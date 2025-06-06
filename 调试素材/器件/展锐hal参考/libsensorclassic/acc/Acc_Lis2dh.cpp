/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>

#include "AccSensor.h"

//#define LIS2DH_DEBUG
#define XR_ACC_DEV_PATH_NAME    "/dev/lis2dh_acc"
#define XR_ACC_INPUT_NAME  "accelerometer"

#define	XR_ACC_IOCTL_BASE 77
/** The following define the IOCTL command values via the ioctl macros */
#define	XR_ACC_IOCTL_SET_DELAY		_IOW(XR_ACC_IOCTL_BASE, 0, int)
#define	XR_ACC_IOCTL_GET_DELAY		_IOR(XR_ACC_IOCTL_BASE, 1, int)
#define	XR_ACC_IOCTL_SET_ENABLE		_IOW(XR_ACC_IOCTL_BASE, 2, int)
#define	XR_ACC_IOCTL_GET_ENABLE		_IOR(XR_ACC_IOCTL_BASE, 3, int)

#define XR_LAYOUT_NEGX    0x01
#define XR_LAYOUT_NEGY    0x02
#define XR_LAYOUT_NEGZ    0x04

#define ACC_SENSOR_RANGE		GRAVITY_EARTH*2.0f//-2G~2G
#define ACC_SENSOR_SENSITIVITY		((ACC_SENSOR_RANGE *2) /1024.0f)     //3.91mg/LSB
#define ACC_UNIT_CONVERSION(value) ((value)*ACC_SENSOR_SENSITIVITY)

#define XR_GSENSOR_DEVICE_NAME			"sys/bus/i2c/devices/1-0019/"
#define XR_GSENSOR_CALI_NAME			"/productinfo/gsensorcali.txt"

static float x_offset, y_offset, z_offset;
/*****************************************************************************/
static struct sensor_t sSensorList[] = {
	{
		"ST LIS2DH12 3-axis Accelerometer",
		"ST",
		1,
		SENSORS_ACCELERATION_HANDLE,
		SENSOR_TYPE_ACCELEROMETER,
		ACC_SENSOR_RANGE,
		ACC_SENSOR_SENSITIVITY,
		0.145f,
		10000,  // fastest is 100Hz
		0,
		0,
		SENSOR_STRING_TYPE_ACCELEROMETER,
		"",
		1000000,
		SENSOR_FLAG_CONTINUOUS_MODE,
		{}
	},
};

static int get_offset_value() {
  FILE *x_offset_fd = NULL;
  FILE *y_offset_fd = NULL;
  FILE *z_offset_fd = NULL;

  x_offset_fd = fopen(ACC_SENSOR_X_AXIS_OFFSET, "r");
  if (x_offset_fd == NULL) {
    ALOGE("open x_axis_offset file failed!(%s)\n", strerror(errno));
    x_offset = 0;
  } else {
    fscanf(x_offset_fd, "%f", &x_offset);
    fclose(x_offset_fd);
  }

  y_offset_fd = fopen(ACC_SENSOR_Y_AXIS_OFFSET, "r");
  if (y_offset_fd == NULL) {
    ALOGE("open y_axis_offset file failed!(%s)\n", strerror(errno));
    y_offset = 0;
  } else {
    fscanf(y_offset_fd, "%f", &y_offset);
    fclose(y_offset_fd);
  }

  z_offset_fd = fopen(ACC_SENSOR_Z_AXIS_OFFSET, "r");
  if (z_offset_fd == NULL) {
    ALOGE("open z_axis_offset file failed!(%s)\n", strerror(errno));
    z_offset = 0;
  } else {
    fscanf(z_offset_fd, "%f", &z_offset);
    fclose(z_offset_fd);
  }

  ALOGD("AccSensor:getoffset from prodnv x_offset=%f, y_offset=%f, z_offset=%f\n", x_offset, y_offset, z_offset);
  return 0;
}

AccSensor::AccSensor() :
	SensorBase(NULL, XR_ACC_INPUT_NAME),
		mEnabled(0), mDelay(-1), mInputReader(32), mHasPendingEvent(false),
		mSensorCoordinate()
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = ID_A;
	mPendingEvent.type = SENSOR_TYPE_ACCELEROMETER;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

	//open_device();
	if (data_fd) {
		strcpy(input_sysfs_path, XR_GSENSOR_DEVICE_NAME);
		input_sysfs_path_len = strlen(input_sysfs_path);
	} else {
		input_sysfs_path[0] = '\0';
		input_sysfs_path_len = 0;
	}
}
AccSensor::~AccSensor()
{
	if (mEnabled) {
		setEnable(0, 0);
	}

	//close_device();
}

int AccSensor::setInitialState()
{
	struct input_absinfo absinfo;
	int clockid = CLOCK_BOOTTIME;
//	float offset[3];
//	int err;
/*
//	if (mEnabled) {
	if(0){
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_X), &absinfo)) {
			mPendingEvent.acceleration.x = XR_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_Y), &absinfo)) {
			mPendingEvent.acceleration.y = XR_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_Z), &absinfo)) {
			mPendingEvent.acceleration.z = XR_UNIT_CONVERSION(absinfo.value);
		}
	}
*/
        if (!ioctl(data_fd, EVIOCSCLOCKID, &clockid))
        {
            ALOGD("AccSensor: set EVIOCSCLOCKID = %d\n", clockid);
        }
        else
        {
            ALOGE("AccSensor: set EVIOCSCLOCKID failed \n");
        }

/*
	mOffset[0] = mOffset[1] = mOffset[2] = 0.0;
	err = read_cali(XR_GSENSOR_CALI_NAME, offset);
	err = err < 0 ? -errno : 0;
	if (!err) {
		mOffset[0] = offset[0];
		mOffset[1] = offset[1];
		mOffset[2] = offset[2];

		ALOGD("AccSensor::read_cali gsensor cali file ok\n");
	}
	ALOGD("AccSensor::read_cali x %f, y %f, z %f\n", mOffset[0], mOffset[1], mOffset[2]);
*/
	return 0;
}

bool AccSensor::hasPendingEvents() const
{
	return mHasPendingEvent;
}

int AccSensor::setEnable(int32_t handle, int enabled)
{
	int err = 0;
//	int opDone = 0;
	char enable[2];
	enable[0] = '\0';
	enable[1] = '\0';

	/* handle check */
	if (handle != ID_A) {
		ALOGE("AccSensor: Invalid handle (%d)", handle);
		return -EINVAL;
	}


	if(enabled==1)
	strcpy(enable,"1");
	else if(enabled==0)
	strcpy(enable,"0");
	else
	strcpy(enable,"2");

	int newState = enabled ? 1 : 0;
//	LOGD("in SensorXr_Gsensor::setEnable enabled=%d\n",enabled);
	ALOGD("AccSensor::handle=0x%x,enabled=0x%x\n", handle, enabled);
	ALOGD("AccSensor::newState=0x%x,(mEnabled & 1)=0x%x\n", uint32_t(newState), (mEnabled & 1));
	if (uint32_t(newState) != (mEnabled & 1))
	{
       get_offset_value();
		strcpy(&input_sysfs_path[input_sysfs_path_len], "enable_device");
		err = write_sys_attribute(input_sysfs_path, enable, 1);
		err = err < 0 ? -errno : 0;
		if (!err)
		{
			mEnabled &= ~(1);
			mEnabled |= uint32_t(newState);
		}
		ALOGD("AccSensor::mEnabled=0x%x,err=0x%x\n", mEnabled,err);
		setInitialState();
	}


	ALOGD("AccSensor: mEnabled = %d", mEnabled);

	return err;
}

int AccSensor::setDelay(int32_t handle, int64_t delay_ns)
{
	int err = 0;
	int ms;
	char buffer[16];
	int bytes;

	/* handle check */
	if (handle != ID_A) {
		ALOGE("AccSensor: Invalid handle (%d)", handle);
		return -EINVAL;
	}
	ALOGE("AccSensor: mDelay(%lld) delay_ns(%lld)", mDelay,delay_ns);
	if (mDelay != delay_ns) {
		ms = delay_ns / 1000000;
		ALOGE("AccSensor: ms(%d) ", ms);
		strcpy(&input_sysfs_path[input_sysfs_path_len], "pollrate_ms");
		bytes = sprintf(buffer, "%d", ms);
		if (write_sys_attribute(input_sysfs_path, buffer, bytes)){
			ALOGE("SensorXr_Gsensor: set delay fail!");
			return -errno;
		}
		mDelay = delay_ns;
	}
	return err;
}

int64_t AccSensor::getDelay(int32_t handle)
{
	return (handle == ID_A) ? mDelay : 0;
}

int AccSensor::getEnable(int32_t handle)
{
	int enable=0;
 	enable = mEnabled & (1);

	if(enable > 0)
		enable = 1;

	return (handle == ID_A) ? enable : 0;
}

int AccSensor::readEvents(sensors_event_t * data, int count)
{
	if (count < 1)
		return -EINVAL;

#ifdef LIS2DH_DEBUG
   ALOGE("*******AccSensor :  mHasPendingEvent =%d,count=%d",mHasPendingEvent,count);
#endif
	if (mHasPendingEvent) {
		mHasPendingEvent = false;
		mPendingEvent.timestamp = getTimestamp();
		*data = mPendingEvent;
		return mEnabled ? 1 : 0;
	}

	ssize_t n = mInputReader.fill(data_fd);
	if (n < 0)
		return n;

	int numEventReceived = 0;
	input_event const *event;
	static float acc_latest_x;
	static float acc_latest_y;
	static float acc_latest_z;

#ifdef LIS2DH_DEBUG
	static int64_t  numAlldata = 0;
	static int64_t previous_timestamp;
	static int64_t  delta_timestamp = 0;
#endif
	while (count && mInputReader.readEvent(&event)) {
		int type = event->type;
		if (type == EV_ABS || type == EV_MSC) {
			float value = event->value;
			if (event->code == EVENT_TYPE_ACCEL_X) {
                                //mPendingEvent.acceleration.x = ACC_UNIT_CONVERSION(value);
                                acc_latest_x = ACC_UNIT_CONVERSION(value);
                        } else if (event->code == EVENT_TYPE_ACCEL_Y) {
                                //mPendingEvent.acceleration.y = ACC_UNIT_CONVERSION(value);
                                acc_latest_y = ACC_UNIT_CONVERSION(value);
                        } else if (event->code == EVENT_TYPE_ACCEL_Z) {
                                //mPendingEvent.acceleration.z = ACC_UNIT_CONVERSION(value);
                                acc_latest_z = ACC_UNIT_CONVERSION(value);
			}
		} else if (type == EV_SYN) {
			mPendingEvent.timestamp = timevalToNano(event->time);
			if (mEnabled) {
				mPendingEvent.acceleration.x = acc_latest_x/4;
				mPendingEvent.acceleration.y = acc_latest_y/4;
				mPendingEvent.acceleration.z = acc_latest_z/4;
             mSensorCoordinate.coordinate_data_convert(mPendingEvent.acceleration.v,
                 INSTALL_DIR);
             mPendingEvent.acceleration.x += x_offset;
             mPendingEvent.acceleration.y += y_offset;
             mPendingEvent.acceleration.z += z_offset;
				*data++ = mPendingEvent;
				count--;
				numEventReceived++;

#ifdef LIS2DH_DEBUG
				numAlldata++;
				if(1 != numAlldata)
					 delta_timestamp = mPendingEvent.timestamp -previous_timestamp;
	                    if (delta_timestamp > mDelay *15/10)
		                   ALOGE("*******AccSensor  exception:  delta_timestamp=%lld ,mDelay=%lld,mPendingEvent.timestamp =%lld, **** \n",delta_timestamp,mDelay,mPendingEvent.timestamp);
	                   ALOGE("*******AccSensor   :  delta_timestamp=%lld ,mDelay=%lld,mPendingEvent.timestamp =%lld, **** \n",delta_timestamp,mDelay,mPendingEvent.timestamp);
				previous_timestamp = mPendingEvent.timestamp;
#endif

			}
		} else {
			ALOGE("AccSensor: unknown event (type=%d, code=%d)",
			      type, event->code);
		}
		mInputReader.next();
	}

	return numEventReceived;
}

int AccSensor::populateSensorList(struct sensor_t *list)
{
	memcpy(list, sSensorList, sizeof(struct sensor_t) * numSensors);
	return numSensors;
}
