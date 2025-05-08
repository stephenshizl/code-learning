
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

#include "GyroSensor.h"

#ifdef LOG_TAG
#undef   LOG_TAG
#define LOG_TAG "gyroscope"
#endif

#define QMI8610_GYRO_DEV_PATH_NAME				"/dev/qmi8610"
#define QMI8610_GYRO_INPUT_NAME					"gyroscope"

#define QMI8610_GYRO_IOCTL_BASE                   78

#define QMI8610_GYRO_IOCTL_SET_ENABLE             _IOW(QMI8610_GYRO_IOCTL_BASE, 0, int)
#define QMI8610_GYRO_IOCTL_SET_DELAY              _IOW(QMI8610_GYRO_IOCTL_BASE, 1, int)

#define GYRO_UNIT_CONVERSION(value)		((value)*(3.1415926f)/(1000.0f)/180.0f)		// mdps-->rad/s   (((value)/1000)*(PI/180))

/*****************************************************************************/
static struct sensor_t sSensorList[] = {
	{
		.name		= "QMI8610 3-axis Gyroscope",
		.vendor 	= "QST",
		.version	= 1,
		.handle 	= SENSORS_GYROSCOPE_HANDLE,
		.type		= SENSOR_TYPE_GYROSCOPE,
		.maxRange	= 2056.0f,//32.0f,
		.resolution = 1/1024.0f,//4.0f/1024.0f,
		.power		= 5000,//130.0f/1000.0f,
		.minDelay   = 10,
		.fifoReservedEventCount = 0,
		.fifoMaxEventCount = 64,
		.stringType = SENSOR_STRING_TYPE_GYROSCOPE,
		.requiredPermission = "",
		.maxDelay = 1000000, //us
		.flags = SENSOR_FLAG_CONTINUOUS_MODE,
		.reserved	= {}
	}

};

GyroSensor::GyroSensor() :
        SensorBase(QMI8610_GYRO_DEV_PATH_NAME, QMI8610_GYRO_INPUT_NAME),
                mEnabled(0), mDelay(-1), mInputReader(32), mHasPendingEvent(false),
                mSensorCoordinate()
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = SENSORS_GYROSCOPE_HANDLE;
    mPendingEvent.type = SENSOR_TYPE_GYROSCOPE;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

	open_device();

    ALOGD("GyroSensor::GyroSensor(Gyro_qmi8610.cpp): construct called");
}

GyroSensor::~GyroSensor()
{
	if (mEnabled) {
		setEnable(0, 0);
	}
	close_device();

    ALOGD("GyroSensor::~GyroSensor(Gyro_qmi8610.cpp): destroy called");
}

int GyroSensor::setInitialState()
{
	struct input_absinfo absinfo;
	int clockid = CLOCK_BOOTTIME;

	//	if (mEnabled) {
	if(1){
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_GYRO_X), &absinfo)) {
			mPendingEvent.acceleration.x = GYRO_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_GYRO_Y), &absinfo)) {
			mPendingEvent.acceleration.y = GYRO_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_GYRO_Z), &absinfo)) {
			mPendingEvent.acceleration.z = GYRO_UNIT_CONVERSION(absinfo.value);
		}
	}
	
	if (!ioctl(data_fd, EVIOCSCLOCKID, &clockid))
	{
		ALOGD("GyroSensor: set EVIOCSCLOCKID = %d\n", clockid);
	}
	else
	{
		ALOGE("GyroSensor: set EVIOCSCLOCKID failed \n");
	}	
		
	return 0;
}

bool GyroSensor::hasPendingEvents() const
{
	return mHasPendingEvent;
}

int GyroSensor::setEnable(int32_t handle, int enabled)
{
	int err = 0;
	//	int opDone = 0;

	/* handle check */
	if (handle != SENSORS_GYROSCOPE_HANDLE)
	{
		ALOGE("GyroSensor: Invalid handle (%d)", handle);
		return -EINVAL;
	}

	if (mEnabled <= 0) {
		if (enabled) {

			setInitialState();

			err = ioctl(dev_fd, QMI8610_GYRO_IOCTL_SET_ENABLE,
					&enabled);
		}
	} else if (mEnabled == 1) {
		if (!enabled) {
			err = ioctl(dev_fd, QMI8610_GYRO_IOCTL_SET_ENABLE,
					&enabled);
		}
	}
	if (err != 0) {
		ALOGE("GyroSensor: IOCTL failed (%s)", strerror(errno));
		return err;
 	}


	if(enabled) {
		mEnabled++;
		if(mEnabled > 32767)
			mEnabled = 32767;
	} else {
		mEnabled--;
		if(mEnabled < 0)
			mEnabled = 0;
	}
	ALOGD("GyroSensor(Gyro_qmi8610.cpp): mEnabled = %d", mEnabled);

	return err;
}

int GyroSensor::setDelay(int32_t handle, int64_t delay_ns)
{
	int err = 0;
	int ms;

	ALOGD("setDelay: (handle=%d, ns=%lld)",handle, delay_ns);
	/* handle check */
	if(handle != SENSORS_GYROSCOPE_HANDLE ) {
		ALOGE("GyroSensor: Invalid handle (%d)", handle);
		return -EINVAL;
	}

	if(mDelay != delay_ns) 
	{
		ms = delay_ns/1000/1000;	
		if (ioctl(dev_fd, QMI8610_GYRO_IOCTL_SET_DELAY, &ms)) {
			return -errno;
		}
		mDelay = delay_ns;
	}

	ALOGD("GyroSensor(Gyro_qmi8610.cpp): delay = %lld", delay_ns);

	return err;
}

int64_t GyroSensor::getDelay(int32_t handle)
{
	return (handle == SENSORS_GYROSCOPE_HANDLE ) ? mDelay : 0;
}

int GyroSensor::getEnable(int32_t handle)
{
	return (handle == SENSORS_GYROSCOPE_HANDLE) ? mEnabled : 0;
}

static float gyro_latest_x = 0.0f;
static float gyro_latest_y = 0.0f;
static float gyro_latest_z = 0.0f;
int GyroSensor::readEvents(sensors_event_t * data, int count)
{
	if(count < 1)
		return -EINVAL;
		
    if(mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }
	ssize_t n = mInputReader.fill(data_fd);
	ALOGD("data_fd %d\n",data_fd);
	if(n < 0)
		return n;

	int numEventReceived = 0;
	input_event const *event;

    while(count && mInputReader.readEvent(&event)) {
        int type = event->type;
		ALOGD("event type %d code %d\n",type,event->code);
        if (type == EV_ABS) {
            if (event->code == EVENT_TYPE_GYRO_X) {
                gyro_latest_x = GYRO_UNIT_CONVERSION(event->value);
            } else if (event->code == EVENT_TYPE_GYRO_Y) {
                gyro_latest_y = GYRO_UNIT_CONVERSION(event->value);
            } else if (event->code == EVENT_TYPE_GYRO_Z) {
                gyro_latest_z = GYRO_UNIT_CONVERSION(event->value);
            } else if (event->code == EVENT_TYPE_GYRO_STATUS) {
                mPendingEvent.gyro.status = event->value;
            }	
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            if (mEnabled) {
                mPendingEvent.gyro.x = gyro_latest_x;
                mPendingEvent.gyro.y = gyro_latest_y;
                mPendingEvent.gyro.z = gyro_latest_z;
				mSensorCoordinate.coordinate_data_convert(
						mPendingEvent.acceleration.v, INSTALL_DIR);
				*data++ = mPendingEvent;
				count--;
				numEventReceived++;
			}
        } else {
                ALOGE("GyroSensor: unknown event (type=%d, code=%d)",
                      type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}

int GyroSensor::populateSensorList(struct sensor_t *list)
{
        memcpy(list, sSensorList, sizeof(struct sensor_t) * numSensors);
		ALOGD("GyroSensor: populateSensorList numSensors=%d", numSensors);
		
        return numSensors;
}

