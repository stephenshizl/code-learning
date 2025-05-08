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
#include <sys/stat.h>

#include <cutils/log.h>

#include "fis210x.h"
#include <utils/SystemClock.h>
#include <utils/Timers.h>
#ifdef LOG_TAG
#undef   LOG_TAG
#define LOG_TAG "ACC_GYRO"
#endif

#define FIS210X_ACC_GYRO_DEV_NAME    "/dev/fis210x"
#define FIS210X_ACC_INPUT_NAME  	"accelerometer"
#define FIS210X_GYRO_INPUT_NAME  	"gyroscope"

#define IGNORE_EVENT_TIME 350000000
#define ACC_UNIT_CONVERSION(value) ((value)*(9.80665f)/(1000.0f))
#define GYRO_UNIT_CONVERSION(value) ((value)*(3.1415926f)/(1000.0f)/180.0f)

#define FIS210X_USE_SYSFS

#define ACC_IOCTL_BASE                   77
#define GYRO_IOCTL_BASE                  78
#define ACC_IOCTL_SET_ENABLE             _IOW(ACC_IOCTL_BASE, 0, int)
#define ACC_IOCTL_SET_DELAY              _IOW(ACC_IOCTL_BASE, 1, int)
#define GYRO_IOCTL_SET_ENABLE             _IOW(GYRO_IOCTL_BASE, 0, int)
#define GYRO_IOCTL_SET_DELAY              _IOW(GYRO_IOCTL_BASE, 1, int)


static struct sensor_t sSensorList_acc[] = {
#if 1
{
	.name		= "FIS210X Accelerometer",
	.vendor 	= "QST",
	.version	= 1,
	.handle 	= ID_A,
	.type		= SENSOR_TYPE_ACCELEROMETER,
	.maxRange	= (GRAVITY_EARTH * 8.0f),//32.0f,
	.resolution = (GRAVITY_EARTH) / 1024.0f,//4.0f/1024.0f,
	.power		= 5000,//130.0f/1000.0f,
	.minDelay	= 10,
	.fifoReservedEventCount = 0,
	.fifoMaxEventCount = 64,
	.reserved	= {}
}
#else
{
	"FIS210X Accelerometer",
	 "QST",
	 1, 
	 ID_A,
	 SENSOR_TYPE_ACCELEROMETER, 
	 (GRAVITY_EARTH * 8.0f),
	 (GRAVITY_EARTH) / 1024.0f,
	 0.2f,
	 5000,
	 0,
	 0,
	 SENSOR_STRING_TYPE_ACCELEROMETER,
	 "",
	 1000000,
	 SENSOR_FLAG_CONTINUOUS_MODE,
	 {}
},
#endif
};

static struct sensor_t sSensorList_gyro[] = {
#if 1
{
	.name		= "FIS210X Gyroscope",
	.vendor 	= "QST",
	.version	= 1,
	.handle 	= ID_GYRO,
	.type		= SENSOR_TYPE_GYROSCOPE,
	.maxRange	= 2056.0f,//32.0f,
	.resolution = 1/1024.0f,
	.power		= 5000,//130.0f/1000.0f,
	.minDelay	= 10,
	.fifoReservedEventCount = 0,
	.fifoMaxEventCount = 64,
	.reserved	= {}
}
#else
{
	"FIS210X Gyroscope",
	 "QST",
	 1, 
	 ID_GYRO,
	 SENSOR_TYPE_GYROSCOPE, 
	 2056.0f,
	 1/1024.0f,
	 0.2f,
	 5000,
	 0,
	 0,
	 SENSOR_STRING_TYPE_GYROSCOPE,
	 "",
	 1000000,
	 SENSOR_FLAG_CONTINUOUS_MODE,
	 {}
},
#endif
};

#if defined(FIS210X_USE_SYSFS)
static int get_sysfs_path(const char *inputName, char* sysfs_path) 
{
  int fd = -1;
  const char *dirname = "/sys/class/input";
  char devname[PATH_MAX];
  char *filename;
  DIR *dir;  
  int path_len = 0;
  struct dirent *de;

  dir = opendir(dirname);
  if(dir == NULL)
  {
 	ALOGD("open dir %s fail", dirname);
  	return -1;
  }

  memset(devname, 0, sizeof(devname));
  snprintf(devname, sizeof(devname), "%s", dirname);
  path_len = strlen(devname);
  filename = devname + path_len;
  *filename++ = '/';
  path_len++;
  while((de = readdir(dir)))
  {
    if(de->d_name[0] == '.' &&
        (de->d_name[1] == '\0' ||
         (de->d_name[1] == '.' && de->d_name[2] == '\0')))
      continue;
	if(de->d_name[0] != 'i')
		continue;

    snprintf(filename, PATH_MAX - path_len, "%s%c%s", de->d_name, '/', "name");
	ALOGD("input path : %s", devname);
    fd = open(devname, O_RDONLY);
    if(fd >= 0)
	{
      char name[20];

	  memset(name, 0, sizeof(name));
	  read(fd, name, sizeof(name));
	  ALOGD("name: %s", name);	  
	  close(fd);
      if(strlen(name) > 0 && !strncmp(name, inputName, strlen(inputName)))
	  {
        ALOGD("sysfs find %s", devname);
		sprintf(sysfs_path, "/sys/class/input/%s", de->d_name);
        break;
      }
    } 
	else
	{
    	ALOGE("open %s fail , fd=%d", devname, fd);
	}
  }
  closedir(dir);

  return 0;
}
#endif

AccSensor::AccSensor() :
        SensorBase(FIS210X_ACC_GYRO_DEV_NAME, FIS210X_ACC_INPUT_NAME),
                mEnabled(0), mDelay(-1), mInputReader(32), mHasPendingEvent(false)
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = ID_A;
	mPendingEvent.type = SENSOR_TYPE_ACCELEROMETER;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

	//	mEnabledTime =0;
	mPendingEvent.timestamp =0;
	//	m_acc_last_ts = 0;
	mDelay = 0;
	open_device();

	memset(sysfs_path, 0, sizeof(sysfs_path));
#if defined(FIS210X_USE_SYSFS)
	#if 1
	get_sysfs_path(FIS210X_ACC_INPUT_NAME, sysfs_path);
	#else	
	sprintf(sysfs_path, "%s", "/sys/class/misc/fis210x");
	#endif
	ALOGD("sysfs_path : %s", sysfs_path);
	sysfs_path_len = strlen(sysfs_path);
#endif
	ALOGD("AccSensor::AccSensor: construct called");
}

AccSensor::~AccSensor()
{
	if (mEnabled) {
		setEnable(0, 0);
	}
	close_device();

    ALOGD("AccSensor::~AccSensor: destroy called");
}

int AccSensor::setInitialState()
{
	struct input_absinfo absinfo;
	int clockid = CLOCK_BOOTTIME;

	if (mEnabled) {
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_X), &absinfo)) {
			mPendingEvent.acceleration.x = ACC_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_Y), &absinfo)) {
			mPendingEvent.acceleration.y = ACC_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_Z), &absinfo)) {
			mPendingEvent.acceleration.z = ACC_UNIT_CONVERSION(absinfo.value);
		}
	}
	
	if (!ioctl(data_fd, EVIOCSCLOCKID, &clockid))
	{
		ALOGD("AccSensor: set EVIOCSCLOCKID = %d\n", clockid);
	}
	else
	{
		ALOGE("AccSensor: set EVIOCSCLOCKID failed \n");
	}	
		
	return 0;
}

bool AccSensor::hasPendingEvents() const
{
	return mHasPendingEvent;
}

int AccSensor::setEnable(int32_t handle, int enabled)
{
#if defined(FIS210X_USE_SYSFS)
	char path[PATH_MAX];
	int fd = -1;
#endif
	
	/* handle check */
	if(handle != ID_A)
	{
		ALOGE("AccSensor: Invalid handle (%d)", handle);
		return -EINVAL;
    }

#if defined(FIS210X_USE_SYSFS)
	memset(path, 0, sizeof(path));
	sprintf(path,"%s/fis210x/enable_acc", sysfs_path);
    fd = open(path, O_RDWR);
	if(fd < 0)
	{
		ALOGE("AccSensor: open %s fail", path);
		return -1;
	}
#endif	
	if(mEnabled <= 0) 
	{
		if(enabled)
		{
#if defined(FIS210X_USE_SYSFS)
			write(fd, "1", sizeof("1"));
			close(fd);
#else
			if(ioctl(dev_fd, ACC_IOCTL_SET_ENABLE, &enabled))
			{
				ALOGE("AccSensor: ACC_IOCTL_SET_ENABLE error");
				return -5;
			}
#endif
			mEnabled = 1;
			setInitialState();
		}
	} 
	else if (mEnabled == 1) 
	{
		if(!enabled) 
		{
#if defined(FIS210X_USE_SYSFS)
			write(fd, "0", sizeof("0"));
			close(fd);
#else
			if(ioctl(dev_fd, ACC_IOCTL_SET_ENABLE, &enabled))
			{
				ALOGE("AccSensor: ACC_IOCTL_SET_ENABLE error");
				return -5;
			}
#endif
			mEnabled = 0;
		}
	}

	ALOGD("AccSensor: mEnabled = %d", mEnabled);

    return 0;
}

int AccSensor::setDelay(int32_t handle, int64_t delay_ns)
{
	int ms;
#if defined(FIS210X_USE_SYSFS)
	int fd = -1;
	char path[PATH_MAX];
	char buf[10];
#endif

	ALOGD("setDelay: (handle=%d, ns=%lld)",handle, delay_ns);
	/* handle check */
	if(handle != ID_A )
	{
			ALOGE("AccSensor: Invalid handle (%d)", handle);
			return -EINVAL;
	}

	if (mDelay != delay_ns) 
	{
		ms = delay_ns/1000/1000;
#if defined(FIS210X_USE_SYSFS)
		memset(path, 0, sizeof(path));
		sprintf(path,"%s/fis210x/delay_acc", sysfs_path);
		fd = open(path, O_RDWR);
		if(fd < 0)
		{
			ALOGE("AccSensor: open %s fail", path);
			return -1;
		}
		memset(buf, 0, sizeof(buf));
		sprintf(buf, "%d", ms);
		write(fd, buf, sizeof(buf));
		close(fd);
#else
		if(ioctl(dev_fd, ACC_IOCTL_SET_DELAY, &ms))
		{
			ALOGE("AccSensor: ACC_IOCTL_SET_DELAY error");
			return -5;
		}
#endif
		mDelay = delay_ns;
	}

	ALOGD("AccSensor: delay = %lld  mDelay = %lld", delay_ns, mDelay);
	return 0;
}

int64_t AccSensor::getDelay(int32_t handle)
{
	return (handle == ID_A ) ? mDelay : 0;
}

int AccSensor::getEnable(int32_t handle)
{
	return (handle == ID_A) ? mEnabled : 0;
}

int AccSensor::readEvents(sensors_event_t * data, int count)
{
	int numEventReceived = 0;
	input_event const *event;
    static float acc_latest_x;
    static float acc_latest_y;
    static float acc_latest_z;

	if(count < 1)
			return -EINVAL;
		
    if(mHasPendingEvent) {
            mHasPendingEvent = false;
            mPendingEvent.timestamp = getTimestamp();
            *data = mPendingEvent;
            return mEnabled ? 1 : 0;
    }
	ssize_t n = mInputReader.fill(data_fd);
//	ALOGD("data_fd %d\n",data_fd);
	if(n < 0)
		return n;

    while(count && mInputReader.readEvent(&event)) {
        int type = event->type;
//		ALOGD("event type %d code %d\n",type,event->code);
        if(type == EV_ABS) {
            float value = event->value;
            if (event->code == EVENT_TYPE_ACCEL_X) {
                    acc_latest_x = ACC_UNIT_CONVERSION(value);
            } else if (event->code == EVENT_TYPE_ACCEL_Y) {
                    acc_latest_y = ACC_UNIT_CONVERSION(value);
            } else if (event->code == EVENT_TYPE_ACCEL_Z) {
                    acc_latest_z = ACC_UNIT_CONVERSION(value);
            }
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            if(mEnabled) 
			{
                mPendingEvent.acceleration.x = acc_latest_x;
                mPendingEvent.acceleration.y = acc_latest_y;
                mPendingEvent.acceleration.z = acc_latest_z;
				*data++ = mPendingEvent;
				count--;
				numEventReceived++;
			}
			else
			{
				*data++ = mPendingEvent;
				count--;
				numEventReceived++;
				
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
        memcpy(list, sSensorList_acc, sizeof(struct sensor_t) * numSensors);
		ALOGD("AccSensor: populateSensorList numSensors=%d", numSensors);
		
        return numSensors;
}



GyroSensor::GyroSensor() :
        SensorBase(FIS210X_ACC_GYRO_DEV_NAME, FIS210X_GYRO_INPUT_NAME),
                mEnabled(0), mDelay(-1), mInputReader(32), mHasPendingEvent(false)
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = ID_GYRO;
	mPendingEvent.type = SENSOR_TYPE_GYROSCOPE;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

	//	mEnabledTime =0;
	mPendingEvent.timestamp =0;
	//	m_acc_last_ts = 0;
	mDelay = 0;
	open_device();

#if defined(FIS210X_USE_SYSFS)
	memset(sysfs_path, 0, sizeof(sysfs_path));
	get_sysfs_path(FIS210X_GYRO_INPUT_NAME, sysfs_path);
	ALOGD("sysfs_path : %s", sysfs_path);
	sysfs_path_len = strlen(sysfs_path);
#endif
	ALOGD("GyroSensor::GyroSensor: construct called");
}

GyroSensor::~GyroSensor()
{
	if (mEnabled) {
		setEnable(0, 0);
	}
	close_device();

    ALOGD("GyroSensor::~GyroSensor: destroy called");
}

int GyroSensor::setInitialState()
{
	struct input_absinfo absinfo;
	int clockid = CLOCK_BOOTTIME;

	if (mEnabled) {
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_GYRO_X), &absinfo)) {
			mPendingEvent.gyro.x = ACC_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_GYRO_Y), &absinfo)) {
			mPendingEvent.gyro.y = ACC_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_GYRO_Z), &absinfo)) {
			mPendingEvent.gyro.z = ACC_UNIT_CONVERSION(absinfo.value);
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
#if defined(FIS210X_USE_SYSFS)
	char path[PATH_MAX];
	int fd = -1;
#endif
	
	/* handle check */
	if(handle != ID_GYRO)
	{
		ALOGE("GyroSensor: Invalid handle (%d)", handle);
		return -EINVAL;
    }

#if defined(FIS210X_USE_SYSFS)
	memset(path, 0, sizeof(path));
	sprintf(path,"%s/fis210x/enable_gyro", sysfs_path);
    fd = open(path, O_RDWR);
	if(fd < 0)
	{
		ALOGE("GyroSensor: open %s fail", path);
		return -1;
	}
#endif	
	if(mEnabled <= 0) 
	{
		if(enabled)
		{
#if defined(FIS210X_USE_SYSFS)
			write(fd, "1", sizeof("1"));
			close(fd);
#else
			if(ioctl(dev_fd, GYRO_IOCTL_SET_ENABLE, &enabled))
			{
				ALOGE("GyroSensor: ACC_IOCTL_SET_ENABLE error");
				return -5;
			}
#endif
			mEnabled = 1;
			setInitialState();
		}
	} 
	else if (mEnabled == 1) 
	{
		if(!enabled) 
		{
#if defined(FIS210X_USE_SYSFS)
			write(fd, "0", sizeof("0"));
			close(fd);
#else
			if(ioctl(dev_fd, GYRO_IOCTL_SET_ENABLE, &enabled))
			{
				ALOGE("GyroSensor: ACC_IOCTL_SET_ENABLE error");
				return -5;
			}
#endif
			mEnabled = 0;

		}
	}

	ALOGD("GyroSensor: mEnabled = %d", mEnabled);

    return 0;
}

int GyroSensor::setDelay(int32_t handle, int64_t delay_ns)
{
	int ms;
#if defined(FIS210X_USE_SYSFS)
	int fd = -1;
	char path[PATH_MAX];
	char buf[10];
#endif

	ALOGD("setDelay: (handle=%d, ns=%lld)",handle, delay_ns);
	/* handle check */
	if(handle != ID_GYRO)
	{
		ALOGE("GyroSensor: Invalid handle (%d)", handle);
		return -EINVAL;
	}

	if (mDelay != delay_ns) 
	{
		ms = delay_ns/1000/1000;
#if defined(FIS210X_USE_SYSFS)
		memset(path, 0, sizeof(path));
		sprintf(path,"%s/fis210x/delay_gyro", sysfs_path);
		fd = open(path, O_RDWR);
		if(fd < 0)
		{
			ALOGE("GyroSensor: open %s fail", path);
			return -1;
		}
		memset(buf, 0, sizeof(buf));
		sprintf(buf, "%d", ms);
		write(fd, buf, sizeof(buf));		
		close(fd);
#else

		if(ioctl(dev_fd, GYRO_IOCTL_SET_DELAY, &ms))
		{
			ALOGE("GyroSensor: ACC_IOCTL_SET_DELAY error");
			return -5;
		}
#endif
		mDelay = delay_ns;
	}

	ALOGD("GyroSensor: delay = %lld  mDelay = %lld", delay_ns, mDelay);
	return 0;
}

int64_t GyroSensor::getDelay(int32_t handle)
{
	return (handle == ID_GYRO) ? mDelay : 0;
}

int GyroSensor::getEnable(int32_t handle)
{
	return (handle == ID_GYRO) ? mEnabled : 0;
}

int GyroSensor::readEvents(sensors_event_t * data, int count)
{
	int numEventReceived = 0;
	input_event const *event;
    static float gyro_latest_x;
    static float gyro_latest_y;
    static float gyro_latest_z;

	if(count < 1)
			return -EINVAL;
		
    if(mHasPendingEvent) {
            mHasPendingEvent = false;
            mPendingEvent.timestamp = getTimestamp();
            *data = mPendingEvent;
            return mEnabled ? 1 : 0;
    }
	ssize_t n = mInputReader.fill(data_fd);
//	ALOGD("data_fd %d\n",data_fd);
	if(n < 0)
		return n;

    while(count && mInputReader.readEvent(&event)) {
        int type = event->type;
//		ALOGD("event type %d code %d\n",type,event->code);
        if(type == EV_ABS) {
            float value = event->value;
            if (event->code == EVENT_TYPE_GYRO_X) {
                    gyro_latest_x = GYRO_UNIT_CONVERSION(value);
            } else if (event->code == EVENT_TYPE_GYRO_Y) {
                    gyro_latest_y = GYRO_UNIT_CONVERSION(value);
            } else if (event->code == EVENT_TYPE_GYRO_Z) {
                    gyro_latest_z = GYRO_UNIT_CONVERSION(value);
            }
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            if(mEnabled) 
			{
                mPendingEvent.gyro.x = gyro_latest_x;
                mPendingEvent.gyro.y = gyro_latest_y;
                mPendingEvent.gyro.z = gyro_latest_z;
				*data++ = mPendingEvent;
				count--;
				numEventReceived++;
			}
			else
			{
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
        memcpy(list, sSensorList_gyro, sizeof(struct sensor_t) * numSensors);
		ALOGD("GyroSensor: populateSensorList numSensors=%d", numSensors);
		
        return numSensors;
}


