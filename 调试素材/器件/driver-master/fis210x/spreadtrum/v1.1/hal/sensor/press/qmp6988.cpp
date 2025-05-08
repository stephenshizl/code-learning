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

#include <utils/SystemClock.h>
#include <utils/Timers.h>
#include "qmp6988.h"

#ifdef LOG_TAG
#undef   LOG_TAG
#define LOG_TAG "pressure"
#endif

#define QMP6988_DEV_NAME    	"/dev/qmp6988"
#define QMP6988_INPUT_NAME  	"qmp6988"

#define IGNORE_EVENT_TIME 350000000
#define ACC_UNIT_CONVERSION(value) ((value)*(9.80665f)/(1000.0f))
#define GYRO_UNIT_CONVERSION(value) ((value)*(3.1415926f)/(1000.0f)/180.0f)

#define PRESS_IOCTL_SET_ENABLE             _IOW(PRESS_IOCTL_BASE, 0, int)
#define PRESS_IOCTL_SET_DELAY              _IOW(PRESS_IOCTL_BASE, 1, int)
#define PRESS_IOCTL_GET_CALI               _IOR(PRESS_IOCTL_BASE, 2, struct qmp6988_calibration_data)
#define CALIDATA_PATH          				"/sys/bus/platform/drivers/barometer/calidata"
#define RAWDATA_PATH           				"/sys/bus/platform/drivers/barometer/sensordata"


static struct sensor_t sSensorList_press[] = {
#if 1
{
	.name		= "QMP6988 Pressure",
	.vendor 	= "QST",
	.version	= 1,
	.handle 	= ID_PRESS,
	.type		= SENSOR_TYPE_PRESSURE,
	.maxRange	= 300.00f,//32.0f,
	.resolution = 1100.0f,//4.0f/1024.0f,
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


static float Conv_A_S[10][2] = {
	{-6.30E-03,4.30E-04},
	{-1.90E-11,1.20E-10},
	{1.00E-01,9.10E-02},
	{1.20E-08,1.20E-06},
	{3.30E-02,1.90E-02},
	{2.10E-07,1.40E-07},
	{-6.30E-10,3.50E-10},
	{2.90E-13,7.60E-13},
	{2.10E-15,1.20E-14},
	{1.30E-16,7.90E-17},
};

static struct qmp6988_calibration_data qmp6988_cali;
static float a0,b00;
static float a1,a2,bt1,bt2,bp1,b11,bp2,b12,b21,bp3;
static float press, tempearture;
static int qmp6988_cali_fd = -1;
static int qmp6988_data_fd = -1;

float qmp6988_calc_press(void)
{
	char databuf[64];
	double Tr = 0;
	double Pr = 0;
	int Dt, Dp;

	if(qmp6988_data_fd < 0)
	{
		qmp6988_data_fd = open(RAWDATA_PATH, O_RDONLY);
		//ALOGD("qmp6988_data_fd = %d", qmp6988_data_fd);
		memset(databuf, 0, sizeof(databuf));
		TEMP_FAILURE_RETRY(read(qmp6988_data_fd, databuf, sizeof(databuf)-1));
		sscanf(databuf, "%d %d\n", &Dp, &Dt);
		close(qmp6988_data_fd);
		qmp6988_data_fd = -1;
	}

	Tr = a0 + a1*Dt + a2*Dt*Dt;
	//Unit centigrade
	tempearture = (float)(Tr / 256.0f);
	//compensation pressure, Unit Pa
	Pr = b00 + bt1*Tr + bp1*Dp + b11*Tr*Dp + bt2*Tr*Tr + bp2*Dp*Dp + b12*Dp*Tr*Tr + b21*Dp*Dp*Tr + bp3*Dp*Dp*Dp;
	press = (float)Pr/100.0f;
	ALOGD("Dp[%d] Dt[%d] T[%f] Pr [%f]",Dp, Dt, tempearture, press);

	return press;
}

float qmp6988_calc_temperature(void)
{
	char databuf[64];
	double Tr = 0;
	int Dt, Dp;

	if(qmp6988_data_fd < 0)
	{
		qmp6988_data_fd = open(RAWDATA_PATH, O_RDONLY);
		ALOGD("qmp6988_data_fd = %d", qmp6988_data_fd);
		memset(databuf, 0, sizeof(databuf));
		TEMP_FAILURE_RETRY(read(qmp6988_data_fd, databuf, sizeof(databuf)-1));
		sscanf(databuf, "%d %d\n", &Dp, &Dt);
		close(qmp6988_data_fd);
		qmp6988_data_fd = -1;
	}

	Tr = a0 + a1*Dt + a2*Dt*Dt;
	//Unit centigrade
	tempearture = (float)(Tr / 256.0f);

	return tempearture;
}


float qmp6988_calc(int Dp, int Dt)
{
	double Tr = 0;
	double Pr = 0;

	Tr = a0 + a1*Dt + a2*Dt*Dt;
	//Unit centigrade
	tempearture = (float)(Tr / 256.0f);
	//compensation pressure, Unit Pa
	Pr = b00 + bt1*Tr + bp1*Dp + b11*Tr*Dp + bt2*Tr*Tr + bp2*Dp*Dp + b12*Dp*Tr*Tr + b21*Dp*Dp*Tr + bp3*Dp*Dp*Dp;
	press = (float)Pr/100.0f;
	ALOGD("Dp[%d] Dt[%d] T[%f] Pr [%f]",Dp, Dt, tempearture, press);

	return press;
}


int qmp6988_algo_init(char *path)
{
	char calibuf[256];

	ALOGD("qmp6988_algo_init");
	if(qmp6988_cali_fd<0)
	{
		qmp6988_cali_fd = open(path, O_RDONLY);
		ALOGD("qmp6988_cali_fd=%d", qmp6988_cali_fd);
	}
	else
	{
		return 0;
	}
	
	if(qmp6988_cali_fd>0)
	{
		int err;
		//err = ioctl(fd, BAROMETER_GET_CALI, &qmp6988_cali);
		//ALOGD("ioctl err = %d", err);
		memset(calibuf, 0, sizeof(calibuf));
		read(qmp6988_cali_fd, calibuf, sizeof(calibuf)-1);
		sscanf(calibuf, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
			&qmp6988_cali.COE_a0,&qmp6988_cali.COE_a1,&qmp6988_cali.COE_a2,
			&qmp6988_cali.COE_b00,&qmp6988_cali.COE_bt1,&qmp6988_cali.COE_bt2,
			&qmp6988_cali.COE_bp1,&qmp6988_cali.COE_b11,
			&qmp6988_cali.COE_bp2,&qmp6988_cali.COE_b12,
			&qmp6988_cali.COE_b21,&qmp6988_cali.COE_bp3);

		close(qmp6988_cali_fd);
		qmp6988_cali_fd = -1;

		a0 = qmp6988_cali.COE_a0 /16.0f;
		b00 = qmp6988_cali.COE_b00 /16.0f;

		a1 = Conv_A_S[0][0] + Conv_A_S[0][1] * qmp6988_cali.COE_a1 / 32767.0f;
		a2 = Conv_A_S[1][0] + Conv_A_S[1][1] * qmp6988_cali.COE_a2 / 32767.0f;
		bt1 = Conv_A_S[2][0] + Conv_A_S[2][1] * qmp6988_cali.COE_bt1 / 32767.0f;
		bt2 = Conv_A_S[3][0] + Conv_A_S[3][1] * qmp6988_cali.COE_bt2 / 32767.0f;
		bp1 = Conv_A_S[4][0] + Conv_A_S[4][1] * qmp6988_cali.COE_bp1 / 32767.0f;
		b11 = Conv_A_S[5][0] + Conv_A_S[5][1] * qmp6988_cali.COE_b11 / 32767.0f;
		bp2 = Conv_A_S[6][0] + Conv_A_S[6][1] * qmp6988_cali.COE_bp2 / 32767.0f;
		b12 = Conv_A_S[7][0] + Conv_A_S[7][1] * qmp6988_cali.COE_b12 / 32767.0f;
		b21 = Conv_A_S[8][0] + Conv_A_S[8][1] * qmp6988_cali.COE_b21 / 32767.0f;
		bp3 = Conv_A_S[9][0] + Conv_A_S[9][1] * qmp6988_cali.COE_bp3 / 32767.0f;

		ALOGD("<-----------get calibration data-------------->\n");
		ALOGD("COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\n",
			qmp6988_cali.COE_a0,qmp6988_cali.COE_a1,qmp6988_cali.COE_a2,qmp6988_cali.COE_b00);
		ALOGD("COE_bt1[%d] COE_bt2[%d] COE_bp1[%d] COE_b11[%d]\n",
			qmp6988_cali.COE_bt1,qmp6988_cali.COE_bt2,qmp6988_cali.COE_bp1,qmp6988_cali.COE_b11);
		ALOGD("COE_bp2[%d] COE_b12[%d] COE_b21[%d] COE_bp3[%d]\n",
			qmp6988_cali.COE_bp2,qmp6988_cali.COE_b12,qmp6988_cali.COE_b21,qmp6988_cali.COE_bp3);
		ALOGD("<-----------calibration data done-------------->\n");

		ALOGD("<----------- float calibration data -------------->\n");
		ALOGD("a0[%lle] a1[%lle] a2[%lle] b00[%lle]\n",a0,a1,a2,b00);
		ALOGD("bt1[%lle]	bt2[%lle]	bp1[%lle]	b11[%lle]\n",bt1,bt2,bp1,b11);
		ALOGD("bp2[%lle]	b12[%lle]	b21[%lle]	bp3[%lle]\n",bp2,b12,b21,bp3);
		ALOGD("<----------- float calibration data dones-------------->\n");

		return 1;
	}
	else
	{
		ALOGE("can not open %s", path);
		return 0;
	}
}


void qmp6988_algo_deinit(void)
{
	if(qmp6988_cali_fd > 0)
	{
		close(qmp6988_cali_fd);
		qmp6988_cali_fd = -1;
	}
}


#if defined(QMP6988_USE_SYSFS)
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

PressSensor::PressSensor() :
        SensorBase(QMP6988_DEV_NAME, QMP6988_INPUT_NAME),
                mEnabled(0), mDelay(-1), mInputReader(32), mHasPendingEvent(false)
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = ID_PRESS;
	mPendingEvent.type = SENSOR_TYPE_PRESSURE;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

	//	mEnabledTime =0;
	mPendingEvent.timestamp =0;
	//	m_acc_last_ts = 0;
	mDelay = 0;
	open_device();

	memset(sysfs_path, 0, sizeof(sysfs_path));
#if defined(QMP6988_USE_SYSFS)
	#if 0
	get_sysfs_path(QMP6988_INPUT_NAME, sysfs_path);
	#else
	sprintf(sysfs_path, "%s", "/sys/class/misc/qmp6988");
	#endif
	ALOGD("sysfs_path : %s", sysfs_path);
	sysfs_path_len = strlen(sysfs_path);
	char cali_path[128];
	sprintf(cali_path, "%s/cali", sysfs_path);
	qmp6988_algo_init(cali_path);
#endif
	ALOGD("PressSensor::PressSensor: construct called");
}

PressSensor::~PressSensor()
{
	if (mEnabled) {
		setEnable(0, 0);
	}
	close_device();

    ALOGD("PressSensor::~PressSensor: destroy called");
}

int PressSensor::setInitialState()
{
	struct input_absinfo absinfo;
	int clockid = CLOCK_BOOTTIME;

	if (mEnabled) {
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_PRESS_X), &absinfo)) {
			mPendingEvent.pressure = 0;
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_PRESS_Y), &absinfo)) {
			mPendingEvent.pressure = 0;
		}
	}
	
	if (!ioctl(data_fd, EVIOCSCLOCKID, &clockid))
	{
		ALOGD("PressSensor: set EVIOCSCLOCKID = %d\n", clockid);
	}
	else
	{
		ALOGE("PressSensor: set EVIOCSCLOCKID failed \n");
	}	
		
	return 0;
}

bool PressSensor::hasPendingEvents() const
{
	return mHasPendingEvent;
}

int PressSensor::setEnable(int32_t handle, int enabled)
{
#if defined(QMP6988_USE_SYSFS)
	char path[PATH_MAX];
	int fd = -1;
#endif
	
	/* handle check */
	if(handle != ID_PRESS)
	{
		ALOGE("PressSensor: Invalid handle (%d)", handle);
		return -EINVAL;
    }

#if defined(QMP6988_USE_SYSFS)
	memset(path, 0, sizeof(path));
	sprintf(path,"%s/enable", sysfs_path);
    fd = open(path, O_RDWR);
	if(fd < 0)
	{
		ALOGE("PressSensor: open %s fail", path);
		return -1;
	}
#endif	
	if(mEnabled <= 0) 
	{
		if(enabled)
		{
#if defined(QMP6988_USE_SYSFS)
			write(fd, "1", sizeof("1"));
			close(fd);
#else
			if(ioctl(dev_fd, PRESS_IOCTL_SET_ENABLE, &enabled))
			{
				ALOGE("PressSensor: ACC_IOCTL_SET_ENABLE error");
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
#if defined(QMP6988_USE_SYSFS)
			write(fd, "0", sizeof("0"));
			close(fd);
#else
			if(ioctl(dev_fd, PRESS_IOCTL_SET_ENABLE, &enabled))
			{
				ALOGE("PressSensor: ACC_IOCTL_SET_ENABLE error");
				return -5;
			}
#endif
			mEnabled = 0;
		}
	}

	ALOGD("PressSensor: mEnabled = %d", mEnabled);

    return 0;
}

int PressSensor::setDelay(int32_t handle, int64_t delay_ns)
{
	int ms;
#if defined(QMP6988_USE_SYSFS)
	int fd = -1;
	char path[PATH_MAX];
	char buf[10];
#endif

	ALOGD("setDelay: (handle=%d, ns=%lld)",handle, delay_ns);
	/* handle check */
	if(handle != ID_PRESS)
	{
			ALOGE("PressSensor: Invalid handle (%d)", handle);
			return -EINVAL;
	}

	if (mDelay != delay_ns) 
	{
		ms = delay_ns/1000/1000;
#if defined(QMP6988_USE_SYSFS)
		memset(path, 0, sizeof(path));
		sprintf(path,"%s/delay", sysfs_path);
		fd = open(path, O_RDWR);
		if(fd < 0)
		{
			ALOGE("PressSensor: open %s fail", path);
			return -1;
		}
		memset(buf, 0, sizeof(buf));
		sprintf(buf, "%d", ms);
		write(fd, buf, sizeof(buf));
		close(fd);
#else
		if(ioctl(dev_fd, PRESS_IOCTL_SET_DELAY, &ms))
		{
			ALOGE("PressSensor: ACC_IOCTL_SET_DELAY error");
			return -5;
		}
#endif
		mDelay = delay_ns;
	}

	ALOGD("PressSensor: delay = %lld  mDelay = %lld", delay_ns, mDelay);
	return 0;
}

int64_t PressSensor::getDelay(int32_t handle)
{
	return (handle == ID_PRESS) ? mDelay : 0;
}

int PressSensor::getEnable(int32_t handle)
{
	return (handle == ID_PRESS) ? mEnabled : 0;
}

int PressSensor::readEvents(sensors_event_t * data, int count)
{
	int numEventReceived = 0;
	input_event const *event;
    static float press_latest;
    static float temp_latest;

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
            if (event->code == EVENT_TYPE_PRESS_X) {
                    press_latest = value;
            } else if (event->code == EVENT_TYPE_PRESS_Y) {
                    temp_latest = value;
            }
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            if(mEnabled) 
			{
                mPendingEvent.pressure = qmp6988_calc(press_latest, temp_latest);
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
                ALOGE("PressSensor: unknown event (type=%d, code=%d)",
                      type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}

int PressSensor::populateSensorList(struct sensor_t *list)
{
        memcpy(list, sSensorList_press, sizeof(struct sensor_t) * numSensors);
		ALOGD("PressSensor: populateSensorList numSensors=%d", numSensors);
		
        return numSensors;
}


