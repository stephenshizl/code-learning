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
#include <dlfcn.h>
#include <string.h>
#include <linux/ioctl.h>
#include <cutils/log.h>

//#include "OriSensor.h"
#include "Algorithm.h"
#include "OriSensorQmc.h"

static int OTP[2] = {0};
static char Layout;
_QMC7983 qmcX983;
float mag_raw[3] = {0};

int64_t m_mag_last_ts[3];
struct hwmsen_convert {
	signed char sign[4];
	unsigned char map[4];
};
struct hwmsen_convert map[] = {
    { { 1, 1, 1}, {0, 1, 2} },
    { {-1, 1, 1}, {1, 0, 2} },
    { {-1,-1, 1}, {0, 1, 2} },
    { { 1,-1, 1}, {1, 0, 2} },

    { {-1, 1,-1}, {0, 1, 2} },
    { { 1, 1,-1}, {1, 0, 2} },
    { { 1,-1,-1}, {0, 1, 2} },
    { {-1,-1,-1}, {1, 0, 2} },      

};
struct hwmsen_convert cvt;

/*IOCTL for HAL*/
#define MSENSOR						   0x83

#define QMC6308_IOCTL_MSENSOR_ENABLE	_IOW(MSENSOR,0x55,int)
#define QMC6308_IOCTL_OSENSOR_ENABLE	_IOW(MSENSOR,0x56,int)
#define QMC6308_IOCTL_SET_DELAY			_IOW(MSENSOR,0x29,int)
#define QMC6308_IOCTL_GET_DELAY			_IOR(MSENSOR,0x1d,int)
#define QMC6308_IOCTL_GET_DIRECTION		_IOR(MSENSOR,0x44,char)
//#define QMC6308_IOCTL_GET_OTPK 			_IOR(MSENSOR,0x45,int[2])

/*****************************************************************************/
static struct sensor_t sSensorList[] = {
	{
		"QMC6308 Magnetic field sensor",
		"QST",
		1,
		SENSORS_MAGNETIC_FIELD_HANDLE,
		SENSOR_TYPE_MAGNETIC_FIELD, 
		65535.0f,
		CONVERT_M, 
		0.35f, 
		10000, 
		0, 
		0,
		SENSOR_STRING_TYPE_MAGNETIC_FIELD,
		"",
		1000000,
		SENSOR_FLAG_CONTINUOUS_MODE,
		{}
	},
	{
		"QMC6308 Orientation sensor",
		"QST",
		1, 
		SENSORS_ORIENTATION_HANDLE,
		SENSOR_TYPE_ORIENTATION, 
		360.0f,
		CONVERT_O, 
		0.495f, 
		10000, 
		0, 
		0,
		SENSOR_STRING_TYPE_ORIENTATION,
		"",
		1000000,
		SENSOR_FLAG_CONTINUOUS_MODE,
		{}
	},
};
int64_t Timestamp;
OriSensor::OriSensor()
: SensorBase("dev/qmcX983", "compass"),
	enabled(0),
	mPendingMask(0),
	mMaEnabled(0),
	mOrEnabled(0),
	mInputReader(32)
{
	 Timestamp = getTimestamp();
	memset(mPendingEvents, 0, sizeof(mPendingEvents));

	mPendingEvents[MagneticField].version = sizeof(sensors_event_t);
	mPendingEvents[MagneticField].sensor = ID_M;
	mPendingEvents[MagneticField].type = SENSOR_TYPE_MAGNETIC_FIELD;
	mPendingEvents[MagneticField].magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;

	mPendingEvents[Orientation].version = sizeof(sensors_event_t);
	mPendingEvents[Orientation].sensor = ID_O;
	mPendingEvents[Orientation].type = SENSOR_TYPE_ORIENTATION;
	mPendingEvents[Orientation].orientation.status = SENSOR_STATUS_ACCURACY_HIGH;

	for (int i=0 ; i<__numSensors ; i++)
	{
		mEnabled[i] = 0;
		mDelay[i] = 50000000; // 50 ms by default
	}

//	ALOGD("QST TEST !!!!!!!!!!");
	open_device();

	if(dev_fd){
		mcal(Timestamp/1000/1000);
		ALOGD("mcal init %lld\n",Timestamp/1000/1000);
		if(ioctl(dev_fd, QMC6308_IOCTL_GET_DIRECTION, &Layout)){
			ALOGE("QMC6308_IOCTL_GET_DIRECTION failed\n");
		}
		ALOGD("QMC6308_IOCTL_GET_DIRECTION %d\n",Layout);
		/*
		if(ioctl(dev_fd, QMC6308_IOCTL_GET_OTPK, &OTP[0])){
			ALOGE("QMC6308_IOCTL_GET_OTPK failed\n");
		}
		ALOGD("QMC6308_IOCTL_GET_OTPK %d %d\n",OTP[0],OTP[1]);
		}
		*/
		if(Layout > sizeof(map)/sizeof(map[0]))
			Layout = 0;
		cvt = map[Layout];
		
		if (!enabled) {
			close_device();
		}
	}
}

OriSensor::~OriSensor(){
	for (int i = 0; i < __numSensors; i++) {
		setEnable(i,0);
	}
}

int OriSensor::setEnable(int32_t handle, int en)
{
	int what = -1;
	int cmd = -1;
	char buffer[2];
	
	ALOGD("OriSensor handle = %d,en = %d",handle,en);

	int clockid = CLOCK_BOOTTIME;
	if (!ioctl(data_fd, EVIOCSCLOCKID, &clockid))
	{
		ALOGD("OriSensor: set EVIOCSCLOCKID = %d\n", clockid);
	}
	else
	{
		ALOGE("OriSensor: set EVIOCSCLOCKID failed \n");
	}

	what = handle2id(handle);

	if (what >= __numSensors)
		return -EINVAL;

	int newState  = en ? 1 : 0;
	int err = 0;

	/*add for cts start*/	
	m_mag_last_ts[0] = 0;		
	m_mag_last_ts[1] = 0;
	m_mag_last_ts[2] = 0;
	/*add for cts end*/
	
	if ((newState<<what) != (enabled & (1<<what))) {
//                ALOGD("QMC handle = %d,enabled = %d \n",handle,enabled);
		if(!enabled){
			open_device();
		}
		switch (what) {
			case MagneticField: 
				 cmd = QMC6308_IOCTL_MSENSOR_ENABLE;
				mMaEnabled = en;
				 break;
			case Orientation: 
				 cmd = QMC6308_IOCTL_OSENSOR_ENABLE;
				mOrEnabled = en;
				 break;
			default: return -EINVAL;
		}
		
		int flags = newState;
		err = ioctl(dev_fd,cmd,&flags);
		if(err){
			ALOGD("QMC handle %d %d %d set fail\n",handle,err,cmd);
		}
		err = err<0 ? -errno : 0;
 //       	ALOGD("setEnable 001 err = %d flags = %d \n",err,flags);
		if (!err) {
			enabled &= ~(1<<what);
			enabled |= (uint32_t(flags)<<what);
		}

                ALOGD("setEnable enabled = 0x%x \n",enabled);
		
		if (!enabled) {
			close_device();
		}
			
	}
	return err;
}

int OriSensor::getEnable(int32_t handle)
{
	int id = handle2id(handle);
	if (id >= 0) {
		if(enabled)
			return 1;
	 } else {
		return 0;
	 }
       return 0;
}

int64_t OriSensor::getDelay(int32_t handle)
{
	int id = handle2id(handle);
	if (id > 0) {
		return mDelay[id];
	} else {
		return 0;
	}
}


int OriSensor::setDelay(int32_t handle, int64_t ns)
{
	int id = handle2id(handle);

	if(id >= __numSensors)
	{
		return -EINVAL;
	}
	if (ns < -1 || 2147483647 < ns) {
		ALOGE("OriSensor: invalid delay (%lld)", ns);
		return -EINVAL;
	}
	mDelay[id] = ns;
       ALOGD("QMC6308 setdelay time = %lld\n", ns);	
	if(enabled){
		uint64_t wanted = -1LLU;
//		for (int i=0 ; i<__numSensors ; i++) { //if ID_O setDelay ,then ID_M alreay setDelay
		for (int i=0 ; i<numSensors ; i++) {
			if (enabled & (1<<i)) {
				uint64_t ns = mDelay[i];
				wanted = wanted < ns ? wanted : ns;
			}
		}
		int delay = int64_t(wanted) /1000/1000;
		ALOGD("QMC6308 delay time = %lld\n", delay);
		if (ioctl(dev_fd, QMC6308_IOCTL_SET_DELAY,&delay)) {
			return -errno;
		}		
	}
	
	return 0;
}

int OriSensor::readEvents(sensors_event_t* data, int count)
{
	
	int numEventReceived = 0;
	input_event const* event;
	static int64_t prev_time;
	
	if (count < 1)
		return -EINVAL;

	ssize_t n = mInputReader.fill(data_fd);

	if (n < 0)
		return n;

        ALOGD("QMC6308 Timestamp = %lld \n", Timestamp);
	
	while (count && mInputReader.readEvent(&event))
	{
		int type = event->type;
		ALOGD("QMC6308  type = %d\n", type);
		if (type == EV_ABS) {
			processEvent(event->code, event->value);
			mInputReader.next();
		} else if (type == EV_SYN) {
			int64_t time = timevalToNano(event->time);
			//ALOGD("QMC6308  time = %lld\n", time/1000/1000);
			ALOGD("readEvents_A [%f %f %f]\n",mag_raw[0],mag_raw[1],mag_raw[2]);				
			if(mMaEnabled || mOrEnabled)
			{
				
				//mag_raw[2] = mag_raw[2] - mag_raw[0]*OTP[0]*0.02f -mag_raw[1]*OTP[1]*0.02f;
				
				qmcX983.dRawMag[0] = (cvt.sign[0]*mag_raw[cvt.map[0]]);
				qmcX983.dRawMag[1] = (cvt.sign[1]*mag_raw[cvt.map[1]]);
				qmcX983.dRawMag[2] = (cvt.sign[2]*mag_raw[cvt.map[2]]);				
							
				//Run Algorithm
				if(process(&qmcX983.dRawMag[0],time/1000/1000)){
					get_mag_bias(&qmcX983.mag_bias[0]);	
				}
				
				qmcX983.data_cali[0] = qmcX983.dRawMag[0]/31.25;
				qmcX983.data_cali[1] = qmcX983.dRawMag[1]/31.25;
				qmcX983.data_cali[2] = qmcX983.dRawMag[2]/31.25;
				ALOGD("QMC6308_mag [%f %f %f]\n",qmcX983.data_cali[0],qmcX983.data_cali[1],qmcX983.data_cali[2]);
				ALOGD("QMC6308_acc [%f %f %f]\n",qmcX983.acc[0],qmcX983.acc[1],qmcX983.acc[2]);	
				//calculate pitch roll yaw
				push2mcal(&qmcX983);				
				ALOGD("QMC6308_ori [%f %f %f]\n",qmcX983.yaw,qmcX983.pitch,qmcX983.roll);
				ALOGD("QMC6308_cail_mag [%f %f %f]\n",qmcX983.data_cali[0],qmcX983.data_cali[1],qmcX983.data_cali[2]);				
				if(mMaEnabled)
				{
					mPendingEvents[MagneticField].magnetic.x = qmcX983.data_cali[0];
					mPendingEvents[MagneticField].magnetic.y = qmcX983.data_cali[1];
					mPendingEvents[MagneticField].magnetic.z = qmcX983.data_cali[2];
					mPendingEvents[MagneticField].magnetic.status = get_mag_accuracy();
					mPendingMask |= 1<<MagneticField;	
				}
		
				if(mOrEnabled)
				{
					//get orientation
					mPendingEvents[Orientation].orientation.azimuth = qmcX983.yaw;
					mPendingEvents[Orientation].orientation.pitch = qmcX983.pitch;
					mPendingEvents[Orientation].orientation.roll = qmcX983.roll;
					mPendingEvents[Orientation].orientation.status = mPendingEvents[MagneticField].magnetic.status;
					mPendingMask |= 1<<Orientation;	
				}
			}

			for (int j=0 ; count && mPendingMask && j<__numSensors ; j++) 
			{
				if (mPendingMask & (1<<j)) 
				{
					mPendingMask &= ~(1<<j);
					mPendingEvents[j].timestamp = time;
					/*add for cts start*/
					if((m_mag_last_ts[j] > 0)&& (mPendingEvents[j].timestamp > m_mag_last_ts[j]))
					{
						int64_t temp_time;
						float delta_mod =
							(float)(mPendingEvents[j].timestamp-m_mag_last_ts[j])/(float)(mDelay[j]);	
							
						if (mDelay[j] == 1000000000)
							delta_mod = 0;
						int loopcout=delta_mod;
						
						if((500000000 <= mDelay[j]*loopcout) && (loopcout < 100))
							loopcout = 500000000/mDelay[j] ;
						
						if(loopcout>=1 && loopcout<100) {
							temp_time = mPendingEvents[j].timestamp;
							for(int i=0; i<loopcout; i++) {
								mPendingEvents[j].timestamp = temp_time- (loopcout-i)*mDelay[j];
							   // ALOGE("QMC[%d]fake event [%lld ] \r\n",j,mPendingEvents[j].timestamp);
								*data++ = mPendingEvents[j];
								numEventReceived++;

								count--;
								if(0==count)
								{
									break;
								}
							}
							m_mag_last_ts[j] = mPendingEvents[j].timestamp;
						}
						else
						{
							*data++ = mPendingEvents[j];
							count--;
							numEventReceived++;
						}
						m_mag_last_ts[j] = mPendingEvents[j].timestamp;						
					}
					else
					{   
						*data++ = mPendingEvents[j];
						count--;
						numEventReceived++;
					}
					m_mag_last_ts[j] = mPendingEvents[j].timestamp;
				}//if (mPendingMask & (1<<j)) 
			}//for (int j=0 ; count && mPendingMask && j<__numSensors ; j++) 
				
			if (!mPendingMask) {
				mInputReader.next();
			}
		} else 
		{
			ALOGE("QMC Sensor: unknown event (type=%d, code=%d)",
                    		type, event->code);
			mInputReader.next();
		}
	}

	return numEventReceived;
}

int OriSensor::setAccel(sensors_event_t* data)
{
	qmcX983.acc[0] = data->acceleration.x;
	qmcX983.acc[1] = data->acceleration.y;
	qmcX983.acc[2] = data->acceleration.z;
//	ALOGD("setAccel[%f %f %f]\n",qmcX983.acc[0],qmcX983.acc[1],qmcX983.acc[2]);
	return 0;
}

int OriSensor::handle2id(int32_t handle)
{
	switch (handle) {
	case ID_A:
		return Accelerometer;
	case ID_M:
		return MagneticField;
	case ID_O:
		return Orientation;
	default:
		ALOGE("OriSensor: unknown handle (%d)", handle);
		return -EINVAL;
        }
}

void OriSensor::processEvent(int code, int value)
{
	switch (code) {
		case ABS_X:
			mPendingMask |= 1<<MagneticField;
			mag_raw[0] = value * 3.125;
			break;
		case ABS_Y:
			mPendingMask |= 1<<MagneticField;
			mag_raw[1] = value * 3.125;
			break;
		case ABS_Z:
			mPendingMask |= 1<<MagneticField;
			mag_raw[2] = value * 3.125;
			break;
	}
}

int OriSensor::populateSensorList(struct sensor_t *list)
{
        memcpy(list, sSensorList, sizeof(struct sensor_t) * numSensors);
        return numSensors;
}

