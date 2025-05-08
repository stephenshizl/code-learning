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

#ifndef QMP6988_H
#define QMP6988_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "sensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"
#include "SensorCoordinate.h"


#define QMP6988_USE_SYSFS
#define PRESS_IOCTL_BASE                   0x87

/*****************************************************************************/
struct input_event;

class PressSensor : public SensorBase {
	public:
	PressSensor();
	virtual ~PressSensor();

	enum {
		Pressure = 0,
	    numSensors
	};

	virtual int readEvents(sensors_event_t* data, int count);
	virtual bool hasPendingEvents() const;
	virtual int setDelay(int32_t handle, int64_t ns);
	virtual int setEnable(int32_t handle, int enabled);
	virtual int64_t getDelay(int32_t handle);
	virtual int getEnable(int32_t handle);
	virtual int populateSensorList(struct sensor_t* list);

	private:
		int mEnabled;
		int64_t mDelay;
		InputEventCircularReader mInputReader;
		sensors_event_t mPendingEvent;
		bool mHasPendingEvent;
		char sysfs_path[PATH_MAX];
		int sysfs_path_len;
		int setInitialState();
};

#define QMP6988_U16_t unsigned short
#define QMP6988_S16_t short
#define QMP6988_U32_t unsigned int
#define QMP6988_S32_t int

struct qmp6988_calibration_data {
	QMP6988_S32_t COE_a0;
	QMP6988_S16_t COE_a1;
	QMP6988_S16_t COE_a2;
	QMP6988_S32_t COE_b00;
	QMP6988_S16_t COE_bt1;
	QMP6988_S16_t COE_bt2;
	QMP6988_S16_t COE_bp1;
	QMP6988_S16_t COE_b11;
	QMP6988_S16_t COE_bp2;
	QMP6988_S16_t COE_b12;
	QMP6988_S16_t COE_b21;
	QMP6988_S16_t COE_bp3;
};


int qmp6988_algo_init(void);
float qmp6988_calc_press(void);
float qmp6988_calc(int Dp, int Dt);


#endif  // QMP6988_H
