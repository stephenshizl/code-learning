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

#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <linux/input.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>
#include <string.h>
__BEGIN_DECLS

/*****************************************************************************/

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))


#define SENSORS_ACCELERATION_HANDLE 0
#define SENSORS_MAGNETIC_FIELD_HANDLE 1
#define SENSORS_ORIENTATION_HANDLE 2
#define SENSORS_LIGHT_HANDLE 3
#define SENSORS_PROXIMITY_HANDLE 4
#define SENSORS_GYROSCOPE_HANDLE 5
#define SENSORS_ROTATION_VECTOR_HANDLE  6

#define ID_A (0)
#define ID_M (1)
#define ID_O (2)
#define ID_L (3)
#define ID_P (4)
#define ID_G (5)
#define ID_R (6)

/*****************************************************************************/

/*
 * The SENSORS Module
 */

/*****************************************************************************/

/* For ADXL346 */
#define EVENT_TYPE_ACCEL_X ABS_X
#define EVENT_TYPE_ACCEL_Y ABS_Y
#define EVENT_TYPE_ACCEL_Z ABS_Z
#define EVENT_TYPE_ACCEL_STATUS ABS_THROTTLE

/* For QMI8610 */
#define EVENT_TYPE_GYRO_X ABS_RX
#define EVENT_TYPE_GYRO_Y ABS_RY
#define EVENT_TYPE_GYRO_Z ABS_RZ
#define EVENT_TYPE_GYRO_STATUS ABS_RUDDER

/* For AK8975 */
#define EVENT_TYPE_MAGV_X ABS_RX
#define EVENT_TYPE_MAGV_Y ABS_RY
#define EVENT_TYPE_MAGV_Z ABS_RZ
#define EVENT_TYPE_MAGV_STATUS ABS_RUDDER

/* Fro AKM Algorithm */
#define EVENT_TYPE_YAW ABS_HAT0X
#define EVENT_TYPE_PITCH ABS_HAT0Y
#define EVENT_TYPE_ROLL ABS_HAT1X
#define EVENT_TYPE_ORIENT_STATUS ABS_HAT1Y

/* Fro AL3006 */
#define EVENT_TYPE_PROXIMITY ABS_DISTANCE
#define EVENT_TYPE_LIGHT ABS_MISC

/* conversion of acceleration data to SI units (m/s^2) */
/* 720 LSB = 1G */
#define LSG (256.0f)
#define AKSC_LSG (720.0f)
#define CONVERT_A (GRAVITY_EARTH / LSG)

/* conversion of magnetic data to uT units */
#define CONVERT_M (0.06f)

/* conversion of orientation data to degree units */
#define CONVERT_O (0.015625f)

/* conversion of rotation vector data to uT units */
#define CONVERT_R (0.015625f)

#define SENSOR_STATE_MASK (0x7FFF)

/*****************************************************************************/

__END_DECLS

#endif  // SENSORS_H_
