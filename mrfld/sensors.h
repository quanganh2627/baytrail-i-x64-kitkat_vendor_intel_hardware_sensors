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

#ifndef ANDROID_SENSORS_H
#define ANDROID_SENSORS_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <linux/input.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>

__BEGIN_DECLS


/*****************************************************************************/

//#define SENSOR_DBG

#ifdef SENSOR_DBG
#define  D(...)  LOGD(__VA_ARGS__)
#else
#define  D(...)  ((void)0)
#endif

#define  E(...)  LOGE(__VA_ARGS__)

/*****************************************************************************/
/*
 * SENSORS_HANDLE_xxx should greater than SENSORS_HANDLE_BASE and must be unique.
*/
#define SENSORS_HANDLE_LIGHT            	1
#define SENSORS_HANDLE_PROXIMITY        	2
#define SENSORS_HANDLE_ACCELERATION     	3
#define SENSORS_HANDLE_MAGNETIC_FIELD   	4
#define SENSORS_HANDLE_GYROSCOPE        	5
#define SENSORS_HANDLE_PRESSURE         	6
#define SENSORS_HANDLE_GRAVITY			7
#define SENSORS_HANDLE_LINEAR_ACCELERATION 	8
#define SENSORS_HANDLE_ROTATION_VECTOR  	9
#define SENSORS_HANDLE_ORIENTATION		10
#define SENSORS_HANDLE_MAX              	10

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

/*
 * The SENSORS Module
 */

#define NSEC_PER_SEC    1000000000L
static inline int64_t timespec_to_ns(const struct timespec *ts)
{
    return ((int64_t) ts->tv_sec * NSEC_PER_SEC) + ts->tv_nsec;
}

/*****************************************************************************/
#define GRAVITY 9.80665f

#define EVENT_TYPE_M_O_X			REL_X
#define EVENT_TYPE_M_O_Y			REL_Y
#define EVENT_TYPE_M_O_Z			REL_Z

// conversion of acceleration data to SI units (m/s^2)
#define RANGE_A                 (2*GRAVITY_EARTH)
#define CONVERT_A               (2*GRAVITY_EARTH / 4096)
#define CONVERT_A_X             (CONVERT_A)
#define CONVERT_A_Y             (CONVERT_A)
#define CONVERT_A_Z             (CONVERT_A)

// conversion of magnetic data to uT units
#define CONVERT_M               (0.5f)
#define CONVERT_M_X             (-CONVERT_M)
#define CONVERT_M_Y             (-CONVERT_M)
#define CONVERT_M_Z             (-CONVERT_M)

// conversion of gyro data to SI units (radian/sec)
#define RANGE_GYRO              (2000.0f*(float)M_PI/180.0f)
#define CONVERT_GYRO            ((1.0f / 10.0f) * ((float)M_PI / 180.0f))
#define CONVERT_GYRO_X          (CONVERT_GYRO)
#define CONVERT_GYRO_Y          (-CONVERT_GYRO)
#define CONVERT_GYRO_Z          (CONVERT_GYRO)

/*****************************************************************************/

__END_DECLS

#endif  // ANDROID_SENSORS_H
