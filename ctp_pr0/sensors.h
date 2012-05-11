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
#define SENSORS_HANDLE_LIGHT            1
#define SENSORS_HANDLE_PROXIMITY        2
#define SENSORS_HANDLE_ACCELERATION     3
#define SENSORS_HANDLE_MAGNETIC_FIELD   4
#define SENSORS_HANDLE_GYROSCOPE        5
#define SENSORS_HANDLE_PRESSURE         6
#define SENSORS_HANDLE_MAX              6

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
/*****************************************************************************/
/*
 * The SENSORS PATH
 */

#define LIGHT_DATA              "/dev/apds990x_lsensor"

#define PROXIMITY_DATA          "/dev/apds990x_psensor"
/*****************************************************************************/

/*
 * The SENSORS Module
 */

/* the GP2A is a binary proximity sensor that triggers around 5 cm on
 * this hardware */
#define PROXIMITY_THRESHOLD_GP2A  5.0f


#define NSEC_PER_SEC    1000000000L
static inline int64_t timespec_to_ns(const struct timespec *ts)
{
    return ((int64_t) ts->tv_sec * NSEC_PER_SEC) + ts->tv_nsec;
}

/*****************************************************************************/
#define GRAVITY 9.80665f

#define EVENT_TYPE_ACCEL_X          REL_X
#define EVENT_TYPE_ACCEL_Y          REL_Y
#define EVENT_TYPE_ACCEL_Z          REL_Z

#define EVENT_TYPE_YAW              REL_RX
#define EVENT_TYPE_PITCH            REL_RY
#define EVENT_TYPE_ROLL             REL_RZ
#define EVENT_TYPE_ORIENT_STATUS    REL_WHEEL

#define EVENT_TYPE_MAGV_X           REL_DIAL
#define EVENT_TYPE_MAGV_Y           REL_HWHEEL
#define EVENT_TYPE_MAGV_Z           REL_MISC

#define EVENT_TYPE_M_O_X			REL_X
#define EVENT_TYPE_M_O_Y			REL_Y
#define EVENT_TYPE_M_O_Z			REL_Z

#define EVENT_TYPE_PROXIMITY        ABS_DISTANCE
#define EVENT_TYPE_LIGHT            ABS_MISC

#define EVENT_TYPE_GYRO_X           REL_RY
#define EVENT_TYPE_GYRO_Y           REL_RX
#define EVENT_TYPE_GYRO_Z           REL_RZ

#define EVENT_TYPE_PRESSURE         REL_X
#define EVENT_TYPE_TEMPERATURE      REL_Y

/* conversion of acceleration data to SI units (m/s^2) */
#define RANGE_A                     (2*GRAVITY_EARTH)
#define CONVERT_A                   (2*GRAVITY_EARTH / 4096)
#define RESOLUTION_A                (GRAVITY_EARTH / 1000)

#ifdef TARGET_MFLD_GI
#define CONVERT_A_X(x)              (((float)x/1000) * GRAVITY * -1.0)
#define CONVERT_A_Y(x)              (((float)x/1000) * GRAVITY * -1.0)
#define CONVERT_A_Z(x)              (((float)x/1000) * GRAVITY)
#else
#define CONVERT_A_X(x)              (((float)x/1000) * GRAVITY * -1.0)
#define CONVERT_A_Y(x)              (((float)x/1000) * GRAVITY)
#define CONVERT_A_Z(x)              (((float)x/1000) * GRAVITY * -1.0)
#endif

/* conversion of magnetic data to uT units */
#define CONVERT_M                   (0.5f)
#define CONVERT_M_X                 (-CONVERT_M)
#define CONVERT_M_Y                 (-CONVERT_M)
#define CONVERT_M_Z                 (-CONVERT_M)
#define RESOLUTION_M                (0.15f)
#define RANGE_M                     (250.0f)

/* conversion of orientation data to degree units */
#define CONVERT_O                   (1.0f/64.0f)
#define CONVERT_O_A                 (CONVERT_O)
#define CONVERT_O_P                 (CONVERT_O)
#define CONVERT_O_R                 (-CONVERT_O)

/* conversion of gyro data to SI units (radian/sec) */
#define RANGE_G                     (2000.0f*(float)M_PI/180.0f)
#define CONVERT_GYRO                ((70.0f / 1000.0f) * ((float)M_PI / 180.0f))
#define RESOLUTION_G                ((70.0f / 1000.0f) * ((float)M_PI / 180.0f))

#define SENSOR_STATE_MASK           (0x7FFF)

/* conversion of light data to SI units (lux) */
#define RANGE_L                     (65535.0f)
#define RESOLUTION_L                (0.1f)

/* conversion of proximity data to cm units */
#define RANGE_PROXIMITY             (6.0f)
#define RESOLUTION_PROXIMITY        (6.0f)

/* conversion of pressure data to mbar units */
#define RANGE_PRESSURE              (1260.0f)
#define RESOLUTION_PRESSURE         (0.24f)

/*****************************************************************************/

__END_DECLS

#endif  // ANDROID_SENSORS_H
