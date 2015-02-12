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

#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <hardware/hardware.h>
#include <hardware/sensors.h>
#include <cmath>

/* Maps senor id's to the sensor list */
enum {
    accel           = 0,
    gyro,
    compass,
    light,
    numSensorDrivers,
    numFds,
};



/*****************************************************************************/

/* Board specific sensor configs. */
#define GRAVITY 9.80665f
#define EVENT_TYPE_ACCEL_X          REL_X
#define EVENT_TYPE_ACCEL_Y          REL_Y
#define EVENT_TYPE_ACCEL_Z          REL_Z

#define EVENT_TYPE_COMP_X           REL_X
#define EVENT_TYPE_COMP_Y           REL_Y
#define EVENT_TYPE_COMP_Z           REL_Z

#define EVENT_TYPE_YAW              REL_RX
#define EVENT_TYPE_PITCH            REL_RY
#define EVENT_TYPE_ROLL             REL_RZ
#define EVENT_TYPE_ORIENT_STATUS    REL_WHEEL

#define EVENT_TYPE_MAGV_X           REL_DIAL
#define EVENT_TYPE_MAGV_Y           REL_HWHEEL
#define EVENT_TYPE_MAGV_Z           REL_MISC

#define EVENT_TYPE_PROXIMITY        ABS_DISTANCE
#define EVENT_TYPE_LIGHT            ABS_MISC

#define EVENT_TYPE_GYRO_X           REL_X
#define EVENT_TYPE_GYRO_Y           REL_Y
#define EVENT_TYPE_GYRO_Z           REL_Z

#define EVENT_TYPE_PRESSURE         REL_X
#define EVENT_TYPE_TEMPERATURE      REL_Y

// 720 LSG = 1G
#define LSG                         (1024.0f)
#define NUMOFACCDATA                (8.0f)

//min delay for accelerometer
#define HID_ACCEL3D_MIN_DELAY  16000
// conversion of acceleration data to SI units (m/s^2)

#define RANGE_A                     (2*GRAVITY_EARTH)
#define RESOLUTION_A                (RANGE_A/(256*NUMOFACCDATA))
#define CONVERT_A                   (GRAVITY_EARTH / LSG / NUMOFACCDATA)
#define CONVERT_A_X(x)              ((float(x)/1000) * (GRAVITY * -1.0))
#define CONVERT_A_Y(x)              ((float(x)/1000) * (GRAVITY * 1.0))
#define CONVERT_A_Z(x)              ((float(x)/1000) * (GRAVITY * 1.0))
// conversion of magnetic data to uT units
#define RANGE_M                     (2048.0f)
#define RESOLUTION_M                (0.01)
#define CONVERT_M                   (1.0f/6.6f)
#define CONVERT_M_X                 (-CONVERT_M)
#define CONVERT_M_Y                 (-CONVERT_M)
#define CONVERT_M_Z                 (CONVERT_M)

/* conversion of orientation data to degree units */
#define CONVERT_O                   (1.0f/64.0f)
#define CONVERT_O_A                 (CONVERT_O)
#define CONVERT_O_P                 (CONVERT_O)
#define CONVERT_O_R                 (-CONVERT_O)

// conversion of gyro data to SI units (radian/sec)
#define RANGE_GYRO                  (2000.0f*(float)M_PI/180.0f)
#define CONVERT_GYRO                ((2000.0f / 32767.0f) * ((float)M_PI / 180.0f))
#define CONVERT_GYRO_X              (-CONVERT_GYRO)
#define CONVERT_GYRO_Y              (-CONVERT_GYRO)
#define CONVERT_GYRO_Z              (CONVERT_GYRO)

// conversion of pressure and temperature data
#define CONVERT_PRESSURE            (1.0f/100.0f)
#define CONVERT_TEMPERATURE         (1.0f/100.0f)

#define RESOLUTION_GYRO             (RANGE_GYRO/(2000*NUMOFACCDATA))
#define SENSOR_STATE_MASK           (0x7FFF)

// Proximity Threshold
#define PROXIMITY_THRESHOLD_GP2A  5.0f

//Used in timespec_to_ns calculations
#define NSEC_PER_SEC    1000000000L

#endif  // SENSOR_CONFIG_H
