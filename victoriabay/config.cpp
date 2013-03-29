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

#include "../sensors.h"

#include "../LightSensor.h"
#include "../ProximitySensor.h"
#include "../AccelSensor.h"
#include "../GyroSensor.h"
#include "../CompassSensor.h"
#include "../PressureSensor.h"

#define RANGE_A                     (2*GRAVITY_EARTH)
#define RESOLUTION_A                (GRAVITY_EARTH / 1000)

#define RANGE_L                     (10000.0f)
#define RESOLUTION_L                (0.1f)

#define RANGE_P                     (6.0f)
#define RESOLUTION_P                (6.0f)

#define RESOLUTION_M                (0.15f)
#define RANGE_M                     (250.0f)

#define RANGE_G                     (2000.0f*(float)M_PI/180.0f)
#define CONVERT_GYRO                ((70.0f / 1000.0f) * ((float)M_PI / 180.0f))
#define RESOLUTION_G                ((70.0f / 1000.0f) * ((float)M_PI / 180.0f))

#define RANGE_PRESSURE              (1260.0f)
#define RESOLUTION_PRESSURE         (0.24f)

/* Sequence of sensor type in sensor_configs and sensor_list should me the same */
static const union sensor_data_t compass_data = {
    compass_filter_en:    1,
};
static const sensor_platform_config_t sensor_configs[] = {
/* accel */
    {
        handle:         SENSORS_HANDLE_ACCELERAMETER,
        name:           "accel",
        activate_path:  "/sys/bus/i2c/devices/5-0019/lis3dh/enable",
        poll_path:      "/sys/bus/i2c/devices/5-0019/lis3dh/poll",
        data_path:      NULL,
        config_path:    NULL,
        mapper:         { AXIS_Y, AXIS_X, AXIS_Z },
        scale:          { -1.0, -1.0, 1.0 },
        range:          { 0 },
        min_delay:      0,
        priv_data:      0,
    },
/* light */
    {
        handle:         SENSORS_HANDLE_LIGHT,
        name:           "light",
        activate_path:  "/dev/apds990x_lsensor",
        poll_path:      NULL,
        data_path:      "/dev/apds990x_lsensor",
        config_path:    NULL,
        mapper:         { 0 },
        scale:          { 0 },
        range:          {0, 10000.0 },
        min_delay:      0,
        priv_data:      0,
    },
/* proximity */
    {
        handle:         SENSORS_HANDLE_PROXIMITY,
        name:           "proximity",
        activate_path:  "/dev/apds990x_psensor",
        poll_path:      NULL,
        data_path:      "/dev/apds990x_psensor",
        config_path:    NULL,
        mapper:         { 0 },
        scale:          { 0 },
        range:          { 0 },
        min_delay:      0,
        priv_data:      0,
    },
/* compass */
    {
        handle:         SENSORS_HANDLE_MAGNETIC_FIELD,
        name:           "lsm303cmp",
        activate_path:  "/sys/bus/i2c/devices/5-001e/lsm303cmp/enable",
        poll_path:      "/sys/bus/i2c/devices/5-001e/lsm303cmp/poll",
        data_path:      NULL,
        config_path:    "/data/compass.conf",
        mapper:         { AXIS_Y, AXIS_X, AXIS_Z },
        scale:          { 670, -670, 600 },
        range:          { 0 },
        min_delay:      0,
        priv_data:      &compass_data,
    },
/* gyro */
    {
        handle:         SENSORS_HANDLE_GYROSCOPE,
        name:           "l3g4200d",
        activate_path:  "/sys/bus/i2c/devices/5-006a/enable;"
                        "/sys/bus/i2c/devices/5-0068/enable",
        poll_path:      "/sys/bus/i2c/devices/5-006a/poll;"
                        "/sys/bus/i2c/devices/5-0068/poll",
        data_path:      NULL,
        config_path:    "/data/gyro.conf",
        mapper:         { AXIS_X, AXIS_Y, AXIS_Z },
        scale:          { 1.0, 1.0, 1.0 },
        range:          { 0 },
        min_delay:      10000000,
        priv_data:      0,
    },
/* pressure */
    {
        handle:         SENSORS_HANDLE_PRESSURE,
        name:           "lps331ap_pressure",
        activate_path:  "/sys/bus/i2c/devices/5-005c/enable",
        poll_path:      "/sys/bus/i2c/devices/5-005c/poll",
        data_path:      NULL,
        config_path:    NULL,
        mapper:         { 0 },
        scale:          { 0 },
        range:          { 0 },
        min_delay:      0,
        priv_data:      0,
    },
};

static const struct sensor_t sensor_list[] = {
    { "MODEL_LSM303DLHC 3-axis Accelerometer",
      "STMicroelectronics",
      1, SENSORS_HANDLE_ACCELERAMETER,
      SENSOR_TYPE_ACCELEROMETER, RANGE_A, RESOLUTION_A, 0.11f, 10000, { } },
    { "Avago APDS-9900 Digital Ambient Light Sensor",
      "Avago",
      1, SENSORS_HANDLE_LIGHT,
      SENSOR_TYPE_LIGHT, RANGE_L, RESOLUTION_L, 0.25f, 0, { } },
    { "Avago APDS-9900 Digital Proximity Sensor",
      "Avago",
      1, SENSORS_HANDLE_PROXIMITY,
      SENSOR_TYPE_PROXIMITY, RANGE_P, RESOLUTION_P, 0.25f, 0, { } },
    { "MODEL_LSM303DLHC 3-axis Magnetic field sensor",
      "STMicroelectronics",
      1, SENSORS_HANDLE_MAGNETIC_FIELD,
      SENSOR_TYPE_MAGNETIC_FIELD, RANGE_M, RESOLUTION_M, 0.1f, 20000, { } },
    { "L3G4200D Gyroscope sensor",
      "STMicroelectronics",
      1, SENSORS_HANDLE_GYROSCOPE,
      SENSOR_TYPE_GYROSCOPE, RANGE_G, RESOLUTION_G, 6.1f, 10000, { } },
    { "ST LPS331AP Pressure Sensor",
      "STMicroelectronics",
      1, SENSORS_HANDLE_PRESSURE,
      SENSOR_TYPE_PRESSURE, RANGE_PRESSURE, RESOLUTION_PRESSURE, 0.001f, 50000, { } },
};

static SensorBase* platform_sensors[ARRAY_SIZE(sensor_list)];

const struct sensor_t* get_platform_sensor_list(int *sensor_num)
{
    int num = ARRAY_SIZE(sensor_list);
    *sensor_num = 0;

    if (ARRAY_SIZE(sensor_list) != ARRAY_SIZE(sensor_configs))
        return NULL;

    for (int i = 0; i < num; i++) {
        if (sensor_list[i].handle != sensor_configs[i].handle) {
            E("Sensor config sequence error\n");
            return NULL;
        }
    }
    *sensor_num = num;

    return sensor_list;
}

SensorBase **get_platform_sensors()
{
    int num = ARRAY_SIZE(sensor_list);

    for (int i = 0; i < num; i++) {
        int handle = sensor_list[i].handle;
        switch (handle) {
        case SENSORS_HANDLE_LIGHT:
            platform_sensors[i] = new LightSensor(&sensor_configs[i]);
            break;
        case SENSORS_HANDLE_PROXIMITY:
            platform_sensors[i] = new ProximitySensor(&sensor_configs[i]);
            break;
        case SENSORS_HANDLE_ACCELERAMETER:
            platform_sensors[i] = new AccelSensor(&sensor_configs[i]);
            break;
        case SENSORS_HANDLE_MAGNETIC_FIELD:
            platform_sensors[i] = new CompassSensor(&sensor_configs[i]);
            break;
        case SENSORS_HANDLE_GYROSCOPE:
            platform_sensors[i] = new GyroSensor(&sensor_configs[i]);
            break;
        case SENSORS_HANDLE_PRESSURE:
            platform_sensors[i] = new PressureSensor(&sensor_configs[i]);
            break;
        default:
            E("Error, no Sensor ID handle %d found\n", handle);
            return NULL;
        }
    }

    return platform_sensors;
}
