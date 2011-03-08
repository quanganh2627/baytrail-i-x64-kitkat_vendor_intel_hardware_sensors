/*
 * Copyright (C) 2009 The Android Open Source Project
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

/* Contains changes by Wind River Systems, 2009-2010 */

#define LOG_TAG "GAID_accelerometer"

#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <hardware/sensors.h>

#include "sensors_gaid.h"

#define ACCEL_SYSFS_DIR    "/sys/devices/platform/lis3lv02d/"
#define ACCEL_DATA         "position"

static int fd_accel = -1;

/* return max value of fd for select() in poll method */
static int gaid_accelerometer_data_open(void)
{
    if (fd_accel < 0) {
        fd_accel = open(ACCEL_SYSFS_DIR ACCEL_DATA, O_RDONLY);
        if (fd_accel < 0) {
            E("%s dev file open failed %d", __FUNCTION__, fd_accel);
        }
    }

    return fd_accel;
}

static void gaid_accelerometer_data_close(void)
{
    if (fd_accel >= 0) {
        close(fd_accel);
        fd_accel = -1;
    }
}

static void gaid_accelerometer_set_fd(fd_set *fds)
{
    FD_SET(fd_accel, fds);
}

static int gaid_accelerometer_is_fd(fd_set *fds)
{
    return FD_ISSET(fd_accel, fds) ? 1 : 0;
}

#define BUFSIZE 100

static int gaid_accelerometer_data_read(sensors_event_t *data)
{
    struct timespec t;
    char buf[BUFSIZE + 1];
    int x,y,z;
    int n;

    n = pread(fd_accel, buf, sizeof(buf), 0);
    if (n < 0) {
        E("%s read error %d", __FUNCTION__,n);
        return n;
    }

    sscanf(buf,"(%d,%d,%d)", &x, &y, &z);
    D("accelerometer raw data: x = %d, y = %d, z = %d", x, y, z);

    clock_gettime(CLOCK_REALTIME, &t);

    /*
     * data is in mG unit, need to convert it into m/s^2 required by android
     * flip axis X & Y to fit with Android platform
     */
    data->acceleration.x = ((float)y / 1000) * GRAVITY_EARTH * -1.0;
    data->acceleration.y = ((float)x / 1000) * GRAVITY_EARTH;
    data->acceleration.z = ((float)z / 1000) * GRAVITY_EARTH;

    data->timestamp = timespec_to_ns(&t);
    data->sensor = S_HANDLE_ACCELEROMETER;
    data->type = SENSOR_TYPE_ACCELEROMETER;
    data->version = sizeof(sensors_event_t);
    data->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

    D("scaled data: x = %f, y = %f, z = %f\n",
      data->acceleration.x, data->acceleration.y, data->acceleration.z);

    return 0;
}

sensors_ops_t gaid_sensors_accelerometer = {
    .sensor_data_open  = gaid_accelerometer_data_open,
    .sensor_set_fd     = gaid_accelerometer_set_fd,
    .sensor_is_fd      = gaid_accelerometer_is_fd,
    .sensor_read       = gaid_accelerometer_data_read,
    .sensor_data_close = gaid_accelerometer_data_close,
    .sensor_list       = {
        .name       = "ST LIS3DH 3-axis Accelerometer",
        .vendor     = "Intel",
        .version    = 1,
        .handle     = S_HANDLE_ACCELEROMETER,
        .type       = SENSOR_TYPE_ACCELEROMETER,
        .maxRange   = 2.0 * GRAVITY_EARTH,
        .resolution = GRAVITY_EARTH / 1000,
        .power      = 0.0f,
        .minDelay   = 20,
        .reserved   = {}
    },
};

