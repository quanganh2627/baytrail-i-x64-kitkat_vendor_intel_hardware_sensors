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

#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <hardware/sensors.h>

#include "sensors_gaid.h"

#define GAID_ACCELEROMETER_SYSFS_PREF \
    "/sys/devices/platform/lis3lv02d/"

#define ACCELEROMETER_DEV "position"

#define CONVERT                    (1.0/100.0) 

static int fd_accel = -1;

/* return max value of fd for select() in poll method */
static int gaid_accelerometer_data_open(void)
{
    fd_accel = open(GAID_ACCELEROMETER_SYSFS_PREF ACCELEROMETER_DEV, O_RDONLY);
    if(fd_accel < 0) {
        E("%s dev file open failed %d", __FUNCTION__, fd_accel);
    }

    return fd_accel;
}

static void gaid_accelerometer_data_close(void)
{
    if(fd_accel>=0) {
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

static int gaid_accelerometer_data_read(sensors_data_t *data)
{
    struct timespec t;
    char buf[BUFSIZE+1];
    int x,y,z;
    int n;

    n = pread(fd_accel,buf,sizeof(buf),0);
    if(n < 0) {
        E("%s read error %d", __FUNCTION__,n);
        return n;
    }
    usleep(1000);
    sscanf(buf,"(%d,%d,%d)", &x, &y, &z);   /* input range : 0 ~ 255 (each x,y,z) */

    clock_gettime(CLOCK_REALTIME, &t);

    /* convert axis X & Y to fit with Android platform */
    data->acceleration.y = (x * CONVERT);
    data->acceleration.x = -(y * CONVERT);
    data->acceleration.z = z * CONVERT;

    data->time = timespec_to_ns(&t);
    data->sensor = SENSOR_TYPE_ACCELEROMETER;
    data->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

    return 0;
}

sensors_ops_t gaid_sensors_accelerometer = {
    .sensor_data_open  = gaid_accelerometer_data_open,
    .sensor_set_fd     = gaid_accelerometer_set_fd,
    .sensor_is_fd      = gaid_accelerometer_is_fd,
    .sensor_read       = gaid_accelerometer_data_read,
    .sensor_data_close = gaid_accelerometer_data_close,
    .sensor_list       = {
        .name       = "GAID 3-axis Accelerometer",
        .vendor     = "The Android Open Source Project",
        .version    = 1,
        .handle     = S_HANDLE_ACCELEROMETER,
        .type       = SENSOR_TYPE_ACCELEROMETER,
        .maxRange   = 2.3f,
        .resolution = 0.018f,
        .power      = 0.0f,
        .reserved   = {}
    },
};

