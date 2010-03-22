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

#ifdef COMPASS_DEBUG
#define  D(...)  LOGD(__VA_ARGS__)
#else
#define  D(...)
#endif

#define GAID_COMPASS_SYSFS_PREF \
    "/sys/class/i2c-adapter/i2c-0/0-0021/hmc6352/"
#define COMPASS_DEV_HEADING "heading"

static int fd_heading = -1;

static int gaid_compass_data_open(void)
{
    int max_fd = 0;

    D("%s\n", __func__);
    fd_heading = open(GAID_COMPASS_SYSFS_PREF COMPASS_DEV_HEADING,
            O_RDONLY);
    if (fd_heading<0) {
        E("%s dev file open failed\n", __func__);
        return fd_heading;
    }
    max_fd = fd_heading;

    return max_fd;
}

static void gaid_compass_data_close(void)
{
    D("%s\n", __func__);
    if (fd_heading > 0) {
        close(fd_heading);
        fd_heading = 0;
    }
}

static void gaid_compass_set_fd(fd_set *fds)
{
    D("%s\n", __func__);
    FD_SET(fd_heading, fds);
}

static int gaid_compass_is_fd(fd_set *fds)
{
    D("%s\n", __func__);
    return FD_ISSET(fd_heading, fds);
}

#define BUFSIZE    32
static int gaid_compass_data_read(sensors_data_t *data)
{
    struct timespec t;
    char buf[BUFSIZE];
    int ret;
    float heading;

    ret = pread(fd_heading, buf, sizeof(buf), 0);
    if (ret<0) {
        E("%s read error\n", __func__);
        return ret;
    }
    usleep(1000);
    heading = atof(buf);

    clock_gettime(CLOCK_REALTIME, &t);

    data->time = (long)(timespec_to_ns(&t));
    data->sensor = SENSOR_TYPE_ORIENTATION;
    data->orientation.azimuth = heading;
    data->orientation.pitch = 0;
    data->orientation.roll = 0;
    data->orientation.status = SENSOR_STATUS_ACCURACY_HIGH;
    D("[%s] time = %lld, azimuth = %f\n", __func__, data->time, heading);

    return 0;
}

sensors_ops_t gaid_sensors_compass = {
    .sensor_data_open   = gaid_compass_data_open,
    .sensor_set_fd      = gaid_compass_set_fd,
    .sensor_is_fd       = gaid_compass_is_fd,
    .sensor_read        = gaid_compass_data_read,
    .sensor_data_close  = gaid_compass_data_close,
    .sensor_list        = {
        .name       = "GAID Compass",
        .vendor     = "The Android Open Source Project",
        .version    = 1,
        .handle     = S_HANDLE_ORIENTATION,
        .type       = SENSOR_TYPE_ORIENTATION,
        .maxRange   = 360.0f,
        .resolution = 0.1f,
        .power      = 0.0f,
        .reserved   = {}
    },
};
