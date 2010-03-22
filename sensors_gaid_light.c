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

#undef D
//#define ALS_DEBUG
#ifdef ALS_DEBUG
#define  D(...)  LOGD(__VA_ARGS__)
#else
#define  D(...)
#endif

#define GAID_ALS_SYSFS_PREF \
    "/sys/class/i2c-adapter/i2c-0/0-0044/isl29020/"
#define ALS_LUX_OUTPUT "lux_output"

static int fd_als = -1;

static int gaid_als_data_open(void)
{
    int max_fd = 0;

    D("%s\n", __func__);
    fd_als = open(GAID_ALS_SYSFS_PREF ALS_LUX_OUTPUT,
            O_RDONLY);
    if (fd_als < 0) {
        E("%s dev file open failed\n", __func__);
        return fd_als;
    }
    max_fd = fd_als;

    return max_fd;
}

static void gaid_als_data_close(void)
{
    D("%s\n", __func__);
    if (fd_als > 0) {
        close(fd_als);
        fd_als = 0;
    }
}

static void gaid_als_set_fd(fd_set *fds)
{
    D("%s\n", __func__);
    FD_SET(fd_als, fds);
}

static int gaid_als_is_fd(fd_set *fds)
{
    D("%s\n", __func__);
    return FD_ISSET(fd_als, fds);
}

#define BUFSIZE    32
static int gaid_als_data_read(sensors_data_t *data)
{
    struct timespec t;
    char buf[BUFSIZE];
    int ret;
    float lux;

    ret = pread(fd_als, buf, sizeof(buf), 0);
    if (ret<0) {
        E("%s read error\n", __func__);
        return ret;
    }
    usleep(1000);
    lux = atof(buf);

    clock_gettime(CLOCK_REALTIME, &t);

    data->time = timespec_to_ns(&t);
    data->sensor = SENSOR_TYPE_LIGHT;
    data->vector.v[0] = lux;
    data->vector.v[1] = lux;
    data->vector.v[2] = lux;
    data->vector.status = SENSOR_STATUS_ACCURACY_MEDIUM;
    D("[%s] time = %lld, lux = %f\n", __func__, data->time, lux);

    return 0;
}

sensors_ops_t gaid_sensors_als = {
    .sensor_data_open   = gaid_als_data_open,
    .sensor_set_fd      = gaid_als_set_fd,
    .sensor_is_fd       = gaid_als_is_fd,
    .sensor_read        = gaid_als_data_read,
    .sensor_data_close  = gaid_als_data_close,
    .sensor_list        = {
        .name       = "GAID Ambient Light Sensor",
        .vendor     = "The Android Open Source Project",
        .version    = 1,
        .handle     = S_HANDLE_LIGHT,
        .type       = SENSOR_TYPE_LIGHT,
        .maxRange   = 1000.0f,
        .resolution = 1.0f,
        .power      = 0.0f,
        .reserved   = {}
    },
};
