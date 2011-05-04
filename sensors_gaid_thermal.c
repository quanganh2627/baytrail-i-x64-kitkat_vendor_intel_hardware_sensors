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

#define LOG_TAG "GAID_thermal"

#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <hardware/sensors.h>

#include "sensors_gaid.h"

#define THERMAL_SYSFS_DIR "/sys/devices/virtual/thermal/thermal_zone3/"
#define THERMAL_DEV "temp"

static int fd_temp = -1;

/* return max value of fd for select() in poll method */
static int gaid_thermal_data_open(void)
{
    if (fd_temp < 0) {
        fd_temp = open(THERMAL_SYSFS_DIR THERMAL_DEV, O_RDONLY);

        if(fd_temp < 0) {
            E("%s dev file open failed %d", __FUNCTION__, fd_temp);
        }
    }

    return fd_temp;
}

static void gaid_thermal_data_close(void)
{
    if(fd_temp >= 0) {
        close(fd_temp);
        fd_temp = -1;
    }
}

static void gaid_thermal_set_fd(fd_set *fds)
{
    FD_SET(fd_temp, fds);
}

static int gaid_thermal_is_fd(fd_set *fds)
{
    return (FD_ISSET(fd_temp, fds)) ? 1 : 0;
}

#define BUFSIZE 100

static int gaid_thermal_data_read(sensors_event_t *data)
{
    struct timespec t;
    char buf[BUFSIZE+1];
    int n;
    float temperature;

    n = pread(fd_temp,buf,sizeof(buf),0);
    if(n < 0) {
        E("%s read error %d", __FUNCTION__,n);
        return n;
    }

    temperature = atof(buf);

    clock_gettime(CLOCK_REALTIME, &t);

    data->temperature = temperature / 1000;
    data->timestamp = timespec_to_ns(&t);
    data->sensor = S_HANDLE_TEMPERATURE;
    data->type = SENSOR_TYPE_TEMPERATURE;
    data->version = sizeof(sensors_event_t);
    data->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

    return 0;
}

sensors_ops_t gaid_sensors_thermal = {
    .sensor_data_open  = gaid_thermal_data_open,
    .sensor_set_fd     = gaid_thermal_set_fd,
    .sensor_is_fd      = gaid_thermal_is_fd,
    .sensor_read       = gaid_thermal_data_read,
    .sensor_data_close = gaid_thermal_data_close,
    .sensor_list       = {
        .name       = "MSIC Thermal",
        .vendor     = "Intel",
        .version    = 1,
        .handle     = S_HANDLE_TEMPERATURE,
        .type       = SENSOR_TYPE_TEMPERATURE,
        .maxRange   = 85.0f,
        .resolution = 1.0f,
        .minDelay   = 500,
        .power      = 0.0f,
        .reserved   = {}
    },
};

