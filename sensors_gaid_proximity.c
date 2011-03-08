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

#define LOG_TAG "GAID_proximity"

#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <hardware/sensors.h>

#include "sensors_gaid.h"

#define PROXIMITY_SYSFS_DIR "/sys/class/i2c-adapter/i2c-5/5-0055/apds9802ps/"
#define PROXIMITY_OUTPUT "proximity_output"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static int fd_proximity = -1;
static int old_proximity;

struct range {
    int cstart, clen;   /* count range */
    float dstart, dlen; /* distance range mapped */
};

/* mannually measured coarse mapping table, need to tune */
static struct range mapping_tbl[] = {
    { 500, 500, 15.75, 10.5 },
    { 1000, 1000, 5.25, 2.625 },
    { 2000, 2000, 2.625, 2.0 }
};

static int gaid_proximity_data_open(void)
{
    old_proximity = -100;

    if (fd_proximity < 0) {
        fd_proximity = open(PROXIMITY_SYSFS_DIR PROXIMITY_OUTPUT, O_RDONLY);
        if (fd_proximity < 0) {
            E("%s dev file open failed\n", __func__);
        }
    }

    return fd_proximity;
}

static void gaid_proximity_data_close(void)
{
    if (fd_proximity >= 0) {
        close(fd_proximity);
        fd_proximity = -1;
    }
}

static void gaid_proximity_set_fd(fd_set *fds)
{
    FD_SET(fd_proximity, fds);
}

static int gaid_proximity_is_fd(fd_set *fds)
{
    return FD_ISSET(fd_proximity, fds);
}

static int valid_data(int new, int old)
{
    if (new >= old + 100 || new <= old - 100)
        return 1;

    return 0;
}

static float count_to_distance(int proximity)
{
    int i;
    float distance;

    if (proximity < 500) {
        /* no object detected, return maxRange */
        return 20.0;
    }
    if (proximity >= 4000) {
        /* object is very close, from 2mm to 10 mm */
        return 0.5;
    }

    for (i = 0; i < (int)ARRAY_SIZE(mapping_tbl); i++) {
        if (proximity >= mapping_tbl[i].cstart &&
            proximity < mapping_tbl[i].cstart + mapping_tbl[i].clen) {
            float prop = proximity - mapping_tbl[i].cstart;

            distance = mapping_tbl[i].dstart + prop / mapping_tbl[i].clen;
        }
    }

    return distance;
}

#define BUFSIZE    32
static int gaid_proximity_data_read(sensors_event_t *data)
{
    struct timespec t;
    char buf[BUFSIZE];
    int ret;
    int proximity;

    ret = pread(fd_proximity, buf, sizeof(buf), 0);
    if (ret < 0) {
        E("%s read error\n", __func__);
        return ret;
    }

    proximity = atoi(buf);

    if (!valid_data(proximity, old_proximity))
        return -1;

    clock_gettime(CLOCK_REALTIME, &t);

    data->timestamp = timespec_to_ns(&t);
    data->sensor = S_HANDLE_PROXIMITY;
    data->type = SENSOR_TYPE_PROXIMITY;
    data->version = sizeof(sensors_event_t);
    data->distance = count_to_distance(proximity);
    D("proximity = %d, distance = %f", proximity, data->distance);

    old_proximity = proximity;

    return 0;
}

sensors_ops_t gaid_sensors_proximity = {
    .sensor_data_open   = gaid_proximity_data_open,
    .sensor_set_fd      = gaid_proximity_set_fd,
    .sensor_is_fd       = gaid_proximity_is_fd,
    .sensor_read        = gaid_proximity_data_read,
    .sensor_data_close  = gaid_proximity_data_close,
    .sensor_list        = {
        .name       = "APDS9802 Proximity Sensor",
        .vendor     = "Intel",
        .version    = 1,
        .handle     = S_HANDLE_PROXIMITY,
        .type       = SENSOR_TYPE_PROXIMITY,
        .maxRange   = 20.0f,
        .resolution = 0.01f,
        .minDelay   = 0,
        .power      = 0.0f,
        .reserved   = {}
    },
};
