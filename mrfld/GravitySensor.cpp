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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <dlfcn.h>
#include <sys/time.h>
#include <cutils/log.h>

#include "GravitySensor.h"

/*****************************************************************************/

GravitySensor::GravitySensor()
: SensorBase("gravity"),
      mEnabled(0)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = SENSORS_HANDLE_GRAVITY;
    mPendingEvent.type = SENSOR_TYPE_GRAVITY;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
}

GravitySensor::~GravitySensor()
{
    if (mEnabled)
        enable(0, 0);
}

int GravitySensor::enable(int32_t handle, int en)
{
    unsigned int flags = en ? 1 : 0;
    int ret;

    D("GravitySensor - %s, flags = %d, mEnabled = %d, data_fd = %d",
                          __FUNCTION__, flags, mEnabled, data_fd);

    if (mHandle == NULL)
        return -1;

    if ((flags != mEnabled) && (flags == 0)) {
        ret = psh_stop_streaming(mHandle);
        if (ret != 0) {
            E("GravitySensor - %s, failed to stop streaming, ret = %d",
                                                  __FUNCTION__, ret);
            return -1;
        }
    }

    mEnabled = flags;
    return 0;
}

#define MAX_DATA_RATE 100

int GravitySensor::setDelay(int32_t handle, int64_t ns)
{
    int data_rate = 0,delay_ms, ret;

    if (mHandle == NULL)
        return -1;

    if (mEnabled == 0)
	return 0;

    D("psh_start_streaming(), data_rate is %d", data_rate);

    delay_ms = ns / 1000000;
    data_rate = 1000 / delay_ms;

    if (data_rate == 0)
        return -1;

    if (data_rate > MAX_DATA_RATE)
        data_rate = MAX_DATA_RATE;

    ret = psh_start_streaming(mHandle, data_rate, 0);
    if (ret != 0) {
        E("psh_start_streaming failed, ret is %d", ret);
        return -1;
    }

    return 0;
}

int GravitySensor::readEvents(sensors_event_t* data, int count)
{
    int size, numEventReceived = 0;
    char buf[512];
    struct gravity_data *p_gravity_data;
    int unit_size = sizeof(struct gravity_data);

    D("Enter %s, count is %d", __FUNCTION__, count);
    if (count < 1)
        return -EINVAL;

    D("GravitySensor::%s, count is %d", __FUNCTION__, count);

    if (mHandle == NULL)
        return 0;

    if ((unit_size * count) <= 512)
        size = unit_size * count;
    else
        size = (512 / unit_size) * unit_size;


    size = read(data_fd, buf, size);

    D("GravitySensor::readEvents read size is %d", size);

    char *p = buf;

    p_gravity_data = (struct gravity_data *)buf;
    while (size > 0) {

        mPendingEvent.data[0] = ((float)p_gravity_data->x / (1000 * 65536)) * GRAVITY;
        mPendingEvent.data[1] = ((float)p_gravity_data->y / (1000 * 65536)) * GRAVITY;
        mPendingEvent.data[2] = ((float)p_gravity_data->z / (1000 * 65536)) * GRAVITY;

        mPendingEvent.timestamp = getTimestamp();

        if (mEnabled == 1) {
           *data++ = mPendingEvent;
            numEventReceived++;
        }

        size = size - unit_size;
        p = p + unit_size;
        p_gravity_data = (struct gravity_data *)p;
    }

    D("GravitySensor::%s, numEventReceived is %d", __FUNCTION__,
                                            numEventReceived);

    return numEventReceived;
}
