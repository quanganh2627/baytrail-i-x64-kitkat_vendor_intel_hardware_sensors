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
#include <cutils/log.h>

#include "LightSensor.h"

/*****************************************************************************/

LightSensor::LightSensor()
    : SensorBase(NULL, "light"),
      mEnabled(0),
      mInputReader(4),
      mHasPendingEvent(false)
{
    data_fd = open(LIGHT_DATA, O_RDONLY);
    LOGE_IF(data_fd < 0, "can't open %s", LIGHT_DATA);

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = SENSORS_HANDLE_LIGHT;
    mPendingEvent.type = SENSOR_TYPE_LIGHT;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
}

LightSensor::~LightSensor()
{
    if (mEnabled)
        enable(0, 0);
}

int LightSensor::enable(int32_t handle, int en)
{
    D("LightSensor - %s - enable=%d", __func__, en);

    if (ioctl(data_fd, 0, en)) {
        E("LightSensor: ioctl set %s failed!", LIGHT_DATA);
        return -1;
    }
    return 0;
}

bool LightSensor::hasPendingEvents() const
{
    return mHasPendingEvent;
}

int LightSensor::readEvents(sensors_event_t* data, int count)
{
    unsigned int val = -1;
    struct timespec t;

    D("LightSensor - %s", __func__);

    if (count < 1 || data == NULL || data_fd < 0)
        return -EINVAL;

    *data = mPendingEvent;
    data->timestamp = getTimestamp();
    read(data_fd, &val, sizeof(unsigned int));
    data->light = (float)val / APDS9900_LUX_OUTPUT_SCALE;
    D("LightSensor - read data val = %f ",data->light);

    return 1;
}
