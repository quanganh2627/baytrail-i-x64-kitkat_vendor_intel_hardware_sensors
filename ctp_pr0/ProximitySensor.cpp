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

#include "ProximitySensor.h"

/*****************************************************************************/

ProximitySensor::ProximitySensor()
    : SensorBase(NULL, "proximity"),
      mEnabled(0),
      mInputReader(4),
      mHasPendingEvent(false)
{
    data_fd = open(PROXIMITY_DATA, O_RDONLY);
    LOGE_IF(data_fd < 0, "can't open %s", PROXIMITY_DATA);

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_P;
    mPendingEvent.type = SENSOR_TYPE_PROXIMITY;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
}

ProximitySensor::~ProximitySensor()
{
    if (mEnabled)
        enable(0, 0);
}

int ProximitySensor::enable(int32_t, int en)
{
    D("ProximitySensor - %s - enable=%d", __func__, en);

    if (ioctl(data_fd, 0, en)) {
        E("ProximitySensor: ioctl set %s failed!", PROXIMITY_DATA);
        return -1;
    }
    return 0;
}

bool ProximitySensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int ProximitySensor::readEvents(sensors_event_t* data, int count)
{
    int val = -1;
    struct timespec t;

    D("ProximitySensor - %s", __func__);
    if (count < 1)
        return -EINVAL;

    if (mHasPendingEvent) {
        D("ProximitySensor - %s has pending event", __func__);
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    if (data == NULL) {
        E("ProximitySensor - %s data is NULL", __func__);
        return -1;
    }
    if (data_fd < 0) {
        E("ProximitySensor - %s data_fd is not valid", __func__);
        return -1;
    }

    data->timestamp = getTimestamp();
    data->type = SENSOR_TYPE_PROXIMITY;
    data->sensor = ID_P;
    data->version = sizeof(sensors_event_t);

    read(data_fd, &val, sizeof(int));
    data->distance = (float)(val == 1 ? 0 : 6);
    D("ProximitySensor - %s read data %f", __func__, data->distance);

    return 1;
}
