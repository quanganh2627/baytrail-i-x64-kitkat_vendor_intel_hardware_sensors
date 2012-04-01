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

#include "PressureSensor.h"

#define PRESSURE_ENABLE     "/sys/bus/i2c/devices/5-005c/enable"
#define PRESSURE_POLL_DELAY "/sys/bus/i2c/devices/5-005c/poll"

/*****************************************************************************/

PressureSensor::PressureSensor()
    : SensorBase(NULL, "pressure"),
      mEnabled(0),
      mInputReader(32),
      mHasPendingEvent(false)
{
    data_fd = SensorBase::openInputDev("lps331ap_pressure");
    LOGE_IF(data_fd < 0, "can't open lps331ap pressure input dev");

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_PR;
    mPendingEvent.type = SENSOR_TYPE_PRESSURE;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
}

PressureSensor::~PressureSensor()
{
    if (mEnabled)
        enable(0, 0);
}

int PressureSensor::setInitialState()
{
    return 0;
}

int PressureSensor::enable(int32_t handle, int en)
{
    int flags = en ? 1 : 0;

    if (flags != mEnabled) {
        int fd;
        fd = open(PRESSURE_ENABLE, O_RDWR);
        if (fd >= 0) {
            char buf[2];
            buf[1] = 0;
            if (flags) {
                buf[0] = '1';
            } else {
                buf[0] = '0';
            }
            int ret = write(fd, buf, sizeof(buf));
            close(fd);
            if (ret == sizeof(buf)) {
                mEnabled = flags;
                return 0;
            }
        }
        return -1;
    }
    return 0;
}

int PressureSensor::setDelay(int32_t handle, int64_t delay_ns)
{
    LOGD("PressureSensor: %s delay_ns=%lld", __FUNCTION__, delay_ns);

    int fd, ms;
    char buf[10] = { 0 };

    if ((fd = open(PRESSURE_POLL_DELAY, O_RDWR)) < 0) {
        LOGE("PressureSensor: Open %s failed!", PRESSURE_POLL_DELAY);
        return -1;
    }

    ms = delay_ns / 1000000;
    snprintf(buf, sizeof(buf), "%d", ms);
    write(fd, buf, sizeof(buf));
    close(fd);

    return 0;
}

bool PressureSensor::hasPendingEvents() const
{
    return mHasPendingEvent;
}

int PressureSensor::readEvents(sensors_event_t* data, int count)
{
    unsigned long pressure = 0;

    if (count < 1)
        return -EINVAL;

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_ABS || type == EV_REL) {
            switch (event->code) {
            case EVENT_TYPE_PRESSURE:
                pressure = event->value;
                break;
            case EVENT_TYPE_TEMPERATURE:
                /* ignore temperature data from sensor */
                break;
            default:
                LOGE("PressureSensor: unknown event (type=%d, code=%d)",
                     type, event->code);
            }
        } else if (type == EV_SYN) {
            mPendingEvent.pressure = (float)pressure / 4096;
            mPendingEvent.timestamp = timevalToNano(event->time);
            if (mEnabled) {
                *data++ = mPendingEvent;
                count -= 1;
                numEventReceived += 1;
            }
        } else {
            LOGE("PressureSensor: unknown event (type=%d, code=%d)",
                 type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}
