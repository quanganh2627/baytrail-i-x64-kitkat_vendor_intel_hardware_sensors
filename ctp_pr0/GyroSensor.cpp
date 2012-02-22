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
#include <sys/stat.h>

#include "GyroSensor.h"
#include "sensors.h"

#define GYRO_ENABLE "/sys/bus/i2c/devices/5-0068/enable"
#define GYRO_POLL_DELAY "/sys/bus/i2c/devices/5-0068/poll"
#define GYRO_MIN_DELAY 10000000


/*****************************************************************************/

GyroSensor::GyroSensor()
    : SensorBase(NULL, "gyro"),
      mEnabled(0),
      mInputReader(4),
      mHasPendingEvent(false)
{
    data_fd = SensorBase::openInputDev("l3g4200d");
    LOGE_IF(data_fd < 0, "can't open l3g4200d gyro input dev");

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_GY;
    mPendingEvent.type = SENSOR_TYPE_GYROSCOPE;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
}

GyroSensor::~GyroSensor()
{
    if (mEnabled)
        enable(0, 0);

    if (conf_fd > -1)
        close(conf_fd);
}

int GyroSensor::setInitialState()
{
    return 0;
}

int GyroSensor::enable(int32_t handle, int en)
{
    int fd, ret = 0;
    int flags = en ? 1 : 0;
    char buf[50];

    D("GyroSensor: en %d", en);

    if (flags == mEnabled)
        return 0;

    if ((fd = open(GYRO_ENABLE, O_RDWR)) < 0) {
        E("GyroSensor: Open %s failed!", GYRO_ENABLE);
        return -1;
    }

    if (flags == 1 && mEnabled == 0) {
        conf_fd = open("/data/gyro.conf", O_RDONLY);
        memset(mCalEvent.data, 0, sizeof(mCalEvent.data));
        if (conf_fd > -1) {
            ret = pread(conf_fd, buf, sizeof(buf), 0);
            if (ret > 0) {
                ret = sscanf(buf, "%f %f %f\n",
                      &mCalEvent.data[0], &mCalEvent.data[1], &mCalEvent.data[2]);
                if (ret != 3)
                    mCalEvent.data[0] = mCalEvent.data[1] = mCalEvent.data[2] = 0;
                D("GyroSensor cal value = [%f, %f, %f]\n", mCalEvent.data[0],
                    mCalEvent.data[1], mCalEvent.data[2]);
            }
            close(conf_fd);
        }
    }

    buf[1] = 0;
    buf[0] = flags ? '1' : '0';
    ret = write(fd, buf, sizeof(buf));
    if (ret > 0) {
        mEnabled = flags;
        ret = 0;
    }
    close(fd);

    return ret;
}

bool GyroSensor::hasPendingEvents() const
{
    return mHasPendingEvent;
}

int GyroSensor::setDelay(int32_t handle, int64_t delay_ns)
{
    int fd;
    unsigned long delay_ms;
    char buf[10] = { 0 };

    D("GyroSensor::%s, delay_ns=%lld", __func__, delay_ns);

    if ((fd = open(GYRO_POLL_DELAY, O_RDWR)) < 0) {
        E("GyroSensor: Open %s failed!", GYRO_POLL_DELAY);
        return -1;
    }

    if (delay_ns < GYRO_MIN_DELAY)
        delay_ns = GYRO_MIN_DELAY;

    delay_ms = delay_ns / 1000000;
    snprintf(buf, sizeof(buf), "%ld", delay_ms);
    write(fd, buf, sizeof(buf));
    close(fd);

    return 0;
}

float processRawData(int value)
{
    return (float)value * CONVERT_GYRO;
}

int GyroSensor::readEvents(sensors_event_t* data, int count)
{
    D("GyroSensor::%s", __func__);

    if (count < 1)
        return -EINVAL;

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_REL) {
            float value = event->value;
            if (event->code == REL_X) {
                mPendingEvent.data[1] =
                    processRawData(value) - mCalEvent.data[1];
            } else if (event->code == REL_Y) {
                mPendingEvent.data[0] =
                    processRawData(value) - mCalEvent.data[0];
            } else if (event->code == REL_Z) {
                mPendingEvent.data[2] =
                    processRawData(value) * -1 - mCalEvent.data[2];
            }
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            if (mEnabled) {
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
                D("gyro = [%f, %f, %f]\n", mPendingEvent.data[0],
                                mPendingEvent.data[1], mPendingEvent.data[2]);
            }
        } else {
            LOGE("GyroSensor: unknown event (type=%d, code=%d)",
                    type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}
