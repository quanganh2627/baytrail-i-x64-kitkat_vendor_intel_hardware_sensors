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

#include <cutils/log.h>

#include "AccelSensor.h"
#include "sensors.h"

#define SENSOR_NOPOLL   0x7fffffff
#define ACCEL_ENABLE    "/sys/bus/i2c/devices/5-0019/lis3dh/enable"
#define ACCEL_DELAY     "/sys/bus/i2c/devices/5-0019/lis3dh/poll"

/*****************************************************************************/

AccelSensor::AccelSensor()
    : SensorBase(NULL, "accel"),
      mEnabled(0),
      mPendingMask(0),
      mInputReader(32)
{
    data_fd = SensorBase::openInputDev("accel");
    LOGE_IF(data_fd < 0, "can't open accel input dev");

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = SENSORS_HANDLE_ACCELERATION;
    mPendingEvent.type = SENSOR_TYPE_ACCELEROMETER;
    mPendingEvent.acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
}

AccelSensor::~AccelSensor()
{
    if (mEnabled)
        enable(0, 0);
}

int AccelSensor::enable(int32_t handle, int en)
{
    unsigned int flags = en ? 1 : 0;

    D("AccelSensor-%s, flags = %d, mEnabled = %d", __func__, flags, mEnabled);

    if (flags != mEnabled) {
        int fd;
        fd = open(ACCEL_ENABLE, O_RDWR);
        D("AccelSensor-%s, fd = %d", __func__, fd);
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
        D("AccelSensor-%s, errno = %d", __func__, errno);
        return -1;
    }

    return 0;
}

int AccelSensor::setDelay(int32_t handle, int64_t ns)
{
    int fd = open(input_setDelay, O_RDWR);
    int ms;
    char buf[10] = { 0 };

    fd = open(ACCEL_DELAY, O_RDWR);
    if (fd < 0) {
	    E("%s: %s", ACCEL_DELAY, strerror(errno));
	    return -1;
    }

    if (ns / 1000 == SENSOR_NOPOLL)
        ms = 0;
    else
        ms = ns / 1000000;

    snprintf(buf, sizeof(buf), "%d", ms);
    write(fd, buf, sizeof(buf));
    close(fd);

    return 0;
}

int AccelSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    D("AccelSensor::%s", __func__);

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        D("AccelSensor::%s, type = %d, code = %d", __func__, type, event->code);
        if (type == EV_REL) {
            float value = event->value;
            if (event->code == EVENT_TYPE_ACCEL_X)
                mPendingEvent.data[1] = CONVERT_A_X(value);
            else if (event->code == EVENT_TYPE_ACCEL_Y)
                mPendingEvent.data[0] = CONVERT_A_Y(value);
            else if (event->code == EVENT_TYPE_ACCEL_Z)
                mPendingEvent.data[2] = CONVERT_A_Z(value);
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            if (mEnabled) {
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
            }
            D("Accel-{%f, %f, %f}", mPendingEvent.data[0],
                                    mPendingEvent.data[1],
                                    mPendingEvent.data[2]);
        } else {
            LOGE("AccelSensor: unknown event (type=%d, code=%d)",
                 type, event->code);
        }
        mInputReader.next();
    }
    D("AccelSensor::%s, numEventReceived= %d", __func__, numEventReceived);
    return numEventReceived;
}
