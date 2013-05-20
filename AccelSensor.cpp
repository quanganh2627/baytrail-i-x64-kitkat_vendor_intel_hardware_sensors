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

#include "AccelSensor.h"

#define SENSOR_NOPOLL   0x7fffffff
#define CONVERT_AXIS(a, b)   (((float)a/1000) * GRAVITY * b)

AccelSensor::AccelSensor(const sensor_platform_config_t *config)
    : SensorBase(config),
      mEnabled(0),
      mInputReader(32),
      inputDataOverrun(0)
{
    if (mConfig->handle != SENSORS_HANDLE_ACCELEROMETER)
        E("AccelSensor: Incorrect sensor config");

    data_fd = SensorBase::openInputDev(mConfig->name);
    LOGE_IF(data_fd < 0, "can't open accel input dev");

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = SENSORS_HANDLE_ACCELEROMETER;
    mPendingEvent.type = SENSOR_TYPE_ACCELEROMETER;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
    mPendingEvent.acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
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
        fd = open(mConfig->activate_path, O_RDWR);
        if (fd >= 0) {
            char buf[2] = { 0 };
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
        E("AccelSensor-%s, errno = %d", __func__, errno);
        return -1;
    }

    return 0;
}

int AccelSensor::setDelay(int32_t handle, int64_t ns)
{
    int fd, ms;
    char buf[10] = { 0 };

    fd = open(mConfig->poll_path, O_RDWR);
    if (fd < 0) {
        E("AccelSensor-%s:%s", __func__, strerror(errno));
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

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        D("AccelSensor::%s, type = %d, code = %d", __func__, type, event->code);
        if (type == EV_REL && !inputDataOverrun) {
            float value = event->value;
            if (event->code == EVENT_TYPE_ACCEL_X)
                mPendingEvent.data[mConfig->mapper[AXIS_X]] =
                                        CONVERT_AXIS(value, mConfig->scale[AXIS_X]);
            else if (event->code == EVENT_TYPE_ACCEL_Y)
                mPendingEvent.data[mConfig->mapper[AXIS_Y]] =
                                        CONVERT_AXIS(value, mConfig->scale[AXIS_Y]);
            else if (event->code == EVENT_TYPE_ACCEL_Z)
                mPendingEvent.data[mConfig->mapper[AXIS_Z]] =
                                        CONVERT_AXIS(value, mConfig->scale[AXIS_Z]);
        } else if (type == EV_SYN) {
            /* drop input event overrun data */
            if (event->code == SYN_DROPPED) {
                E("AccelSensor: input event overrun, dropped event:drop");
                inputDataOverrun = 1;
            } else if (inputDataOverrun) {
                E("AccelSensor: input event overrun, dropped event:sync");
                inputDataOverrun = 0;
            } else {
                mPendingEvent.timestamp = timevalToNano(event->time);
                if (mEnabled) {
                    *data++ = mPendingEvent;
                    count--;
                    numEventReceived++;
                }
                D("Accel-{%f, %f, %f}", mPendingEvent.data[0],
                                        mPendingEvent.data[1],
                                        mPendingEvent.data[2]);
            }
        } else {
            E("AccelSensor: unknown event (type=%d, code=%d)",
                 type, event->code);
        }
        mInputReader.next();
    }
    D("AccelSensor::%s, numEventReceived= %d", __func__, numEventReceived);
    return numEventReceived;
}
