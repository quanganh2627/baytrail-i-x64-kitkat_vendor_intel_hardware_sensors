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

#include "ProximitySensor_apds990x.h"

ProximitySensor::ProximitySensor(const sensor_platform_config_t *config)
    : SensorBase(config),
      mEnabled(0),
      mHasPendingEvent(false)
{
    if (mConfig->handle != SENSORS_HANDLE_PROXIMITY)
        E("ProximitySensor: Incorrect sensor config");

    data_fd = open(mConfig->activate_path, O_RDONLY);
    LOGE_IF(data_fd < 0, "can't open %s", mConfig->activate_path);

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = SENSORS_HANDLE_PROXIMITY;
    mPendingEvent.type = SENSOR_TYPE_PROXIMITY;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
}

ProximitySensor::~ProximitySensor()
{
    if (mEnabled)
        enable(0, 0);
}

int ProximitySensor::calibThresh(int raw_data)
{
    int ret, fd, thresh = APDS990X_MAX_THRESH;
    off_t offset = 0;
    struct flock lock;
    ps_calib_t calib;

    if ((fd = open(SENSOR_CALIB_FILE, O_RDWR | O_CREAT, S_IRWXU)) < 0) {
        E("ProximitySensor: open %s failed, %s",
                                        SENSOR_CALIB_FILE, strerror(errno));
        return fd;
    }

    lock.l_type = F_WRLCK;
    lock.l_start = 0;
    lock.l_whence = SEEK_SET;
    lock.l_len = 0;
    if (fcntl(fd, F_SETLK, &lock) < 0) {
        LOGI("ProximitySensor: File lock failed failed, %s", strerror(errno));
        close(fd);
        return -1;
    }

    memset(&calib, 0, sizeof(calib));
    while ((ret = pread(fd, &calib, sizeof(calib), offset)) > 0) {
        LOGI("ProximitySensor: pread %d bytes from seonsr config file", ret);
        if (calib.type == SENSOR_TYPE_PROXIMITY) {
            LOGI("ProximitySensor: thresh=%d, raw_data=%d",
                                                    calib.thresh, raw_data);
            thresh = calib.thresh;
            break;
        }
        offset += sizeof(calib);
    }
    if (raw_data < thresh && raw_data >= APDS990X_MIN_THRESH) {
        thresh = raw_data;
        calib.thresh = thresh;
        calib.type = SENSOR_TYPE_PROXIMITY;
        if ((ret = pwrite(fd, &calib, sizeof(calib), offset)) > 0)
            LOGI("ProximitySensor: write %d bytes to sensor config file", ret);
        else
            LOGI("ProximitySensor: write data failed, %s", strerror(errno));
    }

    lock.l_type = F_UNLCK;
    if (fcntl(fd, F_SETLK, &lock) < 0) {
        LOGI("ProximitySensor: File unlock failed, %s", strerror(errno));
        thresh = -1;
    }
    close(fd);

    return thresh;
}

int ProximitySensor::enable(int32_t, int en)
{
    int fd, thresh, raw_data = 0, len;
    int flag = en ? 1 : 0;
    char buf[16] = { 0 };

    LOGI("ProximitySensor - %s - enable=%d", __func__, en);
    if (flag == mEnabled)
        return 0;

    if (ioctl(data_fd, 0, en)) {
        E("ProximitySensor: ioctl set %s failed", mConfig->activate_path);
        return -1;
    }
    mEnabled = flag;
    if (!flag)
        return 0;

    if ((fd = open(mConfig->data_path, O_RDONLY)) < 0) {
        LOGI("ProximitySensor: open %s failed, %s!",
				mConfig->data_path, strerror(errno));
        return 0;
    }
    for (int i = 0; i < APDS990X_ENABLE_TRY; i++) {
        usleep(100000);
        len = read(fd, &buf, sizeof(buf) - 1);
        if (len < 0) {
            LOGI("ProximitySensor: read %s failed, %s!",
				    mConfig->data_path, strerror(errno));

            close(fd);
            return 0;
        }
        LOGI("ProximitySensor - buf: %s", buf);
        sscanf(buf, "%d\n", &raw_data);
        if (raw_data > 0)
            break;
    }
    close(fd);

    LOGI("ProximitySensor - raw data for calibration:%d", raw_data);
    if ((thresh = calibThresh(raw_data)) < 0) {
        LOGI("ProximitySensor - calibration failed");
        return 0;
    }

    thresh += APDS990X_DISTANCE_THRESH;
    if ((fd = open(mConfig->config_path, O_WRONLY)) < 0) {
        LOGI("ProximitySensor: open %s failed, %s",
			mConfig->config_path, strerror(errno));
        return 0;
    }
    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "%d\n", thresh);
    if (write(fd, buf, strlen(buf)) < 0) {
        LOGI("ProximitySensor: write %s failed, %s",
                mConfig->config_path, strerror(errno));
        close(fd);
        return 0;
    }
    LOGI("ProximitySensor: set thresh to %s.", buf);
    close(fd);
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
    if (count < 1 || data == NULL || data_fd < 0)
        return -EINVAL;

    *data = mPendingEvent;
    data->timestamp = getTimestamp();
    read(data_fd, &val, sizeof(int));
    data->distance = (float)(val == 1 ? 0 : 6);
    LOGI("ProximitySensor - read data %f, %s",
            data->distance, data->distance > 0 ? "far" : "near");

    return 1;
}
