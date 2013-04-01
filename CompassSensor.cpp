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

#include "CompassSensor.h"
#include "CompassCalibration.h"

#define COMPASS_CONVERT(x, gain) ((x) * 100 / gain)
#define VEC_DIFF_THRESHOLD_SQR 64
#define TOTAL_LARGE_LIMIT 5

CompassSensor::CompassSensor(const sensor_platform_config_t *config)
        : SensorBase(config),
          mEnabled(0),
          mInputReader(32),
          inputDataOverrun(0)
{
    if (mConfig->handle != SENSORS_HANDLE_MAGNETIC_FIELD)
        E("CompassSensor: Incorrect sensor config");

    data_fd = SensorBase::openInputDev(mConfig->name);
    LOGE_IF(data_fd < 0, "can't open compass input dev");

    if (!config->priv_data)
        mFilterEn = 0;
    else
        mFilterEn =
            ((union sensor_data_t *)config->priv_data)->compass_filter_en;

    mMagneticEvent.version = sizeof(sensors_event_t);
    mMagneticEvent.sensor = SENSORS_HANDLE_MAGNETIC_FIELD;
    mMagneticEvent.type = SENSOR_TYPE_MAGNETIC_FIELD;
    mMagneticEvent.magnetic.status = SENSOR_STATUS_ACCURACY_LOW;
    mMagneticEvent.magnetic.x = 0;
    mMagneticEvent.magnetic.y = 0;
    mMagneticEvent.magnetic.x = 0;

    mCalDataFile = -1;
    mCaled = 0;
    memset(&mCalData, 0, sizeof(CompassCalData));
}

CompassSensor::~CompassSensor()
{
    if (mEnabled)
        enable(0, 0);

    if (mCalDataFile > -1)
        close(mCalDataFile);
}

void CompassSensor::readCalibrationData()
{
    char buf[512];
    memset(buf, 0, 512);

    int ret = pread(mCalDataFile, buf, sizeof(buf), 0);
    if (ret > 0) {
        ret = sscanf(buf, "%d %lf %lf %lf %lf %lf %lf %lf\n", &mCaled,
            &mCalData.off_x, &mCalData.off_y, &mCalData.off_z, &mCalData.w11,
            &mCalData.w22, &mCalData.w33, &mCalData.bfield);
        if (ret != 8)
            mCaled = 0;
    } else {
        mCaled = 0;
    }
    if (!mCaled) {
        I("CompassSensor - %s: calibration data is not ready.", __func__);
        memset(&mCalData, 0, sizeof(CompassCalData));
    }

    CompassCal_init(mCaled, mCalData);
}

void CompassSensor::storeCalibrationData()
{
    char buf[512];
    int ret;
    memset(buf, 0, 512);
    snprintf(buf, sizeof(buf), "%d %f %f %f %f %f %f %f\n",
            mCaled, mCalData.off_x, mCalData.off_y, mCalData.off_z,
            mCalData.w11, mCalData.w22, mCalData.w33, mCalData.bfield);
    ret = pwrite(mCalDataFile, buf, sizeof(buf), 0);
    if (ret < 0) {
        E("CompassSensor - Store calibration data failed: %s", strerror(ret));
        E("CompassSensor - calibration data: %d %f %f %f %f %f %f %f\n",
                mCaled, mCalData.off_x, mCalData.off_y, mCalData.off_z,
                mCalData.w11, mCalData.w22, mCalData.w33, mCalData.bfield);
    }
}

int CompassSensor::enable(int32_t handle, int en)
{
    unsigned int flags = en ? 1 : 0;

    D("CompassSensor - %s, flags = %d, mEnabled = %d",
         __func__, flags, mEnabled);

    if (flags == 1 && mEnabled == 0) {
        mCalDataFile = open(mConfig->config_path, O_RDWR | O_CREAT, S_IRWXU);
        if (mCalDataFile > -1) {
            struct flock lock;
            lock.l_type = F_WRLCK;
            lock.l_start = 0;
            lock.l_whence = SEEK_SET;
            lock.l_len = 0;
            if (fcntl(mCalDataFile, F_SETLK, &lock) < 0)
                E("CompassSensor - calibration file lock fail");

            readCalibrationData();
            filter_index = -1;
        }
    } else if (flags == 0 && mEnabled == 1) {
        if (mCalDataFile > -1) {
            struct flock lock;
            lock.l_type = F_UNLCK;
            lock.l_start = 0;
            lock.l_whence = SEEK_SET;
            lock.l_len = 0;
            if (fcntl(mCalDataFile, F_SETLK, &lock) < 0)
                E("CompassSensor - calibration file unlock fail");

            storeCalibrationData();
            close(mCalDataFile);
        }
    }

    if (flags != mEnabled) {
        int fd;
        fd = open(mConfig->activate_path, O_RDWR);
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
        D("CompassSensor - %s, errno = %d", __func__, errno);
        return -1;
    }

    return 0;
}

int CompassSensor::setDelay(int32_t handle, int64_t ns)
{
    D("%s setDelay ns = %lld\n", __func__, ns);

    int fd;
    fd = open(mConfig->poll_path, O_RDWR);
    if (fd >= 0) {
        char buf[80];
        int ms = ns / 1000 / 1000;

        sprintf(buf, "%d", ms);
        write(fd, buf, strlen(buf)+1);
        close(fd);
        return 0;
    }

    E("%s, errno = %d", __func__, errno);
    return -1;
}

void CompassSensor::filter()
{
    int pre_index;
    int i;
    static int too_large_count = 0;

    /* reset filter if data is not following
     * the previous one.
     */
    if (filter_index != -1) {
        pre_index = (filter_index + FILTER_LENGTH - 1) % FILTER_LENGTH;
        if (mMagneticEvent.timestamp - filter_buffer[pre_index].timestamp >= FILTER_VALID_TIME) {
            filter_index = -1;
            D("compass sensor filter, reset filter because long time delay, pre = %lld, now = %lld",
                filter_buffer[pre_index].timestamp, mMagneticEvent.timestamp);
        }

        /* if the module difference of the magnetic vector is continuously larger
           than a threshold(for example, 8 ut), the device may have been significantly
           moved, we flush the filter buffer to quickly respond to the change.*/
        double x =  (double)mMagneticEvent.magnetic.x;
        double y =  (double)mMagneticEvent.magnetic.y;
        double z =  (double)mMagneticEvent.magnetic.z;
        double delta_x = x - filter_buffer[filter_index].magnetic.x;
        double delta_y = y - filter_buffer[filter_index].magnetic.y;
        double delta_z = z - filter_buffer[filter_index].magnetic.z;

        if (delta_x * delta_x + delta_y * delta_y + delta_z * delta_z > VEC_DIFF_THRESHOLD_SQR)
            ++too_large_count;
        else
            too_large_count = 0;

        if (too_large_count >= TOTAL_LARGE_LIMIT) {
            filter_index = -1;
            too_large_count = 0;
        }
    }

    if (filter_index == -1) {
        /* init filter data using current data */
        for (i = 0; i < FILTER_LENGTH; i ++) {
            filter_buffer[i].magnetic.x = mMagneticEvent.magnetic.x;
            filter_buffer[i].magnetic.y = mMagneticEvent.magnetic.y;
            filter_buffer[i].magnetic.z = mMagneticEvent.magnetic.z;
            filter_buffer[i].timestamp = mMagneticEvent.timestamp;
        }
        /* init filter sum value */
        filter_sum[0] = mMagneticEvent.magnetic.x * FILTER_LENGTH;
        filter_sum[1] = mMagneticEvent.magnetic.y * FILTER_LENGTH;
        filter_sum[2] = mMagneticEvent.magnetic.z * FILTER_LENGTH;

        filter_index = 0;
        D("compass sensor filter, first data read");
        return;
    }

    /* remove data from sum value */
    filter_sum[0] -= filter_buffer[filter_index].magnetic.x;
    filter_sum[1] -= filter_buffer[filter_index].magnetic.y;
    filter_sum[2] -= filter_buffer[filter_index].magnetic.z;

    /* replace a buffer slot with new data */
    filter_buffer[filter_index].magnetic.x = mMagneticEvent.magnetic.x;
    filter_buffer[filter_index].magnetic.y = mMagneticEvent.magnetic.y;
    filter_buffer[filter_index].magnetic.z = mMagneticEvent.magnetic.z;
    filter_buffer[filter_index].timestamp = mMagneticEvent.timestamp;

    /* update sum value using new data */
    filter_sum[0] += filter_buffer[filter_index].magnetic.x;
    filter_sum[1] += filter_buffer[filter_index].magnetic.y;
    filter_sum[2] += filter_buffer[filter_index].magnetic.z;

    filter_index = (filter_index + 1) % FILTER_LENGTH;

    /* use avg value of filter buffer to report data */
    mMagneticEvent.magnetic.x = filter_sum[0] / FILTER_LENGTH;
    mMagneticEvent.magnetic.y = filter_sum[1] / FILTER_LENGTH;
    mMagneticEvent.magnetic.z = filter_sum[2] / FILTER_LENGTH;

    return;
}

void CompassSensor::calibration(int64_t time)
{
    long current_time_ms = time / 1000000;
    CompassCal_collectData(mMagneticEvent.magnetic.x,
        mMagneticEvent.magnetic.y, mMagneticEvent.magnetic.z,
        current_time_ms);

    if (CompassCal_readyCheck()) {
        CompassCal_computeCal(&mCalData);
        mCaled = 1;
    }

    if (mCaled == 1) {
        mMagneticEvent.magnetic.x = (mMagneticEvent.magnetic.x - mCalData.off_x) * mCalData.w11;
        mMagneticEvent.magnetic.y = (mMagneticEvent.magnetic.y - mCalData.off_y) * mCalData.w22;
        mMagneticEvent.magnetic.z = (mMagneticEvent.magnetic.z - mCalData.off_z) * mCalData.w33;
        mMagneticEvent.magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
    } else {
        mMagneticEvent.magnetic.status = SENSOR_STATUS_ACCURACY_LOW;
    }
}

int CompassSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    D("%s count = %d ", __func__, count);

    while (count && mInputReader.readEvent(&event)) {
        D("readEvents event->type = %d, code=%d, value=%d",
          event->type, event->code, event->value);

        int type = event->type;
        if (type == EV_REL && !inputDataOverrun) {
            if (event->code == EVENT_TYPE_M_O_X)
                mMagneticEvent.data[mConfig->mapper[AXIS_X]] =
                        COMPASS_CONVERT(event->value, mConfig->scale[AXIS_X]);
            else if (event->code == EVENT_TYPE_M_O_Y)
                mMagneticEvent.data[mConfig->mapper[AXIS_Y]] =
                        COMPASS_CONVERT(event->value, mConfig->scale[AXIS_Y]);
            else if (event->code == EVENT_TYPE_M_O_Z)
                mMagneticEvent.data[mConfig->mapper[AXIS_Z]] =
                        COMPASS_CONVERT(event->value, mConfig->scale[AXIS_Z]);
        } else if (type == EV_SYN) {
            int64_t time = timevalToNano(event->time);

            /* drop input event overrun data */
            if (event->code == SYN_DROPPED) {
                E("CompassSensor: input event overrun, dropped event:drop");
                inputDataOverrun = 1;
            } else if (inputDataOverrun) {
                E("CompassSensor: input event overrun, dropped event:sync");
                inputDataOverrun = 0;
            } else {
                if (mEnabled) {
                    mMagneticEvent.timestamp = time;

                    /* compass calibration */
                    calibration(time);

                    D("CompassSensor magnetic befor filter=[%f, %f, %f] accuracy=%d, time=%lld",
                        mMagneticEvent.magnetic.x,
                        mMagneticEvent.magnetic.y,
                        mMagneticEvent.magnetic.z,
                        (int)mMagneticEvent.magnetic.status,
                        mMagneticEvent.timestamp);

                    /* data filter: used to mitigate data floating */
                    if (mFilterEn)
                        filter();

                    *data++ = mMagneticEvent;
                    count--;
                    numEventReceived++;
                    D("CompassSensor magnetic=[%f, %f, %f] accuracy=%d, time=%lld",
                      mMagneticEvent.magnetic.x,
                      mMagneticEvent.magnetic.y,
                      mMagneticEvent.magnetic.z,
                      (int)mMagneticEvent.magnetic.status,
                      mMagneticEvent.timestamp);
                }
            }
        } else {
            E("CompassSensor: unknown event (type=%d, code=%d)",
              type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}
