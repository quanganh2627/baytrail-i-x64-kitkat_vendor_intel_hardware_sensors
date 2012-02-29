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

#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <dlfcn.h>
#include <cutils/log.h>

#include "CompassSensor.h"

#define COMPASS_SENSOR_DATA_NAME "compass"
#define COMPASS_CALIB_DATA "/data/compass.conf"
#define COMPASS_XY_GAIN 670
#define COMPASS_Z_GAIN  600
#define COMPASS_CONVERT_XY(xy) ((xy) * 100 / COMPASS_XY_GAIN)
#define COMPASS_CONVERT_Z(z) ((z) * 100 / COMPASS_Z_GAIN)

/* calibration indicators
 * MAGNETIC_MAX and MAGNETIC_MIN are
 * defined for max and min magnetic
 * values on earth, which is 30ut to
 * 60 ut. Value lager then MAGNETIC_MAX
 * means device is in high mag field
 */
#define MAGNETIC_MAX 80.0f /* 80 ut */
#define MAGNETIC_MIN 25.0f /* 25 ut */

#define COMPASS_ENABLE  "/sys/bus/i2c/devices/5-001e/lsm303cmp/enable"
#define COMPASS_POLL    "/sys/bus/i2c/devices/5-001e/lsm303cmp/poll"

CompassSensor::CompassSensor()
        : SensorBase(NULL, COMPASS_SENSOR_DATA_NAME),
          mEnabled(0),
          mInputReader(32)
{
    D("add: CompassSensor Sensor");

    data_fd = SensorBase::openInputDev("lsm303cmp");
    LOGE_IF(data_fd < 0, "can't open lsm303cmp compass input dev");

    mMagneticEvent.version = sizeof(sensors_event_t);
    mMagneticEvent.sensor = ID_M;
    mMagneticEvent.type = SENSOR_TYPE_MAGNETIC_FIELD;
    mMagneticEvent.magnetic.status = SENSOR_STATUS_ACCURACY_LOW;

    mDelay  = 200000000; // 200 ms by default
    mCalDataFile = -1;
    calibrated = 0;
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
    char buf[128];
    memset(buf, 0, 128);

    int ret = pread(mCalDataFile, buf, sizeof(buf), 0);
    if (ret > 0) {
        ret = sscanf(buf, "%d %f %f %f %f %f %f\n", &calibrated,
                     &Xmin, &Xmax, &Ymin, &Ymax, &Zmin, &Zmax);
        if (ret != 7)
            calibrated = Xmin = Xmax = Ymin = Ymax = Zmin = Zmax = 0;
    }
    magOffsetX = (Xmin + Xmax) / 2;
    magOffsetY = (Ymin + Ymax) / 2;
    magOffsetZ = (Zmin + Zmax) / 2;
    LOGD("readCalibrationData --calibrated-- = %d", calibrated);
    LOGD("readCalibrationData --x-- min = %f, max = %f", Xmin, Xmax);
    LOGD("readCalibrationData --y-- min = %f, max = %f", Ymin, Ymax);
    LOGD("readCalibrationData --z-- min = %f, max = %f", Zmin, Zmax);
}

void CompassSensor::storeCalibrationData()
{
    char buf[128];
    memset(buf, 0, 128);
    sprintf(buf, "%d %f %f %f %f %f %f\n", calibrated, Xmin,
        Xmax, Ymin, Ymax, Zmin, Zmax);
    pwrite(mCalDataFile, buf, sizeof(buf), 0);

    LOGD("storeCalibrationData --calibrated-- = %d", calibrated);
    LOGD("storeCalibrationData --x-- min = %f, max = %f", Xmin, Xmax);
    LOGD("storeCalibrationData --y-- min = %f, max = %f", Ymin, Ymax);
    LOGD("storeCalibrationData --z-- min = %f, max = %f", Zmin, Zmax);
}


int CompassSensor::enable(int32_t handle, int en)
{
    unsigned int flags = en ? 1 : 0;

    LOGD("CompassSensor - %s, flags = %d, mEnabled = %d",
         __func__, flags, mEnabled);

    if (flags == 1 && mEnabled == 0) {
        mCalDataFile = open(COMPASS_CALIB_DATA, O_RDWR | O_CREAT, S_IRWXU);
        if (mCalDataFile > -1) {
            struct flock lock;
            lock.l_type = F_WRLCK;
            lock.l_start = 0;
            lock.l_whence = SEEK_SET;
            lock.l_len = 0;
            if (fcntl(mCalDataFile, F_SETLK, &lock) < 0)
                LOGD("compass calibration file lock fail");

            readCalibrationData();
        }
    } else if (flags == 0 && mEnabled == 1) {
        if (mCalDataFile > -1) {
            struct flock lock;
            lock.l_type = F_UNLCK;
            lock.l_start = 0;
            lock.l_whence = SEEK_SET;
            lock.l_len = 0;
            if (fcntl(mCalDataFile, F_SETLK, &lock) < 0)
                LOGD("compass calibration file unlock fail");

            storeCalibrationData();
            close(mCalDataFile);
        }
    }

    if (flags != mEnabled) {
        int fd;
        fd = open(COMPASS_ENABLE, O_RDWR);
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
    LOGD("%s setDelay ns = %lld\n", __func__, ns);

    int fd;
    fd = open(COMPASS_POLL, O_RDWR);
    if (fd >= 0) {
        char buf[80];
        int ms = ns / 1000 / 1000;

        sprintf(buf, "%d", ms);
        write(fd, buf, strlen(buf)+1);
        close(fd);
        return 0;
    }

    D("%s, errno = %d", __func__, errno);
    return -1;
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
        if (type == EV_REL) {
            if (event->code == EVENT_TYPE_M_O_X)
                mRawX = event->value;
            else if (event->code == EVENT_TYPE_M_O_Y)
                mRawY = event->value;
            else if (event->code == EVENT_TYPE_M_O_Z)
                mRawZ = event->value;
        } else if (type == EV_SYN) {
            int64_t time = timevalToNano(event->time);
            if (mEnabled) {
                if (!ignoreCal())
                    calcEvent();
                convertEventUnit();
                mMagneticEvent.timestamp = time;
                if (calibrated)
                    mMagneticEvent.magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
                else
                    mMagneticEvent.magnetic.status = SENSOR_STATUS_ACCURACY_LOW;
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
        } else {
            E("CompassSensor: unknown event (type=%d, code=%d)",
              type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}

bool CompassSensor::ignoreCal()
{
    mMagneticEvent.magnetic.x = mRawX - magOffsetX;
    mMagneticEvent.magnetic.y = mRawY - magOffsetY;
    mMagneticEvent.magnetic.z = mRawZ - magOffsetZ;
    if (pow(COMPASS_CONVERT_XY(mMagneticEvent.magnetic.x), 2) +
            pow(COMPASS_CONVERT_XY(mMagneticEvent.magnetic.y), 2) +
            pow(COMPASS_CONVERT_Z(mMagneticEvent.magnetic.z), 2) >=
            pow(MAGNETIC_MAX, 2)) {
        D("CompassCal, data[%f, %f, %f] is not used for calibration",
            mMagneticEvent.magnetic.x,
            mMagneticEvent.magnetic.y,
            mMagneticEvent.magnetic.z);
        return true;
    }
    return false;
}

void  CompassSensor::calcEvent()
{
    if (Xmin == 0 && Xmax == 0) {
        Xmin = Xmax = mRawX;
    } else {
        if (mRawX > Xmax)
            Xmax = mRawX;
        if (mRawX < Xmin)
            Xmin = mRawX;
    }
    magOffsetX = (Xmax + Xmin) / 2;
    mMagneticEvent.magnetic.x = mRawX - magOffsetX;

    if (Ymin == 0 && Ymax == 0) {
        Ymin = Ymax = mRawY;
    } else {
        if (mRawY > Ymax)
            Ymax = mRawY;
        if (mRawY < Ymin)
            Ymin = mRawY;
    }
    magOffsetY = (Ymax + Ymin) / 2;
    mMagneticEvent.magnetic.y = mRawY - magOffsetY;

    if (Zmin == 0 && Zmax == 0) {
        Zmin = Zmax = mRawZ;
    } else {
        if (mRawZ > Zmax)
            Zmax = mRawZ;
        if (mRawZ < Zmin)
            Zmin = mRawZ;
    }
    magOffsetZ = (Zmax + Zmin) / 2;
    mMagneticEvent.magnetic.z = mRawZ - magOffsetZ;

    /* adjust if calibration is done */
    if (!calibrated) {
        if (COMPASS_CONVERT_XY(Xmax - Xmin) >= 2 * MAGNETIC_MIN &&
            COMPASS_CONVERT_XY(Ymax - Ymin) >= 2 * MAGNETIC_MIN &&
            COMPASS_CONVERT_Z(Zmax - Zmin) >= 2 * MAGNETIC_MIN)
            calibrated = 1;
    }
}

void CompassSensor::convertEventUnit()
{

    // convert from LSB/Gauss to micro-Tesla
    mMagneticEvent.magnetic.x = COMPASS_CONVERT_XY(mMagneticEvent.magnetic.x);
    mMagneticEvent.magnetic.y = COMPASS_CONVERT_XY(mMagneticEvent.magnetic.y);
    mMagneticEvent.magnetic.z = COMPASS_CONVERT_Z(mMagneticEvent.magnetic.z);

    //switch axis
    float temp = mMagneticEvent.magnetic.x;
    mMagneticEvent.magnetic.x = mMagneticEvent.magnetic.y;
    mMagneticEvent.magnetic.y = temp;
    mMagneticEvent.magnetic.z *= -1;
}
