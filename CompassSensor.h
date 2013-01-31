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

#ifndef ANDROID_COMPASS_SENSOR_H
#define ANDROID_COMPASS_SENSOR_H

#include "SensorBase.h"

#define FILTER_LENGTH 100
#define FILTER_VALID_TIME (100L * 1000L * 1000L) /* 100ms */

class CompassSensor : public SensorBase {
public:
    CompassSensor(const sensor_platform_config_t *config);
    virtual ~CompassSensor();
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int enable(int32_t handle, int enabled);
    virtual int readEvents(sensors_event_t* data, int count);

private:
    void readCalibrationData();
    void storeCalibrationData();
    void calibration(int64_t time);
    void filter();

private:
    uint32_t mEnabled;
    InputEventCircularReader mInputReader;
    sensors_event_t mMagneticEvent;
    int inputDataOverrun;

    /* for calibration */
    int mCalDataFile;
    int mCaled;
    float mMinX, mMinY, mMinZ, mMaxX, mMaxY, mMaxZ;
    float mKxx, mKyy, mKzz;

    /* data filter */
    int mFilterEn;
    sensors_event_t filter_buffer[FILTER_LENGTH];
    float filter_sum[3];
    int filter_index;
};
#endif
