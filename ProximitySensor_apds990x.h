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

#ifndef ANDROID_PROXIMITY_SENSOR_H
#define ANDROID_PROXIMITY_SENSOR_H

#include "SensorBase.h"

#define APDS990X_MAX_CROSSTALK            900
#define APDS990X_MIN_CROSSTALK            20
#define APDS990X_PS_INIT_DATA             0xffff

#define SAMPLE_MAX_NUM                    20
#define SENSOR_CALIB_FILE                 "/data/proximity.conf"

typedef struct ps_calib {
    int num;
    int average;
    int crosstalk[SAMPLE_MAX_NUM];
} ps_calib_t;

class ProximitySensor : public SensorBase {
    int mEnabled;
    sensors_event_t mPendingEvent;
    bool mHasPendingEvent;

private:
    int calibCrosstalk(int raw);
    int getThresh(ps_calib_t *calib);

public:
    ProximitySensor(const sensor_platform_config_t *config);
    virtual ~ProximitySensor();
    virtual int readEvents(sensors_event_t* data, int count);
    virtual bool hasPendingEvents() const;
    virtual int enable(int32_t handle, int enabled);
};

#endif  // ANDROID_PROXIMITY_SENSOR_H
