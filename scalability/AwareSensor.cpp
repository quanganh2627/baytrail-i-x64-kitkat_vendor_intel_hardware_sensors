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
/*****************************************************************************/
#include "AwareSensor.hpp"
#include <dlfcn.h>
#undef LOG_TAG
#define LOG_TAG "AwareSensor"
VirtualLogical * (*AwareSensor::method_get_virtualsensor)(int, int);
AwareSensor::AwareSensor(SensorDevice &device) : PSHSensor(device) {
        VirtualSensorMethodsInitialize();
        if (method_get_virtualsensor != NULL) {
                virtualSensor = method_get_virtualsensor(device.getType(),
                                device.getHandle());
        } else {
                LOGE("method_get_virtualsensor == NULL");
        }
}

AwareSensor::~AwareSensor() {
        LOGI("~AwareSensor");
        if (virtualSensor != NULL) {
                delete virtualSensor;
                virtualSensor = NULL;
        }
        VirtualSensorMethodsFinallize();
}

bool AwareSensor::VirtualSensorMethodsInitialize() {
        virtualsensorHandle = dlopen("/system/lib/libvirtualsensors.so",
                        RTLD_LAZY);
        if (virtualsensorHandle == NULL) {
                LOGE("dlopen: /system/lib/libvirtualsensors.so error!");
                return false;
        }

        if (method_get_virtualsensor == NULL) {
                method_get_virtualsensor =
                                reinterpret_cast<VirtualLogical * (*)(int, int)>(dlsym(virtualsensorHandle, "createSensor"));
                if (method_get_virtualsensor == NULL) {
                        LOGE("dlsym: method_get_virtualsensor error!");
                        VirtualSensorMethodsFinallize();
                        return false;
                } else {
                        LOGD("handle method_get_virtualsensor success!!");
                }
        }
        return true;
}

bool AwareSensor::VirtualSensorMethodsFinallize() {
        if (virtualsensorHandle != NULL) {
                int err = dlclose(virtualsensorHandle);
                if (err != 0) {
                        LOGE("dlclose error! %d", err);
                        return false;
                }
        }
        return true;
}

bool AwareSensor::selftest() {
        LOGI("selftest");
        if (virtualSensor != NULL) {
                return virtualSensor->selftest();
        } else {
                return false;
        }
}

int AwareSensor::activate(int32_t handle, int en) {
        LOGI("activate");
        if (virtualSensor != NULL) {
                return virtualSensor->activate(handle, en);
        } else {
                return -1;
        }
}

int AwareSensor::setDelay(int32_t handle, int64_t ns) {
        LOGI("setDelay ns = %lld", ns);
        if (virtualSensor != NULL) {
                return virtualSensor->setDelay(handle, ns);
        } else {
                return -1;
        }
}

int AwareSensor::getPollfd() {
        if (virtualSensor != NULL) {
                return virtualSensor->getPollfd();
        } else {
                return 0;
        }
}

int AwareSensor::getData(std::queue<sensors_event_t> &eventQue) {
        LOGI("getData");
        if (virtualSensor != NULL) {
                return virtualSensor->getData(eventQue);
        } else {
                return 0;
        }
}

int AwareSensor::flush(int handle) {
        LOGD("flush");
        if (virtualSensor != NULL) {
                return virtualSensor->flush(handle);
        } else {
                return -EINVAL;
        }
}

void AwareSensor::resetEventHandle() {
        LOGD("resetEventHandle");
        if (virtualSensor != NULL) {
                virtualSensor->resetEventHandle(device.getHandle());
        }
}
