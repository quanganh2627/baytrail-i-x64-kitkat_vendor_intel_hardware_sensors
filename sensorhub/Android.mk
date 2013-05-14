# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


ifeq ($(ENABLE_SENSOR_HUB),true)

LOCAL_PATH := $(call my-dir)

# HAL module implemenation, not prelinked, and stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)

LOCAL_MODULE := sensors.$(TARGET_DEVICE)

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE_TAGS := optional

LOCAL_C_INCLUDES :=  $(COMMON_INCLUDES) \
        $(TOP)/frameworks/base/include/ \
        $(ANDROID_BUILD_TOP)/external/stlport/stlport/ \
        $(ANDROID_BUILD_TOP)/external/stlport/stlport/stl \
        $(ANDROID_BUILD_TOP)/external/stlport/stlport/using/h/ \
        $(ANDROID_BUILD_TOP)/external/icu4c/common \
        $(ANDROID_BUILD_TOP)/external/libxml2/include \
        $(ANDROID_BUILD_TOP)/bionic \
        $(TOP)/vendor/intel/Aware/core/Physical/libs \
        $(TOP)/vendor/intel/Aware/core/Gesture/libgesture

LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\" -DINSTANT_MODE
LOCAL_SRC_FILES :=     config.cpp                 \
                       sensors.cpp                \
                       GyroSensor.cpp             \
                       SensorBase.cpp             \
                       LightSensor.cpp            \
                       ProximitySensor.cpp        \
                       AccelSensor.cpp            \
                       MagneticSensor.cpp         \
                       PressureSensor.cpp         \
                       GravitySensor.cpp          \
                       LinearAccelSensor.cpp      \
                       RotationVectorSensor.cpp   \
                       OrientationSensor.cpp      \
                       TerminalSensor.cpp         \
                       GestureFlickSensor.cpp     \
                       GestureSensor.cpp          \
                       PhysicalActivitySensor.cpp \
                       PedometerSensor.cpp \
                       ShakeSensor.cpp

LOCAL_SHARED_LIBRARIES := liblog libcutils libdl libicuuc libsensorhub libutils libstlport
LOCAL_STATIC_LIBRARIES := libxml2
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)

endif
