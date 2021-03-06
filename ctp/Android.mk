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

ifeq ($(REF_DEVICE_NAME), $(filter $(REF_DEVICE_NAME), victoriabay))

LOCAL_PATH := $(call my-dir)

ifneq ($(TARGET_SIMULATOR),true)

# HAL module implemenation, not prelinked, and stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)

LOCAL_MODULE := sensors.$(TARGET_DEVICE)
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE_TAGS := optional

LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\"
LOCAL_SRC_FILES := config.cpp                   \
                   ../InputEventReader.cpp      \
                   ../sensors.cpp               \
                   ../SensorBase.cpp

LOCAL_SRC_FILES +=  ../AccelSensor.cpp          \
                    ../LightSensor.cpp          \
                    ../ProximitySensor_apds990x.cpp \
                    ../CompassSensor.cpp        \
                    ../CompassCalibration.cpp   \
                    ../GyroSensor.cpp           \
                    ../PressureSensor.cpp

LOCAL_C_INCLUDES := $(COMMON_INCLUDES) \
                    $(call include-path-for, icu4c-common) \
                    $(call include-path-for, libxml2)
LOCAL_SHARED_LIBRARIES := liblog libcutils libdl libicuuc
LOCAL_STATIC_LIBRARIES := libxml2

LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)

endif # !TARGET_SIMULATOR

endif
