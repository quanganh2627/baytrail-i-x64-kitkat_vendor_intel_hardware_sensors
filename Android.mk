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

ifeq ($(ENABLE_SENSOR_HAL_ISH),true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := sensors.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_OWNER := intel
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_MODULE_TAGS := optional

LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\"
LOCAL_SRC_FILES := SensorHAL.cpp    \
                   PlatformConfig.cpp \
                   PSHSensor.cpp \
                   Sensor.cpp \
                   SensorDevice.cpp \
                   PSHCommonSensor.cpp \
                   SensorHubHelper.cpp \
                   utils.cpp

LOCAL_C_INCLUDES := $(COMMON_INCLUDES) \
                    $(call include-path-for, stlport) \
                    $(call include-path-for, stlport)/stl \
                    $(call include-path-for, stlport)/using/h/ \
                    $(call include-path-for, icu4c-common) \
                    $(call include-path-for, libsensorhub) \
                    $(TARGET_OUT_HEADERS)/awarelibs

LOCAL_SHARED_LIBRARIES := liblog libcutils libdl libicuuc libstlport libutils libsensorhub

include external/stlport/libstlport.mk

LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)

endif
