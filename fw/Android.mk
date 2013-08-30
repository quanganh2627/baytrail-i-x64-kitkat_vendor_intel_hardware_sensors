
ifeq ($(USE_GENERAL_SENSOR_DRIVER),true)

#Fix me for new platform
ifeq ($(REF_DEVICE_NAME), $(filter $(REF_DEVICE_NAME), redhookbay))

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := sensor_parser.c
LOCAL_C_INCLUDES +=  $(COMMON_INCLUDES) \
		external/libxml2/include \
		external/icu4c/common \
LOCAL_CFLAGS += -g -Wall
LOCAL_STATIC_LIBRARIES := libxml2
LOCAL_SHARED_LIBRARIES := libicuuc
LOCAL_MODULE := sensor_parser
include $(BUILD_HOST_EXECUTABLE)

#Select XML driver config for each platform
ifeq ($(REF_DEVICE_NAME),redhookbay)
SENSOR_DRIVER_XML := device/intel/clovertrail/board/redhookbay/sensors/sensor_driver_config.xml
endif

include $(CLEAR_VARS)
LOCAL_MODULE := sensor_config.bin
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/firmware/
include $(BUILD_SYSTEM)/base_rules.mk
$(LOCAL_BUILT_MODULE): sensor_parser
	@echo "Generating Sensor Driver Firmware image..."
	$(hide)mkdir -p $(dir $@)
	$(hide)sensor_parser -p -x $(SENSOR_DRIVER_XML) -f $@

endif # REF_DEVICE_NAME

endif # USE_GENERAL_SENSOR_DRIVER
