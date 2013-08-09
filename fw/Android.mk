
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

include $(CLEAR_VARS)

LOCAL_MODULE := sensor_config.bin
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/firmware/

include $(BUILD_SYSTEM)/base_rules.mk

SENSOR_DRIVER_XML := device/intel/clovertrail/board/redhookbay/sensors/sensor_driver_config.xml

$(LOCAL_BUILT_MODULE): sensor_parser
	@echo "Generating Sensor Driver Firmware image..."
	$(hide)mkdir -p $(dir $@)
	$(hide)sensor_parser -p -x $(SENSOR_DRIVER_XML) -f $@
