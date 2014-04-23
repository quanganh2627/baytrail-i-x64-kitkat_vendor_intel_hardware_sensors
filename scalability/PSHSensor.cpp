#include "PSHSensor.hpp"
#include <dlfcn.h>

struct sensor_hub_methods PSHSensor::methods;

PSHSensor::PSHSensor()
{
        SensorHubMethodsInitialize();
}

PSHSensor::PSHSensor(SensorDevice &mDevice)
        :Sensor(mDevice)
{
        SensorHubMethodsInitialize();
}

PSHSensor::~PSHSensor()
{
        SensorHubMethodsFinallize();
}

bool PSHSensor::SensorHubMethodsInitialize()
{
        methodsHandle = dlopen("/system/lib/libsensorhub.so", RTLD_LAZY);
        if (methodsHandle == NULL) {
                LOGE("dlopen: /system/lib/libsensorhub.so error!");
                return false;
        }

        if (methods.psh_open_session == NULL) {
                methods.psh_open_session = reinterpret_cast<handle_t (*)(psh_sensor_t)>(dlsym(methodsHandle, "psh_open_session"));
                if (methods.psh_open_session == NULL) {
                        LOGE("dlsym: psh_open_session error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_get_fd == NULL) {
                methods.psh_get_fd = reinterpret_cast<int (*)(handle_t)>(dlsym(methodsHandle, "psh_get_fd"));
                if (methods.psh_get_fd == NULL) {
                        LOGE("dlsym: psh_get_fd error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_close_session == NULL) {
                methods.psh_close_session = reinterpret_cast<void (*)(handle_t)>(dlsym(methodsHandle, "psh_close_session"));
                if (methods.psh_close_session == NULL) {
                        LOGE("dlsym: psh_close_session error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_start_streaming == NULL) {
                methods.psh_start_streaming = reinterpret_cast<error_t (*)(handle_t, int, int)>(dlsym(methodsHandle, "psh_start_streaming"));
                if (methods.psh_start_streaming == NULL) {
                        LOGE("dlsym: psh_start_streaming error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_start_streaming_with_flag == NULL) {
                methods.psh_start_streaming_with_flag = reinterpret_cast<error_t (*)(handle_t, int, int, streaming_flag)>(dlsym(methodsHandle, "psh_start_streaming_with_flag"));
                if (methods.psh_start_streaming_with_flag == NULL) {
                        LOGE("dlsym: psh_start_streaming_with_flag error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_stop_streaming == NULL) {
                methods.psh_stop_streaming = reinterpret_cast<error_t (*)(handle_t)>(dlsym(methodsHandle, "psh_stop_streaming"));
                if (methods.psh_stop_streaming == NULL) {
                        LOGE("dlsym: psh_stop_streaming error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_set_property == NULL) {
                methods.psh_set_property = reinterpret_cast<error_t (*)(handle_t, property_type, void *)>(dlsym(methodsHandle, "psh_set_property"));
                if (methods.psh_set_property == NULL) {
                        LOGE("dlsym: psh_set_property error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_set_property_with_size == NULL) {
                methods.psh_set_property_with_size = reinterpret_cast<error_t (*)(handle_t, property_type, int, void *)>(dlsym(methodsHandle, "psh_set_property_with_size"));
                if (methods.psh_set_property_with_size == NULL) {
                        LOGE("dlsym: psh_set_property_with_size error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_flush_streaming == NULL) {
                methods.psh_flush_streaming = reinterpret_cast<error_t (*)(handle_t, unsigned int)>(dlsym(methodsHandle, "psh_flush_streaming"));
                if (methods.psh_flush_streaming == NULL) {
                        LOGE("dlsym: psh_flush_streaming error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        return true;
}

bool PSHSensor::SensorHubMethodsFinallize()
{
        if (methodsHandle != NULL) {
                int err = dlclose(methodsHandle);
                if (err != 0) {
                        LOGE("dlclose error! %d", err);
                        return false;
                }
        }
        return true;
}

int PSHSensor::flush(int handle)
{
        error_t err;

        if (handle != device.getHandle()) {
                LOGE("%s: line: %d: %s handle not match! handle: %d required handle: %d",
                     __FUNCTION__, __LINE__, device.getName(), device.getHandle(), handle);
                return -EINVAL;
        }

        if (!state.getActivated()) {
                LOGW("%s line: %d %s not activated", __FUNCTION__, __LINE__, device.getName());
                return -EINVAL;
        }

        if (methods.psh_flush_streaming == NULL) {
                LOGE("psh_flush_streaming not initialized!");
                return -EINVAL;
        }

        err = methods.psh_flush_streaming(sensorHandle, SensorHubHelper::getUnitSize(device.getType()));

        return err == ERROR_NONE ? 0 : -EINVAL;
}
