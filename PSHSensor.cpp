#include "PSHSensor.hpp"
#include <dlfcn.h>

struct sensor_hub_methods PSHSensor::methods;

PSHSensor::PSHSensor()
{
        SensorHubMethodsInitialize();
        activated = false;
}

PSHSensor::PSHSensor(SensorDevice &mDevice)
        :Sensor(mDevice)
{
        SensorHubMethodsInitialize();
        activated = false;
}

PSHSensor::~PSHSensor()
{
        SensorHubMethodsFinallize();
}

bool PSHSensor::SensorHubMethodsInitialize()
{
        methodsHandle = dlopen("/system/lib/libsensorhub.so", RTLD_LAZY);
        if (methodsHandle == NULL) {
                ALOGE("dlopen: /system/lib/libsensorhub.so error!");
                return false;
        }

        if (methods.psh_open_session == NULL) {
                methods.psh_open_session = reinterpret_cast<handle_t (*)(ish_sensor_t)>(dlsym(methodsHandle, "ish_open_session"));
                if (methods.psh_open_session == NULL) {
                        ALOGE("dlsym: ish_open_session error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_get_fd == NULL) {
                methods.psh_get_fd = reinterpret_cast<int (*)(handle_t)>(dlsym(methodsHandle, "ish_get_fd"));
                if (methods.psh_get_fd == NULL) {
                        ALOGE("dlsym: psh_get_fd error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_close_session == NULL) {
                methods.psh_close_session = reinterpret_cast<void (*)(handle_t)>(dlsym(methodsHandle, "ish_close_session"));
                if (methods.psh_close_session == NULL) {
                        ALOGE("dlsym: ish_close_session error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_start_streaming == NULL) {
                methods.psh_start_streaming = reinterpret_cast<error_t (*)(handle_t, int, int)>(dlsym(methodsHandle, "ish_start_streaming"));
                if (methods.psh_start_streaming == NULL) {
                        ALOGE("dlsym: ish_start_streaming error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_start_streaming_with_flag == NULL) {
                methods.psh_start_streaming_with_flag = reinterpret_cast<error_t (*)(handle_t, int, int, streaming_flag)>(dlsym(methodsHandle, "ish_start_streaming_with_flag"));
                if (methods.psh_start_streaming_with_flag == NULL) {
                        ALOGE("dlsym: ish_start_streaming_with_flag error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_stop_streaming == NULL) {
                methods.psh_stop_streaming = reinterpret_cast<error_t (*)(handle_t)>(dlsym(methodsHandle, "ish_stop_streaming"));
                if (methods.psh_stop_streaming == NULL) {
                        ALOGE("dlsym: ish_stop_streaming error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.psh_set_property == NULL) {
                methods.psh_set_property = reinterpret_cast<error_t (*)(handle_t, property_type, void *)>(dlsym(methodsHandle, "ish_set_property"));
                if (methods.psh_set_property == NULL) {
                        ALOGE("dlsym: ish_set_property error!");
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
                        ALOGE("dlclose error! %d", err);
                        return false;
                }
        }
        return true;
}
