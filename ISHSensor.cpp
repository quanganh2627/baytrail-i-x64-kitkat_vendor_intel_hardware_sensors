#include "ISHSensor.hpp"
#include "utils.hpp"
#include <dlfcn.h>

struct sensor_hub_methods ISHSensor::methods;

ISHSensor::ISHSensor()
        :sensorHandle(NULL)
{
        SensorHubMethodsInitialize();
}

ISHSensor::ISHSensor(SensorDevice &mDevice)
        :Sensor(mDevice), sensorHandle(NULL)
{
        SensorHubMethodsInitialize();
}

ISHSensor::~ISHSensor()
{
        SensorHubMethodsFinallize();
}

bool ISHSensor::SensorHubMethodsInitialize()
{
        methodsHandle = dlopen("libsensorhub.so", RTLD_LAZY);
        if (methodsHandle == NULL) {
                log_message(CRITICAL, "dlopen: libsensorhub.so error!");
                return false;
        }

        if (methods.ish_open_session == NULL) {
                methods.ish_open_session = reinterpret_cast<handle_t (*)(ish_sensor_t)>(dlsym(methodsHandle, "ish_open_session"));
                if (methods.ish_open_session == NULL) {
                        log_message(CRITICAL, "dlsym: ish_open_session error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.ish_get_fd == NULL) {
                methods.ish_get_fd = reinterpret_cast<int (*)(handle_t)>(dlsym(methodsHandle, "ish_get_fd"));
                if (methods.ish_get_fd == NULL) {
                        log_message(CRITICAL, "dlsym: ish_get_fd error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.ish_close_session == NULL) {
                methods.ish_close_session = reinterpret_cast<void (*)(handle_t)>(dlsym(methodsHandle, "ish_close_session"));
                if (methods.ish_close_session == NULL) {
                        log_message(CRITICAL, "dlsym: ish_close_session error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.ish_start_streaming == NULL) {
                methods.ish_start_streaming = reinterpret_cast<error_t (*)(handle_t, int, int)>(dlsym(methodsHandle, "ish_start_streaming"));
                if (methods.ish_start_streaming == NULL) {
                        log_message(CRITICAL, "dlsym: ish_start_streaming error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.ish_start_streaming_with_flag == NULL) {
                methods.ish_start_streaming_with_flag = reinterpret_cast<error_t (*)(handle_t, int, int, streaming_flag)>(dlsym(methodsHandle, "ish_start_streaming_with_flag"));
                if (methods.ish_start_streaming_with_flag == NULL) {
                        log_message(CRITICAL, "dlsym: ish_start_streaming_with_flag error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.ish_stop_streaming == NULL) {
                methods.ish_stop_streaming = reinterpret_cast<error_t (*)(handle_t)>(dlsym(methodsHandle, "ish_stop_streaming"));
                if (methods.ish_stop_streaming == NULL) {
                        log_message(CRITICAL, "dlsym: ish_stop_streaming error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.ish_set_property == NULL) {
                methods.ish_set_property = reinterpret_cast<error_t (*)(handle_t, property_type, void *)>(dlsym(methodsHandle, "ish_set_property"));
                if (methods.ish_set_property == NULL) {
                        log_message(CRITICAL, "dlsym: ish_set_property error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.ish_set_property_with_size == NULL) {
                methods.ish_set_property_with_size = reinterpret_cast<error_t (*)(handle_t, property_type, int, void *)>(dlsym(methodsHandle, "ish_set_property_with_size"));
                if (methods.ish_set_property_with_size == NULL) {
                        log_message(CRITICAL, "dlsym: ish_set_property_with_size error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        if (methods.ish_flush_streaming == NULL) {
                methods.ish_flush_streaming = reinterpret_cast<error_t (*)(handle_t, unsigned int)>(dlsym(methodsHandle, "ish_flush_streaming"));
                if (methods.ish_flush_streaming == NULL) {
                        log_message(CRITICAL, "dlsym: ish_flush_streaming error!");
                        SensorHubMethodsFinallize();
                        return false;
                }
        }

        return true;
}

bool ISHSensor::SensorHubMethodsFinallize()
{
        if (methodsHandle != NULL) {
                int err = dlclose(methodsHandle);
                if (err != 0) {
                        log_message(CRITICAL, "dlclose error! %d", err);
                        return false;
                }
        }
        return true;
}

int ISHSensor::flush(int handle)
{
        error_t err;

        if (handle != device.getHandle()) {
                log_message(CRITICAL, "%s: line: %d: %s handle not match! handle: %d required handle: %d",
                     __FUNCTION__, __LINE__, device.getName(), device.getHandle(), handle);
                return -EINVAL;
        }

        if (!state.getActivated()) {
                log_message(CRITICAL, "%s line: %d %s not activated", __FUNCTION__, __LINE__, device.getName());
                return -EINVAL;
        }

        if (methods.ish_flush_streaming == NULL) {
                log_message(CRITICAL, "ish_flush_streaming not initialized!");
                return -EINVAL;
        }

        if ((device.getFlags() & ~SENSOR_FLAG_WAKE_UP) == SENSOR_FLAG_ONE_SHOT_MODE) {
                log_message(CRITICAL, "%s line: %d error: one-shot sensor: %s", __FUNCTION__, __LINE__, device.getName());
                return -EINVAL;
        }

        err = methods.ish_flush_streaming(sensorHandle, SensorHubHelper::getUnitSize(device.getType()));

        return err == ERROR_NONE ? 0 : -EINVAL;
}
