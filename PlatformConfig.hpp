#ifndef _PLATFORM_CONFIG_HPP_
#define _PLATFORM_CONFIG_HPP_
#include <string>
#include <map>
#include <vector>
#include <libsensorhub.h>
#include "SensorDevice.hpp"
#include "utils.hpp"

class PlatformConfig {
        std::vector<SensorDevice> devices;
	sensor_info_t info[SENSOR_MAX];
        int count;
        bool initialized;
        bool addSensorDevice(sensor_info_t info);
        int getType(ish_sensor_t sensor_type);
        std::string getName(ish_sensor_t sensor_type);
	const char* getStringType(int sensorType);
        uint32_t getFlags(int sensorType);
        sensors_event_property_t getEventProperty(int type);
public:
        PlatformConfig();
        unsigned int size() { return devices.size(); }
        bool getSensorDevice(int id, SensorDevice &device);
};
#endif
