#ifndef _SENSOR_HUB_HELPER_
#define _SENSOR_HUB_HELPER_

#include <cstring>
#include <unistd.h>
#include <libsensorhub.h>

typedef unsigned char byte;

struct sensorhub_event_t {
        int data[6];
        int accuracy;
        int64_t timestamp;
};

class SensorHubHelper {
        struct sensorhub_event_t sensorhubEvent;
        static size_t getUnitSize(int sensorType);
public:
        static ish_sensor_t getType(int sensorType, sensors_subname subname);
        static ssize_t readSensorhubEvents(int fd, struct sensorhub_event_t* event, size_t count, int sensorType);
        static float ConvertToFloat(int value, int sensorType);
        static void getStartStreamingParameters(int sensorType, int &dataRate, int &bufferDelay, streaming_flag &flag);
        static bool setPSHPropertyIfNeeded(int sensorType, struct sensor_hub_methods methods, handle_t handler);
        static int getGestureFlickEvent(struct gesture_flick_data data);
        static int getTerminalEvent(struct tc_data data);
        static int getShakeEvent(struct shaking_data data);
        static int getSimpleTappingEvent(struct stap_data data);
        static int getMoveDetectEvent(struct md_data data);
};

#endif
