#ifndef _PSH_COMMON_SENSOR_HPP_
#define _PSH_COMMON_SENSOR_HPP_

#include "PSHSensor.hpp"

class PSHCommonSensor : public PSHSensor {
        struct sensorhub_event_t sensorhubEvent[32];

public:
        PSHCommonSensor(SensorDevice &mDevice) :PSHSensor(mDevice)
        {
                memset(sensorhubEvent, 0, 32 * sizeof(struct sensorhub_event_t));
        }
        ~PSHCommonSensor()
        {
                if (sensorHandle != NULL)
                        methods.psh_close_session(sensorHandle);
        }
        int getPollfd();
        int activate(int handle, int enabled);
        int setDelay(int handle, int64_t ns);
        int getData(std::queue<sensors_event_t> &eventQue);
        bool selftest();
};

#endif
