#ifndef _ISH_COMMON_SENSOR_HPP_
#define _ISH_COMMON_SENSOR_HPP_

#include "ISHSensor.hpp"
#include <pthread.h>

class ISHCommonSensor : public ISHSensor {
protected:
        struct sensorhub_event_t sensorhubEvent[32];
        int bufferDelay;
        streaming_flag flag;
        pthread_mutex_t lock;
public:
        ISHCommonSensor(SensorDevice &mDevice) :ISHSensor(mDevice)
        {
                memset(sensorhubEvent, 0, 32 * sizeof(struct sensorhub_event_t));
                bufferDelay = 0;
                flag = STOP_WHEN_SCREEN_OFF;
                pthread_mutex_init(&lock, NULL);
        }
        virtual ~ISHCommonSensor()
        {
                if (sensorHandle != NULL)
                        methods.ish_close_session(sensorHandle);
                pthread_mutex_destroy(&lock);
        }
        int getPollfd();
        int activate(int handle, int enabled);
        int setDelay(int handle, int64_t ns);
        virtual int getData(std::queue<sensors_event_t> &eventQue);
        int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
        virtual bool selftest();
        int calcDataRate(int64_t period_ns);
        int hardwareSet(bool activated);
};

#endif
