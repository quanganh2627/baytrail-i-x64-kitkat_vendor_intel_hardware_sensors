#include "ISHCommonSensor.hpp"
#include "utils.hpp"

int ISHCommonSensor::getPollfd()
{
        if (pollfd >= 0)
                return pollfd;

        if (methods.ish_open_session == NULL || methods.ish_get_fd == NULL) {
                log_message(CRITICAL, "ish_open_session/ish_get_fd not initialized!");
                return -1;
        }

        ish_sensor_t ISHType = SensorHubHelper::getType(device.getType(), device.getSubname());
        sensorHandle = methods.ish_open_session(ISHType);
        if (sensorHandle == NULL) {
                log_message(CRITICAL, "ish_open_session error!");
                return -1;
        }

        pollfd = methods.ish_get_fd(sensorHandle);

        return pollfd;
}

int ISHCommonSensor::calcDataRate(int64_t period_ns)
{
        int dataRate = 5;
        int delay = 200;
        int minDelay = device.getMinDelay() / US_TO_MS;
        int maxDelay = device.getMaxDelay() / US_TO_MS;

        if (period_ns / 1000 != SENSOR_NOPOLL)
                delay = period_ns / NS_TO_MS;
        else
                delay = 200;

        if (delay < minDelay)
                delay = minDelay;

        if (delay > maxDelay)
                delay = maxDelay;

        if (delay != 0)
                dataRate = 1000 / delay;

        if (dataRate <= 0)
                dataRate = 1;

        return dataRate;
}

int ISHCommonSensor::hardwareSet(bool activated)
{
        int dataRate = state.getDataRate();
        int ret = 0;

        if (!activated) {
                state.setActivated(activated);
                error_t err = methods.ish_stop_streaming(sensorHandle);
                if (err != ERROR_NONE) {
                        log_message(CRITICAL, "ish_stop_streaming error %d", err);
                        return -1;
                }
        } else {
                if (device.getFlags() & SENSOR_FLAG_WAKE_UP)
                        flag = NO_STOP_WHEN_SCREEN_OFF;
                /* Some ISH session need to set different rate and delay */
                SensorHubHelper::getStartStreamingParameters(device.getType(), dataRate, bufferDelay, flag);

                if (dataRate < 0) {
                        log_message(CRITICAL, "Invalid data rate: %d", dataRate);
                        return -1;
                }
                if (!state.getActivated()) {
                        /* Some ISH session need to set property */
                        if (!SensorHubHelper::setISHPropertyIfNeeded(device.getType(), methods, sensorHandle)) {
                                log_message(CRITICAL, "Set property failed for sensor type %d ret: %d", device.getType(), ret);
                                return -1;
                        }
                }

                error_t err;
                err = methods.ish_start_streaming_with_flag(sensorHandle, dataRate, bufferDelay, flag);
                if (err != ERROR_NONE) {
                        log_message(CRITICAL, "ish_start_streaming(_with_flag) error %d name:%s handle: %x %d %d",
                             err, device.getName(), sensorHandle, dataRate, flag);
                        return -1;
                }

                state.setActivated(activated);
                state.setDataRate(dataRate);
        }

        return 0;
}

int ISHCommonSensor::batch(int handle, int flags, int64_t period_ns, int64_t timeout) {
        int delay = period_ns / NS_TO_MS;
        int ret = 0;
        static int oldDataRate = -1;
        static int oldBufferDelay = -1;
        static streaming_flag oldFlag;

        UNUSED(flags);

        if (handle != device.getHandle()) {
                log_message(CRITICAL, "%s: line: %d: %s handle not match! handle: %d required handle: %d",
                     __FUNCTION__, __LINE__, device.getName(), device.getHandle(), handle);
                return -EINVAL;
        }

        if (period_ns < 0 || timeout < 0)
                return -EINVAL;
        pthread_mutex_lock(&lock);
        bufferDelay = timeout / NS_TO_MS;

        state.setDataRate(calcDataRate(period_ns));

        if (oldDataRate == -1) {
                oldDataRate = state.getDataRate();
                oldBufferDelay = bufferDelay;
                oldFlag = flag;
                goto err_batch;
        }

        if (oldDataRate != state.getDataRate() || oldBufferDelay != bufferDelay || oldFlag != flag) {
                oldDataRate = state.getDataRate();
                oldBufferDelay = bufferDelay;
                oldFlag = flag;
                if (state.getActivated())
                        ret = hardwareSet(true);
        }
err_batch:
        pthread_mutex_unlock(&lock);
        return ret;
}

int ISHCommonSensor::activate(int handle, int enabled) {
        int ret = 0;

        UNUSED(handle);

        if (methods.ish_start_streaming == NULL || methods.ish_stop_streaming == NULL || sensorHandle == NULL) {
                log_message(CRITICAL, "ish_start_streaming/ish_stop_streaming/sensorHandle not initialized!");
                return -1;
        }

        pthread_mutex_lock(&lock);
        if(enabled == 0) {
                if (state.getActivated())
                        ret = hardwareSet(false);
        } else if (!state.getActivated())
                ret = hardwareSet(true);
        pthread_mutex_unlock(&lock);

        return ret;
}

int ISHCommonSensor::setDelay(int handle, int64_t period_ns) {
        int dataRate = calcDataRate(period_ns);
        int ret = 0;

        UNUSED(handle);

        pthread_mutex_lock(&lock);
        if (state.getActivated() && dataRate != state.getDataRate()) {
                state.setDataRate(dataRate);
                ret = hardwareSet(true);
        } else
                state.setDataRate(dataRate);
        pthread_mutex_unlock(&lock);

        return ret;
}

int ISHCommonSensor::getData(std::queue<sensors_event_t> &eventQue) {
        int count = 32;

        if (state.getFlushSuccess() == true) {
                eventQue.push(metaEvent);
                state.setFlushSuccess(false);
        }

        count = SensorHubHelper::readSensorhubEvents(pollfd, sensorhubEvent, count, device.getType());
        for (int i = 0; i < count; i++) {
                if (sensorhubEvent[i].timestamp == ISH_BATCH_MODE_FLUSH_DONE) {
                        /*   sensors_meta_data_event_t.version must be META_DATA_VERSION
                         *   sensors_meta_data_event_t.sensor must be 0
                         *   sensors_meta_data_event_t.type must be SENSOR_TYPE_META_DATA
                         *   sensors_meta_data_event_t.reserved must be 0
                         *   sensors_meta_data_event_t.timestamp must be 0
                         */
                        eventQue.push(metaEvent);
                        state.setFlushSuccess(false);
                } else {
                        if (device.getType() == SENSOR_TYPE_STEP_COUNTER)
                                event.u64.step_counter = sensorhubEvent[i].step_counter;
                        else {
                                for (int j = 0; j < AXIS_MAX; j++)
                                        event.data[j] = sensorhubEvent[i].data[j] * device.getScale(j);

                                if (sensorhubEvent[i].accuracy != 0)
                                        event.acceleration.status = sensorhubEvent[i].accuracy;
                        }
                        event.timestamp = sensorhubEvent[i].timestamp * 125000;

                        /* auto disable one-shot sensor */
                        if ((device.getFlags() & ~SENSOR_FLAG_WAKE_UP) == SENSOR_FLAG_ONE_SHOT_MODE)
                                activate(device.getHandle(), 0);
                        eventQue.push(event);
                }
        }

        return 0;
}

bool ISHCommonSensor::selftest() {
        if (getPollfd() >= 0)
                return true;
        return false;
}
