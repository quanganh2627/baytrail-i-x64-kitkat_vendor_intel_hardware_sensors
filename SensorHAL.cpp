#include <dirent.h>
#include <fstream>
#include <poll.h>
#include <signal.h>
#include <hardware/hardware.h>
#include "ISHCommonSensor.hpp"
#include "PlatformConfig.hpp"
#include "utils.hpp"

static int open(const struct hw_module_t* module, const char* id,
                struct hw_device_t** device);

static struct hw_module_methods_t sensors_module_methods = {
open: open,
};

struct SensorModule {
        struct sensor_t* list;
        std::vector<Sensor*> sensors;
        struct pollfd *pollfds;
        int count;
};

static struct SensorModule mModule;

static int get_sensors_list(struct sensors_module_t* module, struct sensor_t const** list)
{
        UNUSED(module);

        *list = mModule.list;
        return mModule.count;
}

struct sensors_module_t HAL_MODULE_INFO_SYM = {
common: {
        tag: HARDWARE_MODULE_TAG,
        module_api_version: SENSORS_MODULE_API_VERSION_0_1,
        hal_api_version: 0,
        id: SENSORS_HARDWARE_MODULE_ID,
        name: "Intel Sensor module",
        author: "Han, He <he.han@intel.com>, Even, Xu <even.xu@intel.com>, Intel Inc.",
        methods: &sensors_module_methods,
        dso: 0,
        reserved: { 0 },
},
get_sensors_list: get_sensors_list,
};

static bool initSensors()
{
        PlatformConfig mConfig;
        SensorDevice mDevice;
        Sensor* mSensor = NULL;
        unsigned int size;

        size = mConfig.size();
        mModule.sensors.reserve(size);
        int newId = 0;

        for (unsigned int i = 0; i < size; i++) {
                if (!mConfig.getSensorDevice(i, mDevice)) {
                        log_message(CRITICAL,"Sensor Device config error\n");
                        return false;
                }

                switch (mDevice.getType()) {
                        case SENSOR_TYPE_ACCELEROMETER:
                        case SENSOR_TYPE_MAGNETIC_FIELD:
                        case SENSOR_TYPE_GYROSCOPE:
                        case SENSOR_TYPE_PRESSURE:
                        case SENSOR_TYPE_LIGHT:
                        case SENSOR_TYPE_PROXIMITY:
                        case SENSOR_TYPE_TEMPERATURE:
                        case SENSOR_TYPE_RELATIVE_HUMIDITY:
                        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
                        case SENSOR_TYPE_ORIENTATION:
                        case SENSOR_TYPE_GRAVITY:
                        case SENSOR_TYPE_LINEAR_ACCELERATION:
                        case SENSOR_TYPE_ROTATION_VECTOR:
                        case SENSOR_TYPE_GESTURE_FLICK:
                        case SENSOR_TYPE_TERMINAL:
                        case SENSOR_TYPE_SHAKE:
                        case SENSOR_TYPE_SIMPLE_TAPPING:
                        case SENSOR_TYPE_MOVE_DETECT:
                        case SENSOR_TYPE_STEP_DETECTOR:
                        case SENSOR_TYPE_STEP_COUNTER:
                        case SENSOR_TYPE_SIGNIFICANT_MOTION:
                        case SENSOR_TYPE_GAME_ROTATION_VECTOR:
                        case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
			case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
			case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
			case SENSOR_TYPE_MOTION_DETECT:
                        case SENSOR_TYPE_PAN_ZOOM:
                        case SENSOR_TYPE_LIFT:
                        case SENSOR_TYPE_INSTANT_ACTIVITY:
                                mSensor = new ISHCommonSensor(mDevice);
                                break;

                        default:
                                log_message(CRITICAL,"%s Unsupported sensor type: %d\n", __FUNCTION__, mDevice.getType());
                                return false;
                }

                if (mSensor) {
                        if (mSensor->selftest()) {
                                // Need to reset ids and handles, since some unfunctional sensors are removed
                                mSensor->getDevice().setId(newId);
                                mSensor->getDevice().setHandle(SensorDevice::idToHandle(newId));
                                mSensor->resetEventHandle();
                                mModule.sensors.push_back(mSensor);
                                newId++;
                        } else {

                                delete mSensor;
                        }
                        mSensor = NULL;
                }
        }

        mModule.count = mModule.sensors.size();
        mModule.list = new sensor_t[mModule.count];

        for (int i = 0; i < mModule.count; i++) {
                mModule.sensors[i]->getDevice().copyItem(mModule.list + i);
        }

        mModule.pollfds = new struct pollfd[mModule.count];
        for (int i = 0; i < mModule.count; i++) {
                mModule.pollfds[i].fd = mModule.sensors[i]->getPollfd();
                mModule.pollfds[i].events = POLLIN;
                mModule.pollfds[i].revents = 0;
        }
		log_message(DEBUG,"initSensors finished well\n");

        return true;
}

int sensorActivate(struct sensors_poll_device_t *dev, int handle, int enabled)
{
        int id = SensorDevice::handleToId(handle);
        if (id < 0) {
                log_message(CRITICAL,"%s: line:%d Invalid handle: handle: %d; id: %d\n",
                     __FUNCTION__, __LINE__, handle, id);
                return -1;
        }

        UNUSED(dev);

	log_message(DEBUG, "%s: sensor %s enabled %d\n", __FUNCTION__, mModule.sensors[id]->getDevice().getName(), enabled);

        return mModule.sensors[id]->activate(handle, enabled);
}

int sensorSetDelay(struct sensors_poll_device_t *dev, int handle, int64_t ns)
{
        int id = SensorDevice::handleToId(handle);
        if (id < 0) {
                log_message(CRITICAL,"%s: line:%d Invalid handle: handle: %d; id: %d\n",
                     __FUNCTION__, __LINE__, handle, id);
                return -1;
        }

        UNUSED(dev);

	log_message(DEBUG, "%s: sensor %s ns %lld\n", __FUNCTION__, mModule.sensors[id]->getDevice().getName(), ns);

        return mModule.sensors[id]->setDelay(handle, ns);
}

int sensorBatch(struct sensors_poll_device_1* dev,
                int handle, int flags, int64_t period_ns, int64_t timeout)
{
        int id = SensorDevice::handleToId(handle);
        if (id < 0) {
                log_message(CRITICAL, "%s: line:%d Invalid handle: handle: %d; id: %d\n",
                     __FUNCTION__, __LINE__, handle, id);
                return -1;
        }

        UNUSED(dev);

	log_message(DEBUG, "%s: sensor %s ns %lld timeout %lld\n", __FUNCTION__, mModule.sensors[id]->getDevice().getName(), period_ns, timeout);

        return mModule.sensors[id]->batch(handle, flags, period_ns, timeout);
}

int sensorFlush(struct sensors_poll_device_1* dev, int handle)
{
        int id = SensorDevice::handleToId(handle);
        if (id < 0) {
                log_message(CRITICAL, "%s: line:%d Invalid handle: handle: %d; id: %d\n",
                     __FUNCTION__, __LINE__, handle, id);
                return -1;
        }

        UNUSED(dev);

	log_message(DEBUG, "%s: sensor %s\n", __FUNCTION__, mModule.sensors[id]->getDevice().getName());

        return mModule.sensors[id]->flush(handle);
}

int sensorPoll(struct sensors_poll_device_t *dev, sensors_event_t* data, int count)
{
        static std::queue<sensors_event_t> eventQue;
        int eventNum = 0;
        int num, err;
	FILE * file;

        UNUSED(dev);

        while (true) {
                while (eventQue.size() > 0 && eventNum < count) {
                        data[eventNum] = eventQue.front();
                        eventQue.pop();
                        eventNum++;
                }

                if (eventNum > 0)
                        return eventNum;

                num = poll(mModule.pollfds, mModule.count, -1);
                if (num <= 0) {
                        err = errno;
                        log_message(CRITICAL,"%s: line: %d poll error: %d %s\n", __FUNCTION__, __LINE__, err, strerror(err));

                        return -err;
                }

                for (int i = 0; i < mModule.count; i++) {
                        if (mModule.pollfds[i].revents & POLLIN)
                                mModule.sensors[i]->getData(eventQue);
                        else if (mModule.pollfds[i].revents != 0)
                                log_message(CRITICAL,"%s: line: %d poll error: %d fd: %d type: %d\n", __FUNCTION__, __LINE__,
						mModule.pollfds[i].revents, mModule.pollfds[i].fd, mModule.sensors[i]->getDevice().getType());

                        mModule.pollfds[i].revents = 0;
                }
        }

	return -1;
}

int close(struct hw_device_t* device)
{
        UNUSED(device);

        if (mModule.list != NULL)
                delete mModule.list;

        for (unsigned int i = 0; i < mModule.sensors.size(); i++)
                if (mModule.sensors[i])
                        delete mModule.sensors[i];

        if (mModule.pollfds)
                delete [] mModule.pollfds;

        return 0;
}

static int open(const struct hw_module_t* module, const char* id,
                struct hw_device_t** device)
{
        static struct sensors_poll_device_1 dev;

        UNUSED(id);

        dev.common.tag = HARDWARE_DEVICE_TAG;
        dev.common.version = SENSORS_DEVICE_API_VERSION_1_3;
        dev.common.module  = const_cast<hw_module_t*>(module);
        dev.common.close   = close;
        dev.activate       = sensorActivate;
        dev.setDelay       = sensorSetDelay;
        dev.poll           = sensorPoll;
        dev.batch          = sensorBatch;
        dev.flush          = sensorFlush;

        *device = &dev.common;

        if(initSensors())
                return 0;

        return -1;
}
