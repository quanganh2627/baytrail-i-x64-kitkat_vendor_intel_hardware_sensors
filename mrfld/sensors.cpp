/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "Sensors"

#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>

#include <linux/input.h>

#include <utils/Atomic.h>
#include <utils/Log.h>

#include "sensors.h"
#include "LightSensor.h"
#include "ProximitySensor.h"
#include "AccelSensor.h"
#include "GyroSensor.h"
#include "MagneticSensor.h"
#include "PressureSensor.h"
#include "LinearAccelSensor.h"
#include "GravitySensor.h"
#include "RotationVectorSensor.h"
#include "OrientationSensor.h"

/* The SENSORS Module */
static const struct sensor_t sSensorList[] = {
        { "PSH Accelerometer",
          "Intel Inc.",
          1, SENSORS_HANDLE_ACCELERATION,
          SENSOR_TYPE_ACCELEROMETER, RANGE_A, CONVERT_A, 0.006f, 10000, { } },
        { "PSH Magnetic field sensor",
          "Intel Inc.",
          1, SENSORS_HANDLE_MAGNETIC_FIELD,
          SENSOR_TYPE_MAGNETIC_FIELD, 800.0f, CONVERT_M, 0.1f, 10000, { } },
        { "PSH Ambient Light sensor",
          "Intel Inc.",
          1, SENSORS_HANDLE_LIGHT,
          SENSOR_TYPE_LIGHT, 50000.0f, 1.0f, 0.35f, 0, { } },
        { "PSH Proximity sensor",
          "Intel Inc.",
          1, SENSORS_HANDLE_PROXIMITY,
          SENSOR_TYPE_PROXIMITY, 15.0f, 15.0f, 0.35f, 0, { } },
        { "PSH Gyroscope sensor",
          "Intel Inc.",
          1, SENSORS_HANDLE_GYROSCOPE,
          SENSOR_TYPE_GYROSCOPE, RANGE_GYRO, CONVERT_GYRO, 6.1f, 10000, { } },
        { "PSH Pressure sensor",
          "Intel Inc.",
          1, SENSORS_HANDLE_PRESSURE,
          SENSOR_TYPE_PRESSURE, 120000, 1.0f, 0.001f, 50000, { } },
        { "PSH Gravity sensor",
          "Intel Inc.",
          1, SENSORS_HANDLE_GRAVITY,
          SENSOR_TYPE_GRAVITY,
	  RANGE_A, CONVERT_A, 0.006f, 10000, { }},
	{ "PSH Linear Acceleration sensor",
	  "Intel Inc.",
	  1, SENSORS_HANDLE_LINEAR_ACCELERATION,
	  SENSOR_TYPE_LINEAR_ACCELERATION,
	  RANGE_A, CONVERT_A, 0.006f, 10000, { }},
	{ "PSH Rotation Vector sensor",
	  "Intel Inc.",
	  1, SENSORS_HANDLE_ROTATION_VECTOR,
	  SENSOR_TYPE_ROTATION_VECTOR,
	  1, // Max-range, copy from android's virtual sensor.
	  1.0f/ (1<<24), //Resolution, copy from android's virtual sensor.
	  0.106f, //Power, acc.power + mag.power, too bad to hardcode it.
	  10000,{}}, // The same as accelerometer
	{ "PSH Orientation sensor",
	  "Intel Inc.",
	  1, SENSORS_HANDLE_ORIENTATION,
	  SENSOR_TYPE_ORIENTATION,
	  360, //Max-range, copy from android's virtual sensor.
	  1.0f, //Resolution, copy from android's virtual sensor.
	// Below is the same as Rotation vector sensor.
	  0.106f,
	  10000,{}},
};


static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device);


static int sensors__get_sensors_list(struct sensors_module_t* module,
                                     struct sensor_t const** list)
{
        *list = sSensorList;
        return ARRAY_SIZE(sSensorList);
}

static struct hw_module_methods_t sensors_module_methods = {
        open: open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
        common: {
                tag: HARDWARE_MODULE_TAG,
                version_major: 1,
                version_minor: 0,
                id: SENSORS_HARDWARE_MODULE_ID,
                name: "Sensor module for libsensorhub",
                author: "Borqs Electronic Company, Intel Asia&Pacific R&D Ltd.",
                methods: &sensors_module_methods,
        },
        get_sensors_list: sensors__get_sensors_list,
};

struct sensors_poll_context_t {
    struct sensors_poll_device_t device; // must be first

        sensors_poll_context_t();
        ~sensors_poll_context_t();
    int activate(int handle, int enabled);
    int setDelay(int handle, int64_t ns);
    int pollEvents(sensors_event_t* data, int count);

private:
    size_t wake;
    static const char WAKE_MESSAGE = 'W';
    struct pollfd mPollFds[SENSORS_HANDLE_MAX + 1];
    int mWritePipeFd;
    int mNumSensors;
    SensorBase *mSensors[SENSORS_HANDLE_MAX + 1]; // reserved 0 for SENSOR_HANDLE_BASE
};

/*****************************************************************************/

sensors_poll_context_t::sensors_poll_context_t()
{
    wake = mNumSensors = ARRAY_SIZE(sSensorList);
    for (int i = 0; i < mNumSensors; i++) {
	    int handle = sSensorList[i].handle;
	    switch (handle) {
		    case SENSORS_HANDLE_LIGHT:
			    mSensors[handle] = new LightSensor();
			    break;
		    case SENSORS_HANDLE_PROXIMITY:
			    mSensors[handle] = new ProximitySensor();
			    break;
		    case SENSORS_HANDLE_ACCELERATION:
			    mSensors[handle] = new AccelSensor();
			    break;
		    case SENSORS_HANDLE_MAGNETIC_FIELD:
			    mSensors[handle] = new MagneticSensor();
			    break;
		    case SENSORS_HANDLE_GYROSCOPE:
			    mSensors[handle] = new GyroSensor();
			    break;
		    case SENSORS_HANDLE_PRESSURE:
			    mSensors[handle] = new PressureSensor();
			    break;
		    case SENSORS_HANDLE_GRAVITY:
			    mSensors[handle] = new GravitySensor();
			    break;
	            case SENSORS_HANDLE_LINEAR_ACCELERATION:
			    mSensors[handle] = new LinearAccelSensor();
			    break;
		    case SENSORS_HANDLE_ROTATION_VECTOR:
			    mSensors[handle] = new RotationVectorSensor();
			    break;
		    case SENSORS_HANDLE_ORIENTATION:
			    mSensors[handle] = new OrientationSensor();
			    break;
		    default:
			    LOGE("No Sensor id handle %d found\n", handle);
			    continue;
	    }
	    mPollFds[i].fd = mSensors[handle]->getFd();
	    mPollFds[i].events = POLLIN;
	    mPollFds[i].revents = 0;
    }
    int  wakeFds[2];
    int result = pipe(wakeFds);
    LOGE_IF(result<0, "error creating wake pipe (%s)", strerror(errno));
    fcntl(wakeFds[0], F_SETFL, O_NONBLOCK);
    fcntl(wakeFds[1], F_SETFL, O_NONBLOCK);
    mWritePipeFd = wakeFds[1];

    mPollFds[wake].fd = wakeFds[0];
    mPollFds[wake].events = POLLIN;
    mPollFds[wake].revents = 0;
}

sensors_poll_context_t::~sensors_poll_context_t() {
    for (int i=0 ; i<mNumSensors ; i++) {
        delete mSensors[sSensorList[i].handle];
    }
    close(mPollFds[wake].fd);
    close(mWritePipeFd);
}

int sensors_poll_context_t::activate(int handle, int enabled) {
   D("sensors_poll_context_t::activate, handle = %d, enabled = %d",
      handle, enabled);

   if (handle <= SENSORS_HANDLE_BASE || handle > SENSORS_HANDLE_MAX)
	   return (handle > 0 ? -handle : handle);

   int err =  mSensors[handle]->enable(handle, enabled);
    if (enabled && !err) {
        const char wakeMessage(WAKE_MESSAGE);
        int result = write(mWritePipeFd, &wakeMessage, 1);
        LOGE_IF(result<0, "error sending wake message (%s)", strerror(errno));
    }
    return err;
}

int sensors_poll_context_t::setDelay(int handle, int64_t ns) {
    if (handle <= SENSORS_HANDLE_BASE || handle > SENSORS_HANDLE_MAX)
	    return (handle > 0 ? -handle : handle);

    return mSensors[handle]->setDelay(handle, ns);
}

int sensors_poll_context_t::pollEvents(sensors_event_t* data, int count)
{
    int nbEvents = 0;
    int n = 0;

    do {
        // see if we have some leftover from the last poll()
        for (int i=0 ; count && i<mNumSensors ; i++) {
            SensorBase* const sensor(mSensors[sSensorList[i].handle]);
            if ((mPollFds[i].revents & POLLIN)
                   || (sensor->hasPendingEvents())) {
                int nb = sensor->readEvents(data, count);
                if (nb < count) {
                    // no more data for this sensor
                    mPollFds[i].revents = 0;
                }
                count -= nb;
                nbEvents += nb;
                data += nb;
            }
        }
        if (count) {
            // we still have some room, so try to see if we can get
            // some events immediately or just wait if we don't have
            // anything to return
            n = poll(mPollFds, mNumSensors + 1, nbEvents ? 0 : -1);
            if (n<0) {
                LOGE("poll() failed (%s)", strerror(errno));
                return -errno;
            }
            if (mPollFds[wake].revents & POLLIN) {
                char msg;
                int result = read(mPollFds[wake].fd, &msg, 1);
                LOGE_IF(result<0, "error reading from wake pipe (%s)",
                                                     strerror(errno));
                LOGE_IF(msg != WAKE_MESSAGE,
                        "unknown message on wake queue (0x%02x)", int(msg));
                mPollFds[wake].revents = 0;
            }
        }
        // if we have events and space, go read them
    } while (n && count);
    D("sensors_poll_context_t::pollEvents(), return: nbEvents = %d", nbEvents);
    return nbEvents;
}

/*****************************************************************************/

static int poll__close(struct hw_device_t *dev)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    if (ctx) {
        delete ctx;
    }
    return 0;
}

static int poll__activate(struct sensors_poll_device_t *dev,
        int handle, int enabled) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->activate(handle, enabled);
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
        int handle, int64_t ns) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->setDelay(handle, ns);
}

static int poll__poll(struct sensors_poll_device_t *dev,
        sensors_event_t* data, int count) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->pollEvents(data, count);
}

/*****************************************************************************/

/** Open a new instance of a sensor device using name */
static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device)
{
        int status = -EINVAL;
        sensors_poll_context_t *dev = new sensors_poll_context_t();

        memset(&dev->device, 0, sizeof(sensors_poll_device_t));

        dev->device.common.tag = HARDWARE_DEVICE_TAG;
        dev->device.common.version  = 0;
        dev->device.common.module   = const_cast<hw_module_t*>(module);
        dev->device.common.close    = poll__close;
        dev->device.activate        = poll__activate;
        dev->device.setDelay        = poll__setDelay;
        dev->device.poll            = poll__poll;

        *device = &dev->device.common;
        status = 0;

        return status;
}

