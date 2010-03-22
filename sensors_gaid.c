/*
 * Copyright (C) 2009 The Android Open Source Project
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

/* Contains changes by Wind River Systems, 2009-2010 */

/* this implements a sensors hardware library for the Android emulator.
 * the following code should be built as a shared library that will be
 * placed into /system/lib/hw/sensors.goldfish.so
 *
 * it will be loaded by the code in hardware/libhardware/hardware.c
 * which is itself called from com_android_server_SensorService.cpp
 */

#define LOG_TAG "GAID_Sensors"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <cutils/log.h>
#include <cutils/native_handle.h>
#include <hardware/sensors.h>

#include "sensors_gaid.h"

static int timeout = 100000;

static struct {
    int handle;
    int fd;
    sensors_ops_t* ops;
} _sensorIds[] =
{
    GAID_SENSORS_DATA
};

#define MAX_NUM_SENSORS ((int)(sizeof(_sensorIds)/sizeof(_sensorIds[0])))

/** SENSORS CONTROL DEVICE
 **
 ** This one is used to send commands to the sensors drivers.
 ** We implement this by sending directly commands to the emulator
 ** through the QEMUD channel.
 **/

typedef struct SensorControl {
    struct sensors_control_device_t  device;
    uint32_t                         active_sensors;
} SensorControl;


/* this must return a file descriptor that will be used to read
 * the sensors data (it is passed to data_open() below
 */
static native_handle_t* control_open_data_source(
                        struct sensors_control_device_t *dev)
{
    native_handle_t *handle;
    D("%s: dev=%p", __FUNCTION__, dev);
    handle = native_handle_create(0 ,0);
	return handle;
}

static void control_close_data_source(struct sensors_control_device_t *dev)
{
    D("%s: dev=%p", __FUNCTION__, dev);
	return ;
}

static int
control_activate(struct sensors_control_device_t *dev,
                  int handle,
                  int enabled)
{
    D("%s: dev=%p", __FUNCTION__, dev);
    return 0;
}

static int control_set_delay(struct sensors_control_device_t *dev
                           , int32_t ms)
{
    SensorControl*  ctl = (void*)dev;

	timeout = 1000 * ms;

    D("%s: dev=%p delay-ms=%d", __FUNCTION__, dev, ms);
	/* ToDo : */
    return 0;
}

/* this function is used to force-stop the blocking read() in
 * data_poll. In order to keep the implementation as simple
 * as possible here, we send a command to the emulator which
 * shall send back an appropriate data block to the system.
 */
static int control_wake(struct sensors_control_device_t *dev)
{
    SensorControl*  ctl = (void*)dev;
    D("%s: dev=%p", __FUNCTION__, dev);
	/* ToDo : */
    return 0;
}


static int control_close(struct hw_device_t *dev)
{
    D("%s: dev=%p", __FUNCTION__, dev);
    return 0;
}


/** SENSORS DATA DEVICE
 **
 ** This one is used to read sensor data from the hardware.
 ** We implement this by simply reading the data from the
 ** emulator through the QEMUD channel.
 **/

typedef struct SensorData {
    struct sensors_data_device_t  device;
    sensors_data_t                sensors[MAX_NUM_SENSORS];
    int                           max_fd;
    unsigned int                  pending_mask;
} SensorData;

static int data_open(struct sensors_data_device_t *dev, native_handle_t *handle)
{
    SensorData*  data = (void*)dev;
    int i,ret;

    D("%s: dev=%p handle=%p", __FUNCTION__, dev, handle);
    memset(&data->sensors, 0, sizeof(data->sensors));
    data->pending_mask = 0;

    native_handle_delete(handle);

    data->max_fd = 0;
    for(i=0;i<MAX_NUM_SENSORS;i++) {
        ret = _sensorIds[i].ops->sensor_data_open();
        if(ret<0) {
            _sensorIds[i].fd = -1;
        } else {
            _sensorIds[i].fd = ret;
            data->max_fd = data->max_fd > ret ? data->max_fd : ret;
        }
    }
    return 0;
}

static int data_common_close(struct hw_device_t *dev)
{
    int i;
    SensorData* data = (SensorData*)dev;
    D("%s: dev=%p", __FUNCTION__, dev);
    if(!data) return 0;

    for(i=0;i<MAX_NUM_SENSORS;i++) {
        if(_sensorIds[i].fd < 0) continue;
        _sensorIds[i].ops->sensor_data_close();
    }

	free(data);
    return 0;
}

static int data_close(struct sensors_data_device_t *dev)
{
    int i;
    SensorData* data = (SensorData*)dev;
    D("%s: dev=%p", __FUNCTION__, dev);
    if(!data) return 0;

    for(i=0;i<MAX_NUM_SENSORS;i++) {
        if(_sensorIds[i].fd < 0) continue;
        _sensorIds[i].ops->sensor_data_close();
    }

    return 0;
}


static int fill_data(SensorData* data,sensors_data_t* values)
{
    int i;

    D("%s: ", __FUNCTION__);

    for(i=0;i<MAX_NUM_SENSORS;i++) {
        if(_sensorIds[i].fd<0) continue;

        if(data->pending_mask & (1<<i)) {
            memcpy(values, &data->sensors[i],sizeof(sensors_data_t));
            data->pending_mask &= ~(1<<i);
            return _sensorIds[i].ops->sensor_list.handle;
        }
    }

    return -1;
}

static int data_poll(struct sensors_data_device_t *dev
                   , sensors_data_t* values)
{
    fd_set fds;
    struct timeval t;
    int ret;
    SensorData*  data = (void*)dev;
    int i;

    D("%s: dev=%p", __FUNCTION__, dev);

    if(data->pending_mask) {
         return fill_data(data,values);
    }

    FD_ZERO(&fds);

    for(i=0;i<MAX_NUM_SENSORS;i++) {
        if(_sensorIds[i].fd < 0) continue;
        _sensorIds[i].ops->sensor_set_fd(&fds);
    }

    t.tv_sec = timeout/1000000;
    t.tv_usec = timeout%1000000;

    ret = select(data->max_fd+1,&fds,NULL,NULL,&t);
    if(ret < 0) {
        D("%s: select error %d", __FUNCTION__, ret);
    } else if(ret == 0) { /* timeout */
    } else /* if(ret > 0) */ {
         for(i=0;i<MAX_NUM_SENSORS;i++) {
             if(_sensorIds[i].fd < 0) continue;
             if(_sensorIds[i].ops->sensor_is_fd(&fds)) {
                 ret = _sensorIds[i].ops->sensor_read(&data->sensors[i]);
                 if(ret < 0) {
                     E("%s: read sensor error %d", __FUNCTION__, ret);
                     continue;
                 }
                 data->pending_mask |= (1<<i);
             }
         }
         usleep(t.tv_usec+1000000*t.tv_sec);
         return fill_data(data,values);
    }

    usleep(t.tv_usec+1000000*t.tv_sec);
    return 0;
}


/** MODULE REGISTRATION SUPPORT
 **
 ** This is required so that hardware/libhardware/hardware.c
 ** will dlopen() this library appropriately.
 **/

/*
 * the following is the list of all supported sensors.
 * this table is used to build sSensorList declared below
 * according to which hardware sensors are reported as
 * available from the emulator (see get_sensors_list below)
 *
 * note: numerical values for maxRange/resolution/power were
 *       taken from the reference AK8976A implementation
 */
static struct sensor_t sSensorListInit[MAX_NUM_SENSORS];
static int sensors_get_sensors_list(struct sensors_module_t* module,
        struct sensor_t const** list)
{
    int i;

    for(i=0;i<MAX_NUM_SENSORS;i++) {
        memcpy(&sSensorListInit[i],&(_sensorIds[i].ops->sensor_list)
             , sizeof(struct sensor_t));
    }

    *list = sSensorListInit;
    return MAX_NUM_SENSORS;
}


static int open_sensors(const struct hw_module_t* module,
             const char*               name,
             struct hw_device_t*      *device)
{
    int  status = -EINVAL;

    D("%s: name=%s", __FUNCTION__, name);

    if (!strcmp(name, SENSORS_HARDWARE_CONTROL))
    {
        SensorControl *dev = malloc(sizeof(*dev));

        memset(dev, 0, sizeof(*dev));

        dev->device.common.tag       = HARDWARE_DEVICE_TAG;
        dev->device.common.version   = 0;
        dev->device.common.module    = (struct hw_module_t*) module;
        dev->device.common.close     = control_close;
        dev->device.open_data_source = control_open_data_source;
        dev->device.activate         = control_activate;
        dev->device.set_delay        = control_set_delay;
        dev->device.wake             = control_wake;

        *device = &dev->device.common;
        status  = 0;
    }
    else if (!strcmp(name, SENSORS_HARDWARE_DATA)) {
        SensorData *dev = malloc(sizeof(*dev));

        memset(dev, 0, sizeof(*dev));

        dev->device.common.tag     = HARDWARE_DEVICE_TAG;
        dev->device.common.version = 0;
        dev->device.common.module  = (struct hw_module_t*) module;
        dev->device.common.close   = data_common_close;
        dev->device.data_open      = data_open;
        dev->device.data_close     = data_close;
        dev->device.poll           = data_poll;

        *device = &dev->device.common;
        status  = 0;
    }
    return status;
}

static struct hw_module_methods_t sensors_module_methods = {
    .open = open_sensors
};

const struct sensors_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .version_major = 1,
        .version_minor = 0,
        .id = SENSORS_HARDWARE_MODULE_ID,
        .name = "GAID SENSORS Module",
        .author = "The Android Open Source Project",
        .methods = &sensors_module_methods,
    },
  .get_sensors_list = sensors_get_sensors_list
};


