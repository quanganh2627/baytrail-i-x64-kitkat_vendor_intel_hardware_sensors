#include <iostream>
#include <cstdlib>
#include <cutils/properties.h>
#include "PlatformConfig.hpp"
#include "VirtualSensor.hpp"

PlatformConfig::PlatformConfig()
{
        int ret = get_sensors_list(USE_CASE_HAL, &info, &count);
        if (ret)
                return;

        for (int i = 0; i < count; i++)
                addSensorDevice(info[i]);
}

bool PlatformConfig::addSensorDevice(sensor_info_t info)
{
        float value = 0;
        SensorDevice mSensor;
        int sensorType;
        std::string name;
        const char* sensorStringType;
        uint32_t flags, mode;
        sensors_event_property_t eventProperty;
	unsigned int i;

        mSensor.setId(devices.size());
        mSensor.setHandle(mSensor.idToHandle(mSensor.getId()));
        sensorType = getType(info.sensor_type);
        mSensor.setType(sensorType);
        eventProperty = getEventProperty(sensorType);
        mSensor.setEventProperty(eventProperty);
        sensorStringType = getStringType(sensorType);
        mSensor.setStringType(sensorStringType);
        flags = getFlags(sensorType);
        mSensor.setFlags(flags);

	for (i = 0; i < info.axis_num; i++) {
        	mSensor.setMapper(i, i);
        	mSensor.setScale(i, info.axis_scale[i]);
	}

	name = getName(info.sensor_type);
        mSensor.setName(reinterpret_cast<const char *>(name.c_str()));
        mSensor.setVendor(reinterpret_cast<const char *>(info.vendor));
        mSensor.setVersion(info.version);
        mSensor.setMaxRange(info.max_range);
        mSensor.setResolution(info.resolution);
        mSensor.setPower(info.power);

        mode = flags & REPORTING_MODE_MASK;
        if (mode == SENSOR_FLAG_SPECIAL_REPORTING_MODE || mode == SENSOR_FLAG_ON_CHANGE_MODE) {
                mSensor.setMinDelay(0);
        } else if (mode == SENSOR_FLAG_ONE_SHOT_MODE) {
                mSensor.setMinDelay(-1);
        } else if (info.min_delay < 10000) {
                /* FixMe: current mindelay repoorted by firmware is 2ms, but not real supported by firmware */
                mSensor.setMinDelay(10000);
                log_message(CRITICAL, "setmindelay %d\n", info.min_delay);
	} else {
        	mSensor.setMinDelay(info.min_delay);
                log_message(CRITICAL, "setmindelay %d\n", info.min_delay);
	}

        if (mode == SENSOR_FLAG_SPECIAL_REPORTING_MODE || mode == SENSOR_FLAG_ON_CHANGE_MODE || mode == SENSOR_FLAG_ONE_SHOT_MODE) {
                mSensor.setMaxDelay(0);
        } else {
                mSensor.setMaxDelay(info.max_delay);
        }

        if (mode == SENSOR_FLAG_ONE_SHOT_MODE) {
                mSensor.setFifoMaxEventCount(0);
                mSensor.setFifoReservedEventCount(0);
        } else {
                mSensor.setFifoMaxEventCount(info.fifo_max_event_count);
                mSensor.setFifoReservedEventCount(info.fifo_reserved_event_count);
        }

        if (sensorType == SENSOR_TYPE_HEART_RATE)
                mSensor.setRequiredPermission(SENSOR_PERMISSION_BODY_SENSORS);
        else if (mSensor.getRequiredPermission() == NULL)
                mSensor.setRequiredPermission("");

        switch (sensorType) {
        case SENSOR_TYPE_SIGNIFICANT_MOTION:
        case SENSOR_TYPE_WAKE_GESTURE:
        case SENSOR_TYPE_GLANCE_GESTURE:
        case SENSOR_TYPE_PICK_UP_GESTURE:
        case SENSOR_TYPE_TILT_DETECTOR:
                flags = mode | SENSOR_FLAG_WAKE_UP;
                mSensor.setFlags(flags);
                break;
        default:
                break;
        }

        devices.push_back(mSensor);

        return true;
}

int PlatformConfig::getType(ish_sensor_t sensor_type)
{
        if (sensor_type == SENSOR_ALS)
                return SENSOR_TYPE_LIGHT;
        else if (sensor_type == SENSOR_PROXIMITY)
                return SENSOR_TYPE_PROXIMITY;
        else if (sensor_type == SENSOR_ACCELEROMETER)
                return SENSOR_TYPE_ACCELEROMETER;
        else if (sensor_type == SENSOR_COMP)
                return SENSOR_TYPE_MAGNETIC_FIELD;
        else if (sensor_type == SENSOR_GYRO)
                return SENSOR_TYPE_GYROSCOPE;
        else if (sensor_type == SENSOR_BARO)
                return SENSOR_TYPE_PRESSURE;
        else if (sensor_type == SENSOR_GRAVITY)
                return SENSOR_TYPE_GRAVITY;
        else if (sensor_type == SENSOR_LINEAR_ACCEL)
                return SENSOR_TYPE_LINEAR_ACCELERATION;
        else if (sensor_type == SENSOR_ROTATION_VECTOR)
                return SENSOR_TYPE_ROTATION_VECTOR;
        else if (sensor_type == SENSOR_ORIENTATION)
                return SENSOR_TYPE_ORIENTATION;
        else if (sensor_type == SENSOR_STEPDETECTOR)
                return SENSOR_TYPE_STEP_DETECTOR;
        else if (sensor_type == SENSOR_STEPCOUNTER)
                return SENSOR_TYPE_STEP_COUNTER;
        else if (sensor_type == SENSOR_SIGNIFICANT_MOTION)
                return SENSOR_TYPE_SIGNIFICANT_MOTION;
        else if (sensor_type == SENSOR_GESTURE_FLICK)
                return SENSOR_TYPE_GESTURE_FLICK;
        else if (sensor_type == SENSOR_TC)
                return SENSOR_TYPE_TERMINAL;
        else if (sensor_type == SENSOR_SHAKING)
                return SENSOR_TYPE_SHAKE;
        else if (sensor_type == SENSOR_MOVE_DETECT)
                return SENSOR_TYPE_MOVE_DETECT;
        else if (sensor_type == SENSOR_PEDOMETER)
                return SENSOR_TYPE_PEDOMETER;
	else if (sensor_type == SENSOR_MOTION_DETECT)
                return SENSOR_TYPE_MOTION_DETECT;
	else if (sensor_type == SENSOR_UNCAL_GYRO)
		return SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
	else if (sensor_type == SENSOR_UNCAL_ACC)
		return SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
        else if (sensor_type == SENSOR_GAME_ROTATION_VECTOR)
                return SENSOR_TYPE_GAME_ROTATION_VECTOR;
	else if (sensor_type == SENSOR_UNCAL_COMP)
		return SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
        else if (sensor_type == SENSOR_GEOMAGNETIC_ROTATION_VECTOR)
                return SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR;
        else if (sensor_type == SENSOR_STAP)
                return SENSOR_TYPE_SIMPLE_TAPPING;
        else if (sensor_type == SENSOR_PAN_TILT_ZOOM)
                return SENSOR_TYPE_PAN_ZOOM;
        else if (sensor_type == SENSOR_LIFT)
                return SENSOR_TYPE_LIFT;
        else if (sensor_type == SENSOR_INSTANT_ACTIVITY)
                return SENSOR_TYPE_INSTANT_ACTIVITY;

        log_message(CRITICAL, "%s: unsupported sensor: %d\n", __FUNCTION__, sensor_type);
        return 0;
}

std::string PlatformConfig::getName(ish_sensor_t sensor_type)
{
        if (sensor_type == SENSOR_ALS)
                return "Light sensor";
        else if (sensor_type == SENSOR_PROXIMITY)
                return "Proximity sensor";
        else if (sensor_type == SENSOR_ACCELEROMETER)
                return "Accelerometer sensor";
        else if (sensor_type == SENSOR_COMP)
                return "Magnetometer sensor";
        else if (sensor_type == SENSOR_GYRO)
                return "Gyroscope sensor";
        else if (sensor_type == SENSOR_BARO)
                return "Barometer sensor";
        else if (sensor_type == SENSOR_GRAVITY)
                return "Gravity sensor";
        else if (sensor_type == SENSOR_LINEAR_ACCEL)
                return "Linear Acceleration sensor";
        else if (sensor_type == SENSOR_ROTATION_VECTOR)
                return "Rotation vector sensor";
        else if (sensor_type == SENSOR_ORIENTATION)
                return "Orientation sensor";
        else if (sensor_type == SENSOR_STEPDETECTOR)
                return "Step detecotr sensor";
        else if (sensor_type == SENSOR_STEPCOUNTER)
                return "Step counter sensor";
        else if (sensor_type == SENSOR_SIGNIFICANT_MOTION)
                return "Significant motion sensor";
        else if (sensor_type == SENSOR_GESTURE_FLICK)
                return "Gesture flick sensor";
        else if (sensor_type == SENSOR_TC)
                return "Termial sensor";
        else if (sensor_type == SENSOR_SHAKING)
                return "Shake sensor";
        else if (sensor_type == SENSOR_MOVE_DETECT)
                return "Move detector sensor";
        else if (sensor_type == SENSOR_PEDOMETER)
                return "Pedometer sensor";
        else if (sensor_type == SENSOR_ACTIVITY)
                return "Physical activity sensor";
	else if (sensor_type == SENSOR_MOTION_DETECT)
                return "Motion detector sensor";
	else if (sensor_type == SENSOR_UNCAL_GYRO)
		return "Uncalibrated Gyro sensor";
	else if (sensor_type == SENSOR_UNCAL_ACC)
		return "Uncalibrated Accelerometer sensor";
        else if (sensor_type == SENSOR_GAME_ROTATION_VECTOR)
                return "Game rotation vector sensor";
	else if (sensor_type == SENSOR_UNCAL_COMP)
		return "Uncalibrated Compass sensor";
        else if (sensor_type == SENSOR_GEOMAGNETIC_ROTATION_VECTOR)
                return "Geomagnetic rotation vector sensor";
        else if (sensor_type == SENSOR_STAP)
                return "Tap sensor";
        else if (sensor_type == SENSOR_PAN_TILT_ZOOM)
                return "Pan Zoom sensor";
        else if (sensor_type == SENSOR_LIFT)
                return "Lift sensor";
        else if (sensor_type == SENSOR_INSTANT_ACTIVITY)
                return "Instant Activity sensor";

        log_message(CRITICAL, "%s: unsupported sensor: %d\n", __FUNCTION__, sensor_type);
        return "Unknow sensor";
}

const char* PlatformConfig::getStringType(int sensorType)
{
        switch (sensorType) {
        case SENSOR_TYPE_LIGHT:
                return SENSOR_STRING_TYPE_LIGHT;
        case SENSOR_TYPE_PROXIMITY:
                return SENSOR_STRING_TYPE_PROXIMITY;
        case SENSOR_TYPE_ACCELEROMETER:
                return SENSOR_STRING_TYPE_ACCELEROMETER;
        case SENSOR_TYPE_MAGNETIC_FIELD:
                return SENSOR_STRING_TYPE_MAGNETIC_FIELD;
        case SENSOR_TYPE_GYROSCOPE:
                return SENSOR_STRING_TYPE_GYROSCOPE;
        case SENSOR_TYPE_PRESSURE:
                return SENSOR_STRING_TYPE_PRESSURE;
        case SENSOR_TYPE_TEMPERATURE:
                return SENSOR_STRING_TYPE_TEMPERATURE;
        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
                return SENSOR_STRING_TYPE_AMBIENT_TEMPERATURE;
        case SENSOR_TYPE_RELATIVE_HUMIDITY:
                return SENSOR_STRING_TYPE_RELATIVE_HUMIDITY;
        case SENSOR_TYPE_GRAVITY:
                return SENSOR_STRING_TYPE_GRAVITY;
        case SENSOR_TYPE_INSTANT_ACTIVITY:
                return SENSOR_STRING_TYPE_INSTANT_ACTIVITY;
        case SENSOR_TYPE_LINEAR_ACCELERATION:
                return SENSOR_STRING_TYPE_LINEAR_ACCELERATION;
        case SENSOR_TYPE_ROTATION_VECTOR:
                return SENSOR_STRING_TYPE_ROTATION_VECTOR;
        case SENSOR_TYPE_ORIENTATION:
                return SENSOR_STRING_TYPE_ORIENTATION;
        case SENSOR_TYPE_STEP_DETECTOR:
                return SENSOR_STRING_TYPE_STEP_DETECTOR;
        case SENSOR_TYPE_STEP_COUNTER:
                return SENSOR_STRING_TYPE_STEP_COUNTER;
        case SENSOR_TYPE_SIGNIFICANT_MOTION:
                return SENSOR_STRING_TYPE_SIGNIFICANT_MOTION;
        case SENSOR_TYPE_GESTURE_FLICK:
                return SENSOR_STRING_TYPE_GESTURE_FLICK;
        case SENSOR_TYPE_TERMINAL:
                return SENSOR_STRING_TYPE_TERMINAL;
        case SENSOR_TYPE_SHAKE:
                return SENSOR_STRING_TYPE_SHAKE;
        case SENSOR_TYPE_SIMPLE_TAPPING:
                return SENSOR_STRING_TYPE_SIMPLE_TAPPING;
        case SENSOR_TYPE_MOVE_DETECT:
                return SENSOR_STRING_TYPE_MOVE_DETECT;
        case SENSOR_TYPE_PEDOMETER:
                return SENSOR_STRING_TYPE_PEDOMETER;
        case SENSOR_TYPE_AUDIO_CLASSIFICATION:
                return SENSOR_STRING_TYPE_AUDIO_CLASSIFICATION;
        case SENSOR_TYPE_GAME_ROTATION_VECTOR:
                return SENSOR_STRING_TYPE_GAME_ROTATION_VECTOR;
        case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
                return SENSOR_STRING_TYPE_GEOMAGNETIC_ROTATION_VECTOR;
        case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
                return SENSOR_STRING_TYPE_ACCELEROMETER_UNCALIBRATED;
        case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
                return SENSOR_STRING_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
        case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
                return SENSOR_STRING_TYPE_GYROSCOPE_UNCALIBRATED;
        case SENSOR_TYPE_GESTURE_HMM:
                return SENSOR_STRING_TYPE_GESTURE_HMM;
        case SENSOR_TYPE_GESTURE_EARTOUCH:
                return SENSOR_STRING_TYPE_GESTURE_EARTOUCH;
        case SENSOR_TYPE_GESTURE:
                return SENSOR_STRING_TYPE_GESTURE;
        case SENSOR_TYPE_DEVICE_POSITION:
                return SENSOR_STRING_TYPE_DEVICE_POSITION;
        case SENSOR_TYPE_LIFT:
                return SENSOR_STRING_TYPE_LIFT;
        case SENSOR_TYPE_PAN_ZOOM:
                return SENSOR_STRING_TYPE_PAN_ZOOM;
        case SENSOR_TYPE_HEART_RATE:
                return SENSOR_STRING_TYPE_HEART_RATE;
        case SENSOR_TYPE_TILT_DETECTOR:
                return SENSOR_STRING_TYPE_TILT_DETECTOR;
        case SENSOR_TYPE_WAKE_GESTURE:
                return SENSOR_STRING_TYPE_WAKE_GESTURE;
        case SENSOR_TYPE_GLANCE_GESTURE:
                return SENSOR_STRING_TYPE_GLANCE_GESTURE;
        case SENSOR_TYPE_PICK_UP_GESTURE:
                return SENSOR_STRING_TYPE_PICK_UP_GESTURE;
        default:
                log_message(CRITICAL, "%s: unsupported sensor: %d\n", __FUNCTION__, sensorType);
                break;
        }
        return NULL;
}

uint32_t PlatformConfig::getFlags(int sensorType)
{
        switch (sensorType) {
        case SENSOR_TYPE_LIGHT:
        case SENSOR_TYPE_STEP_COUNTER:
        case SENSOR_TYPE_HEART_RATE:
        case SENSOR_TYPE_TERMINAL:
        case SENSOR_TYPE_MOVE_DETECT:
        case SENSOR_TYPE_DEVICE_POSITION:
        case SENSOR_TYPE_PEDOMETER:
                return SENSOR_FLAG_ON_CHANGE_MODE;
        case SENSOR_TYPE_PROXIMITY:
                return SENSOR_FLAG_ON_CHANGE_MODE | SENSOR_FLAG_WAKE_UP;
        case SENSOR_TYPE_ACCELEROMETER:
        case SENSOR_TYPE_MAGNETIC_FIELD:
        case SENSOR_TYPE_GYROSCOPE:
        case SENSOR_TYPE_PRESSURE:
        case SENSOR_TYPE_TEMPERATURE:
        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
        case SENSOR_TYPE_RELATIVE_HUMIDITY:
        case SENSOR_TYPE_GRAVITY:
        case SENSOR_TYPE_LINEAR_ACCELERATION:
        case SENSOR_TYPE_ROTATION_VECTOR:
        case SENSOR_TYPE_ORIENTATION:
        case SENSOR_TYPE_GAME_ROTATION_VECTOR:
        case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
        case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
        case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
        case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
        case SENSOR_TYPE_CALIBRATION:
        case SENSOR_TYPE_PAN_ZOOM:
        case SENSOR_TYPE_AUDIO_CLASSIFICATION:
                return SENSOR_FLAG_CONTINUOUS_MODE;
        case SENSOR_TYPE_STEP_DETECTOR:
        case SENSOR_TYPE_GESTURE_FLICK:
        case SENSOR_TYPE_SHAKE:
        case SENSOR_TYPE_SIMPLE_TAPPING:
        case SENSOR_TYPE_GESTURE:
        case SENSOR_TYPE_INSTANT_ACTIVITY:
        case SENSOR_TYPE_LIFT:
                return SENSOR_FLAG_SPECIAL_REPORTING_MODE;
        case SENSOR_TYPE_SIGNIFICANT_MOTION:
        case SENSOR_TYPE_WAKE_GESTURE:
        case SENSOR_TYPE_GLANCE_GESTURE:
        case SENSOR_TYPE_PICK_UP_GESTURE:
                return SENSOR_FLAG_ONE_SHOT_MODE | SENSOR_FLAG_WAKE_UP;
        case SENSOR_TYPE_TILT_DETECTOR:
        case SENSOR_TYPE_GESTURE_HMM:
        case SENSOR_TYPE_GESTURE_EARTOUCH:
                return SENSOR_FLAG_SPECIAL_REPORTING_MODE | SENSOR_FLAG_WAKE_UP;
        default:
                log_message(CRITICAL, "%s: unsupported sensor: %d\n", __FUNCTION__, sensorType);
                break;
        }
        return SENSOR_FLAG_CONTINUOUS_MODE;
}

sensors_event_property_t PlatformConfig::getEventProperty(int type)
{
        switch (type) {
        case SENSOR_TYPE_ACCELEROMETER:
        case SENSOR_TYPE_MAGNETIC_FIELD:
        case SENSOR_TYPE_ORIENTATION:
        case SENSOR_TYPE_GYROSCOPE:
                return VECTOR;
        case SENSOR_TYPE_LIGHT:
        case SENSOR_TYPE_PRESSURE:
        case SENSOR_TYPE_TEMPERATURE:
        case SENSOR_TYPE_PROXIMITY:
        case SENSOR_TYPE_RELATIVE_HUMIDITY:
        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
                return SCALAR;
        default:
                return OTHER;
        }
        return OTHER;
}

bool PlatformConfig::getSensorDevice(int id, SensorDevice &device)
{
        if (id < (int)devices.size()) {
                device = devices[id];
                return true;
        }

        return false;
}
