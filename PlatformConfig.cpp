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
        sensors_event_property_t eventProperty;

        mSensor.setId(devices.size());
        mSensor.setHandle(mSensor.idToHandle(mSensor.getId()));
        sensorType = getType(info.sensor_type);
        mSensor.setType(sensorType);
        eventProperty = getEventProperty(sensorType);
        mSensor.setEventProperty(eventProperty);

        mSensor.setMapper(0, AXIS_X);
        mSensor.setScale(0, info.axis_scale[0]);
	if (info.axis_num > 1) {
                 mSensor.setMapper(1, AXIS_Y);
                 mSensor.setScale(1, info.axis_scale[1]);
	}
	if (info.axis_num > 2) {
                 mSensor.setMapper(2, AXIS_Z);
                 mSensor.setScale(2, info.axis_scale[2]);
	}
	if (info.axis_num > 3) {
                 mSensor.setMapper(3, AXIS_W);
                 mSensor.setScale(3, info.axis_scale[3]);
	}

	name = getName(info.sensor_type);
        mSensor.setName(reinterpret_cast<const char *>(name.c_str()));
        mSensor.setVendor(reinterpret_cast<const char *>(info.vendor));
        mSensor.setVersion(info.version);
        mSensor.setMaxRange(info.max_range);
        mSensor.setResolution(info.resolution);
        mSensor.setPower(info.power);
        mSensor.setMinDelay(info.min_delay);

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

        ALOGW("%s: unsupported sensor: %d", __FUNCTION__, sensor_type);
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

        ALOGW("%s: unsupported sensor: %d", __FUNCTION__, sensor_type);
        return "Unknow sensor";
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
        if (id < devices.size()) {
                device = devices[id];
                return true;
        }
        return false;
}
