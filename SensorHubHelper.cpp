#include "PSHSensor.hpp"
#include "SensorConfig.h"

ish_sensor_t SensorHubHelper::getType(int sensorType, sensors_subname subname)
{
	log_message(CRITICAL, "%s sensor type = %d\n", __func__, sensorType);

        switch (sensorType) {
        case SENSOR_TYPE_ACCELEROMETER:
                if (subname == SECONDARY)
                        return SENSOR_ACCELEROMETER_SEC;
                return SENSOR_ACCELEROMETER;
        case SENSOR_TYPE_MAGNETIC_FIELD:
                if (subname == SECONDARY)
                        return SENSOR_COMP_SEC;
                return SENSOR_COMP;
        case SENSOR_TYPE_ORIENTATION:
                return SENSOR_ORIENTATION;
        case SENSOR_TYPE_GYROSCOPE:
                if (subname == SECONDARY)
                        return SENSOR_GYRO_SEC;
                return SENSOR_GYRO;
        case SENSOR_TYPE_LIGHT:
                if (subname == SECONDARY)
                        return SENSOR_ALS_SEC;
                return SENSOR_ALS;
        case SENSOR_TYPE_PRESSURE:
                if (subname == SECONDARY)
                        return SENSOR_BARO_SEC;
                return SENSOR_BARO;
        case SENSOR_TYPE_PROXIMITY:
                if (subname == SECONDARY)
                        return SENSOR_PROXIMITY_SEC;
                return SENSOR_PROXIMITY;
        case SENSOR_TYPE_GRAVITY:
                return SENSOR_GRAVITY;
        case SENSOR_TYPE_LINEAR_ACCELERATION:
                return SENSOR_LINEAR_ACCEL;
        case SENSOR_TYPE_ROTATION_VECTOR:
                return SENSOR_ROTATION_VECTOR;
        case SENSOR_TYPE_GESTURE_FLICK:
                return SENSOR_GESTURE_FLICK;
        case SENSOR_TYPE_TERMINAL:
                return SENSOR_TC;
        case SENSOR_TYPE_SHAKE:
                return SENSOR_SHAKING;
        case SENSOR_TYPE_SIMPLE_TAPPING:
                return SENSOR_STAP;
        case SENSOR_TYPE_MOVE_DETECT:
                return SENSOR_MOVE_DETECT;
        case SENSOR_TYPE_STEP_DETECTOR:
                return SENSOR_STEPDETECTOR;
        case SENSOR_TYPE_STEP_COUNTER:
                return SENSOR_STEPCOUNTER;
        case SENSOR_TYPE_SIGNIFICANT_MOTION:
                return SENSOR_SIGNIFICANT_MOTION;
        case SENSOR_TYPE_GAME_ROTATION_VECTOR:
                return SENSOR_GAME_ROTATION_VECTOR;
        case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
                return SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
	case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
		return SENSOR_UNCAL_COMP;
	case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		return SENSOR_UNCAL_GYRO;
	case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
                return SENSOR_UNCAL_ACC;
	case SENSOR_TYPE_MOTION_DETECT:
		return SENSOR_MOTION_DETECT;
        case SENSOR_TYPE_INSTANT_ACTIVITY:
                return SENSOR_INSTANT_ACTIVITY;
        case SENSOR_TYPE_LIFT:
                return SENSOR_LIFT;
        case SENSOR_TYPE_PAN_ZOOM:
                return SENSOR_PAN_TILT_ZOOM;

        case SENSOR_TYPE_GESTURE:
        case SENSOR_TYPE_PEDOMETER:
        case SENSOR_TYPE_TEMPERATURE:
        case SENSOR_TYPE_RELATIVE_HUMIDITY:
        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
        case SENSOR_TYPE_AUDIO_CLASSIFICATION:
        default:
                log_message(CRITICAL,"%s: Unsupported Sensor Type: %d", __FUNCTION__, sensorType);
                break;
        }
        return SENSOR_INVALID;
}

void SensorHubHelper::getStartStreamingParameters(int sensorType, int &dataRate, int &bufferDelay, streaming_flag &flag)
{
        switch (sensorType) {
        case SENSOR_TYPE_PROXIMITY:
                flag = NO_STOP_WHEN_SCREEN_OFF;
        case SENSOR_TYPE_GESTURE_FLICK:
                dataRate = GF_SAMPLE_RATE;
                bufferDelay = GF_BUF_DELAY;
                break;
        case SENSOR_TYPE_TERMINAL:
                dataRate = TERM_RATE;
                bufferDelay = TERM_DELAY;
                break;
        case SENSOR_TYPE_SHAKE:
                dataRate = SHAKING_SAMPLE_RATE;
                bufferDelay = SHAKING_BUF_DELAY;
                break;
        case SENSOR_TYPE_SIMPLE_TAPPING:
                dataRate = STAP_SAMPLE_RATE;
                bufferDelay = STAP_BUF_DELAY;
                break;
        default:
                break;
        }
}

bool SensorHubHelper::setPSHPropertyIfNeeded(int sensorType, struct sensor_hub_methods methods, handle_t handler)
{
        return true;
}

int SensorHubHelper::getGestureFlickEvent(struct gesture_flick_data data)
{
        return data.flick;
}

int SensorHubHelper::getTerminalEvent(struct tc_data data)
{
        return data.state;
}

int SensorHubHelper::getShakeEvent(struct accel_data data)
{
        return data.motion;
}

int SensorHubHelper::getSimpleTappingEvent(struct stap_data data)
{
        return data.stap;
}

int SensorHubHelper::getMoveDetectEvent(struct md_data data)
{
	return data.state;
}

size_t SensorHubHelper::getUnitSize(int sensorType)
{
        switch (sensorType) {
        case SENSOR_TYPE_ACCELEROMETER:
                return sizeof(struct accel_data);
        case SENSOR_TYPE_MAGNETIC_FIELD:
                return sizeof(struct compass_raw_data);
        case SENSOR_TYPE_ORIENTATION:
                return sizeof(struct orientation_data);
        case SENSOR_TYPE_GYROSCOPE:
                return sizeof(struct gyro_raw_data);
        case SENSOR_TYPE_LIGHT:
                return sizeof(struct als_raw_data);
        case SENSOR_TYPE_PRESSURE:
                return sizeof(struct baro_raw_data);
        case SENSOR_TYPE_PROXIMITY:
                return sizeof(struct ps_phy_data);
        case SENSOR_TYPE_GRAVITY:
                return sizeof(struct gravity_data);
        case SENSOR_TYPE_LINEAR_ACCELERATION:
                return sizeof(struct linear_accel_data);
        case SENSOR_TYPE_ROTATION_VECTOR:
                return sizeof(struct rotation_vector_data);
        case SENSOR_TYPE_GESTURE_FLICK:
                return sizeof(struct gesture_flick_data);
        case SENSOR_TYPE_GESTURE:
                return sizeof(struct gs_data);
        case SENSOR_TYPE_TERMINAL:
                return sizeof(struct tc_data);
        case SENSOR_TYPE_PEDOMETER:
                return sizeof(struct pedometer_data);
        case SENSOR_TYPE_SHAKE:
                return sizeof(struct accel_data);
        case SENSOR_TYPE_SIMPLE_TAPPING:
                return sizeof(struct stap_data);
        case SENSOR_TYPE_MOVE_DETECT:
                return sizeof(struct md_data);
        case SENSOR_TYPE_STEP_DETECTOR:
                return sizeof(struct stepdetector_data);
        case SENSOR_TYPE_STEP_COUNTER:
                return sizeof(struct stepcounter_data);
        case SENSOR_TYPE_SIGNIFICANT_MOTION:
                return sizeof(struct sm_data);
        case SENSOR_TYPE_GAME_ROTATION_VECTOR:
                return sizeof(struct game_rotation_vector_data);
        case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
                return sizeof(struct geomagnetic_rotation_vector_data);
	case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
		return sizeof(struct uncalib_compass_data);
	case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		return sizeof(struct uncalib_gyro_data);
	case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
                return sizeof(struct uncalib_acc_data);
	case SENSOR_TYPE_MOTION_DETECT:
		return sizeof(struct motion_detect_data);
        case SENSOR_TYPE_PAN_ZOOM:
                return sizeof(struct pz_data);
        case SENSOR_TYPE_LIFT:
                return sizeof(struct lift_data);
        case SENSOR_TYPE_INSTANT_ACTIVITY:
                return sizeof(struct instant_activity_data);
        case SENSOR_TYPE_TEMPERATURE:
        case SENSOR_TYPE_RELATIVE_HUMIDITY:
        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
        case SENSOR_TYPE_AUDIO_CLASSIFICATION:
        default:
                log_message(CRITICAL,"%s: Unsupported Sensor Type: %d", __FUNCTION__, sensorType);
                break;
        }
        return -1;
}


float SensorHubHelper::ConvertToFloat(int value, int sensorType)
{
        switch (sensorType) {
		case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
		case SENSOR_TYPE_LINEAR_ACCELERATION:
	        case SENSOR_TYPE_ACCELEROMETER:
			return	CONVERT_A_G_VTF16E14_X(4, -6, value);

		case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
		case SENSOR_TYPE_MAGNETIC_FIELD:
			return	CONVERT_M_MG_VTF16E14_X(4, -3, value);

		case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		case SENSOR_TYPE_GYROSCOPE:
			return CONVERT_G_D_VTF16E14_X(4, -5, value);

		case SENSOR_TYPE_LIGHT:
			return ((float) value) / 1000;

		case SENSOR_TYPE_PRESSURE:
			return ((float) value) / 100;	//exponent is 5 to convert to Bars, and 1 Bar = 1000 hectopascals. Android expect hectopascals.

		case SENSOR_TYPE_PROXIMITY:
			return value;

		case SENSOR_TYPE_MOTION_DETECT:
			return value;

		case SENSOR_TYPE_ROTATION_VECTOR:
		case SENSOR_TYPE_GAME_ROTATION_VECTOR:
		case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
			return ((float) value) / 10000;

		case SENSOR_TYPE_GRAVITY:
			return ((float)value * 9.8) / 1000000;

		case SENSOR_TYPE_STEP_DETECTOR:
			return 1.0;

                case SENSOR_TYPE_PAN_ZOOM:
                case SENSOR_TYPE_LIFT:
                case SENSOR_TYPE_INSTANT_ACTIVITY:
                case SENSOR_TYPE_SIMPLE_TAPPING:
                        return (float) value;
		}

		log_message(CRITICAL, "%s: unsupported convert. sensorType: %d, value: %d\n", __func__, sensorType, value);
		return (float)value;
}

ssize_t SensorHubHelper::readSensorhubEvents(int fd, struct sensorhub_event_t* events, size_t count, int sensorType)
{
        if (fd < 0)
                return fd;

        if (count <= 0)
                return 0;

        size_t unitSize = getUnitSize(sensorType);
        size_t streamSize = unitSize * count;
        byte* stream = new byte[streamSize];
        streamSize = read(fd, reinterpret_cast<void *>(stream), streamSize);
        if (streamSize % unitSize != 0) {
                log_message(CRITICAL,"%s line: %d: invalid stream size: type: %d size: %d unit_size: %d\n",
                     __FUNCTION__, __LINE__, sensorType, streamSize, unitSize);
                delete[] stream;
                return -1;
        }

        count = streamSize / unitSize;

        switch (sensorType) {
        case SENSOR_TYPE_ACCELEROMETER:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct accel_data*>(stream))[i].x;
                        events[i].data[1] = (reinterpret_cast<struct accel_data*>(stream))[i].y;
                        events[i].data[2] = (reinterpret_cast<struct accel_data*>(stream))[i].z;
                        events[i].accuracy = SENSOR_STATUS_ACCURACY_MEDIUM;
                        events[i].timestamp = (reinterpret_cast<struct accel_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_MAGNETIC_FIELD:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct compass_raw_data *>(stream))[i].x;
                        events[i].data[1] = (reinterpret_cast<struct compass_raw_data *>(stream))[i].y;
                        events[i].data[2] = (reinterpret_cast<struct compass_raw_data *>(stream))[i].z;
                        events[i].timestamp = (reinterpret_cast<struct compass_raw_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_ORIENTATION:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct orientation_data*>(stream))[i].tiltx;
                        events[i].data[1] = (reinterpret_cast<struct orientation_data*>(stream))[i].tilty;
                        events[i].data[2] = (reinterpret_cast<struct orientation_data*>(stream))[i].tiltz;
                        events[i].accuracy = SENSOR_STATUS_ACCURACY_MEDIUM;
                        events[i].timestamp = (reinterpret_cast<struct orientation_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_GYROSCOPE:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct gyro_raw_data*>(stream))[i].x;
                        events[i].data[1] = (reinterpret_cast<struct gyro_raw_data*>(stream))[i].y;
                        events[i].data[2] = (reinterpret_cast<struct gyro_raw_data*>(stream))[i].z;
                        events[i].timestamp = (reinterpret_cast<struct gyro_raw_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_LIGHT:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct als_raw_data *>(stream))[i].lux;
                        events[i].timestamp = (reinterpret_cast<struct als_raw_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_PRESSURE:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct baro_raw_data*>(stream))[i].p;
                        events[i].timestamp = (reinterpret_cast<struct baro_raw_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_PROXIMITY:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct ps_phy_data*>(stream))[i].near == 0 ? 1 : 0;
                        events[i].timestamp = (reinterpret_cast<struct ps_phy_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_GRAVITY:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct gravity_data*>(stream))[i].x;
                        events[i].data[1] = (reinterpret_cast<struct gravity_data*>(stream))[i].y;
                        events[i].data[2] = (reinterpret_cast<struct gravity_data*>(stream))[i].z;
                        events[i].accuracy = SENSOR_STATUS_ACCURACY_MEDIUM;
                        events[i].timestamp = (reinterpret_cast<struct gravity_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_LINEAR_ACCELERATION:
		for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct linear_accel_data*>(stream))[i].x;
                        events[i].data[1] = (reinterpret_cast<struct linear_accel_data*>(stream))[i].y;
                        events[i].data[2] = (reinterpret_cast<struct linear_accel_data*>(stream))[i].z;
                        events[i].accuracy = SENSOR_STATUS_ACCURACY_MEDIUM;
                        events[i].timestamp = (reinterpret_cast<struct linear_accel_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_ROTATION_VECTOR:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct rotation_vector_data*>(stream))[i].x;
                        events[i].data[1] = (reinterpret_cast<struct rotation_vector_data*>(stream))[i].y;
                        events[i].data[2] = (reinterpret_cast<struct rotation_vector_data*>(stream))[i].z;
                        events[i].data[3] = (reinterpret_cast<struct rotation_vector_data*>(stream))[i].w;
                        events[i].timestamp = (reinterpret_cast<struct rotation_vector_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_GESTURE_FLICK:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = getGestureFlickEvent((reinterpret_cast<struct gesture_flick_data*>(stream))[i]);
                        events[i].timestamp = (reinterpret_cast<struct gesture_flick_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_TERMINAL:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = getTerminalEvent((reinterpret_cast<struct tc_data*>(stream))[i]);
                        events[i].timestamp = (reinterpret_cast<struct tc_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_SHAKE:
                for (unsigned int i = 0; i < count; i++) {
                        /* workaround: shaking data is combined to accelerometer data.
                        events[i].data[0] = getShakeEvent((reinterpret_cast<struct shaking_data*>(stream))[i]);
                        events[i].timestamp = (reinterpret_cast<struct shaking_data*>(stream))[i].ts;
                        */
                        events[i].data[0] = getShakeEvent((reinterpret_cast<struct accel_data*>(stream))[i]);
                        events[i].timestamp = (reinterpret_cast<struct accel_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_SIMPLE_TAPPING:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = getSimpleTappingEvent((reinterpret_cast<struct stap_data*>(stream))[i]);
                        events[i].timestamp = (reinterpret_cast<struct stap_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_MOVE_DETECT:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = getMoveDetectEvent((reinterpret_cast<struct md_data*>(stream))[i]);
                        events[i].timestamp = (reinterpret_cast<struct md_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_STEP_DETECTOR:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = ((reinterpret_cast<struct stepdetector_data*>(stream))[i]).step_event_counter;
                        events[i].timestamp = (reinterpret_cast<struct stepdetector_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_STEP_COUNTER:
                for (unsigned int i = 0; i < count; i++) {
			int run_counter = ((reinterpret_cast<struct stepcounter_data*>(stream))[i]).run_step_count;
			int walk_counter = ((reinterpret_cast<struct stepcounter_data*>(stream))[i]).walk_step_count;
                        events[i].data[0] = run_counter + walk_counter;
                        events[i].timestamp = (reinterpret_cast<struct stepcounter_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_SIGNIFICANT_MOTION:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = ((reinterpret_cast<struct sm_data*>(stream))[i]).state;
                        events[i].timestamp = (reinterpret_cast<struct sm_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_LIFT:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = ((reinterpret_cast<struct lift_data*>(stream))[i]).look;
                        events[i].data[1] = ((reinterpret_cast<struct lift_data*>(stream))[i]).vertical;
                        events[i].timestamp = (reinterpret_cast<struct lift_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_PAN_ZOOM:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = ((reinterpret_cast<struct pz_data*>(stream))[i]).deltX;
                        events[i].data[1] = ((reinterpret_cast<struct pz_data*>(stream))[i]).deltY;
                        events[i].timestamp = (reinterpret_cast<struct pz_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_INSTANT_ACTIVITY:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = ((reinterpret_cast<struct instant_activity_data*>(stream))[i]).typeclass;
                        events[i].timestamp = (reinterpret_cast<struct instant_activity_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_GAME_ROTATION_VECTOR:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct game_rotation_vector_data*>(stream))[i].x;
                        events[i].data[1] = (reinterpret_cast<struct game_rotation_vector_data*>(stream))[i].y;
                        events[i].data[2] = (reinterpret_cast<struct game_rotation_vector_data*>(stream))[i].z;
                        events[i].data[3] = (reinterpret_cast<struct game_rotation_vector_data*>(stream))[i].w;
                        events[i].timestamp = (reinterpret_cast<struct game_rotation_vector_data*>(stream))[i].ts;
                }
                break;
        case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct geomagnetic_rotation_vector_data*>(stream))[i].x;
                        events[i].data[1] = (reinterpret_cast<struct geomagnetic_rotation_vector_data*>(stream))[i].y;
                        events[i].data[2] = (reinterpret_cast<struct geomagnetic_rotation_vector_data*>(stream))[i].z;
                        events[i].data[3] = (reinterpret_cast<struct geomagnetic_rotation_vector_data*>(stream))[i].w;
                        events[i].timestamp = (reinterpret_cast<struct geomagnetic_rotation_vector_data*>(stream))[i].ts;
                }
                break;
	case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct uncalib_compass_data*>(stream))[i].x_uncalib;
                        events[i].data[1] = (reinterpret_cast<struct uncalib_compass_data*>(stream))[i].y_uncalib;
                        events[i].data[2] = (reinterpret_cast<struct uncalib_compass_data*>(stream))[i].z_uncalib;
                        events[i].data[3] = (reinterpret_cast<struct uncalib_compass_data*>(stream))[i].x_calib - events[i].data[0];
			events[i].data[4] = (reinterpret_cast<struct uncalib_compass_data*>(stream))[i].y_calib - events[i].data[1];
			events[i].data[5] = (reinterpret_cast<struct uncalib_compass_data*>(stream))[i].z_calib - events[i].data[2];
                        events[i].timestamp = (reinterpret_cast<struct uncalib_compass_data*>(stream))[i].ts;
                }
                break;
	case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct uncalib_gyro_data*>(stream))[i].x_uncalib;
                        events[i].data[1] = (reinterpret_cast<struct uncalib_gyro_data*>(stream))[i].y_uncalib;
                        events[i].data[2] = (reinterpret_cast<struct uncalib_gyro_data*>(stream))[i].z_uncalib;
                        events[i].data[3] = (reinterpret_cast<struct uncalib_gyro_data*>(stream))[i].x_calib - events[i].data[0];
                        events[i].data[4] = (reinterpret_cast<struct uncalib_gyro_data*>(stream))[i].y_calib - events[i].data[1];
                        events[i].data[5] = (reinterpret_cast<struct uncalib_gyro_data*>(stream))[i].z_calib - events[i].data[2];
			events[i].timestamp = (reinterpret_cast<struct uncalib_gyro_data*>(stream))[i].ts;
                }
                break;
	case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
                for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct uncalib_acc_data*>(stream))[i].x_uncalib;
                        events[i].data[1] = (reinterpret_cast<struct uncalib_acc_data*>(stream))[i].y_uncalib;
                        events[i].data[2] = (reinterpret_cast<struct uncalib_acc_data*>(stream))[i].z_uncalib;
                        events[i].timestamp = (reinterpret_cast<struct uncalib_acc_data*>(stream))[i].ts;
                }
                break;
	case SENSOR_TYPE_MOTION_DETECT:
		for (unsigned int i = 0; i < count; i++) {
                        events[i].data[0] = (reinterpret_cast<struct motion_detect_data*>(stream))[i].eventData1;
                        events[i].data[1] = (reinterpret_cast<struct motion_detect_data*>(stream))[i].eventData2;
                        events[i].data[2] = (reinterpret_cast<struct motion_detect_data*>(stream))[i].eventData3;
                        events[i].data[3] = (reinterpret_cast<struct motion_detect_data*>(stream))[i].eventData4;
                        events[i].data[4] = (reinterpret_cast<struct motion_detect_data*>(stream))[i].eventData5;
                        events[i].timestamp = (reinterpret_cast<struct motion_detect_data*>(stream))[i].ts;
                }
                break;

        case SENSOR_TYPE_TEMPERATURE:
        case SENSOR_TYPE_RELATIVE_HUMIDITY:
        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
        case SENSOR_TYPE_AUDIO_CLASSIFICATION:
        case SENSOR_TYPE_GESTURE:
        case SENSOR_TYPE_PEDOMETER:
        default:
                log_message(CRITICAL,"%s: Unsupported Sensor Type: %d", __FUNCTION__, sensorType);
                break;
        }

        delete[] stream;

        return count;
}
