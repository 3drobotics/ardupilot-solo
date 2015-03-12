/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "AP_InertialSensor_PX4.h"

const extern AP_HAL::HAL& hal;

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_hrt.h>

#include <stdio.h>

AP_InertialSensor_PX4::AP_InertialSensor_PX4(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu),
    _last_get_sample_timestamp(0),
    _last_sample_timestamp(0),
    _last_gyro_filter_hz(-1),
    _last_accel_filter_hz(-1)
{
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _delta_angle_accumulator[i].zero();
        _delta_velocity_accumulator[i].zero();
    }
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_PX4::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_PX4 *sensor = new AP_InertialSensor_PX4(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

/*
  calculate the right queue depth for a device with the given sensor
  sample rate
 */
uint8_t AP_InertialSensor_PX4::_queue_depth(uint16_t sensor_sample_rate) const
{
    uint16_t requested_sample_rate = get_sample_rate_hz();
    uint8_t min_depth = (sensor_sample_rate+requested_sample_rate-1)/requested_sample_rate;
    // add 5ms more worth of queue to account for possible timing jitter
    uint8_t ret = min_depth + (5 * sensor_sample_rate) / 1000;
    return ret;
}

bool AP_InertialSensor_PX4::_init_sensor(void) 
{
    // assumes max 3 instances
    _accel_fd[0] = open(ACCEL_BASE_DEVICE_PATH "0", O_RDONLY);
    _accel_fd[1] = open(ACCEL_BASE_DEVICE_PATH "1", O_RDONLY);
    _accel_fd[2] = open(ACCEL_BASE_DEVICE_PATH "2", O_RDONLY);
    _gyro_fd[0] = open(GYRO_BASE_DEVICE_PATH "0", O_RDONLY);
    _gyro_fd[1] = open(GYRO_BASE_DEVICE_PATH "1", O_RDONLY);
    _gyro_fd[2] = open(GYRO_BASE_DEVICE_PATH "2", O_RDONLY);

    _num_accel_instances = 0;
    _num_gyro_instances = 0;
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        if (_accel_fd[i] >= 0) {
            _num_accel_instances = i+1;
            _accel_instance[i] = _imu.register_accel();
        }
        if (_gyro_fd[i] >= 0) {
            _num_gyro_instances = i+1;
            _gyro_instance[i] = _imu.register_gyro();
        }
    }
    if (_num_accel_instances == 0) {
        return false;
    }
    if (_num_gyro_instances == 0) {
        return false;
    }

    for (uint8_t i=0; i<_num_gyro_instances; i++) {
        int fd = _gyro_fd[i];
        int devid = (ioctl(fd, DEVIOCGDEVICEID, 0) & 0x00FF0000)>>16;

        // software LPF off
        ioctl(fd, GYROIOCSLOWPASS, 0);
        // 2000dps range
        ioctl(fd, GYROIOCSRANGE, 2000);

        switch(devid) {
            case DRV_GYR_DEVTYPE_MPU6000:
                // hardware LPF off
                ioctl(fd, GYROIOCSHWLOWPASS, 256);
                // khz sampling
                ioctl(fd, GYROIOCSSAMPLERATE, 1000);
                // set queue depth
                ioctl(fd, SENSORIOCSQUEUEDEPTH, _queue_depth(1000));
                break;
            case DRV_GYR_DEVTYPE_L3GD20:
                // hardware LPF as high as possible
                ioctl(fd, GYROIOCSHWLOWPASS, 100);
                // ~khz sampling
                ioctl(fd, GYROIOCSSAMPLERATE, 800);
                // 10ms queue depth
                ioctl(fd, SENSORIOCSQUEUEDEPTH, _queue_depth(800));
                break;
            default:
                break;
        }
    }

    for (uint8_t i=0; i<_num_accel_instances; i++) {
        int fd = _accel_fd[i];
        int devid = (ioctl(fd, DEVIOCGDEVICEID, 0) & 0x00FF0000)>>16;

        // software LPF off
        ioctl(fd, ACCELIOCSLOWPASS, 0);
        // 16g range
        ioctl(fd, ACCELIOCSRANGE, 16);

        switch(devid) {
            case DRV_ACC_DEVTYPE_MPU6000:
                // hardware LPF off
                ioctl(fd, ACCELIOCSHWLOWPASS, 256);
                // khz sampling
                ioctl(fd, ACCELIOCSSAMPLERATE, 1000);
                // 10ms queue depth
                ioctl(fd, SENSORIOCSQUEUEDEPTH, _queue_depth(1000));
                break;
            case DRV_ACC_DEVTYPE_LSM303D:
                // hardware LPF to ~1/10th sample rate for antialiasing
                ioctl(fd, ACCELIOCSHWLOWPASS, 194);
                // ~khz sampling
                ioctl(fd, ACCELIOCSSAMPLERATE, 1600);
                ioctl(fd,SENSORIOCSPOLLRATE, 1600);
                // 10ms queue depth
                ioctl(fd, SENSORIOCSQUEUEDEPTH, _queue_depth(1600));
                break;
            default:
                break;
        }
    }

    _set_accel_filter_frequency(_accel_filter_cutoff());
    _set_gyro_filter_frequency(_gyro_filter_cutoff());

#if  CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    _product_id = AP_PRODUCT_ID_VRBRAIN;
#else
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
    _product_id = AP_PRODUCT_ID_PX4_V2;
#else
    _product_id = AP_PRODUCT_ID_PX4;
#endif
#endif
    return true;
}

/*
  set the accel filter frequency
 */
void AP_InertialSensor_PX4::_set_accel_filter_frequency(uint8_t filter_hz)
{
    for (uint8_t i=0; i<_num_accel_instances; i++) {
        int samplerate = ioctl(_accel_fd[i],  ACCELIOCGSAMPLERATE, 0);
        if(samplerate < 100 || samplerate > 2000) {
            // sample rate doesn't seem sane, turn off filter
            _accel_filter[i].set_cutoff_frequency(0, 0);
        } else {
            _accel_filter[i].set_cutoff_frequency(samplerate, filter_hz);
        }
    }
}

/*
  set the gyro filter frequency
 */
void AP_InertialSensor_PX4::_set_gyro_filter_frequency(uint8_t filter_hz)
{
    for (uint8_t i=0; i<_num_gyro_instances; i++) {
        int samplerate = ioctl(_gyro_fd[i],  GYROIOCGSAMPLERATE, 0);
        if(samplerate < 100 || samplerate > 2000) {
            // sample rate doesn't seem sane, turn off filter
            _gyro_filter[i].set_cutoff_frequency(0, 0);
        } else {
            _gyro_filter[i].set_cutoff_frequency(samplerate, filter_hz);
        }
    }
}

bool AP_InertialSensor_PX4::update(void) 
{
    // get the latest sample from the sensor drivers
    _get_sample();

    for (uint8_t k=0; k<_num_accel_instances; k++) {
        Vector3f accel = _accel_in[k];
        // calling _publish_accel sets the sensor healthy,
        // so we only want to do this if we have new data from it
        if (_last_accel_timestamp[k] != _last_accel_update_timestamp[k]) {
            _publish_accel(_accel_instance[k], accel, false);
            _publish_delta_velocity(_accel_instance[k], _delta_velocity_accumulator[k]);
            _last_accel_update_timestamp[k] = _last_accel_timestamp[k];
        }
    }

    for (uint8_t k=0; k<_num_gyro_instances; k++) {
        Vector3f gyro = _gyro_in[k];
        // calling _publish_accel sets the sensor healthy,
        // so we only want to do this if we have new data from it
        if (_last_gyro_timestamp[k] != _last_gyro_update_timestamp[k]) {
            _publish_gyro(_gyro_instance[k], gyro, false);
            _publish_delta_angle(_gyro_instance[k], _delta_angle_accumulator[k]);
            _last_gyro_update_timestamp[k] = _last_gyro_timestamp[k];
        }
    }

    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _delta_angle_accumulator[i].zero();
        _delta_velocity_accumulator[i].zero();
    }

    if (_last_accel_filter_hz != _accel_filter_cutoff()) {
        _set_accel_filter_frequency(_accel_filter_cutoff());
        _last_accel_filter_hz = _accel_filter_cutoff();
    }

    if (_last_gyro_filter_hz != _gyro_filter_cutoff()) {
        _set_gyro_filter_frequency(_gyro_filter_cutoff());
        _last_gyro_filter_hz = _gyro_filter_cutoff();
    }
    
    return true;
}

void AP_InertialSensor_PX4::_new_accel_sample(uint8_t i, accel_report &accel_report)
{
    Vector3f accel = Vector3f(accel_report.x, accel_report.y, accel_report.z);
    uint8_t frontend_instance = _accel_instance[i];

    // apply corrections
    _rotate_and_correct_accel(frontend_instance, accel);

    // apply filter for control path
    _accel_in[i] = _accel_filter[i].apply(accel);

    // compute time since last sample
    float dt = (accel_report.timestamp - _last_accel_timestamp[i]) * 1.0e-6f;

    // compute delta velocity
    Vector3f delVel = Vector3f(accel.x, accel.y, accel.z) * dt;

    // integrate delta velocity accumulator
    _delta_velocity_accumulator[i] += delVel;

    // save last timestamp
    _last_accel_timestamp[i] = accel_report.timestamp;

    // report error count
    _set_accel_error_count(frontend_instance, accel_report.error_count);

#ifdef AP_INERTIALSENSOR_PX4_DEBUG
    _accel_dt_max[i] = max(_accel_dt_max[i],dt);

    _accel_meas_count[i] ++;

    if(_accel_meas_count[i] >= 10000) {
        uint32_t tnow = hal.scheduler->micros();

        ::printf("a%d %.2f Hz max %.8f s\n", frontend_instance, 10000.0/((tnow-_accel_meas_count_start_us[i])*1.0e-6f),_accel_dt_max[i]);

        _accel_meas_count_start_us[i] = tnow;
        _accel_meas_count[i] = 0;
        _accel_dt_max[i] = 0;
    }
#endif // AP_INERTIALSENSOR_PX4_DEBUG
}

void AP_InertialSensor_PX4::_new_gyro_sample(uint8_t i, gyro_report &gyro_report)
{
    Vector3f gyro = Vector3f(gyro_report.x, gyro_report.y, gyro_report.z);
    uint8_t frontend_instance = _gyro_instance[i];

    // apply corrections
    _rotate_and_correct_gyro(frontend_instance, gyro);

    // apply filter for control path
    _gyro_in[i] = _gyro_filter[i].apply(gyro);

    // compute time since last sample - not more than 50ms
    float dt = min((gyro_report.timestamp - _last_gyro_timestamp[i]) * 1.0e-6f, 0.05f);

    // compute delta angle
    Vector3f delAng = (gyro+_last_gyro[i]) * 0.5f * dt;

    /* compute coning correction
     * see page 26 of:
     * Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
     * Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
     * see also examples/coning.py
     */
    Vector3f delConing = ((_delta_angle_accumulator[i]+_last_delAng[i]*(1.0f/6.0f)) % delAng) * 0.5f;

    // integrate delta angle accumulator
    // the angles and coning corrections are accumulated separately in the
    // referenced paper, but in simulation little difference was found between
    // integrating together and integrating separately (see examples/coning.py)
    _delta_angle_accumulator[i] += delAng + delConing;

    // save previous delta angle for coning correction
    _last_delAng[i] = delAng;
    _last_gyro[i] = gyro;

    // save last timestamp
    _last_gyro_timestamp[i] = gyro_report.timestamp;

    // report error count
    _set_gyro_error_count(_gyro_instance[i], gyro_report.error_count);
#ifdef AP_INERTIALSENSOR_PX4_DEBUG
    _gyro_dt_max[i] = max(_gyro_dt_max[i],dt);

    _gyro_meas_count[i] ++;

    if(_gyro_meas_count[i] >= 10000) {
        uint32_t tnow = hal.scheduler->micros();

        ::printf("g%d %.2f Hz max %.8f s\n", frontend_instance, 10000.0f/((tnow-_gyro_meas_count_start_us[i])*1.0e-6f), _gyro_dt_max[i]);

        _gyro_meas_count_start_us[i] = tnow;
        _gyro_meas_count[i] = 0;
        _gyro_dt_max[i] = 0;
    }
#endif // AP_INERTIALSENSOR_PX4_DEBUG
}

void AP_InertialSensor_PX4::_get_sample()
{
    for (uint8_t i=0; i<max(_num_accel_instances,_num_gyro_instances);i++) {
        struct accel_report accel_report;
        struct gyro_report gyro_report;

        bool gyro_valid = _get_gyro_sample(i,gyro_report);
        bool accel_valid = _get_accel_sample(i,accel_report);

        while(gyro_valid || accel_valid) {
            // interleave accel and gyro samples by time - this will allow sculling corrections later
            // check the next gyro measurement to see if it needs to be integrated first
            if(gyro_valid && accel_valid && gyro_report.timestamp <= accel_report.timestamp) {
                _new_gyro_sample(i,gyro_report);
                gyro_valid = _get_gyro_sample(i,gyro_report);
                continue;
            }
            // if not, try to integrate an accelerometer sample
            if(accel_valid) {
                _new_accel_sample(i,accel_report);
                accel_valid = _get_accel_sample(i,accel_report);
                continue;
            }
            // if not, we've only got gyro samples left in the buffer
            if(gyro_valid) {
                _new_gyro_sample(i,gyro_report);
                gyro_valid = _get_gyro_sample(i,gyro_report);
            }
        }
    }
    _last_get_sample_timestamp = hal.scheduler->micros64();
}

bool AP_InertialSensor_PX4::_get_accel_sample(uint8_t i, struct accel_report &accel_report) {
    return i<_num_accel_instances && _accel_fd[i] != -1 && ::read(_accel_fd[i], &accel_report, sizeof(accel_report)) == sizeof(accel_report) && accel_report.timestamp != _last_accel_timestamp[i];
}

bool AP_InertialSensor_PX4::_get_gyro_sample(uint8_t i, struct gyro_report &gyro_report) {
    return i<_num_gyro_instances && _gyro_fd[i] != -1 && ::read(_gyro_fd[i], &gyro_report, sizeof(gyro_report)) == sizeof(gyro_report) && gyro_report.timestamp != _last_gyro_timestamp[i];
}

bool AP_InertialSensor_PX4::gyro_sample_available(void)
{
    _get_sample();
    for (uint8_t i=0; i<_num_gyro_instances; i++) {
        if (_last_gyro_timestamp[i] != _last_gyro_update_timestamp[i]) {
            return true;
        }
    }    
    return false;
}

bool AP_InertialSensor_PX4::accel_sample_available(void)
{
    _get_sample();
    for (uint8_t i=0; i<_num_accel_instances; i++) {
        if (_last_accel_timestamp[i] != _last_accel_update_timestamp[i]) {
            return true;
        }
    }    
    return false;
}

#endif // CONFIG_HAL_BOARD

