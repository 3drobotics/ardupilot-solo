#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include "AP_InertialSensor_GyroCalib.h"

extern const AP_HAL::HAL& hal;

AP_InertialSensor_GyroCalib::AP_InertialSensor_GyroCalib() :
    _status(WAITING)
{
}

void AP_InertialSensor_GyroCalib::init(enum Rotation &board_orientation)
{
    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // cold start
    hal.console->print_P(PSTR("Gyro Initialisation Started\n"));

    /*
      we do the gyro calibration with no board rotation. This avoids
      having to rotate readings during the calibration
    */
    _saved_orientation = board_orientation;
    board_orientation = ROTATION_NONE;

    // remove existing gyro offsets
    _new_gyro_offset.zero();
    _best_diff = 0;
    _last_average.zero();
    _converged = false;

    set_status(COLLECTION);
    return;
}

void AP_InertialSensor_GyroCalib::collect_samples(Vector3f sample, Vector3f accel_value)
{
    if(_status == COLLECTION){
        if(_sample_cnt == 0){
            _diff_norm = 0;
            _gyro_sum.zero();
            _accel_start = accel_value;
        }
        _gyro_sum += sample;
        if(_sample_cnt == MAX_GYRO_CALIB_SAMPLES){
            set_status(CALIBRATE_STEP);
        }
    } else {
        return;
    }
}

void AP_InertialSensor_GyroCalib::calibrate(Vector3f accel_value)
{
    Vector3f accel_diff = accel_value - _accel_start;
    Vector3f gyro_avg, gyro_diff;

    if (accel_diff.length() > 0.2f) {
        // the accelerometers changed during the gyro sum. Skip
        // this sample. This copes with doing gyro cal on a
        // steadily moving platform. The value 0.2 corresponds
        // with around 5 degrees/second of rotation.
        return;
    }

    gyro_avg = _gyro_sum / _sample_cnt;
    gyro_diff = _last_average - gyro_avg;
    _diff_norm = gyro_diff.length();

    if (_num_steps == 0) {
        _best_diff = _diff_norm;
        _best_avg = gyro_avg;
    } else if (gyro_diff.length() < ToRad(0.1f)) {
        // we want the average to be within 0.1 bit, which is 0.04 degrees/s
        _last_average = (gyro_avg * 0.5f) + (_last_average * 0.5f);
        if (!_converged || _last_average.length() < _new_gyro_offset.length()) {
            _new_gyro_offset = _last_average;
        }
        if (!_converged) {
            _converged = true;
        }
    } else if (_diff_norm < _best_diff) {
        _best_diff = _diff_norm;
        _best_avg = (gyro_avg * 0.5f) + (_last_average * 0.5f);
    }
    _last_average = gyro_avg;
}

bool AP_InertialSensor_GyroCalib::get_new_offsets(enum Rotation &board_orientation, AP_Vector3f &offsets){
    // we've kept the user waiting long enough - use the best pair we
    // found so far
    bool ret;
    if (!_converged) {
        hal.console->printf_P(PSTR("gyro did not converge: diff=%f dps\n"), ToDeg(_best_diff));
        offsets = _best_avg;
        // flag calibration as failed for this gyro
        ret = false;
    } else {
        offsets = _new_gyro_offset;
        ret = true;
    }

    // restore orientation
    board_orientation = _saved_orientation;

    // stop flashing leds
    AP_Notify::flags.initialising = false;
    set_status(WAITING);
    return ret;
}

void AP_InertialSensor_GyroCalib::set_status(enum gyro_calib_status_t status)
{
    switch(status){
        case WAITING:
            _status = WAITING;
            break;
        case INITIAL:
            _status = INITIAL;
            break;
        case COLLECTION:
            _status = COLLECTION;
            break;
        case CALIBRATE_STEP:
            _status = CALIBRATE_STEP;
            break;
        case COMPLETE:
            _status = COMPLETE;
            break;
        case BLOCKING_CALIBRATE:
            break;
        //unhandled
    }
}

bool AP_InertialSensor_GyroCalib::step(Vector3f accel_value)
{
    
    if(_status == CALIBRATE_STEP || _converged == true){
        if(_num_steps >= MAX_GYRO_CALIB_STEPS){
            set_status(COMPLETE);
            return true;
        }
        calibrate(accel_value);
        set_status(COLLECTION);
        _num_steps++;
    }
    return false;
}