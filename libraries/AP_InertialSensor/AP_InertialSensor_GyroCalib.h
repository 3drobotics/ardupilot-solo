/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_GYRO_CALIB_H__
#define __AP_INERTIAL_SENSOR_GYRO_CALIB_H__

#define MAX_GYRO_CALIB_SAMPLES  100
#define MAX_GYRO_CALIB_STEPS    120
enum gyro_calib_status_t {
    WAITING=0,
    INITIAL=1,
    COLLECTION=2,
    CALIBRATE_STEP=3,
    BLOCKING_CALIBRATE=4,
    COMPLETE=5
};

class AP_InertialSensor_GyroCalib {
public:
    AP_InertialSensor_GyroCalib();
    void init(enum Rotation &board_orientation);
    bool step(Vector3f accel_value);
    bool get_new_offsets(enum Rotation &board_orientation, AP_Vector3f &offsets);
    void collect_samples(Vector3f sample, Vector3f accel_value);
    gyro_calib_status_t get_status(){ return _status;}

private:
    void calibrate(Vector3f accel_value);
    void set_status(gyro_calib_status_t status);
    gyro_calib_status_t _status;

    //gyro init variables
    uint16_t _sample_cnt;
    uint16_t _num_steps;
    Vector3f _last_average, _best_avg;
    Vector3f _new_gyro_offset;
    float _best_diff;
    Vector3f _gyro_sum;
    bool _converged;
    Vector3f _accel_start;
    float _diff_norm;
    enum Rotation _saved_orientation;
};

#endif