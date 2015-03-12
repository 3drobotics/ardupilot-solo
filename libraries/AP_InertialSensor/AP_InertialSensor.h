/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_H__
#define __AP_INERTIAL_SENSOR_H__

// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f

/**
   maximum number of INS instances available on this platform. If more
   than 1 then redundent sensors may be available
 */
#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
#define INS_MAX_INSTANCES 3
#define INS_MAX_BACKENDS  6
#else
#define INS_MAX_INSTANCES 1
#define INS_MAX_BACKENDS  1
#endif


#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include "AP_InertialSensor_UserInteract.h"

class AP_InertialSensor_Backend;

/* AP_InertialSensor is an abstraction for gyro and accel measurements
 * which are correctly aligned to the body axes and scaled to SI units.
 *
 * Gauss-Newton accel calibration routines borrowed from Rolfe Schmidt
 * blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
 * original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
 */
class AP_InertialSensor
{
    friend class AP_InertialSensor_Backend;

public:
    AP_InertialSensor();

    enum Start_style {
        COLD_START = 0,
        WARM_START
    };

    // the rate that updates will be available to the application
    enum Sample_rate {
        RATE_50HZ  = 50,
        RATE_100HZ = 100,
        RATE_200HZ = 200,
        RATE_400HZ = 400
    };

    /// Perform startup initialisation.
    ///
    /// Called to initialise the state of the IMU.
    ///
    /// For COLD_START, implementations using real sensors can assume
    /// that the airframe is stationary and nominally oriented.
    ///
    /// For WARM_START, no assumptions should be made about the
    /// orientation or motion of the airframe.  Calibration should be
    /// as for the previous COLD_START call.
    ///
    /// @param style	The initialisation startup style.
    ///
    void init( Start_style style,
               Sample_rate sample_rate);

    /// Register a new gyro/accel driver, allocating an instance
    /// number
    uint8_t register_gyro(void);
    uint8_t register_accel(void);

#if !defined( __AVR_ATmega1280__ )
    // perform accelerometer calibration including providing user instructions
    // and feedback
    bool calibrate_accel(AP_InertialSensor_UserInteract *interact,
                         float& trim_roll,
                         float& trim_pitch);
#endif

    /// calibrated - returns true if the accelerometers have been calibrated
    ///
    /// @note this should not be called while flying because it reads from the eeprom which can be slow
    ///
    bool calibrated() const;

    /// calibrating - returns true if the gyros or accels are currently being calibrated
    bool calibrating() const { return _calibrating; }

    /// Perform cold-start initialisation for just the gyros.
    ///
    /// @note This should not be called unless ::init has previously
    ///       been called, as ::init may perform other work
    ///
    void init_gyro(void);

    /// Fetch the current gyro values
    ///
    /// @returns	vector of rotational rates in radians/sec
    ///
    const Vector3f     &get_gyro(uint8_t i) const { return _gyro[i]; }
    const Vector3f     &get_gyro(void) const { return get_gyro(_primary_gyro); }
    void               set_gyro(uint8_t instance, const Vector3f &gyro);

    // set gyro offsets in radians/sec
    const Vector3f &get_gyro_offsets(uint8_t i) const { return _gyro_offset[i]; }
    const Vector3f &get_gyro_offsets(void) const { return get_gyro_offsets(_primary_gyro); }

    //get delta angle if available
    bool get_delta_angle(uint8_t i, Vector3f &delta_angle) const {
        if(_delta_angle_valid[i]) delta_angle = _delta_angle[i];
        return _delta_angle_valid[i];
    }

    bool get_delta_angle(Vector3f &delta_angle) const { return get_delta_angle(_primary_gyro, delta_angle); }


    //get delta velocity if available
    bool get_delta_velocity(uint8_t i, Vector3f &delta_velocity) const {
        if(_delta_velocity_valid[i]) delta_velocity = _delta_velocity[i];
        return _delta_velocity_valid[i];
    }

    bool get_delta_velocity(Vector3f &delta_velocity) const { return get_delta_velocity(_primary_accel, delta_velocity); }

    /// Fetch the current accelerometer values
    ///
    /// @returns	vector of current accelerations in m/s/s
    ///
    const Vector3f     &get_accel(uint8_t i) const { return _accel[i]; }
    const Vector3f     &get_accel(void) const { return get_accel(_primary_accel); }
    void               set_accel(uint8_t instance, const Vector3f &accel);

    uint32_t get_gyro_error_count(uint8_t i) const { return _gyro_error_count[i]; }
    uint32_t get_accel_error_count(uint8_t i) const { return _accel_error_count[i]; }

    // multi-device interface
    bool get_gyro_health(uint8_t instance) const { return _gyro_healthy[instance]; }
    bool get_gyro_health(void) const { return get_gyro_health(_primary_gyro); }
    bool get_gyro_health_all(void) const;
    uint8_t get_gyro_count(void) const { return _gyro_count; }
    bool gyro_calibrated_ok(uint8_t instance) const { return _gyro_cal_ok[instance]; }
    bool gyro_calibrated_ok_all() const;

    bool get_accel_health(uint8_t instance) const { return _accel_healthy[instance]; }
    bool get_accel_health(void) const { return get_accel_health(_primary_accel); }
    bool get_accel_health_all(void) const;
    uint8_t get_accel_count(void) const { return _accel_count; };

    // get accel offsets in m/s/s
    const Vector3f &get_accel_offsets(uint8_t i) const { return _accel_offset[i]; }
    const Vector3f &get_accel_offsets(void) const { return get_accel_offsets(_primary_accel); }

    // get accel scale
    const Vector3f &get_accel_scale(uint8_t i) const { return _accel_scale[i]; }
    const Vector3f &get_accel_scale(void) const { return get_accel_scale(_primary_accel); }

    /* get_delta_time returns the time period in seconds
     * overwhich the sensor data was collected
     */
    float get_delta_time() const { return _delta_time; }

    // return the maximum gyro drift rate in radians/s/s. This
    // depends on what gyro chips are being used
    float get_gyro_drift_rate(void) const { return ToRad(0.5f/60); }

    // update gyro and accel values from accumulated samples
    void update(void);

    // wait for a sample to be available
    void wait_for_sample(void);

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];

    // set overall board orientation
    void set_board_orientation(enum Rotation orientation) {
        _board_orientation = orientation;
    }

    // return the selected sample rate
    Sample_rate get_sample_rate(void) const { return _sample_rate; }

    uint16_t error_count(void) const { return 0; }
    bool healthy(void) const { return get_gyro_health() && get_accel_health(); }

    uint8_t get_primary_accel(void) const { return 0; }

    // enable HIL mode
    void set_hil_mode(void) { _hil_mode = true; }

private:

    // load backend drivers
    void _add_backend(AP_InertialSensor_Backend *(detect)(AP_InertialSensor &));
    void _detect_backends(void);

    // gyro initialisation
    void _init_gyro();

#if !defined( __AVR_ATmega1280__ )
    // Calibration routines borrowed from Rolfe Schmidt
    // blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
    // original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde

    // _calibrate_accel - perform low level accel calibration
    bool _calibrate_accel(const Vector3f accel_sample[6], Vector3f& accel_offsets, Vector3f& accel_scale, enum Rotation r);
    bool _check_sample_range(const Vector3f accel_sample[6], enum Rotation rotation, 
                             AP_InertialSensor_UserInteract* interact);
    void _calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
    void _calibrate_reset_matrices(float dS[6], float JS[6][6]);
    void _calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);
    void _calculate_trim(const Vector3f &accel_sample, float& trim_roll, float& trim_pitch);
#endif

    // check if we have 3D accel calibration
    void check_3D_calibration(void);

    // save parameters to eeprom
    void  _save_parameters();

    // backend objects
    AP_InertialSensor_Backend *_backends[INS_MAX_BACKENDS];

    // number of gyros and accel drivers. Note that most backends
    // provide both accel and gyro data, so will increment both
    // counters on initialisation
    uint8_t _gyro_count;
    uint8_t _accel_count;
    uint8_t _backend_count;

    // the selected sample rate
    Sample_rate _sample_rate;
    
    // Most recent accelerometer reading
    Vector3f _accel[INS_MAX_INSTANCES];
    Vector3f _delta_velocity[INS_MAX_INSTANCES];
    bool _delta_velocity_valid[INS_MAX_INSTANCES];

    // Most recent gyro reading
    Vector3f _gyro[INS_MAX_INSTANCES];
    Vector3f _delta_angle[INS_MAX_INSTANCES];
    bool _delta_angle_valid[INS_MAX_INSTANCES];

    // product id
    AP_Int16 _product_id;

    // accelerometer scaling and offsets
    AP_Vector3f _accel_scale[INS_MAX_INSTANCES];
    AP_Vector3f _accel_offset[INS_MAX_INSTANCES];
    AP_Vector3f _gyro_offset[INS_MAX_INSTANCES];

    // filtering frequency (0 means default)
    AP_Int8     _accel_filter_cutoff;
    AP_Int8     _gyro_filter_cutoff;

    // board orientation from AHRS
    enum Rotation _board_orientation;

    // calibrated_ok flags
    bool _gyro_cal_ok[INS_MAX_INSTANCES];

    // primary accel and gyro
    uint8_t _primary_gyro;
    uint8_t _primary_accel;

    // has wait_for_sample() found a sample?
    bool _have_sample:1;

    // are we in HIL mode?
    bool _hil_mode:1;

    // do we have offsets/scaling from a 3D calibration?
    bool _have_3D_calibration:1;

    // are gyros or accels currently being calibrated
    bool _calibrating:1;

    // the delta time in seconds for the last sample
    float _delta_time;

    // last time a wait_for_sample() returned a sample
    uint32_t _last_sample_usec;

    // target time for next wait_for_sample() return
    uint32_t _next_sample_usec;
    
    // time between samples in microseconds
    uint32_t _sample_period_usec;

    // health of gyros and accels
    bool _gyro_healthy[INS_MAX_INSTANCES];
    bool _accel_healthy[INS_MAX_INSTANCES];

    uint32_t _accel_error_count[INS_MAX_INSTANCES];
    uint32_t _gyro_error_count[INS_MAX_INSTANCES];
};

#include "AP_InertialSensor_Backend.h"
#include "AP_InertialSensor_MPU6000.h"
#include "AP_InertialSensor_PX4.h"
#include "AP_InertialSensor_Oilpan.h"
#include "AP_InertialSensor_MPU9250.h"
#include "AP_InertialSensor_L3G4200D.h"
#include "AP_InertialSensor_Flymaple.h"
#include "AP_InertialSensor_MPU9150.h"
#include "AP_InertialSensor_HIL.h"
#include "AP_InertialSensor_UserInteract_Stream.h"
#include "AP_InertialSensor_UserInteract_MAVLink.h"

#endif // __AP_INERTIAL_SENSOR_H__
