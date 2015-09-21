// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl.h"
#include <AP_HAL.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] PROGMEM = {

    // 0, 1 were RATE_RP_MAX, RATE_Y_MAX

    // @Param: SLEW_YAW
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 500 18000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("SLEW_YAW",    2, AC_AttitudeControl, _slew_yaw, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT),

    // 3 was for ACCEL_RP_MAX

    // @Param: ACCEL_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 72000
    // @Values: 0:Disabled, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("ACCEL_Y_MAX",  4, AC_AttitudeControl, _accel_yaw_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT),

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedfoward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RATE_FF_ENAB", 5, AC_AttitudeControl, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    // @Param: ACCEL_R_MAX
    // @DisplayName: Acceleration Max for Roll
    // @Description: Maximum acceleration in roll axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_R_MAX", 6, AC_AttitudeControl, _accel_roll_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT),

    // @Param: ACCEL_P_MAX
    // @DisplayName: Acceleration Max for Pitch
    // @Description: Maximum acceleration in pitch axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_P_MAX", 7, AC_AttitudeControl, _accel_pitch_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT),

    AP_GROUPEND
};

//
// high level controllers
//

void AC_AttitudeControl::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // set attitude controller's D term filters
    _pid_rate_roll.set_dt(_dt);
    _pid_rate_pitch.set_dt(_dt);
    _pid_rate_yaw.set_dt(_dt);
}

// relax_bf_rate_controller - ensure body-frame rate controller has zero errors to relax rate controller output
void AC_AttitudeControl::relax_bf_rate_controller()
{
    // ensure zero error in body frame rate controllers
    const Vector3f& gyro = _ahrs.get_gyro_for_control();
    _rate_bf_target = gyro * AC_ATTITUDE_CONTROL_DEGX100;

    _pid_rate_roll.reset_I();
    _pid_rate_pitch.reset_I();
    _pid_rate_yaw.reset_I();
}

// shifts earth frame yaw target by yaw_shift_cd.  yaw_shift_cd should be in centi-degreesa and is added to the current target heading
void AC_AttitudeControl::shift_ef_yaw_target(float yaw_shift_cd)
{
    _angle_ef_target.z = wrap_360_cd_float(_angle_ef_target.z + yaw_shift_cd);
}

//
// methods to be called by upper controllers to request and implement a desired attitude
//

// angle_ef_roll_pitch_rate_ef_yaw_smooth - attempts to maintain a roll and pitch angle and yaw rate (all earth frame) while smoothing the attitude based on the feel parameter
//      smoothing_gain : a number from 1 to 50 with 1 being sluggish and 50 being very crisp
void AC_AttitudeControl::angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain)
{
    float rate_ef_desired;
    float rate_change_limit;
    Vector3f angle_ef_error;    // earth frame angle errors

    // sanity check smoothing gain
    smoothing_gain = constrain_float(smoothing_gain,1.0f,50.0f);

    // if accel limiting and feed forward enabled
    if ((_accel_roll_max > 0.0f) && _rate_bf_ff_enabled) {
        rate_change_limit = _accel_roll_max * _dt;

        // calculate earth-frame feed forward roll rate using linear response when close to the target, sqrt response when we're further away
        rate_ef_desired = sqrt_controller(roll_angle_ef-_angle_ef_target.x, smoothing_gain, _accel_roll_max);

        // apply acceleration limit to feed forward roll rate
        _rate_ef_desired.x = constrain_float(rate_ef_desired, _rate_ef_desired.x-rate_change_limit, _rate_ef_desired.x+rate_change_limit);

        // update earth-frame roll angle target using desired roll rate
        update_ef_roll_angle_and_error(_rate_ef_desired.x, angle_ef_error, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX);
    } else {
        // target roll and pitch to desired input roll and pitch
        _angle_ef_target.x = roll_angle_ef;
        angle_ef_error.x = wrap_180_cd_float(_angle_ef_target.x - _ahrs.roll_sensor);

        // set roll and pitch feed forward to zero
        _rate_ef_desired.x = 0;
    }
    // constrain earth-frame angle targets
    _angle_ef_target.x = constrain_float(_angle_ef_target.x, -_aparm.angle_max, _aparm.angle_max);

    // if accel limiting and feed forward enabled
    if ((_accel_pitch_max > 0.0f) && _rate_bf_ff_enabled) {
        rate_change_limit = _accel_pitch_max * _dt;

        // calculate earth-frame feed forward pitch rate using linear response when close to the target, sqrt response when we're further away
        rate_ef_desired = sqrt_controller(pitch_angle_ef-_angle_ef_target.y, smoothing_gain, _accel_pitch_max);

        // apply acceleration limit to feed forward pitch rate
        _rate_ef_desired.y = constrain_float(rate_ef_desired, _rate_ef_desired.y-rate_change_limit, _rate_ef_desired.y+rate_change_limit);

        // update earth-frame pitch angle target using desired pitch rate
        update_ef_pitch_angle_and_error(_rate_ef_desired.y, angle_ef_error, AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX);
    } else {
        // target roll and pitch to desired input roll and pitch
        _angle_ef_target.y = pitch_angle_ef;
        angle_ef_error.y = wrap_180_cd_float(_angle_ef_target.y - _ahrs.pitch_sensor);

        // set roll and pitch feed forward to zero
        _rate_ef_desired.y = 0;
    }
    // constrain earth-frame angle targets
    _angle_ef_target.y = constrain_float(_angle_ef_target.y, -_aparm.angle_max, _aparm.angle_max);

    if (_accel_yaw_max > 0.0f) {
        // set earth-frame feed forward rate for yaw
        rate_change_limit = _accel_yaw_max * _dt;

        // update yaw rate target with acceleration limit
        _rate_ef_desired.z += constrain_float(yaw_rate_ef - _rate_ef_desired.z, -rate_change_limit, rate_change_limit);

        // calculate yaw target angle and angle error
        update_ef_yaw_angle_and_error(_rate_ef_desired.z, angle_ef_error, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);
    } else {
        // set yaw feed forward to zero
        _rate_ef_desired.z = yaw_rate_ef;
        // calculate yaw target angle and angle error
        update_ef_yaw_angle_and_error(_rate_ef_desired.z, angle_ef_error, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);
    }

    // convert earth-frame angle errors to body-frame angle errors
    frame_conversion_ef_to_bf(angle_ef_error, _angle_bf_error);


    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // add body frame rate feed forward
    if (_rate_bf_ff_enabled) {
        // convert earth-frame feed forward rates to body-frame feed forward rates
        frame_conversion_ef_to_bf(_rate_ef_desired, _rate_bf_desired);
        _rate_bf_target += _rate_bf_desired;
    } else {
        // convert earth-frame feed forward rates to body-frame feed forward rates
        frame_conversion_ef_to_bf(Vector3f(0,0,_rate_ef_desired.z), _rate_bf_desired);
        _rate_bf_target += _rate_bf_desired;
    }

    // body-frame to motor outputs should be called separately
}

//
// methods to be called by upper controllers to request and implement a desired attitude
//

// angle_ef_roll_pitch_rate_ef_yaw - attempts to maintain a roll and pitch angle and yaw rate (all earth frame)
void AC_AttitudeControl::angle_ef_roll_pitch_rate_ef_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef)
{
    Vector3f    angle_ef_error;         // earth frame angle errors

    // set earth-frame angle targets for roll and pitch and calculate angle error
    _angle_ef_target.x = constrain_float(roll_angle_ef, -_aparm.angle_max, _aparm.angle_max);
    angle_ef_error.x = wrap_180_cd_float(_angle_ef_target.x - _ahrs.roll_sensor);

    _angle_ef_target.y = constrain_float(pitch_angle_ef, -_aparm.angle_max, _aparm.angle_max);
    angle_ef_error.y = wrap_180_cd_float(_angle_ef_target.y - _ahrs.pitch_sensor);

    if (_accel_yaw_max > 0.0f) {
        // set earth-frame feed forward rate for yaw
        float rate_change_limit = _accel_yaw_max * _dt;

        float rate_change = yaw_rate_ef - _rate_ef_desired.z;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_ef_desired.z += rate_change;
        // calculate yaw target angle and angle error
        update_ef_yaw_angle_and_error(_rate_ef_desired.z, angle_ef_error, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);
    } else {
        // set yaw feed forward to zero
        _rate_ef_desired.z = yaw_rate_ef;
        // calculate yaw target angle and angle error
        update_ef_yaw_angle_and_error(_rate_ef_desired.z, angle_ef_error, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);
    }

    // convert earth-frame angle errors to body-frame angle errors
    frame_conversion_ef_to_bf(angle_ef_error, _angle_bf_error);

    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // set roll and pitch feed forward to zero
    _rate_ef_desired.x = 0;
    _rate_ef_desired.y = 0;
    // convert earth-frame feed forward rates to body-frame feed forward rates
    frame_conversion_ef_to_bf(_rate_ef_desired, _rate_bf_desired);
    _rate_bf_target += _rate_bf_desired;

    // body-frame to motor outputs should be called separately
}

// angle_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw angle (all earth frame)
//  if yaw_slew is true then target yaw movement will be gradually moved to the new target based on the SLEW_YAW parameter
void AC_AttitudeControl::angle_ef_roll_pitch_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_angle_ef, bool slew_yaw)
{
    Vector3f    angle_ef_error;

    // set earth-frame angle targets
    _angle_ef_target.x = constrain_float(roll_angle_ef, -_aparm.angle_max, _aparm.angle_max);
    _angle_ef_target.y = constrain_float(pitch_angle_ef, -_aparm.angle_max, _aparm.angle_max);
    _angle_ef_target.z = yaw_angle_ef;

    // calculate earth frame errors
    angle_ef_error.x = wrap_180_cd_float(_angle_ef_target.x - _ahrs.roll_sensor);
    angle_ef_error.y = wrap_180_cd_float(_angle_ef_target.y - _ahrs.pitch_sensor);
    angle_ef_error.z = wrap_180_cd_float(_angle_ef_target.z - _ahrs.yaw_sensor);

    // constrain the yaw angle error
    if (slew_yaw) {
        angle_ef_error.z = constrain_float(angle_ef_error.z,-_slew_yaw,_slew_yaw);
    }

    // convert earth-frame angle errors to body-frame angle errors
    frame_conversion_ef_to_bf(angle_ef_error, _angle_bf_error);

    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // body-frame to motor outputs should be called separately
}

// rate_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all earth frame)
void AC_AttitudeControl::rate_ef_roll_pitch_yaw(float roll_rate_ef, float pitch_rate_ef, float yaw_rate_ef)
{
    Vector3f angle_ef_error;
    float rate_change_limit, rate_change;

    if (_accel_roll_max > 0.0f) {
        rate_change_limit = _accel_roll_max * _dt;

        // update feed forward roll rate after checking it is within acceleration limits
        rate_change = roll_rate_ef - _rate_ef_desired.x;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_ef_desired.x += rate_change;
    } else {
        _rate_ef_desired.x = roll_rate_ef;
    }

    if (_accel_pitch_max > 0.0f) {
        rate_change_limit = _accel_pitch_max * _dt;

        // update feed forward pitch rate after checking it is within acceleration limits
        rate_change = pitch_rate_ef - _rate_ef_desired.y;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_ef_desired.y += rate_change;
    } else {
        _rate_ef_desired.y = pitch_rate_ef;
    }

    if (_accel_yaw_max > 0.0f) {
        rate_change_limit = _accel_yaw_max * _dt;

        // update feed forward yaw rate after checking it is within acceleration limits
        rate_change = yaw_rate_ef - _rate_ef_desired.z;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_ef_desired.z += rate_change;
    } else {
        _rate_ef_desired.z = yaw_rate_ef;
    }

    // update earth frame angle targets and errors
    update_ef_roll_angle_and_error(_rate_ef_desired.x, angle_ef_error, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX);
    update_ef_pitch_angle_and_error(_rate_ef_desired.y, angle_ef_error, AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX);
    update_ef_yaw_angle_and_error(_rate_ef_desired.z, angle_ef_error, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);

    // constrain earth-frame angle targets
    _angle_ef_target.x = constrain_float(_angle_ef_target.x, -_aparm.angle_max, _aparm.angle_max);
    _angle_ef_target.y = constrain_float(_angle_ef_target.y, -_aparm.angle_max, _aparm.angle_max);

    // convert earth-frame angle errors to body-frame angle errors
    frame_conversion_ef_to_bf(angle_ef_error, _angle_bf_error);

    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // convert earth-frame rates to body-frame rates
    frame_conversion_ef_to_bf(_rate_ef_desired, _rate_bf_desired);

    // add body frame rate feed forward
    _rate_bf_target += _rate_bf_desired;

    // body-frame to motor outputs should be called separately
}

// rate_bf_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all body frame)
void AC_AttitudeControl::rate_bf_roll_pitch_yaw(float roll_rate_bf, float pitch_rate_bf, float yaw_rate_bf)
{
    Vector3f    angle_ef_error;

    float rate_change, rate_change_limit;

    // update the rate feed forward with angular acceleration limits
    if (_accel_roll_max > 0.0f) {
        rate_change_limit = _accel_roll_max * _dt;

        rate_change = roll_rate_bf - _rate_bf_desired.x;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_bf_desired.x += rate_change;
    } else {
        _rate_bf_desired.x = roll_rate_bf;
    }

    // update the rate feed forward with angular acceleration limits
    if (_accel_pitch_max > 0.0f) {
        rate_change_limit = _accel_pitch_max * _dt;

        rate_change = pitch_rate_bf - _rate_bf_desired.y;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_bf_desired.y += rate_change;
    } else {
        _rate_bf_desired.y = pitch_rate_bf;
    }

    if (_accel_yaw_max > 0.0f) {
        rate_change_limit = _accel_yaw_max * _dt;

        rate_change = yaw_rate_bf - _rate_bf_desired.z;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_bf_desired.z += rate_change;
    } else {
    	_rate_bf_desired.z = yaw_rate_bf;
    }

    // Update angle error
    if (labs(_ahrs.pitch_sensor)<_acro_angle_switch) {
        _acro_angle_switch = 6000;
        // convert body-frame rates to earth-frame rates
        frame_conversion_bf_to_ef(_rate_bf_desired, _rate_ef_desired);

        // update earth frame angle targets and errors
        update_ef_roll_angle_and_error(_rate_ef_desired.x, angle_ef_error, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX);
        update_ef_pitch_angle_and_error(_rate_ef_desired.y, angle_ef_error, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX);
        update_ef_yaw_angle_and_error(_rate_ef_desired.z, angle_ef_error, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX);

        // convert earth-frame angle errors to body-frame angle errors
        frame_conversion_ef_to_bf(angle_ef_error, _angle_bf_error);
    } else {
        _acro_angle_switch = 4500;
        integrate_bf_rate_error_to_angle_errors();
        if (frame_conversion_bf_to_ef(_angle_bf_error, angle_ef_error)) {
            _angle_ef_target.x = wrap_180_cd_float(angle_ef_error.x + _ahrs.roll_sensor);
            _angle_ef_target.y = wrap_180_cd_float(angle_ef_error.y + _ahrs.pitch_sensor);
            _angle_ef_target.z = wrap_360_cd_float(angle_ef_error.z + _ahrs.yaw_sensor);
        }
        if (_angle_ef_target.y > 9000.0f) {
            _angle_ef_target.x = wrap_180_cd_float(_angle_ef_target.x + 18000.0f);
            _angle_ef_target.y = wrap_180_cd_float(18000.0f - _angle_ef_target.y);
            _angle_ef_target.z = wrap_360_cd_float(_angle_ef_target.z + 18000.0f);
        }
        if (_angle_ef_target.y < -9000.0f) {
            _angle_ef_target.x = wrap_180_cd_float(_angle_ef_target.x + 18000.0f);
            _angle_ef_target.y = wrap_180_cd_float(-18000.0f - _angle_ef_target.y);
            _angle_ef_target.z = wrap_360_cd_float(_angle_ef_target.z + 18000.0f);
        }
    }

    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // body-frame rate commands added
    _rate_bf_target += _rate_bf_desired;
}

//
// rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
//      should be called at 100hz or more
//
void AC_AttitudeControl::rate_controller_run()
{
    // call rate controllers and send output to motors object
    // To-Do: should the outputs from get_rate_roll, pitch, yaw be int16_t which is the input to the motors library?
    // To-Do: skip this step if the throttle out is zero?
    _motors.set_roll(rate_bf_to_motor_roll(_rate_bf_target.x));
    _motors.set_pitch(rate_bf_to_motor_pitch(_rate_bf_target.y));
    _motors.set_yaw(rate_bf_to_motor_yaw(_rate_bf_target.z));
}

//
// earth-frame <-> body-frame conversion functions
//
// frame_conversion_ef_to_bf - converts earth frame vector to body frame vector
void AC_AttitudeControl::frame_conversion_ef_to_bf(const Vector3f& ef_vector, Vector3f& bf_vector)
{
    // convert earth frame rates to body frame rates
    bf_vector.x = ef_vector.x - _ahrs.sin_pitch() * ef_vector.z;
    bf_vector.y = _ahrs.cos_roll()  * ef_vector.y + _ahrs.sin_roll() * _ahrs.cos_pitch() * ef_vector.z;
    bf_vector.z = -_ahrs.sin_roll() * ef_vector.y + _ahrs.cos_pitch() * _ahrs.cos_roll() * ef_vector.z;
}

// frame_conversion_bf_to_ef - converts body frame vector to earth frame vector
bool AC_AttitudeControl::frame_conversion_bf_to_ef(const Vector3f& bf_vector, Vector3f& ef_vector)
{
    // avoid divide by zero
    if (_ahrs.cos_pitch() == 0.0f) {
        return false;
    }
    // convert earth frame angle or rates to body frame
    ef_vector.x = bf_vector.x + _ahrs.sin_roll() * (_ahrs.sin_pitch()/_ahrs.cos_pitch()) * bf_vector.y + _ahrs.cos_roll() * (_ahrs.sin_pitch()/_ahrs.cos_pitch()) * bf_vector.z;
    ef_vector.y = _ahrs.cos_roll()  * bf_vector.y - _ahrs.sin_roll() * bf_vector.z;
    ef_vector.z = (_ahrs.sin_roll() / _ahrs.cos_pitch()) * bf_vector.y + (_ahrs.cos_roll() / _ahrs.cos_pitch()) * bf_vector.z;
    return true;
}

//
// protected methods
//

//
// stabilized rate controller (body-frame) methods
//

// update_ef_roll_angle_and_error - update _angle_ef_target.x using an earth frame roll rate request
void AC_AttitudeControl::update_ef_roll_angle_and_error(float roll_rate_ef, Vector3f &angle_ef_error, float overshoot_max)
{
    // calculate angle error with maximum of +- max angle overshoot
    angle_ef_error.x = wrap_180_cd_float(_angle_ef_target.x - _ahrs.roll_sensor);
    angle_ef_error.x  = constrain_float(angle_ef_error.x, -overshoot_max, overshoot_max);

    // update roll angle target to be within max angle overshoot of our roll angle
    _angle_ef_target.x = angle_ef_error.x + _ahrs.roll_sensor;

    // increment the roll angle target
    _angle_ef_target.x += roll_rate_ef * _dt;
    _angle_ef_target.x = wrap_180_cd_float(_angle_ef_target.x);
}

// update_ef_pitch_angle_and_error - update _angle_ef_target.y using an earth frame pitch rate request
void AC_AttitudeControl::update_ef_pitch_angle_and_error(float pitch_rate_ef, Vector3f &angle_ef_error, float overshoot_max)
{
    // calculate angle error with maximum of +- max angle overshoot
    // To-Do: should we do something better as we cross 90 degrees?
    angle_ef_error.y = wrap_180_cd_float(_angle_ef_target.y - _ahrs.pitch_sensor);
    angle_ef_error.y  = constrain_float(angle_ef_error.y, -overshoot_max, overshoot_max);

    // update pitch angle target to be within max angle overshoot of our pitch angle
    _angle_ef_target.y = angle_ef_error.y + _ahrs.pitch_sensor;

    // increment the pitch angle target
    _angle_ef_target.y += pitch_rate_ef * _dt;
    _angle_ef_target.y = wrap_180_cd_float(_angle_ef_target.y);
}

// update_ef_yaw_angle_and_error - update _angle_ef_target.z using an earth frame yaw rate request
void AC_AttitudeControl::update_ef_yaw_angle_and_error(float yaw_rate_ef, Vector3f &angle_ef_error, float overshoot_max)
{
    // calculate angle error with maximum of +- max angle overshoot
    angle_ef_error.z = wrap_180_cd_float(_angle_ef_target.z - _ahrs.yaw_sensor);
    angle_ef_error.z  = constrain_float(angle_ef_error.z, -overshoot_max, overshoot_max);

    // update yaw angle target to be within max angle overshoot of our current heading
    _angle_ef_target.z = angle_ef_error.z + _ahrs.yaw_sensor;


    // increment the yaw angle target
    _angle_ef_target.z += yaw_rate_ef * _dt;
    _angle_ef_target.z = wrap_360_cd_float(_angle_ef_target.z);
}

// update_rate_bf_errors - calculates body frame angle errors
//   body-frame feed forward rates (centi-degrees / second) taken from _angle_bf_error
//   angle errors in centi-degrees placed in _angle_bf_error
void AC_AttitudeControl::integrate_bf_rate_error_to_angle_errors()
{
    // roll - calculate body-frame angle error by integrating body-frame rate error
    _angle_bf_error.x += (_rate_bf_desired.x - (_ahrs.get_gyro_for_control().x * AC_ATTITUDE_CONTROL_DEGX100)) * _dt;
    // roll - limit maximum error
    _angle_bf_error.x = constrain_float(_angle_bf_error.x, -AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX);


    // pitch - calculate body-frame angle error by integrating body-frame rate error
    _angle_bf_error.y += (_rate_bf_desired.y - (_ahrs.get_gyro_for_control().y * AC_ATTITUDE_CONTROL_DEGX100)) * _dt;
    // pitch - limit maximum error
    _angle_bf_error.y = constrain_float(_angle_bf_error.y, -AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX);


    // yaw - calculate body-frame angle error by integrating body-frame rate error
    _angle_bf_error.z += (_rate_bf_desired.z - (_ahrs.get_gyro_for_control().z * AC_ATTITUDE_CONTROL_DEGX100)) * _dt;
    // yaw - limit maximum error
    _angle_bf_error.z = constrain_float(_angle_bf_error.z, -AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX);

    // To-Do: handle case of motors being disarmed or g.rc_3.servo_out == 0 and set error to zero
}

// update_rate_bf_targets - converts body-frame angle error to body-frame rate targets for roll, pitch and yaw axis
//   targets rates in centi-degrees taken from _angle_bf_error
//   results in centi-degrees/sec put into _rate_bf_target
void AC_AttitudeControl::update_rate_bf_targets()
{

    // stab roll calculation
    // constrain roll rate request
    if (_flags.limit_angle_to_rate_request) {
        _rate_bf_target.x = sqrt_controller(_angle_bf_error.x, _p_angle_roll.kP(), constrain_float(_accel_roll_max/2.0f,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX));
    }else{
        _rate_bf_target.x = _p_angle_roll.kP() * _angle_bf_error.x;
    }

    // stab pitch calculation
    // constrain pitch rate request
    if (_flags.limit_angle_to_rate_request) {
        _rate_bf_target.y = sqrt_controller(_angle_bf_error.y, _p_angle_pitch.kP(), constrain_float(_accel_pitch_max/2.0f,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX));
    }else{
        _rate_bf_target.y = _p_angle_pitch.kP() * _angle_bf_error.y;
    }

    // stab yaw calculation
    // constrain yaw rate request
    if (_flags.limit_angle_to_rate_request) {
        _rate_bf_target.z = sqrt_controller(_angle_bf_error.z, _p_angle_yaw.kP(), constrain_float(_accel_yaw_max/2.0f,  AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX));
    }else{
        _rate_bf_target.z = _p_angle_yaw.kP() * _angle_bf_error.z;
    }

    // include roll and pitch rate required to account for precession of the desired attitude about the body frame yaw axes
	_rate_bf_target.x += _angle_bf_error.y * _ahrs.get_gyro_for_control().z;
	_rate_bf_target.y += -_angle_bf_error.x * _ahrs.get_gyro_for_control().z;
}

//
// body-frame rate controller
//

// rate_bf_to_motor_roll - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl::rate_bf_to_motor_roll(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro_for_control().x * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    _pid_rate_roll.set_input_filter_d(rate_error);

    // get p value
    p = _pid_rate_roll.get_p();

    // get i term
    i = _pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_roll.get_i();
    }

    // get d term
    d = _pid_rate_roll.get_d();

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

// rate_bf_to_motor_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl::rate_bf_to_motor_pitch(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro_for_control().y * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    _pid_rate_pitch.set_input_filter_d(rate_error);

    // get p value
    p = _pid_rate_pitch.get_p();

    // get i term
    i = _pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_pitch.get_i();
    }

    // get d term
    d = _pid_rate_pitch.get_d();

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl::rate_bf_to_motor_yaw(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro_for_control().z * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error  = rate_target_cds - current_rate;
    _pid_rate_yaw.set_input_filter_all(rate_error);

    // get p value
    p = _pid_rate_yaw.get_p();

    // get i term
    i = _pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_yaw.get_i();
    }

    // get d value
    d = _pid_rate_yaw.get_d();

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

// accel_limiting - enable or disable accel limiting
void AC_AttitudeControl::accel_limiting(bool enable_limits)
{
    if (enable_limits) {
        // if enabling limits, reload from eeprom or set to defaults
        if (_accel_roll_max == 0.0f) {
            _accel_roll_max.load();
        }
        // if enabling limits, reload from eeprom or set to defaults
        if (_accel_pitch_max == 0.0f) {
            _accel_pitch_max.load();
        }
        if (_accel_yaw_max == 0.0f) {
            _accel_yaw_max.load();
        }
    } else {
        // if disabling limits, set to zero
        _accel_roll_max = 0.0f;
        _accel_pitch_max = 0.0f;
        _accel_yaw_max = 0.0f;
    }
}

//
// throttle functions
//

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void AC_AttitudeControl::set_throttle_out(float throttle_out, bool apply_angle_boost, float filter_cutoff)
{
    _motors.set_stabilizing(true);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        _motors.set_throttle(get_boosted_throttle(throttle_out));
    }else{
        _motors.set_throttle(throttle_out);
        // clear angle_boost for logging purposes
        _angle_boost = 0;
    }
}

// outputs a throttle to all motors evenly with no attitude stabilization
void AC_AttitudeControl::set_throttle_out_unstabilized(float throttle_in, bool reset_attitude_control, float filter_cutoff)
{
    if (reset_attitude_control) {
        relax_bf_rate_controller();
        set_yaw_target_to_current_heading();
    }
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    _motors.set_stabilizing(false);
    _motors.set_throttle(throttle_in);
    _angle_boost = 0;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
float AC_AttitudeControl::get_boosted_throttle(float throttle_in)
{
    // inverted_factor is 1 for tilt angles below 60 degrees
    // reduces as a function of angle beyond 60 degrees
    // becomes zero at 90 degrees
    float min_throttle = _motors.throttle_min();
    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(2.0f*cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f/constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = (throttle_in-min_throttle)*inverted_factor*boost_factor + min_throttle;
    _angle_boost = constrain_float(throttle_out - throttle_in,-32000,32000);
    return throttle_out;
}

// sqrt_controller - response based on the sqrt of the error instead of the more common linear response
float AC_AttitudeControl::sqrt_controller(float error, float p, float second_ord_lim)
{
    if (second_ord_lim == 0.0f || p == 0.0f) {
        return error*p;
    }

    float linear_dist = second_ord_lim/sq(p);

    if (error > linear_dist) {
        return safe_sqrt(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
    } else if (error < -linear_dist) {
        return -safe_sqrt(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
    } else {
        return error*p;
    }
}

// Maximum roll rate step size that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_roll()
{
    float alpha = _pid_rate_roll.get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*_pid_rate_roll.kD())/_dt + _pid_rate_roll.kP());
}

// Maximum pitch rate step size that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_pitch()
{
    float alpha = _pid_rate_pitch.get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*_pid_rate_pitch.kD())/_dt + _pid_rate_pitch.kP());
}

// Maximum yaw rate step size that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_yaw()
{
    float alpha = _pid_rate_yaw.get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*_pid_rate_yaw.kD())/_dt + _pid_rate_yaw.kP());
}

float AC_AttitudeControl::inverse_sqrt_controller(float output, float p, float second_ord_lim)
{
    if (second_ord_lim == 0.0f || p == 0.0f) {
        return output/p;
    }

    float linear_dist = second_ord_lim/(p);

    if (output > linear_dist) {
        return (sq(second_ord_lim) + sq(output) * sq(p)) / (2*second_ord_lim*sq(p));
    } else if (output < -linear_dist) {
        return -(sq(second_ord_lim) + sq(output) * sq(p)) / (2*second_ord_lim*sq(p));
    } else {
        return output/p;
    }
}
