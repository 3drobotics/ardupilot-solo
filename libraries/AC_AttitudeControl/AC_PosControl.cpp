/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_PosControl.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PosControl::var_info[] PROGMEM = {
    // @Param: THR_HOVER
    // @DisplayName: Throttle Hover
    // @Description: The autopilot's estimate of the throttle required to maintain a level hover.  Calculated automatically from the pilot's throttle input while in stabilize mode
    // @Range: 0 1000
    // @Units: Percent*10
    // @User: Advanced
    AP_GROUPINFO("THR_HOVER",       0, AC_PosControl, _throttle_hover, POSCONTROL_THROTTLE_HOVER),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PosControl::AC_PosControl(const AP_AHRS& ahrs, const AP_InertialNav& inav,
                             const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                             AC_P& p_alt_pos, AC_P& p_alt_rate, AC_PID& pid_alt_accel,
                             AC_P& p_pos_xy, AC_PID& pid_rate_lat, AC_PID& pid_rate_lon) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _p_alt_pos(p_alt_pos),
    _p_alt_rate(p_alt_rate),
    _pid_alt_accel(pid_alt_accel),
    _p_pos_xy(p_pos_xy),
    _pid_rate_lat(pid_rate_lat),
    _pid_rate_lon(pid_rate_lon),
    _dt(POSCONTROL_DT_10HZ),
    _dt_xy(POSCONTROL_DT_50HZ),
    _last_update_xy_ms(0),
    _last_update_z_ms(0),
    _speed_down_cms(POSCONTROL_SPEED_DOWN),
    _speed_up_cms(POSCONTROL_SPEED_UP),
    _speed_cms(POSCONTROL_SPEED),
    _accel_z_cms(POSCONTROL_ACCEL_Z),
    _accel_cms(POSCONTROL_ACCEL_XY),
    _leash(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_down_z(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_up_z(POSCONTROL_LEASH_LENGTH_MIN),
    _roll_target(0.0f),
    _pitch_target(0.0f),
    _alt_max(0.0f),
    _distance_to_target(0.0f),
    _accel_target_jerk_limited(0.0f,0.0f),
    _accel_target_filtered(0.0f,0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150
    _flags.slow_cpu = false;
#else
    _flags.slow_cpu = true;
#endif
    _flags.recalc_leash_xy = true;
    _flags.recalc_leash_z = true;
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_rate_to_accel_xy = true;
    _flags.reset_rate_to_accel_z = true;
    _flags.reset_accel_to_throttle = true;
}

///
/// z-axis position controller
///


/// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
void AC_PosControl::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // update rate controller's d filter
    _pid_alt_accel.set_d_lpf_alpha(POSCONTROL_ACCEL_Z_DTERM_FILTER, _dt);

    // update rate z-axis velocity error and accel error filters
    _vel_error_filter.set_cutoff_frequency(_dt,POSCONTROL_VEL_ERROR_CUTOFF_FREQ);
    _accel_error_filter.set_cutoff_frequency(_dt,POSCONTROL_ACCEL_ERROR_CUTOFF_FREQ);
}

/// set_speed_z - sets maximum climb and descent rates
/// To-Do: call this in the main code as part of flight mode initialisation
///     calc_leash_length_z should be called afterwards
///     speed_down should be a negative number
void AC_PosControl::set_speed_z(float speed_down, float speed_up)
{
    // ensure speed_down is always negative
    speed_down = (float)-fabs(speed_down);

    if (((float)fabs(_speed_down_cms-speed_down) > 1.0f) || ((float)fabs(_speed_up_cms-speed_up) > 1.0f)) {
        _speed_down_cms = speed_down;
        _speed_up_cms = speed_up;
        _flags.recalc_leash_z = true;
    }
}

/// set_accel_z - set vertical acceleration in cm/s/s
void AC_PosControl::set_accel_z(float accel_cmss)
{
    if ((float)fabs(_accel_z_cms-accel_cmss) > 1.0f) {
        _accel_z_cms = accel_cmss;
        _flags.recalc_leash_z = true;
    }
}

/// set_alt_target_with_slew - adjusts target towards a final altitude target
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl::set_alt_target_with_slew(float alt_cm, float dt)
{
    float alt_change = alt_cm-_pos_target.z;
    
    _vel_desired.z = constrain_float(alt_change * dt, _speed_down_cms, _speed_up_cms);

    // adjust desired alt if motors have not hit their limits
    if ((alt_change<0 && !_motors.limit.throttle_lower) || (alt_change>0 && !_motors.limit.throttle_upper)) {
        _pos_target.z += constrain_float(alt_change, _speed_down_cms*dt, _speed_up_cms*dt);
    }

    // do not let target get too far from current altitude
    float curr_alt = _inav.get_altitude();
    _pos_target.z = constrain_float(_pos_target.z,curr_alt-_leash_down_z,curr_alt+_leash_up_z);
}

/// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl::set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend)
{
    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((climb_rate_cms<0 && (!_motors.limit.throttle_lower || force_descend)) || (climb_rate_cms>0 && !_motors.limit.throttle_upper && !_limit.pos_up)) {
        _pos_target.z += climb_rate_cms * dt;
    }

    // do not let target alt get above limit
    if (_alt_max > 0 && _pos_target.z > _alt_max) {
        _pos_target.z = _alt_max;
        _limit.pos_up = true;
    }

    _vel_desired.z = climb_rate_cms;
}

// get_alt_error - returns altitude error in cm
float AC_PosControl::get_alt_error() const
{
    return (_pos_target.z - _inav.get_altitude());
}

/// set_target_to_stopping_point_z - returns reasonable stopping altitude in cm above home
void AC_PosControl::set_target_to_stopping_point_z()
{
    // check if z leash needs to be recalculated
    calc_leash_length_z();

    get_stopping_point_z(_pos_target);
}

/// get_stopping_point_z - calculates stopping point based on current position, velocity, vehicle acceleration
void AC_PosControl::get_stopping_point_z(Vector3f& stopping_point) const
{
    const float curr_pos_z = _inav.get_altitude();
    float curr_vel_z = _inav.get_velocity_z();

    float linear_distance;  // half the distance we swap between linear and sqrt and the distance we offset sqrt
    float linear_velocity;  // the velocity we swap between linear and sqrt

    // if position controller is active add current velocity error to avoid sudden jump in acceleration
    if (is_active_z()) {
        curr_vel_z += _vel_error.z;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    linear_velocity = _accel_z_cms/_p_alt_pos.kP();

    if ((float)fabs(curr_vel_z) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        stopping_point.z = curr_pos_z + curr_vel_z/_p_alt_pos.kP();
    } else {
        linear_distance = _accel_z_cms/(2.0f*_p_alt_pos.kP()*_p_alt_pos.kP());
        if (curr_vel_z > 0){
            stopping_point.z = curr_pos_z + (linear_distance + curr_vel_z*curr_vel_z/(2.0f*_accel_z_cms));
        } else {
            stopping_point.z = curr_pos_z - (linear_distance + curr_vel_z*curr_vel_z/(2.0f*_accel_z_cms));
        }
    }
    stopping_point.z = constrain_float(stopping_point.z, curr_pos_z - POSCONTROL_STOPPING_DIST_Z_MAX, curr_pos_z + POSCONTROL_STOPPING_DIST_Z_MAX);
}

/// init_takeoff - initialises target altitude if we are taking off
void AC_PosControl::init_takeoff()
{
    const Vector3f& curr_pos = _inav.get_position();

    _pos_target.z = curr_pos.z + POSCONTROL_TAKEOFF_JUMP_CM;

    // freeze feedforward to avoid jump
    freeze_ff_z();

    // shift difference between last motor out and hover throttle into accelerometer I
    _pid_alt_accel.set_integrator(_motors.get_throttle_out()-_throttle_hover);
}

// is_active_z - returns true if the z-axis position controller has been run very recently
bool AC_PosControl::is_active_z() const
{
    return ((hal.scheduler->millis() - _last_update_z_ms) <= POSCONTROL_ACTIVE_TIMEOUT_MS);
}

/// update_z_controller - fly to altitude in cm above home
void AC_PosControl::update_z_controller()
{
    // check time since last cast
    uint32_t now = hal.scheduler->millis();
    if (now - _last_update_z_ms > POSCONTROL_ACTIVE_TIMEOUT_MS) {
        _flags.reset_rate_to_accel_z = true;
        _flags.reset_accel_to_throttle = true;
    }
    _last_update_z_ms = now;

    // check if leash lengths need to be recalculated
    calc_leash_length_z();

    // call position controller
    if(_gnd_effect_mode) {
        gnd_effect_pos_to_rate_z();
    } else {
        pos_to_rate_z();
    }
}

/// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
///     called by pos_to_rate_z if z-axis speed or accelerations are changed
void AC_PosControl::calc_leash_length_z()
{
    if (_flags.recalc_leash_z) {
        _leash_up_z = calc_leash_length(_speed_up_cms, _accel_z_cms, _p_alt_pos.kP());
        _leash_down_z = calc_leash_length(-_speed_down_cms, _accel_z_cms, _p_alt_pos.kP());
        _flags.recalc_leash_z = false;
    }
}

void AC_PosControl::update_gnd_effect_targets(float dt) {
    _pos_error.z = _pos_target.z - _gnd_effect_pos_target_z;

    // clear position limit flags
    _limit.pos_up = false;
    _limit.pos_down = false;

    // do not let target altitude get too far from current altitude
    if (_pos_error.z > _leash_up_z) {
        _pos_target.z = _gnd_effect_pos_target_z + _leash_up_z;
        _pos_error.z = _leash_up_z;
        _limit.pos_up = true;
    }
    if (_pos_error.z < -_leash_down_z) {
        _pos_target.z = _gnd_effect_pos_target_z - _leash_down_z;
        _pos_error.z = -_leash_down_z;
        _limit.pos_down = true;
    }

    _gnd_effect_vel_desired_z = AC_AttitudeControl::sqrt_controller(_pos_error.z, _p_alt_pos.kP(), _accel_z_cms);
    _gnd_effect_pos_error_z = _gnd_effect_pos_target_z - _inav.get_altitude();

    _gnd_effect_pos_target_z += _gnd_effect_vel_desired_z * dt;
}

void AC_PosControl::gnd_effect_pos_to_rate_z() {
    // update gnd effect pos target
    update_gnd_effect_targets(_dt);

    // calculate _vel_target.z using from _pos_error.z using sqrt controller
    _vel_target.z = AC_AttitudeControl::sqrt_controller(_gnd_effect_pos_error_z, _p_alt_pos.kP()*POSCONTROL_GNDEFFECT_GAIN, _accel_z_cms);
    _vel_target.z += _gnd_effect_vel_desired_z;

    // call rate based throttle controller which will update accel based throttle controller targets
    rate_to_accel_z();
}

// pos_to_rate_z - position to rate controller for Z axis
// calculates desired rate in earth-frame z axis and passes to rate controller
// vel_up_max, vel_down_max should have already been set before calling this method
void AC_PosControl::pos_to_rate_z()
{
    float curr_alt = _inav.get_altitude();

    // clear position limit flags
    _limit.pos_up = false;
    _limit.pos_down = false;

    // calculate altitude error
    _pos_error.z = _pos_target.z - curr_alt;

    // do not let target altitude get too far from current altitude
    if (_pos_error.z > _leash_up_z) {
        _pos_target.z = curr_alt + _leash_up_z;
        _pos_error.z = _leash_up_z;
        _limit.pos_up = true;
    }
    if (_pos_error.z < -_leash_down_z) {
        _pos_target.z = curr_alt - _leash_down_z;
        _pos_error.z = -_leash_down_z;
        _limit.pos_down = true;
    }

    // calculate _vel_target.z using from _pos_error.z using sqrt controller
    _vel_target.z = AC_AttitudeControl::sqrt_controller(_pos_error.z, _p_alt_pos.kP(), _accel_z_cms);

    // call rate based throttle controller which will update accel based throttle controller targets
    rate_to_accel_z();
}

// rate_to_accel_z - calculates desired accel required to achieve the velocity target
// calculates desired acceleration and calls accel throttle controller
void AC_PosControl::rate_to_accel_z()
{
    const Vector3f& curr_vel = _inav.get_velocity();
    float p;                                // used to capture pid values for logging
    float desired_accel;                    // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors

    // check speed limits
    // To-Do: check these speed limits here or in the pos->rate controller
    _limit.vel_up = false;
    _limit.vel_down = false;
    if (_vel_target.z < _speed_down_cms) {
        _vel_target.z = _speed_down_cms;
        _limit.vel_down = true;
    }
    if (_vel_target.z > _speed_up_cms) {
        _vel_target.z = _speed_up_cms;
        _limit.vel_up = true;
    }

    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_z) {
        _vel_last.z = _vel_target.z;
    }

    // feed forward desired acceleration calculation
    if (_dt > 0.0f) {
    	if (!_flags.freeze_ff_z) {
    		_accel_feedforward.z = (_vel_target.z - _vel_last.z)/_dt;
        } else {
    		// stop the feed forward being calculated during a known discontinuity
    		_flags.freeze_ff_z = false;
    	}
    } else {
    	_accel_feedforward.z = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.z = _vel_target.z;

    // reset velocity error and filter if this controller has just been engaged
    if (_flags.reset_rate_to_accel_z) {
        // Reset Filter
        _vel_error.z = 0;
        _vel_error_filter.reset(0);
        _flags.reset_rate_to_accel_z = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        _vel_error.z = _vel_error_filter.apply(_vel_target.z - curr_vel.z);
    }

    // calculate p
    p = _p_alt_rate.kP() * _vel_error.z;

    // consolidate and constrain target acceleration
    desired_accel = _accel_feedforward.z + p;
    desired_accel = constrain_int32(desired_accel, -32000, 32000);

    // set target for accel based throttle controller
    accel_to_throttle(desired_accel);
}

// accel_to_throttle - alt hold's acceleration controller
// calculates a desired throttle which is sent directly to the motors
void AC_PosControl::accel_to_throttle(float accel_target_z)
{
    float z_accel_meas;         // actual acceleration
    int32_t p,i,d;              // used to capture pid values for logging

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;

    // reset target altitude if this controller has just been engaged
    if (_flags.reset_accel_to_throttle) {
        // Reset Filter
        _accel_error.z = 0;
        _accel_error_filter.reset(0);
        _flags.reset_accel_to_throttle = false;
    } else {
        // calculate accel error and Filter with fc = 2 Hz
        _accel_error.z = _accel_error_filter.apply(constrain_float(accel_target_z - z_accel_meas, -32000, 32000));
    }

    // separately calculate p, i, d values for logging
    p = _pid_alt_accel.get_p(_accel_error.z);

    // get i term
    i = _pid_alt_accel.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    // To-Do: should this be replaced with limits check from attitude_controller?
    if ((!_motors.limit.throttle_lower && !_motors.limit.throttle_upper) || (i>0&&_accel_error.z<0) || (i<0&&_accel_error.z>0)) {
        i = _pid_alt_accel.get_i(_accel_error.z, _dt);
    }

    // get d term
    d = _pid_alt_accel.get_d(_accel_error.z, _dt);

    // send throttle to attitude controller with angle boost
    _attitude_control.set_throttle_out((int16_t)p+i+d+_throttle_hover, true);
}

///
/// position controller
///

/// set_accel_xy - set horizontal acceleration in cm/s/s
///     calc_leash_length_xy should be called afterwards
void AC_PosControl::set_accel_xy(float accel_cmss)
{
    if ((float)fabs(_accel_cms-accel_cmss) > 1.0f) {
        _accel_cms = accel_cmss;
        _flags.recalc_leash_xy = true;
    }
}

/// set_speed_xy - set horizontal speed maximum in cm/s
///     calc_leash_length_xy should be called afterwards
void AC_PosControl::set_speed_xy(float speed_cms)
{
    if ((float)fabs(_speed_cms-speed_cms) > 1.0f) {
        _speed_cms = speed_cms;
        _flags.recalc_leash_xy = true;
    }
}

/// set_pos_target in cm from home
void AC_PosControl::set_pos_target(const Vector3f& position)
{
    _pos_target = position;

    // initialise roll and pitch to current roll and pitch.  This avoids a twitch between when the target is set and the pos controller is first run
    // To-Do: this initialisation of roll and pitch targets needs to go somewhere between when pos-control is initialised and when it completes it's first cycle
    //_roll_target = constrain_int32(_ahrs.roll_sensor,-_attitude_control.lean_angle_max(),_attitude_control.lean_angle_max());
    //_pitch_target = constrain_int32(_ahrs.pitch_sensor,-_attitude_control.lean_angle_max(),_attitude_control.lean_angle_max());
}

/// set_xy_target in cm from home
void AC_PosControl::set_xy_target(float x, float y)
{
    _pos_target.x = x;
    _pos_target.y = y;
}

/// set_target_to_stopping_point_xy - sets horizontal target to reasonable stopping position in cm from home
void AC_PosControl::set_target_to_stopping_point_xy()
{
    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    get_stopping_point_xy(_pos_target);
}

/// get_stopping_point_xy - calculates stopping point based on current position, velocity, vehicle acceleration
///     distance_max allows limiting distance to stopping point
///     results placed in stopping_position vector
///     set_accel_xy() should be called before this method to set vehicle acceleration
///     set_leash_length() should have been called before this method
void AC_PosControl::get_stopping_point_xy(Vector3f &stopping_point) const
{
	const Vector3f curr_pos = _inav.get_position();
	Vector3f curr_vel = _inav.get_velocity();
    float linear_distance;      // the distance at which we swap from a linear to sqrt response
    float linear_velocity;      // the velocity above which we swap from a linear to sqrt response
    float stopping_dist;		// the distance within the vehicle can stop
    float kP = _p_pos_xy.kP();

    // add velocity error to current velocity
    if (is_active_xy()) {
        curr_vel.x += _vel_error.x;
        curr_vel.y += _vel_error.y;
    }

    // calculate current velocity
    float vel_total = pythagorous2(curr_vel.x, curr_vel.y);

    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (kP <= 0.0f || _accel_cms <= 0.0f || vel_total == 0.0f) {
        stopping_point.x = curr_pos.x;
        stopping_point.y = curr_pos.y;
        return;
    }

    // calculate point at which velocity switches from linear to sqrt
    linear_velocity = _accel_cms/kP;

    // calculate distance within which we can stop
    if (vel_total < linear_velocity) {
    	stopping_dist = vel_total/kP;
    } else {
        linear_distance = _accel_cms/(2.0f*kP*kP);
        stopping_dist = linear_distance + (vel_total*vel_total)/(2.0f*_accel_cms);
    }

    // constrain stopping distance
    stopping_dist = constrain_float(stopping_dist, 0, _leash);

    // convert the stopping distance into a stopping point using velocity vector
    stopping_point.x = curr_pos.x + (stopping_dist * curr_vel.x / vel_total);
    stopping_point.y = curr_pos.y + (stopping_dist * curr_vel.y / vel_total);
}

/// get_distance_to_target - get horizontal distance to loiter target in cm
float AC_PosControl::get_distance_to_target() const
{
    return _distance_to_target;
}

// is_active_xy - returns true if the xy position controller has been run very recently
bool AC_PosControl::is_active_xy() const
{
    return ((hal.scheduler->millis() - _last_update_xy_ms) <= POSCONTROL_ACTIVE_TIMEOUT_MS);
}

/// init_xy_controller - initialise the xy controller
///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
///     should be called once whenever significant changes to the position target are made
///     this does not update the xy target
void AC_PosControl::init_xy_controller(bool reset_I)
{
    // set roll, pitch lean angle targets to current attitude
    _roll_target = _ahrs.roll_sensor;
    _pitch_target = _ahrs.pitch_sensor;

    // initialise I terms from lean angles
    if (reset_I) {
        // reset last velocity if this controller has just been engaged or dt is zero
        lean_angles_to_accel(_accel_target.x, _accel_target.y);
        _pid_rate_lat.set_integrator(_accel_target.x);
        _pid_rate_lon.set_integrator(_accel_target.y);
    }

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_rate_to_accel_xy = true;
}

/// update_xy_controller - run the horizontal position controller - should be called at 100hz or higher
void AC_PosControl::update_xy_controller(xy_mode mode, float ekfNavVelGainScaler)
{
    // compute dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _last_update_xy_ms) / 1000.0f;
    _last_update_xy_ms = now;

    // sanity check dt - expect to be called faster than ~5hz
    if (dt > POSCONTROL_ACTIVE_TIMEOUT_MS*1.0e-3f) {
        dt = 0.0f;
    }

    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    // translate any adjustments from pilot to loiter target
    desired_vel_to_pos(dt);

    // run position controller's position error to desired velocity step
    pos_to_rate_xy(mode, dt, ekfNavVelGainScaler);

    // run position controller's velocity to acceleration step
    rate_to_accel_xy(dt, ekfNavVelGainScaler);

    // run position controller's acceleration to lean angle step
    accel_to_lean_angles(dt, ekfNavVelGainScaler);
}

float AC_PosControl::time_since_last_xy_update() const
{
    uint32_t now = hal.scheduler->millis();
    return (now - _last_update_xy_ms)*0.001f;
}

/// init_vel_controller_xyz - initialise the velocity controller - should be called once before the caller attempts to use the controller
void AC_PosControl::init_vel_controller_xyz()
{
    // set roll, pitch lean angle targets to current attitude
    _roll_target = _ahrs.roll_sensor;
    _pitch_target = _ahrs.pitch_sensor;

    // reset last velocity if this controller has just been engaged or dt is zero
    lean_angles_to_accel(_accel_target.x, _accel_target.y);
    _pid_rate_lat.set_integrator(_accel_target.x);
    _pid_rate_lon.set_integrator(_accel_target.y);

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_rate_to_accel_xy = true;

    // set target position in xy axis
    const Vector3f& curr_pos = _inav.get_position();
    set_xy_target(curr_pos.x, curr_pos.y);

    // move current vehicle velocity into feed forward velocity
    const Vector3f& curr_vel = _inav.get_velocity();
    set_desired_velocity_xy(curr_vel.x, curr_vel.y);
}

/// update_velocity_controller_xyz - run the velocity controller - should be called at 100hz or higher
///     velocity targets should we set using set_desired_velocity_xyz() method
///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
///     throttle targets will be sent directly to the motors
void AC_PosControl::update_vel_controller_xyz(float ekfNavVelGainScaler)
{
    // capture time since last iteration
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _last_update_xy_ms) / 1000.0f;

    // sanity check dt - expect to be called faster than ~5hz
    if (dt >= POSCONTROL_ACTIVE_TIMEOUT_MS*1.0e-3f) {
        dt = 0.0f;
    }

    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    // apply desired velocity request to position target
    desired_vel_to_pos(dt);

    // run position controller's position error to desired velocity step
    pos_to_rate_xy(XY_MODE_POS_LIMITED_AND_VEL_FF, dt, ekfNavVelGainScaler);

    // run velocity to acceleration step
    rate_to_accel_xy(dt, ekfNavVelGainScaler);

    // run acceleration to lean angle step
    accel_to_lean_angles(dt, ekfNavVelGainScaler);

    // update altitude target
    set_alt_target_from_climb_rate(_vel_desired.z, dt);

    // run z-axis position controller
    update_z_controller();

    // record update time
    _last_update_xy_ms = now;
}

///
/// private methods
///

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration
///     should be called whenever the speed, acceleration or position kP is modified
void AC_PosControl::calc_leash_length_xy()
{
    if (_flags.recalc_leash_xy) {
        _leash = calc_leash_length(_speed_cms, _accel_cms, _p_pos_xy.kP());
        _flags.recalc_leash_xy = false;
    }
}

/// desired_vel_to_pos - move position target using desired velocities
void AC_PosControl::desired_vel_to_pos(float nav_dt)
{
    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    // update target position
    if (_flags.reset_desired_vel_to_pos) {
        _flags.reset_desired_vel_to_pos = false;
    } else {
        _pos_target.x += _vel_desired.x * nav_dt;
        _pos_target.y += _vel_desired.y * nav_dt;
    }
}

/// pos_to_rate_xy - horizontal position error to velocity controller
///     converts position (_pos_target) to target velocity (_vel_target)
///     when use_desired_rate is set to true:
///         desired velocity (_vel_desired) is combined into final target velocity and
///         velocity due to position error is reduce to a maximum of 1m/s
void AC_PosControl::pos_to_rate_xy(xy_mode mode, float dt, float ekfNavVelGainScaler)
{
    Vector3f curr_pos = _inav.get_position();
    float linear_distance;      // the distance we swap between linear and sqrt velocity response
    float kP = ekfNavVelGainScaler * _p_pos_xy.kP(); // scale gains to compensate for noisy optical flow measurement in the EKF

    // avoid divide by zero
    if (kP <= 0.0f) {
        _vel_target.x = 0.0f;
        _vel_target.y = 0.0f;
    }else{
        // calculate distance error
        _pos_error.x = _pos_target.x - curr_pos.x;
        _pos_error.y = _pos_target.y - curr_pos.y;

        // constrain target position to within reasonable distance of current location
        _distance_to_target = pythagorous2(_pos_error.x, _pos_error.y);
        if (_distance_to_target > _leash && _distance_to_target > 0.0f) {
            _pos_target.x = curr_pos.x + _leash * _pos_error.x/_distance_to_target;
            _pos_target.y = curr_pos.y + _leash * _pos_error.y/_distance_to_target;
            // re-calculate distance error
            _pos_error.x = _pos_target.x - curr_pos.x;
            _pos_error.y = _pos_target.y - curr_pos.y;
            _distance_to_target = _leash;
        }

        // calculate the distance at which we swap between linear and sqrt velocity response
        linear_distance = _accel_cms/(2.0f*kP*kP);

        if (_distance_to_target > 2.0f*linear_distance) {
            // velocity response grows with the square root of the distance
            float vel_sqrt = safe_sqrt(2.0f*_accel_cms*(_distance_to_target-linear_distance));
            _vel_target.x = vel_sqrt * _pos_error.x/_distance_to_target;
            _vel_target.y = vel_sqrt * _pos_error.y/_distance_to_target;
        }else{
            // velocity response grows linearly with the distance
            _vel_target.x = _p_pos_xy.kP() * _pos_error.x;
            _vel_target.y = _p_pos_xy.kP() * _pos_error.y;
        }

        if (mode == XY_MODE_POS_LIMITED_AND_VEL_FF) {
            // this mode is for loiter - rate-limiting the position correction
            // allows the pilot to always override the position correction in
            // the event of a disturbance

            // scale velocity within limit
            float vel_total = pythagorous2(_vel_target.x, _vel_target.y);
            if (vel_total > POSCONTROL_VEL_XY_MAX_FROM_POS_ERR) {
                _vel_target.x = POSCONTROL_VEL_XY_MAX_FROM_POS_ERR * _vel_target.x/vel_total;
                _vel_target.y = POSCONTROL_VEL_XY_MAX_FROM_POS_ERR * _vel_target.y/vel_total;
            }

            // add velocity feed-forward
            _vel_target.x += _vel_desired.x;
            _vel_target.y += _vel_desired.y;
        } else {
            if (mode == XY_MODE_POS_AND_VEL_FF) {
                // add velocity feed-forward
                _vel_target.x += _vel_desired.x;
                _vel_target.y += _vel_desired.y;
            }

            // scale velocity within speed limit
            float vel_total = pythagorous2(_vel_target.x, _vel_target.y);
            if (vel_total > _speed_cms) {
                _vel_target.x = _speed_cms * _vel_target.x/vel_total;
                _vel_target.y = _speed_cms * _vel_target.y/vel_total;
            }
        }
    }
}

/// rate_to_accel_xy - horizontal desired rate to desired acceleration
///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
void AC_PosControl::rate_to_accel_xy(float dt, float ekfNavVelGainScaler)
{
    const Vector3f &vel_curr = _inav.get_velocity();  // current velocity in cm/s
    float accel_total;                          // total acceleration in cm/s/s
    float lat_i, lon_i;

    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_xy) {
        _vel_last.x = _vel_target.x;
        _vel_last.y = _vel_target.y;
        _flags.reset_rate_to_accel_xy = false;
    }

    // feed forward desired acceleration calculation
    if (dt > 0.0f) {
    	if (!_flags.freeze_ff_xy) {
    		_accel_feedforward.x = (_vel_target.x - _vel_last.x)/dt;
    		_accel_feedforward.y = (_vel_target.y - _vel_last.y)/dt;
        } else {
    		// stop the feed forward being calculated during a known discontinuity
    		_flags.freeze_ff_xy = false;
    	}
    } else {
    	_accel_feedforward.x = 0.0f;
    	_accel_feedforward.y = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.x = _vel_target.x;
    _vel_last.y = _vel_target.y;

    // calculate velocity error
    _vel_error.x = _vel_target.x - vel_curr.x;
    _vel_error.y = _vel_target.y - vel_curr.y;

    // get current i term
    lat_i = _pid_rate_lat.get_integrator();
    lon_i = _pid_rate_lon.get_integrator();

    // update i term if we have not hit the accel or throttle limits OR the i term will reduce
    if ((!_limit.accel_xy && !_motors.limit.throttle_upper) || ((lat_i>0&&_vel_error.x<0)||(lat_i<0&&_vel_error.x>0))) {
        lat_i = _pid_rate_lat.get_i(_vel_error.x, dt);
    }
    if ((!_limit.accel_xy && !_motors.limit.throttle_upper) || ((lon_i>0&&_vel_error.y<0)||(lon_i<0&&_vel_error.y>0))) {
        lon_i = _pid_rate_lon.get_i(_vel_error.y, dt);
    }

    // combine feed forward accel with PID output from velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    _accel_target.x = _accel_feedforward.x + (_pid_rate_lat.get_p(_vel_error.x) + lat_i + _pid_rate_lat.get_d(_vel_error.x, dt)) * ekfNavVelGainScaler;
    _accel_target.y = _accel_feedforward.y + (_pid_rate_lon.get_p(_vel_error.y) + lon_i + _pid_rate_lon.get_d(_vel_error.y, dt)) * ekfNavVelGainScaler;

    // scale desired acceleration if it's beyond acceptable limit
    // To-Do: move this check down to the accel_to_lean_angle method?
    accel_total = pythagorous2(_accel_target.x, _accel_target.y);
    if (accel_total > POSCONTROL_ACCEL_XY_MAX && accel_total > 0.0f) {
        _accel_target.x = POSCONTROL_ACCEL_XY_MAX * _accel_target.x/accel_total;
        _accel_target.y = POSCONTROL_ACCEL_XY_MAX * _accel_target.y/accel_total;
        _limit.accel_xy = true;     // unused
    } else {
        // reset accel limit flag
        _limit.accel_xy = false;
    }
}

/// accel_to_lean_angles - horizontal desired acceleration to lean angles
///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_PosControl::accel_to_lean_angles(float dt, float ekfNavVelGainScaler)
{
    float accel_right, accel_forward;
    float lean_angle_max = _attitude_control.lean_angle_max();

    float max_delta_accel = dt * 1700.0f;

    Vector2f accel_in(_accel_target.x, _accel_target.y);
    Vector2f accel_change = accel_in-_accel_target_jerk_limited;
    float accel_change_length = accel_change.length();

    if(accel_change_length > max_delta_accel) {
        accel_change *= max_delta_accel/accel_change_length;
        _accel_target_jerk_limited += accel_change;
    }

    // 5Hz lowpass filter on NE accel
    float freq_cut = 5.0f * ekfNavVelGainScaler;
    float alpha = constrain_float(dt/(dt + 1.0f/(2.0f*(float)M_PI*freq_cut)),0.0f,1.0f);
    _accel_target_filtered.x += alpha * (_accel_target_jerk_limited.x - _accel_target_filtered.x);
    _accel_target_filtered.y  += alpha * (_accel_target_jerk_limited.y - _accel_target_filtered.y);

    // rotate accelerations into body forward-right frame
    accel_forward = _accel_target_filtered.x*_ahrs.cos_yaw() + _accel_target_filtered.y*_ahrs.sin_yaw();
    accel_right = -_accel_target_filtered.x*_ahrs.sin_yaw() + _accel_target_filtered.y*_ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    _pitch_target = constrain_float(fast_atan(-accel_forward/(GRAVITY_MSS * 100))*(18000/M_PI),-lean_angle_max, lean_angle_max);
    float cos_pitch_target = cosf(_pitch_target*(float)M_PI/18000);
    _roll_target = constrain_float(fast_atan(accel_right*cos_pitch_target/(GRAVITY_MSS * 100))*(18000/M_PI), -lean_angle_max, lean_angle_max);
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::lean_angles_to_accel(float& accel_x_cmss, float& accel_y_cmss) const
{
    // rotate our roll, pitch angles into lat/lon frame
    accel_x_cmss = (GRAVITY_MSS * 100) * (-(_ahrs.cos_yaw() * _ahrs.sin_pitch() / max(_ahrs.cos_pitch(),0.5f)) - _ahrs.sin_yaw() * _ahrs.sin_roll() / max(_ahrs.cos_roll(),0.5f));
    accel_y_cmss = (GRAVITY_MSS * 100) * (-(_ahrs.sin_yaw() * _ahrs.sin_pitch() / max(_ahrs.cos_pitch(),0.5f)) + _ahrs.cos_yaw() * _ahrs.sin_roll() / max(_ahrs.cos_roll(),0.5f));
}

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
float AC_PosControl::calc_leash_length(float speed_cms, float accel_cms, float kP) const
{
    float leash_length;

    // sanity check acceleration and avoid divide by zero
    if (accel_cms <= 0.0f) {
        accel_cms = POSCONTROL_ACCELERATION_MIN;
    }

    // avoid divide by zero
    if (kP <= 0.0f) {
        return POSCONTROL_LEASH_LENGTH_MIN;
    }

    // calculate leash length
    if(speed_cms <= accel_cms / kP) {
        // linear leash length based on speed close in
        leash_length = speed_cms / kP;
    }else{
        // leash length grows at sqrt of speed further out
        leash_length = (accel_cms / (2.0f*kP*kP)) + (speed_cms*speed_cms / (2.0f*accel_cms));
    }

    // ensure leash is at least 1m long
    if( leash_length < POSCONTROL_LEASH_LENGTH_MIN ) {
        leash_length = POSCONTROL_LEASH_LENGTH_MIN;
    }

    return leash_length;
}

// set the height controller to use method that relies more on forward path velocity and less on height error
// this reduces the sensitivity to baro noise and transient errors in ground effect
void AC_PosControl::setGndEffectMode(bool gndEffectMode)
{
    if(_gnd_effect_mode != gndEffectMode) {
        if(gndEffectMode) {
            // turning feedforward on, initialize
            float curr_alt = _inav.get_altitude();
            _gnd_effect_pos_target_z = curr_alt;
            update_gnd_effect_targets(0.0f);
        } else {
            // turning feedforward off - adjust position target such that velocity demand stays constant:
            // sqrt_controller_gnd(err_gnd)+vel_desired = sqrt_controller_air(err_air)
            // find err_gnd such that this equality is met:
            // err_air = sqrt_controller_air_inv(sqrt_controller_gnd(err_gnd)+vel_desired)
            float curr_alt = _inav.get_altitude();
            float prev_fb_vel_target = AC_AttitudeControl::sqrt_controller(_gnd_effect_pos_error_z, _p_alt_pos.kP()*POSCONTROL_GNDEFFECT_GAIN, _accel_z_cms);
            float new_pos_error = AC_AttitudeControl::inverse_sqrt_controller(prev_fb_vel_target+_gnd_effect_vel_desired_z, _p_alt_pos.kP(), _accel_z_cms);
            _pos_target.z = new_pos_error + curr_alt;
        }
    }
    _gnd_effect_mode = gndEffectMode;
}
