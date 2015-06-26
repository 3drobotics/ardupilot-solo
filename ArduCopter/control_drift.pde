/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_drift.pde - init and run calls for drift flight mode
 */

#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 8.0f
#endif
#ifndef DRIFT_SPEEDLIMIT
 # define DRIFT_SPEEDLIMIT 560.0f
#endif

#ifndef DRIFT_THR_ASSIST_GAIN
 # define DRIFT_THR_ASSIST_GAIN 1.8f    // gain controlling amount of throttle assistance
#endif

#ifndef DRIFT_THR_ASSIST_MAX
 # define DRIFT_THR_ASSIST_MAX  300.0f  // maximum assistance throttle assist will provide
#endif

#ifndef DRIFT_THR_MIN
 # define DRIFT_THR_MIN         213     // throttle assist will be active when pilot's throttle is above this value
#endif
#ifndef DRIFT_THR_MAX
 # define DRIFT_THR_MAX         787     // throttle assist will be active when pilot's throttle is below this value
#endif

// drift_init - initialise drift controller
static bool drift_init(bool ignore_checks)
{
    if (!ignore_checks && failsafe.gps_glitch) {
        return false;
    }

    if (position_ok() || ignore_checks) {
        return true;
    }else{
        return false;
    }
}

// drift_run - runs the drift controller
// should be called at 100hz or more
static void drift_run()
{
    static float breaker = 0.0f;
    static float roll_input = 0.0f;
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or landed and throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || (ap.land_complete && ap.throttle_zero)) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // Grab inertial velocity
    const Vector3f& vel = inertial_nav.get_velocity();

    // rotate roll, pitch input from north facing to vehicle's perspective
    float roll_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw(); // body roll vel
    float pitch_vel = vel.y * ahrs.sin_yaw() + vel.x * ahrs.cos_yaw(); // body pitch vel

    // gain sceduling for Yaw
    float pitch_vel2 = min(fabs(pitch_vel), 2000);
    target_yaw_rate = ((float)target_roll/1.0f) * (1.0f - (pitch_vel2 / 5000.0f)) * g.acro_yaw_p;

    roll_vel = constrain_float(roll_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    pitch_vel = constrain_float(pitch_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    
    roll_input = roll_input * .96f + (float)g.rc_4.control_in * .04f;

    //convert user input into desired roll velocity
    float roll_vel_error = roll_vel - (roll_input / DRIFT_SPEEDGAIN);

    // Roll velocity is feed into roll acceleration to minimize slip
    target_roll = roll_vel_error * -DRIFT_SPEEDGAIN;
    target_roll = constrain_int16(target_roll, -4500, 4500);

    // If we let go of sticks, bring us to a stop
    if(target_pitch == 0){
        // .14/ (.03 * 100) = 4.6 seconds till full breaking
        breaker += .03f;
        breaker = min(breaker, DRIFT_SPEEDGAIN);
        target_pitch = pitch_vel * breaker;
    }else{
        breaker = 0.0f;
    }

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // output pilot's throttle with angle boost
    attitude_control.set_throttle_out(get_throttle_assist(vel.z, pilot_throttle_scaled), true, g.throttle_filt);
}

// get_throttle_assist - return throttle output (range 0 ~ 1000) based on pilot input and z-axis velocity
int16_t get_throttle_assist(float velz, int16_t pilot_throttle_scaled)
{
    // throttle assist - adjusts throttle to slow the vehicle's vertical velocity
    //      Only active when pilot's throttle is between 213 ~ 787
    //      Assistance is strongest when throttle is at mid, drops linearly to no assistance at 213 and 787
    float thr_assist = 0.0f;
    if (pilot_throttle_scaled > g.throttle_min && pilot_throttle_scaled < THR_MAX &&
        pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX) {
        // calculate throttle assist gain
        thr_assist = 1.2f - ((float)abs(pilot_throttle_scaled - 500) / 240.0f);
        thr_assist = constrain_float(thr_assist, 0.0f, 1.0f) * -DRIFT_THR_ASSIST_GAIN * velz;

        // ensure throttle assist never adjusts the throttle by more than 300 pwm
        thr_assist = constrain_float(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);

        // ensure throttle assist never pushes throttle below throttle_min or above throttle_max
        thr_assist = constrain_float(thr_assist, g.throttle_min - pilot_throttle_scaled, THR_MAX - pilot_throttle_scaled);
    }
    
    return pilot_throttle_scaled + thr_assist;
}
