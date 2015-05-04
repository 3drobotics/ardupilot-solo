/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if AUTOTUNE_ENABLED == ENABLED

/*
 * control_autotune.pde - init and run calls for autotune flight mode
 *
 * Instructions:
 *      1) Set up one flight mode switch position to be AltHold.
 *      2) Set the Ch7 Opt or Ch8 Opt to AutoTune to allow you to turn the auto tuning on/off with the ch7 or ch8 switch.
 *      3) Ensure the ch7 or ch8 switch is in the LOW position.
 *      4) Wait for a calm day and go to a large open area.
 *      5) Take off and put the vehicle into AltHold mode at a comfortable altitude.
 *      6) Set the ch7/ch8 switch to the HIGH position to engage auto tuning:
 *          a) You will see it twitch about 20 degrees left and right for a few minutes, then it will repeat forward and back.
 *          b) Use the roll and pitch stick at any time to reposition the copter if it drifts away (it will use the original PID gains during repositioning and between tests).
 *             When you release the sticks it will continue auto tuning where it left off.
 *          c) Move the ch7/ch8 switch into the LOW position at any time to abandon the autotuning and return to the origin PIDs.
 *          d) Make sure that you do not have any trim set on your transmitter or the autotune may not get the signal that the sticks are centered.
 *      7) When the tune completes the vehicle will change back to the original PID gains.
 *      8) Put the ch7/ch8 switch into the LOW position then back to the HIGH position to test the tuned PID gains.
 *      9) Put the ch7/ch8 switch into the LOW position to fly using the original PID gains.
 *      10) If you are happy with the autotuned PID gains, leave the ch7/ch8 switch in the HIGH position, land and disarm to save the PIDs permanently.
 *          If you DO NOT like the new PIDS, switch ch7/ch8 LOW to return to the original PIDs. The gains will not be saved when you disarm
 *
 * What it's doing during each "twitch":
 *      a) invokes 90 deg/sec rate request
 *      b) records maximum "forward" roll rate and bounce back rate
 *      c) when copter reaches 20 degrees or 1 second has passed, it commands level
 *      d) tries to keep max rotation rate between 80% ~ 100% of requested rate (90deg/sec) by adjusting rate P
 *      e) increases rate D until the bounce back becomes greater than 10% of requested rate (90deg/sec)
 *      f) decreases rate D until the bounce back becomes less than 10% of requested rate (90deg/sec)
 *      g) increases rate P until the max rotate rate becomes greater than the request rate (90deg/sec)
 *      h) invokes a 20deg angle request on roll or pitch
 *      i) increases stab P until the maximum angle becomes greater than 110% of the requested angle (20deg)
 *      j) decreases stab P by 25%
 *
 * Notes: AUTOTUNE should not be set-up as a flight mode, it should be invoked only from the ch7/ch8 switch.
 *
 */

#define AUTOTUNE_AXIS_BITMASK_ROLL            1
#define AUTOTUNE_AXIS_BITMASK_PITCH           2
#define AUTOTUNE_AXIS_BITMASK_YAW             4

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500     // restart tuning if pilot has left sticks in middle for 2 seconds
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS    500     // timeout for tuning mode's testing step
#define AUTOTUNE_LEVEL_ANGLE_CD             300     // angle which qualifies as level
#define AUTOTUNE_LEVEL_RATE_RP_CD          1000     // rate which qualifies as level for roll and pitch
#define AUTOTUNE_LEVEL_RATE_Y_CD            750     // rate which qualifies as level for yaw
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     500     // time we require the copter to be level
#define AUTOTUNE_RD_STEP                  0.05f     // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                  0.05f     // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                  0.05f     // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f     // I is set 10x smaller than P during testing
#define AUTOTUNE_PI_RATIO_FINAL            1.0f     // I is set 1x P after testing
#define AUTOTUNE_YAW_PI_RATIO_FINAL        0.1f     // I is set 1x P after testing
#define AUTOTUNE_RD_MIN                  0.004f     // minimum Rate D value
#define AUTOTUNE_RD_MAX                  0.050f     // maximum Rate D value
#define AUTOTUNE_RLPF_MIN                  1.0f     // minimum Rate Yaw filter value
#define AUTOTUNE_RLPF_MAX                 10.0f     // maximum Rate Yaw filter value
#define AUTOTUNE_RP_MIN                   0.01f     // minimum Rate P value
#define AUTOTUNE_RP_MAX                    2.0f     // maximum Rate P value
#define AUTOTUNE_SP_MAX                   20.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    0.5f     // maximum Stab P value
#define AUTOTUNE_RP_ACCEL_MIN          36000.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN            9000.0f     // Minimum acceleration for Yaw
#define AUTOTUNE_Y_FILT_FREQ              10.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_SUCCESS_COUNT                4     // how many successful iterations we need to freeze at current gains
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2f     // The margin below the target that we tune D in
#define AUTOTUNE_RD_BACKOFF                1.0f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RP_BACKOFF                1.0f     // Rate P gains are reduced to 97.5% of their maximum value discovered during tuning
#define AUTOTUNE_SP_BACKOFF                1.0f     // Stab P gains are reduced to 60% of their maximum value discovered during tuning
#define AUTOTUNE_ACCEL_RP_BACKOFF          1.0f     // back off from maximum acceleration
#define AUTOTUNE_ACCEL_Y_BACKOFF          0.75f     // back off from maximum acceleration

// roll and pitch axes
#define AUTOTUNE_TARGET_ANGLE_RLLPIT_CD     2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_RLLPIT_CDS     9000    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD 1000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS 4500    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step

// yaw axis
#define AUTOTUNE_TARGET_ANGLE_YAW_CD        1000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_YAW_CDS        3000    // target yaw rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD     500    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_YAW_CDS    1500    // target yaw rate during AUTOTUNE_STEP_TWITCHING step

// Auto Tune message ids for ground station
#define AUTOTUNE_MESSAGE_STARTED 0
#define AUTOTUNE_MESSAGE_STOPPED 1
#define AUTOTUNE_MESSAGE_SUCCESS 2
#define AUTOTUNE_MESSAGE_FAILED 3
#define AUTOTUNE_MESSAGE_SAVED_GAINS 4

// autotune modes (high level states)
enum AutoTuneTuneMode {
    AUTOTUNE_MODE_UNINITIALISED = 0,        // autotune has never been run
    AUTOTUNE_MODE_TUNING = 1,               // autotune is testing gains
    AUTOTUNE_MODE_SUCCESS = 2,              // tuning has completed, user is flight testing the new gains
    AUTOTUNE_MODE_FAILED = 3,               // tuning has failed, user is flying on original gains
};

// steps performed while in the tuning mode
enum AutoTuneStepType {
    AUTOTUNE_STEP_WAITING_FOR_LEVEL = 0,    // autotune is waiting for vehicle to return to level before beginning the next twitch
    AUTOTUNE_STEP_TWITCHING = 1,            // autotune has begun a twitch and is watching the resulting vehicle movement
    AUTOTUNE_STEP_UPDATE_GAINS = 2          // autotune has completed a twitch and is updating the gains based on the results
};

// things that can be tuned
enum AutoTuneAxisType {
    AUTOTUNE_AXIS_ROLL = 0,                 // roll axis is being tuned (either angle or rate)
    AUTOTUNE_AXIS_PITCH = 1,                // pitch axis is being tuned (either angle or rate)
    AUTOTUNE_AXIS_YAW = 2,                  // pitch axis is being tuned (either angle or rate)
};

// mini steps performed while in Tuning mode, Testing step
enum AutoTuneTuneType {
    AUTOTUNE_TYPE_RD_UP = 0,                // rate D is being tuned up
    AUTOTUNE_TYPE_RD_DOWN = 1,              // rate D is being tuned down
    AUTOTUNE_TYPE_RP_UP = 2,                // rate P is being tuned up
    AUTOTUNE_TYPE_SP_DOWN = 3,              // angle P is being tuned up
    AUTOTUNE_TYPE_SP_UP = 4                 // angle P is being tuned up
};

// autotune_state_struct - hold state flags
struct autotune_state_struct {
    AutoTuneTuneMode    mode                : 2;    // see AutoTuneTuneMode for what modes are allowed
    uint8_t             pilot_override      : 1;    // 1 = pilot is overriding controls so we suspend tuning temporarily
    AutoTuneAxisType    axis                : 2;    // see AutoTuneAxisType for which things can be tuned
    uint8_t             positive_direction  : 1;    // 0 = tuning in negative direction (i.e. left for roll), 1 = positive direction (i.e. right for roll)
    AutoTuneStepType    step                : 2;    // see AutoTuneStepType for what steps are performed
    AutoTuneTuneType    tune_type           : 3;    // see AutoTuneTuneType
    uint8_t             ignore_next         : 1;    // 1 = ignore the next test
} autotune_state;

// variables
static uint32_t autotune_override_time;                         // the last time the pilot overrode the controls
static float    autotune_test_min;                              // the minimum angular rate achieved during TESTING_RATE step
static float    autotune_test_max;                              // the maximum angular rate achieved during TESTING_RATE step
static uint32_t autotune_step_start_time;                       // start time of current tuning step (used for timeout checks)
static uint32_t autotune_step_stop_time;                        // start time of current tuning step (used for timeout checks)
static int8_t   autotune_counter;                               // counter for tuning gains
static float    autotune_target_rate, autotune_start_rate;      // target and start rate
static float    autotune_target_angle, autotune_start_angle;    // target and start angles
static float    autotune_desired_yaw;                           // yaw heading during tune
static float    rate_max, autotune_test_accel_max;              // maximum acceleration variables

LowPassFilterFloat  rotation_rate_filt;                         // filtered rotation rate in radians/second

// backup of currently being tuned parameter values
static float    orig_roll_rp = 0, orig_roll_ri, orig_roll_rd, orig_roll_sp;
static float    orig_pitch_rp = 0, orig_pitch_ri, orig_pitch_rd, orig_pitch_sp;
static float    orig_yaw_rp = 0, orig_yaw_ri, orig_yaw_rd, orig_yaw_rLPF, orig_yaw_sp;

// currently being tuned parameter values
static float    tune_roll_rp, tune_roll_rd, tune_roll_sp, tune_roll_accel;
static float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel;
static float    tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, tune_yaw_accel;

// autotune_init - should be called when autotune mode is selected
static bool autotune_init(bool ignore_checks)
{
    bool success = true;

    switch (autotune_state.mode) {
        case AUTOTUNE_MODE_FAILED:
        // autotune has been run but failed so reset state to uninitialized
            autotune_state.mode = AUTOTUNE_MODE_UNINITIALISED;
            // no break to allow fall through to restart the tuning

        case AUTOTUNE_MODE_UNINITIALISED:
            // autotune has never been run
            success = autotune_start(false);
            if (success) {
                // so store current gains as original gains
                autotune_backup_gains_and_initialise();
                // advance mode to tuning
                autotune_state.mode = AUTOTUNE_MODE_TUNING;
                // send message to ground station that we've started tuning
                autotune_update_gcs(AUTOTUNE_MESSAGE_STARTED);
            }
            break;

        case AUTOTUNE_MODE_TUNING:
            // we are restarting tuning after the user must have switched ch7/ch8 off so we restart tuning where we left off
            success = autotune_start(false);
            if (success) {
                // reset gains to tuning-start gains (i.e. low I term)
                autotune_load_intra_test_gains();
                // write dataflash log even and send message to ground station
                Log_Write_Event(DATA_AUTOTUNE_RESTART);
                autotune_update_gcs(AUTOTUNE_MESSAGE_STARTED);
            }
            break;

        case AUTOTUNE_MODE_SUCCESS:
            // we have completed a tune and the pilot wishes to test the new gains in the current flight mode
            // so simply apply tuning gains (i.e. do not change flight mode)
            autotune_load_tuned_gains();
            Log_Write_Event(DATA_AUTOTUNE_PILOT_TESTING);
            break;
    }

    return success;
}

// autotune_stop - should be called when the ch7/ch8 switch is switched OFF
static void autotune_stop()
{
    // set gains to their original values
    autotune_load_orig_gains();

    // re-enable angle-to-rate request limits
    attitude_control.limit_angle_to_rate_request(true);

    // log off event and send message to ground station
    autotune_update_gcs(AUTOTUNE_MESSAGE_STOPPED);
    Log_Write_Event(DATA_AUTOTUNE_OFF);

    // Note: we leave the autotune_state.mode as it was so that we know how the autotune ended
    // we expect the caller will change the flight mode back to the flight mode indicated by the flight mode switch
}

// autotune_start - Initialize autotune flight mode
static bool autotune_start(bool ignore_checks)
{
    // only allow flip from Stabilize or AltHold flight modes
    if (control_mode != STABILIZE && control_mode != ALT_HOLD) {
        return false;
    }

    // ensure throttle is above zero
    if (g.rc_3.control_in <= 0) {
        return false;
    }

    // ensure we are flying
    if (!motors.armed() || !ap.auto_armed || ap.land_complete) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    return true;
}

// autotune_run - runs the autotune flight mode
// should be called at 100hz or more
static void autotune_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t target_climb_rate;

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // if not auto armed set throttle to zero and exit immediately
    // this should not actually be possible because of the autotune_init() checks
    if (!ap.auto_armed) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(g.rc_3.control_in)-throttle_average);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

    // check for pilot requested take-off - this should not actually be possible because of autotune_init() checks
    if (ap.land_complete && target_climb_rate > 0) {
        // indicate we are taking off
        set_land_complete(false);
        // clear i term when we're taking off
        set_throttle_takeoff();
    }

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out_unstabilized(get_throttle_pre_takeoff(g.rc_3.control_in),true,g.throttle_filt);
        pos_control.set_alt_target_to_current_alt();
    }else{
        // check if pilot is overriding the controls
        if (target_roll != 0 || target_pitch != 0 || target_yaw_rate != 0.0f || target_climb_rate != 0) {
            if (!autotune_state.pilot_override) {
                autotune_state.pilot_override = true;
                // set gains to their original values
                autotune_load_orig_gains();
                attitude_control.limit_angle_to_rate_request(true);
            }
            // reset pilot override time
            autotune_override_time = millis();
        }else if (autotune_state.pilot_override) {
            // check if we should resume tuning after pilot's override
            if (millis() - autotune_override_time > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
                autotune_state.pilot_override = false;             // turn off pilot override
                // set gains to their intra-test values (which are very close to the original gains)
                // autotune_load_intra_test_gains(); //I think we should be keeping the originals here to let the I term settle quickly
                autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL; // set tuning step back from beginning
                autotune_desired_yaw = ahrs.yaw_sensor;
            }
        }

        // if pilot override call attitude controller
        if (autotune_state.pilot_override || autotune_state.mode != AUTOTUNE_MODE_TUNING) {
            attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        }else{
            // somehow get attitude requests from autotuning
            autotune_attitude_control();
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

// autotune_attitude_controller - sets attitude control targets during tuning
static void autotune_attitude_control()
{
    float rotation_rate = 0.0f;        // rotation rate in radians/second
    float lean_angle = 0.0f;
    const float direction_sign = autotune_state.positive_direction ? 1.0 : -1.0;

    // check tuning step
    switch (autotune_state.step) {

    case AUTOTUNE_STEP_WAITING_FOR_LEVEL:
        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)
        // re-enable rate limits
        attitude_control.limit_angle_to_rate_request(true);

        // hold level attitude
        attitude_control.angle_ef_roll_pitch_yaw( 0.0f, 0.0f, autotune_desired_yaw, true);

        // hold the copter level for 0.5 seconds before we begin a twitch
        // reset counter if we are no longer level
        if ((labs(ahrs.roll_sensor) > AUTOTUNE_LEVEL_ANGLE_CD) ||
                (labs(ahrs.pitch_sensor) > AUTOTUNE_LEVEL_ANGLE_CD) ||
                (labs(wrap_180_cd(ahrs.yaw_sensor-(int32_t)autotune_desired_yaw)) > AUTOTUNE_LEVEL_ANGLE_CD) ||
                ((ToDeg(ahrs.get_gyro().x) * 100.0f) > AUTOTUNE_LEVEL_RATE_RP_CD) ||
                ((ToDeg(ahrs.get_gyro().y) * 100.0f) > AUTOTUNE_LEVEL_RATE_RP_CD) ||
                ((ToDeg(ahrs.get_gyro().z) * 100.0f) > AUTOTUNE_LEVEL_RATE_Y_CD) ) {
            autotune_step_start_time = millis();
        }

        // if we have been level for a sufficient amount of time (0.5 seconds) move onto tuning step
        if (millis() - autotune_step_start_time >= AUTOTUNE_REQUIRED_LEVEL_TIME_MS) {
            // initiate variables for next step
            autotune_state.step = AUTOTUNE_STEP_TWITCHING;
            autotune_step_start_time = millis();
            autotune_step_stop_time = autotune_step_start_time + AUTOTUNE_TESTING_STEP_TIMEOUT_MS;
            autotune_test_max = 0.0f;
            autotune_test_min = 0.0f;
            rotation_rate_filt.reset(0.0f);
            rate_max = 0.0f;
            // set gains to their to-be-tested values
            autotune_load_twitch_gains();
        }

        switch (autotune_state.axis) {
        case AUTOTUNE_AXIS_ROLL:
            autotune_target_rate = constrain_float(attitude_control.max_rate_step_bf_roll(), AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            autotune_target_angle = constrain_float(attitude_control.max_angle_step_bf_roll(), AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
            autotune_start_rate = ToDeg(ahrs.get_gyro().x) * 100.0f;
            autotune_start_angle = ahrs.roll_sensor;
            rotation_rate_filt.set_cutoff_frequency(g.pid_rate_roll.filt_hz()*2.0f);
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate_filt.reset(autotune_start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
        break;
        case AUTOTUNE_AXIS_PITCH:
            autotune_target_rate = constrain_float(attitude_control.max_rate_step_bf_pitch(), AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            autotune_target_angle = constrain_float(attitude_control.max_angle_step_bf_pitch(), AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
            autotune_start_rate = ToDeg(ahrs.get_gyro().y) * 100.0f;
            autotune_start_angle = ahrs.pitch_sensor;
            rotation_rate_filt.set_cutoff_frequency(g.pid_rate_pitch.filt_hz()*2.0f);
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate_filt.reset(autotune_start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
            break;
        case AUTOTUNE_AXIS_YAW:
            autotune_target_rate = constrain_float(attitude_control.max_rate_step_bf_yaw()/1.5f, AUTOTUNE_TARGET_MIN_RATE_YAW_CDS, AUTOTUNE_TARGET_RATE_YAW_CDS);
            autotune_target_angle = constrain_float(attitude_control.max_angle_step_bf_yaw(), AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD, AUTOTUNE_TARGET_ANGLE_YAW_CD);
            autotune_start_rate = ToDeg(ahrs.get_gyro().z) * 100.0f;
            autotune_start_angle = ahrs.yaw_sensor;
            rotation_rate_filt.set_cutoff_frequency(AUTOTUNE_Y_FILT_FREQ);
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate_filt.reset(autotune_start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
            break;
        }
        break;

    case AUTOTUNE_STEP_TWITCHING:
        // Run the twitching step
        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)

        // disable rate limits
        attitude_control.limit_angle_to_rate_request(false);

        if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
            // Testing increasing stabilize P gain so will set lean angle target
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                // request roll to 20deg
                attitude_control.angle_ef_roll_pitch_rate_ef_yaw( direction_sign * autotune_target_angle + autotune_start_angle, 0.0f, 0.0f);
                break;
            case AUTOTUNE_AXIS_PITCH:
                // request pitch to 20deg
                attitude_control.angle_ef_roll_pitch_rate_ef_yaw( 0.0f, direction_sign * autotune_target_angle + autotune_start_angle, 0.0f);
                break;
            case AUTOTUNE_AXIS_YAW:
                // request pitch to 20deg
                attitude_control.angle_ef_roll_pitch_yaw( 0.0f, 0.0f, wrap_180_cd_float(direction_sign * autotune_target_angle + autotune_start_angle), false);
                break;
            }
        } else {
            // Testing rate P and D gains so will set body-frame rate targets.
            // Rate controller will use existing body-frame rates and convert to motor outputs
            // for all axes except the one we override here.
            attitude_control.angle_ef_roll_pitch_rate_ef_yaw( 0.0f, 0.0f, 0.0f);
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                // override body-frame roll rate
                attitude_control.rate_bf_roll_target(direction_sign * autotune_target_rate + autotune_start_rate);
                break;
            case AUTOTUNE_AXIS_PITCH:
                // override body-frame pitch rate
                attitude_control.rate_bf_pitch_target(direction_sign * autotune_target_rate + autotune_start_rate);
                break;
            case AUTOTUNE_AXIS_YAW:
                // override body-frame yaw rate
                attitude_control.rate_bf_yaw_target(direction_sign * autotune_target_rate + autotune_start_rate);
                break;
            }
        }

        // capture this iterations rotation rate and lean angle
        // Add filter to measurements
        switch (autotune_state.axis) {
        case AUTOTUNE_AXIS_ROLL:
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().x) * 100.0f), MAIN_LOOP_SECONDS);
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().x) * 100.0f - autotune_start_rate), MAIN_LOOP_SECONDS);
            }
            lean_angle = direction_sign * (ahrs.roll_sensor - (int32_t)autotune_start_angle);
            break;
        case AUTOTUNE_AXIS_PITCH:
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().y) * 100.0f), MAIN_LOOP_SECONDS);
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().y) * 100.0f - autotune_start_rate), MAIN_LOOP_SECONDS);
            }
            lean_angle = direction_sign * (ahrs.pitch_sensor - (int32_t)autotune_start_angle);
            break;
        case AUTOTUNE_AXIS_YAW:
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().z) * 100.0f), MAIN_LOOP_SECONDS);
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().z) * 100.0f - autotune_start_rate), MAIN_LOOP_SECONDS);
            }
            lean_angle = direction_sign * wrap_180_cd(ahrs.yaw_sensor-(int32_t)autotune_start_angle);
            break;
        }

        switch (autotune_state.tune_type) {
        case AUTOTUNE_TYPE_RD_UP:
        case AUTOTUNE_TYPE_RD_DOWN:
            autotune_twitching_test(rotation_rate, autotune_target_rate, autotune_test_min, autotune_test_max);
            autotune_twitching_measure_acceleration(autotune_test_accel_max, rotation_rate, rate_max);
            if (lean_angle >= autotune_target_angle) {
                autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
            }
            break;
        case AUTOTUNE_TYPE_RP_UP:
            autotune_twitching_test(rotation_rate, autotune_target_rate*(1+0.5f*g.autotune_aggressiveness), autotune_test_min, autotune_test_max);
            autotune_twitching_measure_acceleration(autotune_test_accel_max, rotation_rate, rate_max);
            if (lean_angle >= autotune_target_angle) {
                autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
            }
            break;
        case AUTOTUNE_TYPE_SP_DOWN:
        case AUTOTUNE_TYPE_SP_UP:
            autotune_twitching_test(lean_angle, autotune_target_angle, autotune_test_min, autotune_test_max);
            autotune_twitching_measure_acceleration(autotune_test_accel_max, rotation_rate - direction_sign * autotune_start_rate, rate_max);
            break;
        }

        // log this iterations lean angle and rotation rate
        Log_Write_AutoTuneDetails(lean_angle, rotation_rate);
        Log_Write_Rate();
        break;

    case AUTOTUNE_STEP_UPDATE_GAINS:

        // re-enable rate limits
        attitude_control.limit_angle_to_rate_request(true);

        // log the latest gains
        if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_angle, autotune_test_min, autotune_test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp);
                break;
            case AUTOTUNE_AXIS_PITCH:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_angle, autotune_test_min, autotune_test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp);
                break;
            case AUTOTUNE_AXIS_YAW:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_angle, autotune_test_min, autotune_test_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp);
                break;
            }
        } else {
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_rate, autotune_test_min, autotune_test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp);
                break;
            case AUTOTUNE_AXIS_PITCH:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_rate, autotune_test_min, autotune_test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp);
                break;
            case AUTOTUNE_AXIS_YAW:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_rate, autotune_test_min, autotune_test_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp);
                break;
            }
        }

        // Check results after mini-step to increase rate D gain
        switch (autotune_state.tune_type) {
        case AUTOTUNE_TYPE_RD_UP:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_d_up(tune_roll_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_d_up(tune_pitch_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_d_up(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RLPF_MAX, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to decrease rate D gain
        case AUTOTUNE_TYPE_RD_DOWN:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_d_down(tune_roll_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_d_down(tune_pitch_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to increase rate P gain
        case AUTOTUNE_TYPE_RP_UP:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_p_up_d_down(tune_roll_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate*(1+0.5f*g.autotune_aggressiveness), autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_p_up_d_down(tune_pitch_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate*(1+0.5f*g.autotune_aggressiveness), autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_p_up_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate*(1+0.5f*g.autotune_aggressiveness), autotune_test_min, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to increase stabilize P gain
        case AUTOTUNE_TYPE_SP_DOWN:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_p_down(tune_roll_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_p_down(tune_pitch_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_p_down(tune_yaw_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to increase stabilize P gain
        case AUTOTUNE_TYPE_SP_UP:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_p_up(tune_roll_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_p_up(tune_pitch_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_p_up(tune_yaw_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            }
            break;
        }

        // we've complete this step, finalize pids and move to next step
        if (autotune_counter >= AUTOTUNE_SUCCESS_COUNT) {

            // reset counter
            autotune_counter = 0;

            // move to the next tuning type
            switch (autotune_state.tune_type) {
            case AUTOTUNE_TYPE_RD_UP:
                autotune_state.tune_type++;
                break;
            case AUTOTUNE_TYPE_RD_DOWN:
                autotune_state.tune_type++;
                switch (autotune_state.axis) {
                case AUTOTUNE_AXIS_ROLL:
                    tune_roll_rd = max(AUTOTUNE_RD_MIN, tune_roll_rd * AUTOTUNE_RD_BACKOFF);
                    tune_roll_rp = max(AUTOTUNE_RP_MIN, tune_roll_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                case AUTOTUNE_AXIS_PITCH:
                    tune_pitch_rd = max(AUTOTUNE_RD_MIN, tune_pitch_rd * AUTOTUNE_RD_BACKOFF);
                    tune_pitch_rp = max(AUTOTUNE_RP_MIN, tune_pitch_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                case AUTOTUNE_AXIS_YAW:
                    tune_yaw_rLPF = max(AUTOTUNE_RLPF_MIN, tune_yaw_rLPF * AUTOTUNE_RD_BACKOFF);
                    tune_yaw_rp = max(AUTOTUNE_RP_MIN, tune_yaw_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                }
                break;
            case AUTOTUNE_TYPE_RP_UP:
                autotune_state.tune_type++;
                switch (autotune_state.axis) {
                case AUTOTUNE_AXIS_ROLL:
                    tune_roll_rp = max(AUTOTUNE_RP_MIN, tune_roll_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                case AUTOTUNE_AXIS_PITCH:
                    tune_pitch_rp = max(AUTOTUNE_RP_MIN, tune_pitch_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                case AUTOTUNE_AXIS_YAW:
                    tune_yaw_rp = max(AUTOTUNE_RP_MIN, tune_yaw_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                }
                break;
            case AUTOTUNE_TYPE_SP_DOWN:
                autotune_state.tune_type++;
                break;
            case AUTOTUNE_TYPE_SP_UP:
                // we've reached the end of a D-up-down PI-up-down tune type cycle
                autotune_state.tune_type = AUTOTUNE_TYPE_RD_UP;

                // advance to the next axis
                bool autotune_complete = false;
                switch (autotune_state.axis) {
                case AUTOTUNE_AXIS_ROLL:
                    tune_roll_sp = max(AUTOTUNE_SP_MIN, tune_roll_sp * AUTOTUNE_SP_BACKOFF);
                    tune_roll_accel = max(AUTOTUNE_RP_ACCEL_MIN, autotune_test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    if (autotune_pitch_enabled()) {
                        autotune_state.axis = AUTOTUNE_AXIS_PITCH;
                    } else if (autotune_yaw_enabled()) {
                        autotune_state.axis = AUTOTUNE_AXIS_YAW;
                    } else {
                        autotune_complete = true;
                    }
                    break;
                case AUTOTUNE_AXIS_PITCH:
                    tune_pitch_sp = max(AUTOTUNE_SP_MIN, tune_pitch_sp * AUTOTUNE_SP_BACKOFF);
                    tune_pitch_accel = max(AUTOTUNE_RP_ACCEL_MIN, autotune_test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    if (autotune_yaw_enabled()) {
                        autotune_state.axis = AUTOTUNE_AXIS_YAW;
                    } else {
                        autotune_complete = true;
                    }
                    break;
                case AUTOTUNE_AXIS_YAW:
                    tune_yaw_sp = max(AUTOTUNE_SP_MIN, tune_yaw_sp * AUTOTUNE_SP_BACKOFF);
                    tune_yaw_accel = max(AUTOTUNE_Y_ACCEL_MIN, autotune_test_accel_max * AUTOTUNE_ACCEL_Y_BACKOFF);
                    autotune_complete = true;
                    break;
                }

                // if we've just completed all axes we have successfully completed the autotune
                    // change to TESTING mode to allow user to fly with new gains
                if (autotune_complete) {
                    autotune_state.mode = AUTOTUNE_MODE_SUCCESS;
                    autotune_update_gcs(AUTOTUNE_MESSAGE_SUCCESS);
                    Log_Write_Event(DATA_AUTOTUNE_SUCCESS);
                    AP_Notify::events.autotune_complete = 1;
                } else {
                    AP_Notify::events.autotune_next_axis = 1;
                }
                break;
            }
        }

        // reverse direction
        autotune_state.positive_direction = !autotune_state.positive_direction;

        if (autotune_state.axis == AUTOTUNE_AXIS_YAW) {
            attitude_control.angle_ef_roll_pitch_yaw( 0.0f, 0.0f, ahrs.yaw_sensor, false);
        }

        // set gains to their intra-test values (which are very close to the original gains)
        autotune_load_intra_test_gains();

        // reset testing step
        autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL;
        autotune_step_start_time = millis();
        break;
    }
}

// autotune_backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
static void autotune_backup_gains_and_initialise()
{
    // initialise state because this is our first time
    if (autotune_roll_enabled()) {
        autotune_state.axis = AUTOTUNE_AXIS_ROLL;
    } else if (autotune_pitch_enabled()) {
        autotune_state.axis = AUTOTUNE_AXIS_PITCH;
    } else if (autotune_yaw_enabled()) {
        autotune_state.axis = AUTOTUNE_AXIS_YAW;
    }
    autotune_state.positive_direction = false;
    autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL;
    autotune_step_start_time = millis();
    autotune_state.tune_type = AUTOTUNE_TYPE_RD_UP;

    autotune_desired_yaw = ahrs.yaw_sensor;

    g.autotune_aggressiveness = constrain_float(g.autotune_aggressiveness, 0.05, 0.1);

    // backup original pids and initialise tuned pid values
    if (autotune_roll_enabled()) {
        orig_roll_rp = g.pid_rate_roll.kP();
        orig_roll_ri = g.pid_rate_roll.kI();
        orig_roll_rd = g.pid_rate_roll.kD();
        orig_roll_sp = g.p_stabilize_roll.kP();
        tune_roll_rp = g.pid_rate_roll.kP();
        tune_roll_rd = g.pid_rate_roll.kD();
        tune_roll_sp = g.p_stabilize_roll.kP();
    }
    if (autotune_pitch_enabled()) {
        orig_pitch_rp = g.pid_rate_pitch.kP();
        orig_pitch_ri = g.pid_rate_pitch.kI();
        orig_pitch_rd = g.pid_rate_pitch.kD();
        orig_pitch_sp = g.p_stabilize_pitch.kP();
        tune_pitch_rp = g.pid_rate_pitch.kP();
        tune_pitch_rd = g.pid_rate_pitch.kD();
        tune_pitch_sp = g.p_stabilize_pitch.kP();
    }
    if (autotune_yaw_enabled()) {
        orig_yaw_rp = g.pid_rate_yaw.kP();
        orig_yaw_ri = g.pid_rate_yaw.kI();
        orig_yaw_rd = g.pid_rate_yaw.kD();
        orig_yaw_rLPF = g.pid_rate_yaw.filt_hz();
        orig_yaw_sp = g.p_stabilize_yaw.kP();
        tune_yaw_rp = g.pid_rate_yaw.kP();
        tune_yaw_rLPF = g.pid_rate_yaw.filt_hz();
        tune_yaw_sp = g.p_stabilize_yaw.kP();
    }

    Log_Write_Event(DATA_AUTOTUNE_INITIALISED);
}

// autotune_load_orig_gains - set gains to their original values
//  called by autotune_stop and autotune_failed functions
static void autotune_load_orig_gains()
{
    // sanity check the gains
    bool failed = false;
    if (autotune_roll_enabled()) {
        if ((orig_roll_rp != 0) || (orig_roll_sp != 0)) {
            g.pid_rate_roll.kP(orig_roll_rp);
            g.pid_rate_roll.kI(orig_roll_ri);
            g.pid_rate_roll.kD(orig_roll_rd);
            g.p_stabilize_roll.kP(orig_roll_sp);
        } else {
            failed = true;
        }
    }
    if (autotune_pitch_enabled()) {
        if ((orig_pitch_rp != 0) || (orig_pitch_sp != 0)) {
            g.pid_rate_pitch.kP(orig_pitch_rp);
            g.pid_rate_pitch.kI(orig_pitch_ri);
            g.pid_rate_pitch.kD(orig_pitch_rd);
            g.p_stabilize_pitch.kP(orig_pitch_sp);
        } else {
            failed = true;
        }
    }
    if (autotune_yaw_enabled()) {
        if ((orig_yaw_rp != 0) || (orig_yaw_sp != 0) || (orig_yaw_rLPF != 0)) {
            g.pid_rate_yaw.kP(orig_yaw_rp);
            g.pid_rate_yaw.kI(orig_yaw_ri);
            g.pid_rate_yaw.kD(orig_yaw_rd);
            g.pid_rate_yaw.filt_hz(orig_yaw_rLPF);
            g.p_stabilize_yaw.kP(orig_yaw_sp);
        } else {
            failed = true;
        }
    }
    if (failed) {
        // log an error message and fail the autotune
        Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
    }
}

// autotune_load_tuned_gains - load tuned gains
static void autotune_load_tuned_gains()
{
    // sanity check the gains
    bool failed = true;
    if (autotune_roll_enabled()) {
        if (tune_roll_rp != 0) {
            g.pid_rate_roll.kP(tune_roll_rp);
            g.pid_rate_roll.kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
            g.pid_rate_roll.kD(tune_roll_rd);
            g.p_stabilize_roll.kP(tune_roll_sp);
            failed = false;
        }
    }
    if (autotune_pitch_enabled()) {
        if (tune_pitch_rp != 0) {
            g.pid_rate_pitch.kP(tune_pitch_rp);
            g.pid_rate_pitch.kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
            g.pid_rate_pitch.kD(tune_pitch_rd);
            g.p_stabilize_pitch.kP(tune_pitch_sp);
            failed = false;
        }
    }
    if (autotune_yaw_enabled()) {
        if (tune_yaw_rp != 0) {
            g.pid_rate_yaw.kP(tune_yaw_rp);
            g.pid_rate_yaw.kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
            g.pid_rate_yaw.kD(0.0f);
            g.pid_rate_yaw.filt_hz(tune_yaw_rLPF);
            g.p_stabilize_yaw.kP(tune_yaw_sp);
            failed = false;
        }
    }
    if (failed) {
        // log an error message and fail the autotune
        Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
    }
}

// autotune_load_intra_test_gains - gains used between tests
//  called during testing mode's update-gains step to set gains ahead of return-to-level step
static void autotune_load_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    // sanity check the gains
    bool failed = true;
    if (autotune_roll_enabled() && (orig_roll_rp != 0)) {
        g.pid_rate_roll.kP(orig_roll_rp);
        g.pid_rate_roll.kI(orig_roll_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        g.pid_rate_roll.kD(orig_roll_rd);
        g.p_stabilize_roll.kP(orig_roll_sp);
        failed = false;
    }
    if (autotune_pitch_enabled() && (orig_pitch_rp != 0)) {
        g.pid_rate_pitch.kP(orig_pitch_rp);
        g.pid_rate_pitch.kI(orig_pitch_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        g.pid_rate_pitch.kD(orig_pitch_rd);
        g.p_stabilize_pitch.kP(orig_pitch_sp);
        failed = false;
    }
    if (autotune_yaw_enabled() && (orig_yaw_rp != 0)) {
        g.pid_rate_yaw.kP(orig_yaw_rp);
        g.pid_rate_yaw.kI(orig_yaw_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        g.pid_rate_yaw.kD(orig_yaw_rd);
        g.pid_rate_yaw.filt_hz(orig_yaw_rLPF);
        g.p_stabilize_yaw.kP(orig_yaw_sp);
        failed = false;
    }
    if (failed) {
        // log an error message and fail the autotune
        Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
    }
}

// autotune_load_twitch_gains - load the to-be-tested gains for a single axis
// called by autotune_attitude_control() just before it beings testing a gain (i.e. just before it twitches)
static void autotune_load_twitch_gains()
{
    bool failed = true;
    switch (autotune_state.axis) {
        case AUTOTUNE_AXIS_ROLL:
            if (tune_roll_rp != 0) {
                g.pid_rate_roll.kP(tune_roll_rp);
                g.pid_rate_roll.kI(tune_roll_rp*0.01f);
                g.pid_rate_roll.kD(tune_roll_rd);
                g.p_stabilize_roll.kP(tune_roll_sp);
                failed = false;
            }
            break;
        case AUTOTUNE_AXIS_PITCH:
            if (tune_pitch_rp != 0) {
                g.pid_rate_pitch.kP(tune_pitch_rp);
                g.pid_rate_pitch.kI(tune_pitch_rp*0.01f);
                g.pid_rate_pitch.kD(tune_pitch_rd);
                g.p_stabilize_pitch.kP(tune_pitch_sp);
                failed = false;
            }
            break;
        case AUTOTUNE_AXIS_YAW:
            if (tune_yaw_rp != 0) {
                g.pid_rate_yaw.kP(tune_yaw_rp);
                g.pid_rate_yaw.kI(tune_yaw_rp*0.01f);
                g.pid_rate_yaw.kD(0.0f);
                g.pid_rate_yaw.filt_hz(tune_yaw_rLPF);
                g.p_stabilize_yaw.kP(tune_yaw_sp);
                failed = false;
            }
            break;
    }
    if (failed) {
        // log an error message and fail the autotune
        Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
    }
}

// autotune_save_tuning_gains - save the final tuned gains for each axis
// save discovered gains to eeprom if autotuner is enabled (i.e. switch is in the high position)
static void autotune_save_tuning_gains()
{
    // if we successfully completed tuning
    if (autotune_state.mode == AUTOTUNE_MODE_SUCCESS) {
        // sanity check the rate P values
        if (autotune_roll_enabled() && (tune_roll_rp != 0)) {
            // rate roll gains
            g.pid_rate_roll.kP(tune_roll_rp);
            g.pid_rate_roll.kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
            g.pid_rate_roll.kD(tune_roll_rd);
            g.pid_rate_roll.save_gains();
            // stabilize roll
            g.p_stabilize_roll.kP(tune_roll_sp);
            g.p_stabilize_roll.save_gains();

            if (attitude_control.get_bf_feedforward()) {
                attitude_control.save_accel_roll_max(tune_roll_accel);
            }

            // resave pids to originals in case the autotune is run again
            orig_roll_rp = g.pid_rate_roll.kP();
            orig_roll_ri = g.pid_rate_roll.kI();
            orig_roll_rd = g.pid_rate_roll.kD();
            orig_roll_sp = g.p_stabilize_roll.kP();
        }

        if (autotune_pitch_enabled() && (tune_pitch_rp != 0)) {
            // rate pitch gains
            g.pid_rate_pitch.kP(tune_pitch_rp);
            g.pid_rate_pitch.kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
            g.pid_rate_pitch.kD(tune_pitch_rd);
            g.pid_rate_pitch.save_gains();
            // stabilize pitch
            g.p_stabilize_pitch.kP(tune_pitch_sp);
            g.p_stabilize_pitch.save_gains();

            if (attitude_control.get_bf_feedforward()) {
                attitude_control.save_accel_pitch_max(tune_pitch_accel);
            }

            // resave pids to originals in case the autotune is run again
            orig_pitch_rp = g.pid_rate_pitch.kP();
            orig_pitch_ri = g.pid_rate_pitch.kI();
            orig_pitch_rd = g.pid_rate_pitch.kD();
            orig_pitch_sp = g.p_stabilize_pitch.kP();
        }

        if (autotune_yaw_enabled() && (tune_yaw_rp != 0)) {
            // rate yaw gains
            g.pid_rate_yaw.kP(tune_yaw_rp);
            g.pid_rate_yaw.kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
            g.pid_rate_yaw.kD(0.0f);
            g.pid_rate_yaw.filt_hz(tune_yaw_rLPF);
            g.pid_rate_yaw.save_gains();
            // stabilize yaw
            g.p_stabilize_yaw.kP(tune_yaw_sp);
            g.p_stabilize_yaw.save_gains();

            if (attitude_control.get_bf_feedforward()) {
                attitude_control.save_accel_yaw_max(tune_yaw_accel);
            }

            // resave pids to originals in case the autotune is run again
            orig_yaw_rp = g.pid_rate_yaw.kP();
            orig_yaw_ri = g.pid_rate_yaw.kI();
            orig_yaw_rd = g.pid_rate_yaw.kD();
            orig_yaw_rLPF = g.pid_rate_yaw.filt_hz();
            orig_yaw_sp = g.p_stabilize_yaw.kP();
        }
        // update GCS and log save gains event
        autotune_update_gcs(AUTOTUNE_MESSAGE_SAVED_GAINS);
        Log_Write_Event(DATA_AUTOTUNE_SAVEDGAINS);
        // reset Autotune so that gains are not saved again and autotune can be run again.
        autotune_state.mode = AUTOTUNE_MODE_UNINITIALISED;
    }
}

// autotune_update_gcs - send message to ground station
void autotune_update_gcs(uint8_t message_id)
{
    switch (message_id) {
        case AUTOTUNE_MESSAGE_STARTED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Started"));
            break;
        case AUTOTUNE_MESSAGE_STOPPED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Stopped"));
            break;
        case AUTOTUNE_MESSAGE_SUCCESS:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Success"));
            break;
        case AUTOTUNE_MESSAGE_FAILED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Failed"));
            break;
        case AUTOTUNE_MESSAGE_SAVED_GAINS:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Saved Gains"));
            break;
    }
}

// axis helper functions
inline bool autotune_roll_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_ROLL;
}

inline bool autotune_pitch_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_PITCH;
}

inline bool autotune_yaw_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_YAW;
}

// autotune_twitching_test - twitching tests
// update min and max and test for end conditions
void autotune_twitching_test(float measurement, float target, float &measurement_min, float &measurement_max)
{
    // capture maximum measurement
    if (measurement > measurement_max) {
        // the measurement is continuing to increase without stopping
        measurement_max = measurement;
        measurement_min = measurement;
    }

    // capture minimum measurement after the measurement has peaked (aka "bounce back")
    if ((measurement < measurement_min) && (measurement_max > target * 0.5f)) {
        // the measurement is bouncing back
        measurement_min = measurement;
    }

    // calculate early stopping time based on the time it takes to get to 90%
    if (measurement_max < target * 0.9f) {
        // the measurement not reached the 90% threshold yet
        autotune_step_stop_time = autotune_step_start_time + (millis() - autotune_step_start_time) * 3.0f;
        autotune_step_stop_time = min(autotune_step_stop_time, autotune_step_start_time + AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
    }

    if (measurement_max > target) {
        // the measurement has passed the target
        autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
    }

    if (measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) {
        // the measurement has passed 50% of the target and bounce back is larger than the threshold
        autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
    }

    if (millis() >= autotune_step_stop_time) {
        // we have passed the maximum stop time
        autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
    }
}

// autotune_updating_d_up - increase D and adjust P to optimize the D term for a little bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void autotune_updating_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target) {
        // if maximum measurement was higher than target
        // reduce P gain (which should reduce maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D
            tune_p = tune_p_min;
            tune_d -= tune_d*tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D gain so stop tuning
                tune_d = tune_d_min;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }else if ((measurement_max < target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }else{
        // we have a good measurement of bounce back
        if (measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) {
            // ignore the next result unless it is the same as this one
            autotune_state.ignore_next = 1;
            // bounce back is bigger than our threshold so increment the success counter
            autotune_counter++;
        }else{
            if (autotune_state.ignore_next == 0){
                // bounce back is smaller than our threshold so decrement the success counter
                if (autotune_counter > 0 ) {
                    autotune_counter--;
                }
                // increase D gain (which should increase bounce back)
                tune_d += tune_d*tune_d_step_ratio*2.0f;
                // stop tuning if we hit maximum D
                if (tune_d >= tune_d_max) {
                    tune_d = tune_d_max;
                    autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                }
            } else {
                autotune_state.ignore_next = 0;
            }
        }
    }
}

// autotune_updating_d_down - decrease D and adjust P to optimize the D term for no bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void autotune_updating_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target) {
        // if maximum measurement was higher than target
        // reduce P gain (which should reduce maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D gain
            tune_p = tune_p_min;
            tune_d -= tune_d*tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D so stop tuning
                tune_d = tune_d_min;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }else if ((measurement_max < target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }else{
        // we have a good measurement of bounce back
        if (measurement_max-measurement_min < measurement_max*g.autotune_aggressiveness) {
            if (autotune_state.ignore_next == 0){
                // bounce back is less than our threshold so increment the success counter
                autotune_counter++;
            } else {
                autotune_state.ignore_next = 0;
            }
        }else{
            // ignore the next result unless it is the same as this one
            autotune_state.ignore_next = 1;
            // bounce back is larger than our threshold so decrement the success counter
            if (autotune_counter > 0 ) {
                autotune_counter--;
            }
            // decrease D gain (which should decrease bounce back)
            tune_d -= tune_d*tune_d_step_ratio;
            // stop tuning if we hit minimum D
            if (tune_d <= tune_d_min) {
                tune_d = tune_d_min;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }
}

// autotune_updating_p_down - decrease P until we don't reach the target before time out
// P is decreased to ensure we are not overshooting the target
void autotune_updating_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float target, float measurement_max)
{
    if (measurement_max < target) {
        if (autotune_state.ignore_next == 0){
            // if maximum measurement was lower than target so increment the success counter
            autotune_counter++;
        } else {
            autotune_state.ignore_next = 0;
        }
    }else{
        // ignore the next result unless it is the same as this one
        autotune_state.ignore_next = 1;
        // if maximum measurement was higher than target so decrement the success counter
        if (autotune_counter > 0 ) {
            autotune_counter--;
        }
        // decrease P gain (which should decrease the maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit maximum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            autotune_counter = AUTOTUNE_SUCCESS_COUNT;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }
}

// autotune_updating_p_up - increase P to ensure the target is reached
// P is increased until we achieve our target within a reasonable time
void autotune_updating_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float target, float measurement_max)
{
    if (measurement_max > target) {
        // ignore the next result unless it is the same as this one
        autotune_state.ignore_next = 1;
        // if maximum measurement was greater than target so increment the success counter
        autotune_counter++;
    }else{
        if (autotune_state.ignore_next == 0){
            // if maximum measurement was lower than target so decrement the success counter
            if (autotune_counter > 0 ) {
                autotune_counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            autotune_state.ignore_next = 0;
        }
    }
}

// autotune_updating_p_up - increase P to ensure the target is reached while checking bounce back isn't increasing
// P is increased until we achieve our target within a reasonable time while reducing D if bounce back increases above the threshold
void autotune_updating_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target) {
        // ignore the next result unless it is the same as this one
        autotune_state.ignore_next = 1;
        // if maximum measurement was greater than target so increment the success counter
        autotune_counter++;
    }else if ((measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) && (tune_d > tune_d_min)) {
        // if bounce back was larger than the threshold so decrement the success counter
        if (autotune_counter > 0 ) {
            autotune_counter--;
        }
        // decrease D gain (which should decrease bounce back)
        tune_d -= tune_d*tune_d_step_ratio;
        // stop tuning if we hit minimum D
        if (tune_d <= tune_d_min) {
            tune_d = tune_d_min;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
        // decrease P gain to match D gain reduction
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit minimum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
        // cancel change in direction
        autotune_state.positive_direction = !autotune_state.positive_direction;
    }else{
        if (autotune_state.ignore_next == 0){
            // if maximum measurement was lower than target so decrement the success counter
            if (autotune_counter > 0 ) {
                autotune_counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            autotune_state.ignore_next = 0;
        }
    }
}

// autotune_twitching_measure_acceleration - measure rate of change of measurement
void autotune_twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max)
{
    if (rate_measurement_max < rate_measurement) {
        rate_measurement_max = rate_measurement;
        rate_of_change = (1000.0f*rate_measurement_max)/(millis() - autotune_step_start_time);
    }
}

#endif  // AUTOTUNE_ENABLED == ENABLED
