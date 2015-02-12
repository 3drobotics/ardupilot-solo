/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// counter to verify landings
static uint16_t land_detector = LAND_DETECTOR_TRIGGER;  // we assume we are landed

// land_complete_maybe - return true if we may have landed (used to reset loiter targets during landing)
static bool land_complete_maybe()
{
    return (ap.land_complete || ap.land_complete_maybe);
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at 50hz
static void update_land_detector()
{
    bool climb_rate_low = (abs(climb_rate) < LAND_DETECTOR_CLIMBRATE_MAX);
    bool target_climb_rate_low = !pos_control.is_active_z() || (pos_control.get_desired_velocity().z <= LAND_DETECTOR_DESIRED_CLIMBRATE_MAX);
    bool motor_at_lower_limit = motors.limit.throttle_lower;
    bool throttle_low = (FRAME_CONFIG == HELI_FRAME) || (motors.get_throttle_out() < get_non_takeoff_throttle());
    bool not_rotating_fast = (ahrs.get_gyro().length() < LAND_DETECTOR_ROTATION_MAX);

    if (climb_rate_low && target_climb_rate_low && motor_at_lower_limit && throttle_low && not_rotating_fast) {
        if (!ap.land_complete) {
            // increase counter until we hit the trigger then set land complete flag
            if( land_detector < LAND_DETECTOR_TRIGGER) {
                land_detector++;
            }else{
                set_land_complete(true);
                land_detector = LAND_DETECTOR_TRIGGER;
            }
        }
    } else {
        // we've sensed movement up or down so reset land_detector
        land_detector = 0;
        // if throttle output is high then clear landing flag
        if (motors.get_throttle_out() > get_non_takeoff_throttle()) {
            set_land_complete(false);
        }
    }

    // set land maybe flag
    set_land_complete_maybe(land_detector >= LAND_DETECTOR_MAYBE_TRIGGER);
}

// update_throttle_low_comp - sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
static void update_throttle_low_comp()
{
    // manual throttle
    if (mode_has_manual_throttle(control_mode)) {
        if(!motors.armed() || g.rc_3.control_in <= 0) {
            motors.set_throttle_low_comp(0.1f);
        } else {
            motors.set_throttle_low_comp(0.5f);
        }
    } else {
        // autopilot controlled throttle
        const Vector3f angle_target = attitude_control.angle_ef_targets();
        if (pythagorous2(angle_target.x, angle_target.y) > 1500.0f) {
            // if target lean angle is over 15 degrees set high
            motors.set_throttle_low_comp(0.9f);
        } else {
            motors.set_throttle_low_comp(0.1f);
        }
    }
}

static void update_ground_effect_detector(void)
{
    if(!motors.armed()) {
        // disarmed - disable ground effect and return
        ap.in_ground_effect = false;
        ahrs.setGndEffectMode(ap.in_ground_effect);
        pos_control.setGndEffectMode(ap.in_ground_effect);
        return;
    }

    // variable initialization
    uint32_t tnow_ms = hal.scheduler->millis();
    static uint32_t takeoff_time_ms;
    static float takeoff_alt_cm;
    float xy_des_speed_cms = 0.0f;
    float xy_speed_cms = 0.0f;
    float des_climb_rate_cms = pos_control.get_desired_velocity().z;

    if(pos_control.is_active_xy()) {
        Vector3f vel_target = pos_control.get_vel_target();
        vel_target.z = 0.0f;
        xy_des_speed_cms = vel_target.length();
    }

    if(position_ok() || optflow_position_ok()) {
        Vector3f vel = inertial_nav.get_velocity();
        vel.z = 0.0f;
        xy_speed_cms = vel.length();
    }

    // takeoff logic
    bool throttle_up = motors.armed() && g.rc_3.servo_out >= g.throttle_min;
    bool takeoffExpected;
    if (!throttle_up || ap.land_complete) {
        takeoff_time_ms = tnow_ms;
        takeoff_alt_cm = current_loc.alt;
    }

    if (throttle_up && tnow_ms-takeoff_time_ms < 10000 && current_loc.alt-takeoff_alt_cm < 150.0f) {
        takeoffExpected = true;
    } else {
        takeoffExpected = false;
    }

    // landing logic
    bool xy_speed_low = (position_ok() || optflow_position_ok()) && xy_speed_cms <= 100.0f;
    bool xy_speed_demand_low = pos_control.is_active_xy() && xy_des_speed_cms <= 50.0f;
    bool slow_horizontal = xy_speed_demand_low || (xy_speed_low && !pos_control.is_active_xy());
    
    bool descent_demanded = pos_control.is_active_z() && des_climb_rate_cms < 0.0f;
    bool slow_descent_demanded = descent_demanded && des_climb_rate_cms >= -100.0f;
    bool z_speed_low = abs(climb_rate) <= LAND_DETECTOR_CLIMBRATE_MAX*2.0f;
    bool slow_descent = (slow_descent_demanded || (z_speed_low && descent_demanded));

    bool height_low = current_loc.alt < 1000;

    bool touchdownExpected = slow_horizontal && slow_descent && height_low;

    // Prepare the EKF and height controller for ground effect if either takeoff or touchdown is expected.
    ap.in_ground_effect = takeoffExpected || touchdownExpected;
    ahrs.setGndEffectMode(ap.in_ground_effect);
    pos_control.setGndEffectMode(ap.in_ground_effect);
}
