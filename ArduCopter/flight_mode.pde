/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * flight.pde - high level calls to set and update flight modes
 *      logic for individual flight modes is in control_acro.pde, control_stabilize.pde, etc
 */

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was succesfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
// If we are in a mode that doesn't require a stable GPs position, then hard reset of the EKF position following a GPS glitch or outage is allowed for faster recovery.
static bool set_mode(uint8_t mode)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !motors.armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        return true;
    }

    switch(mode) {
        case ACRO:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_acro_init(ignore_checks);
            #else
                success = acro_init(ignore_checks);
            #endif
            if (success) ahrs.setAllowHardPosResets(true);
            break;

        case STABILIZE:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_stabilize_init(ignore_checks);
            #else
                success = stabilize_init(ignore_checks);
            #endif
            if (success) ahrs.setAllowHardPosResets(true);
            break;

        case ALT_HOLD:
            success = althold_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(true);
            break;

        case AUTO:
            success = auto_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(false);
            break;

        case CIRCLE:
            success = circle_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(false);
            break;

        case LOITER:
            success = loiter_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(false);
            break;

        case GUIDED:
            success = guided_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(false);
            break;

        case LAND:
            success = land_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(false);
            break;

        case RTL:
            success = rtl_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(false);
            break;

        case DRIFT:
            success = drift_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(false);
            break;

        case SPORT:
            success = sport_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(true);
            break;

        case FLIP:
            success = flip_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(false);
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            success = autotune_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(true);
            break;
#endif

#if POSHOLD_ENABLED == ENABLED
        case POSHOLD:
            success = poshold_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(false);
            break;
#endif

        case STOP:
            success = stop_init(ignore_checks);
            if (success) ahrs.setAllowHardPosResets(false);
            break;

        default:
            success = false;
            if (success) ahrs.setAllowHardPosResets(false);
            break;
    }

    // update flight mode
    if (success) {
        // perform any cleanup required by previous flight mode
        exit_mode(control_mode, mode);
        control_mode = mode;
        DataFlash.Log_Write_Mode(control_mode);

#if AC_FENCE == ENABLED
        // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
        // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
        // but it should be harmless to disable the fence temporarily in these situations as well
        fence.manual_recovery_start();
#endif
    }else{
        // Log error that we failed to enter desired flight mode
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
    }

    // update notify object
    if (success) {
        notify_flight_mode(control_mode);
    }

    // return success or failure
    return success;
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
static void update_flight_mode()
{
#if AP_AHRS_NAVEKF_AVAILABLE
    // Update EKF speed limit - used to limit speed when we are using optical flow
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
#endif
    switch (control_mode) {
        case ACRO:
            #if FRAME_CONFIG == HELI_FRAME
                heli_acro_run();
            #else
                acro_run();
            #endif
            break;

        case STABILIZE:
            #if FRAME_CONFIG == HELI_FRAME
                heli_stabilize_run();
            #else
                stabilize_run();
            #endif
            break;

        case ALT_HOLD:
            althold_run();
            break;

        case AUTO:
            auto_run();
            break;

        case CIRCLE:
            circle_run();
            break;

        case LOITER:
            loiter_run();
            break;

        case GUIDED:
            guided_run();
            break;

        case LAND:
            land_run();
            break;

        case RTL:
            rtl_run();
            break;

        case DRIFT:
            drift_run();
            break;

        case SPORT:
            sport_run();
            break;

        case FLIP:
            flip_run();
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            autotune_run();
            break;
#endif

#if POSHOLD_ENABLED == ENABLED
        case POSHOLD:
            poshold_run();
            break;
#endif

        case STOP:
            stop_run();
            break;
    }
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
static void exit_mode(uint8_t old_control_mode, uint8_t new_control_mode)
{
#if AUTOTUNE_ENABLED == ENABLED
    if (old_control_mode == AUTOTUNE) {
        autotune_stop();
    }
#endif

    // stop mission when we leave auto mode
    if (old_control_mode == AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if MOUNT == ENABLED
        camera_mount.set_mode_to_default();
#endif  // MOUNT == ENABLED
    }

    // smooth throttle transition when switching from manual to automatic flight modes
    if (mode_has_manual_throttle(old_control_mode) && !mode_has_manual_throttle(new_control_mode) && motors.armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
    }

    // cancel any takeoffs in progress
    takeoff_stop();

#if FRAME_CONFIG == HELI_FRAME
    // firmly reset the flybar passthrough to false when exiting acro mode.
    if (old_control_mode == ACRO) {
        attitude_control.use_flybar_passthrough(false);
    }
#endif //HELI_FRAME
}

// returns true or false whether mode requires GPS
static bool mode_requires_GPS(uint8_t mode) {
    switch(mode) {
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case DRIFT:
        case POSHOLD:
        case STOP:
            return true;
        default:
            return false;
    }

    return false;
}

static bool mode_disarms_on_land(uint8_t mode) {
    return mode_allows_arming(mode, false) && !mode_has_manual_throttle(mode);
}

// mode_has_manual_throttle - returns true if the flight mode has a manual throttle (i.e. pilot directly controls throttle)
static bool mode_has_manual_throttle(uint8_t mode) {
    switch(mode) {
        case ACRO:
        case STABILIZE:
            return true;
        default:
            return false;
    }

    return false;
}

// mode_allows_arming - returns true if vehicle can be armed in the specified mode
//  arming_from_gcs should be set to true if the arming request comes from the ground station
static bool mode_allows_arming(uint8_t mode, bool arming_from_gcs) {
    if (mode_has_manual_throttle(mode) || mode == LOITER || mode == ALT_HOLD || mode == POSHOLD || (arming_from_gcs && mode == GUIDED)) {
        return true;
    }
    return false;
}

// notify_flight_mode - sets notify object based on flight mode.  Only used for OreoLED notify device
static void notify_flight_mode(uint8_t mode) {
    switch(mode) {
        case AUTO:
        case GUIDED:
        case RTL:
        case CIRCLE:
        case LAND:
            // autopilot modes
            AP_Notify::flags.autopilot_mode = true;
            break;
        default:
            // all other are manual flight modes
            AP_Notify::flags.autopilot_mode = false;
            break;
    }
}

//
// print_flight_mode - prints flight mode to serial port.
//
static void
print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case STABILIZE:
        port->print_P(PSTR("STABILIZE"));
        break;
    case ACRO:
        port->print_P(PSTR("ACRO"));
        break;
    case ALT_HOLD:
        port->print_P(PSTR("ALT_HOLD"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case GUIDED:
        port->print_P(PSTR("GUIDED"));
        break;
    case LOITER:
        port->print_P(PSTR("LOITER"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    case CIRCLE:
        port->print_P(PSTR("CIRCLE"));
        break;
    case LAND:
        port->print_P(PSTR("LAND"));
        break;
    case OF_LOITER:
        port->print_P(PSTR("OF_LOITER"));
        break;
    case DRIFT:
        port->print_P(PSTR("DRIFT"));
        break;
    case SPORT:
        port->print_P(PSTR("SPORT"));
        break;
    case FLIP:
        port->print_P(PSTR("FLIP"));
        break;
    case AUTOTUNE:
        port->print_P(PSTR("AUTOTUNE"));
        break;
    case POSHOLD:
        port->print_P(PSTR("POSHOLD"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}

