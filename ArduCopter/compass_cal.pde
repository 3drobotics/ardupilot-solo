/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define COMPASS_CAL_STICK_DELAY 2.0f // 2 seconds
#define COMPASS_CAL_DELAY 5.0f

static void compass_cal_update() {
    compass.compass_cal_update();

    static bool cal_has_run = false;
    if (compass.is_calibrating()) {
        camera_mount.set_mode(MAV_MOUNT_MODE_RETRACT);
        cal_has_run = true;
        if(!motors.armed() && g.rc_4.control_in < -4000 && g.rc_3.control_in > 900) {
            compass.cancel_calibration_all();
        }
        return;
    } else if (cal_has_run) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }

    bool stick_gesture_detected = !motors.armed() && g.rc_4.control_in > 4000 && g.rc_3.control_in > 900;
    static uint32_t stick_gesture_begin = 0;
    uint32_t tnow = millis();

    if(!stick_gesture_detected) {
        stick_gesture_begin = tnow;
    } else {
        if(tnow-stick_gesture_begin > 1000*COMPASS_CAL_STICK_DELAY) {
            compass.start_calibration_all(true,true,COMPASS_CAL_DELAY);
        }
    }
}

static void accel_cal_update() {
    ins.acal_update();
    if (motors.armed()) {
        return;
    }

    uint32_t tnow = millis();
    static uint32_t start_gesture_begin = 0;
    static uint32_t last_continue = 0;

    bool start_gesture_detected = g.rc_1.control_in > 4000 && g.rc_2.control_in < -4000;
    bool cancel_gesture_detected = g.rc_1.control_in < -4000 && g.rc_2.control_in < -4000;
    bool continue_gesture_detected = g.rc_1.control_in > 4000 && g.rc_2.control_in > 4000;

    if (!start_gesture_detected) {
        start_gesture_begin = tnow;
    }

    if (ins.acal_is_calibrating()) {
        if (cancel_gesture_detected) {
            ins.acal_cancel();
        }

        if (continue_gesture_detected && tnow-last_continue > 2000) {
            last_continue = tnow;
            ins.acal_collect_sample();
        }

        camera_mount.set_mode(MAV_MOUNT_MODE_RETRACT);

        return;
    }

    if(start_gesture_detected && tnow-start_gesture_begin > 2000) {
        ins.acal_start();
    }
}
