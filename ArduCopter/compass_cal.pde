/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define COMPASS_CAL_STICK_DELAY 2.0f // 2 seconds
#define COMPASS_CAL_DELAY 5.0f

static AP_InertialSensor_UserInteract_MAVLink interact(NULL);
static uint8_t acal_orient_step;

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
    if (motors.armed()) {
        return;
    }

    float trim_roll, trim_pitch;
    static bool _trim_saved;

    //update states of calibration instances
    ins.acal_update(trim_roll, trim_pitch);
    camera_mount.acal_update();

    if(ins.acal_failed() || camera_mount.acal_failed()) {
        //Calibration failed clear everything and notify GCS
        interact.printf_P(PSTR("Calibration FAILED\n"));
        ins.acal_stop();
        camera_mount.acal_stop();
        acal_orient_step = 0;
    }

    if (ins.acal_is_calibrating()) {
        //set gimbal to body fixed mode
        camera_mount.set_mode(MAV_MOUNT_MODE_RETRACT);
        _trim_saved = false;
        return;
    } else if(ins.acal_completed() && !_trim_saved) {
        printf("Trim OK: roll=%.5f pitch=%.5f\n",
                              degrees(trim_roll),
                              degrees(trim_pitch));
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
        _trim_saved = true;
    }
    if(ins.acal_completed() && camera_mount.acal_completed()) {
        interact.printf_P(PSTR("Calibration successful\n"));
        acal_orient_step = 0;
        ins.acal_stop();
        camera_mount.acal_stop();
        //accel calibration complete notify GCS and reboot the system
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }

}

static void acal_collect_sample( GCS_MAVLINK* interact_gcs) {
    const prog_char_t *msg;

    interact = AP_InertialSensor_UserInteract_MAVLink(interact_gcs);

    if(ins.acal_collecting_sample() || camera_mount.acal_collecting_sample()) {
        return;
    }
     // display message to user
     switch ( acal_orient_step++ ) {
         case 0:
             msg = PSTR("on its LEFT side");
             break;
         case 1:
             msg = PSTR("on its RIGHT side");
             break;
         case 2:
             msg = PSTR("nose DOWN");
             break;
         case 3:
             msg = PSTR("nose UP");
             break;
         default:    // default added to avoid compiler warning
         case 4:
             msg = PSTR("on its BACK");
             break;
     }
     ins.acal_collect_sample();
     camera_mount.acal_collect_sample();

     interact.printf_P(PSTR("Place vehicle %S and press any key.\n"), msg);
}
