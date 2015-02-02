// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
#if HIL_MODE == HIL_MODE_DISABLED
static int8_t   test_baro(uint8_t argc,                 const Menu::arg *argv);
#endif
static int8_t   test_radio_pwm(uint8_t argc,            const Menu::arg *argv);
static int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_compass(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_optflow(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_input(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_motor_torture(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_motor_step(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_motor_slew(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_motor_filter(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_curve(uint8_t argc,                const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
static int8_t   test_shell(uint8_t argc,                const Menu::arg *argv);
#endif
#if HIL_MODE == HIL_MODE_DISABLED
static int8_t   test_sonar(uint8_t argc,                const Menu::arg *argv);
#endif

static bool do_curve = false;

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command test_menu_commands[] PROGMEM = {
#if HIL_MODE == HIL_MODE_DISABLED
    {"baro",                test_baro},
#endif
    {"pwm",                 test_radio_pwm},
    {"radio",               test_radio},
    {"compass",             test_compass},
    {"ins",                 test_ins},
    {"optflow",             test_optflow},
    {"relay",               test_relay},
    {"input",               test_input},
    {"motor-torture",       test_motor_torture},
    {"motor-filter",        test_motor_filter},
    {"motor-slew",          test_motor_slew},
    {"motor-step",          test_motor_step},
    {"motor-curve",               test_curve},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    {"shell", 				test_shell},
#endif
#if HIL_MODE == HIL_MODE_DISABLED
    {"rangefinder",         test_sonar},
#endif
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
    test_menu.run();
    return 0;
}

#if HIL_MODE == HIL_MODE_DISABLED
static int8_t
test_baro(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    init_barometer(true);

    while(1) {
        delay(100);
        read_barometer();

        if (!barometer.healthy()) {
            cliSerial->println_P(PSTR("not healthy"));
        } else {
            cliSerial->printf_P(PSTR("Alt: %0.2fm, Raw: %f Temperature: %.1f\n"),
                                baro_alt / 100.0,
                                barometer.get_pressure(), 
                                barometer.get_temperature());
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}
#endif

static int8_t
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);

        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        // servo Yaw
        //APM_RC.OutputCh(CH_7, g.rc_4.radio_out);

        cliSerial->printf_P(PSTR("IN: 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
                        g.rc_1.radio_in,
                        g.rc_2.radio_in,
                        g.rc_3.radio_in,
                        g.rc_4.radio_in,
                        g.rc_5.radio_in,
                        g.rc_6.radio_in,
                        g.rc_7.radio_in,
                        g.rc_8.radio_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);
        read_radio();


        cliSerial->printf_P(PSTR("IN  1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\n"),
                        g.rc_1.control_in,
                        g.rc_2.control_in,
                        g.rc_3.control_in,
                        g.rc_4.control_in,
                        g.rc_5.control_in,
                        g.rc_6.control_in,
                        g.rc_7.control_in);

        //cliSerial->printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d\n"), (g.rc_1.servo_out / 100), (g.rc_2.servo_out / 100), g.rc_3.servo_out, (g.rc_4.servo_out / 100));

        /*cliSerial->printf_P(PSTR(	"min: %d"
         *                                               "\t in: %d"
         *                                               "\t pwm_in: %d"
         *                                               "\t sout: %d"
         *                                               "\t pwm_out %d\n"),
         *                                               g.rc_3.radio_min,
         *                                               g.rc_3.control_in,
         *                                               g.rc_3.radio_in,
         *                                               g.rc_3.servo_out,
         *                                               g.rc_3.pwm_out
         *                                               );
         */
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


static int8_t
test_compass(uint8_t argc, const Menu::arg *argv)
{
    uint8_t delta_ms_fast_loop;
    uint8_t medium_loopCounter = 0;

    if (!g.compass_enabled) {
        cliSerial->printf_P(PSTR("Compass: "));
        print_enabled(false);
        return (0);
    }

    if (!compass.init()) {
        cliSerial->println_P(PSTR("Compass initialisation failed!"));
        return 0;
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_compass(&compass);
#if OPTFLOW == ENABLED
    ahrs.set_optflow(&optflow);
#endif
    report_compass();

    // we need the AHRS initialised for this test
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ahrs.reset();
    int16_t counter = 0;
    float heading = 0;

    print_hit_enter();

    while(1) {
        delay(20);
        if (millis() - fast_loopTimer > 19) {
            delta_ms_fast_loop      = millis() - fast_loopTimer;
            G_Dt                    = (float)delta_ms_fast_loop / 1000.f;                       // used by DCM integrator
            fast_loopTimer          = millis();

            // INS
            // ---
            ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter == 5) {
                if (compass.read()) {
                    // Calculate heading
                    const Matrix3f &m = ahrs.get_dcm_matrix();
                    heading = compass.calculate_heading(m);
                    compass.learn_offsets();
                }
                medium_loopCounter = 0;
            }

            counter++;
            if (counter>20) {
                if (compass.healthy()) {
                    const Vector3f &mag_ofs = compass.get_offsets();
                    const Vector3f &mag = compass.get_field();
                    cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                        (wrap_360_cd(ToDeg(heading) * 100)) /100,
                                        mag.x,
                                        mag.y,
                                        mag.z,
                                        mag_ofs.x,
                                        mag_ofs.y,
                                        mag_ofs.z);
                } else {
                    cliSerial->println_P(PSTR("compass not healthy"));
                }
                counter=0;
            }
        }
        if (cliSerial->available() > 0) {
            break;
        }
    }

    // save offsets. This allows you to get sane offset values using
    // the CLI before you go flying.
    cliSerial->println_P(PSTR("saving offsets"));
    compass.save_offsets();
    return (0);
}

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
    Vector3f gyro, accel;
    print_hit_enter();
    cliSerial->printf_P(PSTR("INS\n"));
    delay(1000);

    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    cliSerial->printf_P(PSTR("...done\n"));

    delay(50);

    while(1) {
        ins.update();
        gyro = ins.get_gyro();
        accel = ins.get_accel();

        float test = accel.length() / GRAVITY_MSS;

        cliSerial->printf_P(PSTR("a %7.4f %7.4f %7.4f g %7.4f %7.4f %7.4f t %7.4f \n"),
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            test);

        delay(40);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_optflow(uint8_t argc, const Menu::arg *argv)
{
#if OPTFLOW == ENABLED
    if(optflow.enabled()) {
        cliSerial->printf_P(PSTR("dev id: %d\t"),(int)optflow.device_id());
        print_hit_enter();

        while(1) {
            delay(200);
            optflow.update();
            const Vector2f& flowRate = optflow.flowRate();
            cliSerial->printf_P(PSTR("flowX : %7.4f\t flowY : %7.4f\t flow qual : %d\n"),
                            flowRate.x,
                            flowRate.y,
                            (int)optflow.quality());

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf_P(PSTR("OptFlow: "));
        print_enabled(false);
    }
    return (0);
#else
    return (0);
#endif      // OPTFLOW == ENABLED
}


static int8_t test_input(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    cliSerial->printf_P(PSTR("Input\n"));

    int16_t i;
    int16_t scaled;
    
    // make sure throttle is 1000-2000
    for (i = 1000; i <= 2000; i++){
        g.rc_3.set_pwm(i);
        scaled = get_pilot_desired_throttle(g.rc_3.control_in);
        cliSerial->printf_P(PSTR("pwm in:%d, control_in %d, scaled: %d\n"), i, g.rc_3.control_in, scaled);
    }    
    cliSerial->printf_P(PSTR("\n\nComplete\n"));
    return (0);
}


static int8_t test_curve(uint8_t argc, const Menu::arg *argv) {
    if(do_curve) {
        cliSerial->printf("Disabling throttle curve\n");
        do_curve = false;
    } else {
        cliSerial->printf("Enabling throttle curve\n");
        do_curve = true;
    }
    return 0;
}

static float curve_throttle(float thr)
{
    if(!do_curve) {
        return thr;
    }

    float out_min_pwm = g.rc_3.radio_min+130.0f;
    float out_max_pwm = g.rc_3.radio_max;

    float throttle_out = (thr-out_min_pwm)/(out_max_pwm-out_min_pwm);

    throttle_out = constrain_float(-0.269231f+0.769231f*safe_sqrt(0.1225f+2.6f*throttle_out*0.93f),0.0f,1.0f);

    throttle_out = throttle_out * (out_max_pwm-out_min_pwm) + out_min_pwm;

    throttle_out = constrain_float(throttle_out, g.rc_3.radio_min+130.0f, g.rc_3.radio_max);

    return throttle_out;
}

static void step_throttle_filtered(float from, float to, float freq_cut) {
    float elapsed = 0.0f;
    uint32_t tbegin = micros();
    uint32_t tlast = micros();
    uint32_t delay_start = micros();
    float filtered_thr = from;

    while(elapsed < 10.0f) {
        uint32_t tnow = micros();
        float dt = (tnow - tlast)*1.0e-6f;

        float alpha = constrain_float(dt/(dt + 1.0f/(2.0f*(float)M_PI*freq_cut)),0.0f,1.0f);
        filtered_thr += (to - filtered_thr) * alpha;

        motors.set_throttle(filtered_thr);
        motors.output();

        if((filtered_thr - from) / (to-from) > 0.95f) {
            if(tnow-delay_start > 0.5e6f) {
                break;
            }
        } else {
            delay_start = tnow;
        }

        delay(2);
        tlast = tnow;
        elapsed = (tnow-tbegin)*1.0e-6;
    }
}

static int8_t test_motor_torture(uint8_t argc, const Menu::arg *argv)
{
    if (argc < 4) {
        cliSerial->printf("%s <iterations> <max throttle> <freq cut>\n", argv[0].str);
        return 0;
    }

    motors.armed(true);
    motors.enable();

    float high_thr = constrain_float(argv[2].f*10.0f,130,1000);

    for(int32_t i=0; i<argv[1].i; i++) {
        motors.set_roll(0);
        motors.set_pitch(0);
        motors.set_yaw(0);
        motors.set_throttle(130);
        motors.output();
        delay(500);

        step_throttle_filtered(130,high_thr,argv[3].f);

        step_throttle_filtered(high_thr,130,argv[3].f);

        motors.set_pitch(-4500);
        motors.set_throttle(high_thr);
        motors.output();
        delay(500);

        motors.set_pitch(4500);
        motors.output();
        delay(500);

        motors.set_pitch(-4500);
        motors.output();
        delay(500);
    }

    motors.throttle_pass_through(g.rc_3.radio_min);
    return 0;
}

static int8_t test_motor_step(uint8_t argc, const Menu::arg *argv)
{
    if (argc < 3) {
        cliSerial->printf("%s <from> <to>\n", argv[0].str);
        return 0;
    }

    motors.armed(true);
    motors.enable();

    float throttle_range = g.rc_3.radio_max - g.rc_3.radio_min;
    float low_thr = constrain_float(g.rc_3.radio_min + throttle_range*argv[1].f*0.01f, g.rc_3.radio_min+130.0f, g.rc_3.radio_max);
    float high_thr = constrain_float(g.rc_3.radio_min + throttle_range*argv[2].f*0.01f, g.rc_3.radio_min+130.0f, g.rc_3.radio_max);

    cliSerial->printf("stepping from %f to %f\n", low_thr, high_thr);

    motors.throttle_pass_through(curve_throttle(low_thr));

    delay(1000);

    motors.throttle_pass_through(curve_throttle(high_thr));

    delay(1000);

    motors.throttle_pass_through(g.rc_3.radio_min);
    return 0;
}

static int8_t test_motor_slew(uint8_t argc, const Menu::arg *argv)
{
    if (argc < 4) {
        cliSerial->printf("%s <from> <to> <percent/sec>\n", argv[0].str);
        return 0;
    }

    motors.armed(true);
    motors.enable();

    float throttle_range = g.rc_3.radio_max - g.rc_3.radio_min;
    float low_thr = constrain_float(g.rc_3.radio_min + throttle_range*argv[1].f*0.01f, g.rc_3.radio_min+130.0f, g.rc_3.radio_max);
    float high_thr = constrain_float(g.rc_3.radio_min + throttle_range*argv[2].f*0.01f, g.rc_3.radio_min+130.0f, g.rc_3.radio_max);

    motors.throttle_pass_through(curve_throttle(low_thr));

    float filtered_thr = low_thr;
    float slew_rate = throttle_range * argv[3].f * 0.01f; // pwm * 100/s & 1/100 = pwm/s

    cliSerial->printf("slewing from %f to %f over %f sec\n", low_thr, high_thr, (high_thr - low_thr)/slew_rate);

    delay(1000);

    uint32_t tbegin = micros();
    uint32_t tlast = micros();
    float elapsed = 0.0f;

    while(elapsed < 10.0f && elapsed < (high_thr - low_thr)/slew_rate + 0.5f) {
        uint32_t tnow = micros();
        float dt = (tnow - tlast)*1.0e-6f;

        filtered_thr = constrain_float(filtered_thr + dt * slew_rate,low_thr,high_thr);

        motors.throttle_pass_through(curve_throttle(filtered_thr));

        delay(2);
        tlast = tnow;
        elapsed = (tnow-tbegin)*1.0e-6;
    }
    motors.throttle_pass_through(g.rc_3.radio_min);
    return 0;
}

static int8_t test_motor_filter(uint8_t argc, const Menu::arg *argv)
{
    if (argc < 4) {
        cliSerial->printf("%s <from> <to> <freq>\n", argv[0].str);
        return 0;
    }

    motors.armed(true);
    motors.enable();

    float throttle_range = g.rc_3.radio_max - g.rc_3.radio_min;
    float low_thr = constrain_float(g.rc_3.radio_min + throttle_range*argv[1].f*0.01f, g.rc_3.radio_min+130.0f, g.rc_3.radio_max);
    float high_thr = constrain_float(g.rc_3.radio_min + throttle_range*argv[2].f*0.01f, g.rc_3.radio_min+130.0f, g.rc_3.radio_max);

    motors.throttle_pass_through(curve_throttle(low_thr));

    float filtered_thr = low_thr;
    float freq_cut = argv[3].f;

    cliSerial->printf("stepping from %f to %f with %fhz LPF\n", low_thr, high_thr, freq_cut);

    delay(1000);

    float elapsed = 0.0f;
    uint32_t tbegin = micros();
    uint32_t tlast = micros();
    uint32_t delay_start = micros();

    while(elapsed < 10.0f) {
        uint32_t tnow = micros();
        float dt = (tnow - tlast)*1.0e-6f;

        float alpha = constrain_float(dt/(dt + 1.0f/(2.0f*(float)M_PI*freq_cut)),0.0f,1.0f);
        filtered_thr += (high_thr - filtered_thr) * alpha;

        motors.throttle_pass_through(curve_throttle(filtered_thr));

        if((filtered_thr - low_thr) / (high_thr-low_thr) > 0.95f) {
            if(tnow-delay_start > 0.5e6f) {
                break;
            }
        } else {
            delay_start = tnow;
        }

        delay(2);
        tlast = tnow;
        elapsed = (tnow-tbegin)*1.0e-6;
    }
    motors.throttle_pass_through(g.rc_3.radio_min);
    return 0;
}


static int8_t test_relay(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        cliSerial->printf_P(PSTR("Relay on\n"));
        relay.on(0);
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }

        cliSerial->printf_P(PSTR("Relay off\n"));
        relay.off(0);
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
/*
 *  run a debug shell
 */
static int8_t
test_shell(uint8_t argc, const Menu::arg *argv)
{
    hal.util->run_debug_shell(cliSerial);
    return 0;
}
#endif

#if HIL_MODE == HIL_MODE_DISABLED
/*
 *  test the rangefinders
 */
static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
#if CONFIG_SONAR == ENABLED
	sonar.init();

    cliSerial->printf_P(PSTR("RangeFinder: %d devices detected\n"), sonar.num_sensors());

    print_hit_enter();
    while(1) {
        delay(100);
        sonar.update();

        cliSerial->printf_P(PSTR("Primary: health %d distance_cm %d \n"), (int)sonar.healthy(), sonar.distance_cm());
        cliSerial->printf_P(PSTR("All: device_0 type %d health %d distance_cm %d, device_1 type %d health %d distance_cm %d\n"), 
        (int)sonar._type[0], (int)sonar.healthy(0), sonar.distance_cm(0), (int)sonar._type[1], (int)sonar.healthy(1), sonar.distance_cm(1));

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
#endif
    return (0);
}
#endif

static void print_hit_enter()
{
    cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
}

#endif // CLI_ENABLED
