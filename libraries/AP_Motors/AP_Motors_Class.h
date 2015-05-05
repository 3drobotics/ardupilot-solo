// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_MOTORS_CLASS_H__
#define __AP_MOTORS_CLASS_H__

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_Notify.h>      // Notify library
#include <RC_Channel.h>     // RC Channel Library
#include <Filter.h>         // filter library

// offsets for motors in motor_out, _motor_filtered and _motor_to_channel_map arrays
#define AP_MOTORS_MOT_1 0
#define AP_MOTORS_MOT_2 1
#define AP_MOTORS_MOT_3 2
#define AP_MOTORS_MOT_4 3
#define AP_MOTORS_MOT_5 4
#define AP_MOTORS_MOT_6 5
#define AP_MOTORS_MOT_7 6
#define AP_MOTORS_MOT_8 7

#define APM1_MOTOR_TO_CHANNEL_MAP CH_1,CH_2,CH_3,CH_4,CH_7,CH_8,CH_10,CH_11
#define APM2_MOTOR_TO_CHANNEL_MAP CH_1,CH_2,CH_3,CH_4,CH_5,CH_6,CH_7,CH_8

#define AP_MOTORS_MAX_NUM_MOTORS 8

#define AP_MOTORS_DEFAULT_MIN_THROTTLE  130
#define AP_MOTORS_DEFAULT_MID_THROTTLE  500
#define AP_MOTORS_DEFAULT_MAX_THROTTLE  1000

// APM board definitions
#define AP_MOTORS_APM1  1
#define AP_MOTORS_APM2  2

// frame definitions
#define AP_MOTORS_PLUS_FRAME        0
#define AP_MOTORS_X_FRAME           1
#define AP_MOTORS_V_FRAME           2
#define AP_MOTORS_H_FRAME           3   // same as X frame but motors spin in opposite direction
#define AP_MOTORS_VTAIL_FRAME       4   // Lynxmotion Hunter VTail 400/500
#define AP_MOTORS_ATAIL_FRAME       5   // A-Shaped VTail Quads
#define AP_MOTORS_NEW_PLUS_FRAME    10  // NEW frames are same as original 4 but with motor orders changed to be clockwise from the front
#define AP_MOTORS_NEW_X_FRAME       11
#define AP_MOTORS_NEW_V_FRAME       12
#define AP_MOTORS_NEW_H_FRAME       13   // same as X frame but motors spin in opposite direction

// motor update rate
#define AP_MOTORS_SPEED_DEFAULT     490 // default output rate to the motors

#define THROTTLE_CURVE_ENABLED      1   // throttle curve disabled by default
#define THROTTLE_CURVE_MID_THRUST   52  // throttle which produces 1/2 the maximum thrust.  expressed as a percentage of the full throttle range (i.e 0 ~ 100)
#define THROTTLE_CURVE_MAX_THRUST   93  // throttle which produces the maximum thrust.  expressed as a percentage of the full throttle range (i.e 0 ~ 100)

#define AP_MOTORS_SPIN_WHEN_ARMED   70  // spin motors at this PWM value when armed

#define AP_MOTORS_YAW_HEADROOM_DEFAULT  200

#define AP_MOTORS_THR_LOW_CMP_DEFAULT   0.5f // ratio controlling the max throttle output during competing requests of low throttle from the pilot (or autopilot) and higher throttle for attitude control.  Higher favours Attitude over pilot input
#define AP_MOTORS_THST_EXPO_DEFAULT     0.5f // set to 0 for linear and 1 for second order approximation
#define AP_MOTORS_THST_MAX_DEFAULT      0.95f   // throttle which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_THST_BAT_MAX_DEFAULT  0.0f
#define AP_MOTORS_THST_BAT_MIN_DEFAULT  0.0f
#define AP_MOTORS_CURR_MAX_DEFAULT      0.0f    // current limiting max default
#define AP_MOTORS_BATT_VOLT_FILT_HZ     0.5f    // battery voltage filtered at 0.5hz
#define AP_MOTORS_THR_MIX_MIN_DEFAULT   0.1f    // minimum throttle mix
#define AP_MOTORS_THR_MIX_MID_DEFAULT   0.5f    // manual throttle mix
#define AP_MOTORS_THR_MIX_MAX_DEFAULT   0.9f    // maximum throttle mix

// bit mask for recording which limits we have reached when outputting to motors
#define AP_MOTOR_NO_LIMITS_REACHED  0x00
#define AP_MOTOR_ROLLPITCH_LIMIT    0x01
#define AP_MOTOR_YAW_LIMIT          0x02
#define AP_MOTOR_THROTTLE_LIMIT     0x04
#define AP_MOTOR_ANY_LIMIT          0xFF

// To-Do: replace this hard coded counter with a timer
#if HAL_CPU_CLASS < HAL_CPU_CLASS_75 || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
 // slow start increments - throttle increase per (100hz) iteration.  i.e. 5 = full speed in 2 seconds
 #define AP_MOTOR_SLOW_START_INCREMENT           10      // max throttle ramp speed (i.e. motors can reach full throttle in 1 second)
 #define AP_MOTOR_SLOW_START_LOW_END_INCREMENT   2       // min throttle ramp speed (i.e. motors will speed up from zero to _spin_when_armed speed in about 1 second)
#else
 // slow start increments - throttle increase per (400hz) iteration.  i.e. 1 = full speed in 2.5 seconds
 #define AP_MOTOR_SLOW_START_INCREMENT           3       // max throttle ramp speed (i.e. motors can reach full throttle in 0.8 seconds)
 #define AP_MOTOR_SLOW_START_LOW_END_INCREMENT   1       // min throttle ramp speed (i.e. motors will speed up from zero to _spin_when_armed speed in about 0.3 second)
#endif
/// @class      AP_Motors
class AP_Motors {
public:

    // Constructor
    AP_Motors(RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw, uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // init
    virtual void        Init() {}

    // set update rate to motors - a value in hertz
    virtual void        set_update_rate( uint16_t speed_hz ) { _speed_hz = speed_hz; };

    // set frame orientation (normally + or X)
    virtual void        set_frame_orientation( uint8_t new_orientation ) { _flags.frame_orientation = new_orientation; };

    // enable - starts allowing signals to be sent to motors
    virtual void        enable() = 0;

    // arm, disarm or check status status of motors
    bool                armed() const { return _flags.armed; };
    void                armed(bool arm);

    // set_min_throttle - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
    void                set_min_throttle(uint16_t min_throttle);

    // set_hover_throttle - sets the mid throttle which is close to the hover throttle of the copter
    // this is used to limit the amount that the stability patch will increase the throttle to give more room for roll, pitch and yaw control
    void                set_hover_throttle(uint16_t hov_thr) { _hover_out = hov_thr; }

    // get_hover_throttle_as_pwm - converts hover throttle to pwm (i.e. range 1000 ~ 2000)
    int16_t             get_hover_throttle_as_pwm() const;

    int16_t             throttle_min() const { return _min_throttle;}
    int16_t             throttle_max() const { return _max_throttle;}

    // set_roll, set_pitch, set_yaw, set_throttle
    void                set_roll(int16_t roll_in) { _rc_roll.servo_out = roll_in; };                    // range -4500 ~ 4500
    void                set_pitch(int16_t pitch_in) { _rc_pitch.servo_out = pitch_in; };                // range -4500 ~ 4500
    void                set_yaw(int16_t yaw_in) { _rc_yaw.servo_out = yaw_in; };                        // range -4500 ~ 4500
    void                set_throttle(float throttle_in) { _throttle_in = throttle_in; };    // range 0 ~ 1000
    void                set_stabilizing(bool stabilizing) { _flags.stabilizing = stabilizing; }

    // accessors for roll, pitch, yaw and throttle inputs to motors
    int16_t             get_roll() const { return _rc_roll.servo_out; }
    int16_t             get_pitch() const { return _rc_pitch.servo_out; }
    int16_t             get_yaw() const { return _rc_yaw.servo_out; }
    int16_t             get_throttle_out() const { return _rc_throttle.servo_out; }

    void                set_throttle_filter_cutoff(float filt_hz) { _throttle_filter.set_cutoff_frequency(filt_hz); }

    // output - sends commands to the motors
    void                output();

    // output_min - sends minimum values out to the motors
    virtual void        output_min() = 0;

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm) = 0;

    // throttle_pass_through - passes provided pwm directly to all motors - dangerous but used for initialising ESCs
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        throttle_pass_through(int16_t pwm);

    // set_yaw_headroom - set yaw headroom (yaw is given at least this amount of pwm)
    virtual void        set_yaw_headroom(int16_t pwm) { _yaw_headroom = pwm; }

    // set_voltage - set voltage to be used for output scaling
    virtual void        set_voltage(float volts){ _batt_voltage = volts; }

    // set_current - set current to be used for output scaling
    virtual void        set_current(float current){ _batt_current = current; }

    // set_density_ratio - sets air density as a proportion of sea level density
    void                set_air_density_ratio(float ratio) { _air_density_ratio = ratio; }

    // set_throttle_thr_mix - set desired throttle_thr_mix (actual throttle_thr_mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void                set_throttle_mix_min() { _throttle_thr_mix_desired = _thr_mix_min; }
    void                set_throttle_mix_mid() { _throttle_thr_mix_desired = AP_MOTORS_THR_MIX_MID_DEFAULT; }
    void                set_throttle_mix_max() { _throttle_thr_mix_desired = AP_MOTORS_THR_MIX_MAX_DEFAULT; }

    // get_throttle_thr_mix - get low throttle compensation value
    float               is_throttle_mix_min() { return (_throttle_thr_mix < 1.25f*_thr_mix_min); }

    // get_lift_max - get maximum lift ratio
    float               get_lift_max() { return _lift_max; }

    // get_batt_voltage_filt - get battery voltage ratio
    float               get_batt_voltage_filt() { return _batt_voltage_filt.get(); }

    // get_batt_resistance - get battery resistance approximation
    float               get_batt_resistance() { return _batt_resistance; }

    // get_throttle_limit - throttle limit ratio
    float               get_throttle_limit() { return _throttle_limit; }

    // returns warning throttle
    float               get_throttle_warn() { return rel_pwm_to_thr_range(_spin_when_armed); }

    // 1 if motor is enabled, 0 otherwise
    bool                motor_enabled[AP_MOTORS_MAX_NUM_MOTORS];

    // slow_start - set to true to slew motors from current speed to maximum
    // Note: this must be set immediately before a step up in throttle
    void                slow_start(bool true_false);

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask() = 0;

    // structure for holding motor limit flags
    struct AP_Motors_limit {
        uint8_t roll_pitch      : 1; // we have reached roll or pitch limit
        uint8_t yaw             : 1; // we have reached yaw limit
        uint8_t throttle_lower  : 1; // we have reached throttle's lower limit
        uint8_t throttle_upper  : 1; // we have reached throttle's upper limit
    } limit;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

protected:
    // output functions that should be overloaded by child classes
    virtual void        output_armed_stabilizing()=0;
    virtual void        output_armed_not_stabilizing()=0;
    virtual void        output_disarmed()=0;

    // update the throttle input filter
    void                update_throttle_filter();

    // update_max_throttle - updates the limits on _max_throttle for slow_start and current limiting flag
    void                update_max_throttle();

    // current_limit_max_throttle - current limit maximum throttle (called from update_max_throttle)
    void                current_limit_max_throttle();

    // apply_thrust_curve_and_volt_scaling - thrust curve and voltage adjusted pwm value (i.e. 1000 ~ 2000)
    int16_t             apply_thrust_curve_and_volt_scaling(int16_t pwm_out, int16_t pwm_min, int16_t pwm_max) const;

    // update_lift_max_from_batt_voltage - used for voltage compensation
    void                update_lift_max_from_batt_voltage();

    // update_battery_resistance - calculate battery resistance when throttle is above hover_out
    void                update_battery_resistance();

    // update_throttle_thr_mix - updates thr_low_comp value towards the target
    void                update_throttle_thr_mix();

    // return gain scheduling gain based on voltage and air density
    float               get_compensation_gain() const;

    float               rel_pwm_to_thr_range(float pwm) const;
    float               thr_range_to_rel_pwm(float thr) const;

    // flag bitmask
    struct AP_Motors_flags {
        uint8_t armed              : 1;    // 0 if disarmed, 1 if armed
        uint8_t stabilizing        : 1;    // 0 if not controlling attitude, 1 if controlling attitude
        uint8_t frame_orientation  : 4;    // PLUS_FRAME 0, X_FRAME 1, V_FRAME 2, H_FRAME 3, NEW_PLUS_FRAME 10, NEW_X_FRAME, NEW_V_FRAME, NEW_H_FRAME
        uint8_t slow_start         : 1;    // 1 if slow start is active
        uint8_t slow_start_low_end : 1;    // 1 just after arming so we can ramp up the spin_when_armed value
    } _flags;

    // mapping of motor number (as received from upper APM code) to RC channel output - used to account for differences between APM1 and APM2
    static const uint8_t _motor_to_channel_map[AP_MOTORS_MAX_NUM_MOTORS] PROGMEM;

    // parameters
    AP_Int16            _spin_when_armed;       // used to control whether the motors always spin when armed.  pwm value above radio_min

    AP_Int16            _yaw_headroom;          // yaw control is given at least this pwm range
    AP_Float            _thrust_curve_expo;     // curve used to linearize pwm to thrust conversion.  set to 0 for linear and 1 for second order approximation
    AP_Float            _thrust_curve_max;      // throttle which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
    AP_Float            _batt_voltage_max;      // maximum voltage used to scale lift
    AP_Float            _batt_voltage_min;      // minimum voltage used to scale lift
    AP_Float            _batt_current_max;      // current over which maximum throttle is limited
    AP_Float            _thr_mix_min;           // current over which maximum throttle is limited

    // internal variables
    RC_Channel&         _rc_roll;               // roll input in from users is held in servo_out
    RC_Channel&         _rc_pitch;              // pitch input in from users is held in servo_out
    RC_Channel&         _rc_throttle;           // throttle input in from users is held in servo_out
    RC_Channel&         _rc_yaw;                // yaw input in from users is held in servo_out
    uint16_t            _loop_rate;             // rate at which output() function is called (normally 400hz)
    uint16_t            _speed_hz;              // speed in hz to send updates to motors
    int16_t             _min_throttle;          // the minimum throttle to be sent to the motors when they're on (prevents motors stalling while flying)
    int16_t             _max_throttle;          // the maximum throttle to be sent to the motors (sometimes limited by slow start)
    int16_t             _hover_out;             // the estimated hover throttle as pct * 10 (i.e. 0 ~ 1000)
    int16_t             _spin_when_armed_ramped;// equal to _spin_when_armed parameter but slowly ramped up from zero
    float               _throttle_thr_mix;      // mix between throttle and hover throttle for 0 to 1 and ratio above hover throttle for >1
    float               _throttle_thr_mix_desired; // desired throttle_low_comp value, actual throttle_low_comp is slewed towards this value over 1~2 seconds

    // battery voltage compensation variables
    float               _batt_voltage;          // latest battery voltage reading
    float               _batt_voltage_resting;  // battery voltage reading at minimum throttle
    LowPassFilterFloat  _batt_voltage_filt;     // filtered battery voltage expressed as a percentage (0 ~ 1.0) of batt_voltage_max
    float               _batt_current;          // latest battery current reading
    float               _batt_current_resting;  // battery's current when motors at minimum
    float               _batt_resistance;       // battery's resistance calculated by comparing resting voltage vs in flight voltage
    int16_t             _batt_timer;            // timer used in battery resistance calcs
    float               _air_density_ratio;     // air density / sea level density - decreases in altitude
    float               _lift_max;              // maximum lift ratio from battery voltage
    float               _throttle_limit;        // ratio of throttle limit between hover and maximum
    float               _throttle_in;           // last throttle input from set_throttle caller
    LowPassFilterFloat  _throttle_filter;       // throttle input filter
};
#endif  // __AP_MOTORS_CLASS_H__
