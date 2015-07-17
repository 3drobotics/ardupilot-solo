// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Mount_Backend.h>

extern const AP_HAL::HAL& hal;

// set_angle_targets - sets angle targets in degrees
void AP_Mount_Backend::set_angle_targets(float roll, float tilt, float pan)
{
    // set angle targets
    _angle_ef_target_rad.x = radians(roll);
    _angle_ef_target_rad.y = radians(tilt);
    _angle_ef_target_rad.z = radians(pan);

    // set the mode to mavlink targeting
    _frontend.set_mode(_instance, MAV_MOUNT_MODE_MAVLINK_TARGETING);
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount_Backend::set_roi_target(const struct Location &target_loc)
{
    // set the target gps location
    _state._roi_target = target_loc;

    // set the mode to GPS tracking mode
    _frontend.set_mode(_instance, MAV_MOUNT_MODE_GPS_POINT);
}

// configure_msg - process MOUNT_CONFIGURE messages received from GCS
void AP_Mount_Backend::configure_msg(mavlink_message_t* msg)
{
    __mavlink_mount_configure_t packet;
    mavlink_msg_mount_configure_decode(msg, &packet);

    // set mode
    _frontend.set_mode(_instance,(enum MAV_MOUNT_MODE)packet.mount_mode);

    // set which axis are stabilized
    _state._stab_roll = packet.stab_roll;
    _state._stab_tilt = packet.stab_pitch;
    _state._stab_pan = packet.stab_yaw;
}

// control_msg - process MOUNT_CONTROL messages received from GCS
void AP_Mount_Backend::control_msg(mavlink_message_t *msg)
{
    __mavlink_mount_control_t packet;
    mavlink_msg_mount_control_decode(msg, &packet);

    // interpret message fields based on mode
    switch (_frontend.get_mode(_instance)) {
        case MAV_MOUNT_MODE_RETRACT:
        case MAV_MOUNT_MODE_NEUTRAL:
            // do nothing with request if mount is retracted or in neutral position
            break;

        // set earth frame target angles from mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            set_angle_targets(packet.input_b*0.01f, packet.input_a*0.01f, packet.input_c*0.01f);
            break;

        // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
        case MAV_MOUNT_MODE_RC_TARGETING:
            // do nothing if pilot is controlling the roll, pitch and yaw
            break;

        // set lat, lon, alt position targets from mavlink message
        case MAV_MOUNT_MODE_GPS_POINT:
            Location target_location;
            memset(&target_location, 0, sizeof(target_location));
            target_location.lat = packet.input_a;
            target_location.lng = packet.input_b;
            target_location.alt = packet.input_c;
            target_location.flags.relative_alt = true;
            set_roi_target(target_location);
            break;

        default:
            // do nothing
            break;
    }
}

// update_targets_from_rc - updates angle targets using input from receiver
void AP_Mount_Backend::update_targets_from_rc()
{
#define rc_ch(i) RC_Channel::rc_channel(i-1)

    uint8_t roll_rc_in = _state._roll_rc_in;
    uint8_t tilt_rc_in = _state._tilt_rc_in;
    uint8_t pan_rc_in = _state._pan_rc_in;

    RC_Channel* roll_rc_ch = rc_ch(roll_rc_in);
    RC_Channel* tilt_rc_ch = rc_ch(tilt_rc_in);
    RC_Channel* pan_rc_ch = rc_ch(pan_rc_in);

    if (roll_rc_in && roll_rc_ch) {
        if (!_roll_rc_valid) {
            _roll_rc_valid = (roll_rc_ch->radio_in != 0);
        }

        if (_roll_rc_valid && roll_rc_ch->radio_in != 0) {
            if (_frontend._joystick_speed) {
                _angle_ef_target_rad.x += roll_rc_ch->norm_input_dz() * 0.0001f * _frontend._joystick_speed;
                constrain_float(_angle_ef_target_rad.x, radians(_state._roll_angle_min*0.01f), radians(_state._roll_angle_max*0.01f));
            } else {
                _angle_ef_target_rad.x = angle_input_rad(roll_rc_ch, _state._roll_angle_min, _state._roll_angle_max);
            }
        }
    }

    if (tilt_rc_in && tilt_rc_ch) {
        if (!_tilt_rc_valid) {
            _tilt_rc_valid = (tilt_rc_ch->radio_in != 0);
        }

        if (_tilt_rc_valid && tilt_rc_ch->radio_in != 0) {
            if (_frontend._joystick_speed) {
                _angle_ef_target_rad.y += tilt_rc_ch->norm_input_dz() * 0.0001f * _frontend._joystick_speed;
                constrain_float(_angle_ef_target_rad.y, radians(_state._tilt_angle_min*0.01f), radians(_state._tilt_angle_max*0.01f));
            } else {
                _angle_ef_target_rad.y = angle_input_rad(tilt_rc_ch, _state._tilt_angle_min, _state._tilt_angle_max);
            }
        }
    }

    if (pan_rc_in && pan_rc_ch) {
        if (!_pan_rc_valid) {
            _pan_rc_valid = (pan_rc_ch->radio_in != 0);
        }

        if (_pan_rc_valid && pan_rc_ch->radio_in != 0) {
            if (_frontend._joystick_speed) {
                _angle_ef_target_rad.z += pan_rc_ch->norm_input_dz() * 0.0001f * _frontend._joystick_speed;
                constrain_float(_angle_ef_target_rad.z, radians(_state._pan_angle_min*0.01f), radians(_state._pan_angle_max*0.01f));
            } else {
                _angle_ef_target_rad.z = angle_input_rad(pan_rc_ch, _state._pan_angle_min, _state._pan_angle_max);
            }
        }
    }
}

// returns the angle (degrees*100) that the RC_Channel input is receiving
int32_t AP_Mount_Backend::angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return (rc->get_reverse() ? -1 : 1) * (rc->radio_in - rc->radio_min) * (int32_t)(angle_max - angle_min) / (rc->radio_max - rc->radio_min) + (rc->get_reverse() ? angle_max : angle_min);
}

// returns the angle (radians) that the RC_Channel input is receiving
float AP_Mount_Backend::angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return radians(angle_input(rc, angle_min, angle_max)*0.01f);
}

// calc_angle_to_location - calculates the earth-frame roll, tilt and pan angles (and radians) to point at the given target
void AP_Mount_Backend::calc_angle_to_location(const struct Location &target, Vector3f& angles_to_target_rad, bool calc_tilt, bool calc_pan)
{
    float GPS_vector_x = (target.lng-_frontend._current_loc.lng)*cosf(ToRad((_frontend._current_loc.lat+target.lat)*0.00000005f))*0.01113195f;
    float GPS_vector_y = (target.lat-_frontend._current_loc.lat)*0.01113195f;
    float GPS_vector_z = (target.alt-_frontend._current_loc.alt);                 // baro altitude(IN CM) should be adjusted to known home elevation before take off (Set altimeter).
    float target_distance = 100.0f*pythagorous2(GPS_vector_x, GPS_vector_y);      // Careful , centimeters here locally. Baro/alt is in cm, lat/lon is in meters.

    // initialise all angles to zero
    angles_to_target_rad.zero();

    // tilt calcs
    if (calc_tilt) {
        angles_to_target_rad.y = atan2f(GPS_vector_z, target_distance);
    }

    // pan calcs
    if (calc_pan) {
        // calc absolute heading and then onvert to vehicle relative yaw
        angles_to_target_rad.z = wrap_PI(atan2f(GPS_vector_x, GPS_vector_y) - _frontend._ahrs.yaw);
    }
}
