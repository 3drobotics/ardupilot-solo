// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Gimbal.h>
#include <GCS.h>
#include <GCS_MAVLink.h>

const AP_Param::GroupInfo AP_Gimbal::var_info[] PROGMEM = {
    AP_GROUPEND
};

uint16_t feedback_error_count;

void AP_Gimbal::receive_feedback(mavlink_message_t *msg)
{
    mavlink_msg_gimbal_feedback_decode(msg, &_state.measuraments);
    update_state();
    send_control();
}

void AP_Gimbal::update_state()
{
    /*
    _state.measuraments.accx; ///< X acceleration m/s/s
    _state.measuraments.accy; ///< Y acceleration m/s/s
    _state.measuraments.accz; ///< Z acceleration m/s/s
    
    _state.measuraments.gyrox; ///< Angular speed around X axis rad/s
    _state.measuraments.gyroy; ///< Angular speed around Y axis rad/s
    _state.measuraments.gyroz; ///< Angular speed around Z axis rad/s
    _state.measuraments.joint_az; ///<  Azimuth joint angle (-10000,10000)
    _state.measuraments.joint_roll; ///<  Roll joint angle (-10000,10000)
    _state.measuraments.joint_el; ///<  Elevation joint angle (-10000,10000)

    _ahrs.get_dcm_matrix();
    */

    // TODO add EKF code here

}

void AP_Gimbal::send_control()
{
    mavlink_message_t msg;
    mavlink_gimbal_control_t control;
    control.ratex = _state.target_rate[0];
    control.ratey = _state.target_rate[1];
    control.ratez = _state.target_rate[2];
    control.target_system = _state.sysid;
    control.target_component = _state.compid;
    control.id = _state.measuraments.id;

    mavlink_msg_gimbal_control_encode(1, 1, &msg, &control);

    GCS_MAVLINK::routing.forward(&msg);
}


void AP_Gimbal::status_msg(mavlink_channel_t chan)
{
    mavlink_msg_mount_status_send(chan,0,0, _state.target_angles[EL]*100, _state.target_angles[ROLL]*100, _state.target_angles[AZ]*100);
}

/// This one should be called periodically
void AP_Gimbal::update_position()
{
}