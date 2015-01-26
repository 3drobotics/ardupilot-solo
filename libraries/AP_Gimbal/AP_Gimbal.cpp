// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Gimbal.h>
#include <GCS.h>

const AP_Param::GroupInfo AP_Gimbal::var_info[] PROGMEM = {
    AP_GROUPEND
};


/// This one should be called periodically
void AP_Gimbal::update_position()
{
    ::printf("update position\n");
    send_control();
}

void AP_Gimbal::send_control()
{
    mavlink_message_t msg;
    mavlink_mount_control_t mount_control;
    mount_control.input_a = _el_angle*100;
    mount_control.input_b = _roll_angle*100;
    mount_control.input_c = _az_angle*100;
    mount_control.target_system = _sysid;
    mount_control.target_component = _compid;


    mount_control.input_a = 0;
    mount_control.input_b = 0;
    mount_control.input_c = 0;

    mavlink_msg_mount_control_encode(1, 1, &msg, &mount_control);
    GCS_MAVLINK::routing.forward(&msg);
}

/// Return mount status information
void AP_Gimbal::status_msg(mavlink_channel_t chan)
{
    mavlink_msg_mount_status_send(chan,0,0, _el_angle*100, _roll_angle*100, _az_angle*100);
}
