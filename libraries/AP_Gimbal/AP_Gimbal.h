// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/************************************************************
* AP_Gimbal -- library to control a 3 axis rate gimbal.		*
*															*
* Author:  Arthur Benemann;									*
*															*
* Purpose:                          						*
*															*
* Usage:	           										*
*															*
* Comments:                         						*
************************************************************/
#ifndef __AP_GIMBAL_H__
#define __AP_GIMBAL_H__

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <GCS_MAVLink.h>


class AP_Gimbal
{
public:
    //Constructor
    AP_Gimbal(const struct Location *current_loc, const AP_AHRS &ahrs, uint8_t sysid, uint8_t compid) :
        _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _current_loc = current_loc;
        _state.sysid = sysid;
        _state.compid = compid;
    }
    
   
        
    // MAVLink methods
    void                    status_msg(mavlink_channel_t chan);
    void                    receive_feedback(mavlink_message_t *msg);

    // should be called periodically
    void                    update_position();

    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

private:

    enum GIMBAL_AXIS{
        AZ,ROLL,EL
    };

    struct Gimbal_State {
        uint8_t sysid;
        uint8_t compid;
        mavlink_gimbal_feedback_t measuraments;
        float target_angles[3];     // degrees
        float target_rate[3];       // degrees/s
    };

    struct Gimbal_State             _state;
    const AP_AHRS                   &_ahrs;             //  Rotation matrix from earth to plane.
    const struct Location           *_current_loc;

    void                    send_control();
    void                    update_state();
};

#endif // __AP_MOUNT_H__
