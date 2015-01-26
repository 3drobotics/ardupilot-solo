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
        _ahrs(ahrs),
        _roll_angle(0.0f),
        _el_angle(0.0f),
        _az_angle(0.0f),
        _sysid(sysid),
        _compid(compid)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _current_loc = current_loc;
    }
        
    // MAVLink methods
    void                    status_msg(mavlink_channel_t chan);

    // should be called periodically
    void                    update_position();

    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

private:
    const uint8_t                   _sysid,_compid;     // Target system id

    const AP_AHRS                   &_ahrs;             //  Rotation matrix from earth to plane.
    const struct Location           *_current_loc;

    float                           _roll_angle; ///< degrees
    float                           _el_angle; ///< degrees
    float                           _az_angle;  ///< degrees

    void                    send_control();
};

#endif // __AP_MOUNT_H__
