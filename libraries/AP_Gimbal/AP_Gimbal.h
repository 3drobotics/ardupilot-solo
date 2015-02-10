// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/************************************************************
* AP_Gimbal -- library to control a 3 axis rate gimbal.		*
*															*
* Author:  Arthur Benemann, Paul Riseborough;									*
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
    AP_Gimbal(const AP_AHRS &ahrs, uint8_t sysid, uint8_t compid) :
        _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _sysid = sysid;
        _compid = compid;
    }
        
    // MAVLink methods
    void                    receive_feedback(mavlink_message_t *msg);

    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

private:
    uint8_t _sysid;
    uint8_t _compid;

    enum GIMBAL_JOINTS{
        AZ,ROLL,EL
    };

    enum GIMBAL_AXIS{
        X,Y,Z
    };

    struct Measurament {
        uint8_t id;
        Vector3f delta_angles;
        Vector3f delta_velocity;
        Vector3f joint_angles;
    } _measurament;


    Vector3f gimbalRateDemVec;       // degrees/s

    const AP_AHRS                   &_ahrs;             //  Rotation matrix from earth to plane.

    void                    send_control();
    void                    update_state();
    void                    decode_feedback(mavlink_message_t *msg);
};

#endif // __AP_MOUNT_H__
