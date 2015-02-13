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
#include <AP_SmallEKF.h>
#include <AP_NavEKF.h>


class AP_Gimbal
{
public:
    //Constructor
    AP_Gimbal(const AP_AHRS_NavEKF &ahrs, uint8_t sysid, uint8_t compid) :
        _ahrs(ahrs),
        _ekf(ahrs),
        _joint_offsets(0.05,0.18,-0.02)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _sysid = sysid;
        _compid = compid;
        _initialised = false;        
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
    float vehicleYawRateDem;        // vehicle yaw rate demand


    const AP_AHRS_NavEKF             &_ahrs;             //  Rotation matrix from earth to plane.

    // internal variables
    bool _initialised;              // true once the driver has been initialised

    // state of small EKF for gimbal
    SmallEKF _ekf;


    void                    send_control();
    void                    update_state();
    void                    decode_feedback(mavlink_message_t *msg);

    
    float const delta_time = 1.0/100.0;

        // maximum vehicle yaw rate in rad/sec
    float const vehYawRateLim = 1.0f;

    // gimbal yaw offset relative to vehicle reference frame in radians, used to centre relative to visual or mechanical limits
    float const gimbalYawOffset = 0.03f;

    // filtered yaw rate from the vehicle
    float vehicleYawRateFilt = 0.0f;

    // circular frequency (rad/sec) constant of filter applied to forward path vehicle yaw rate
    // this frequency must not be larger than the update rate (Hz).
    // reducing it makes the gimbal yaw less responsive to vehicle yaw
    // increasing it makes the gimbal yawe more responsive to vehicle yaw
    float const yawRateFiltPole = 10.0f;

    // amount of yaw angle that we permit the gimbal to lag the vehicle when operating in slave mode
    // reducing this makes the gimbal respond more to vehicle yaw disturbances
    float const yawErrorLimit = 0.1f;

    Vector3f const _joint_offsets;
};

#endif // __AP_MOUNT_H__
