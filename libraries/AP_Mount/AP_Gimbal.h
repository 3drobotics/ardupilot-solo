// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/************************************************************
* AP_Gimbal -- library to control a 3 axis rate gimbal.     *
*                                                           *
* Author:  Arthur Benemann, Paul Riseborough;               *
*                                                           *
************************************************************/
#ifndef __AP_GIMBAL_H__
#define __AP_GIMBAL_H__

#include <AP_HAL.h>
#include <AP_AHRS.h>

#if AP_AHRS_NAVEKF_AVAILABLE

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <GCS_MAVLink.h>
#include <AP_Mount.h>
#include <AP_SmallEKF.h>
#include <AP_NavEKF.h>

class AP_Gimbal
{
public:
    //Constructor
    AP_Gimbal(const AP_AHRS_NavEKF &ahrs, AP_Gimbal_Parameters &parameters) :
        _ekf(ahrs),
        _ahrs(ahrs),
        _gimbalParams(parameters),
        vehicleYawRateFilt(0.0f),
        yawRateFiltPole(10.0f),
        lockedToBody(false),
        yawErrorLimit(0.1f),
        vehicle_delta_angles(),
        vehicle_to_gimbal_quat(),
        vehicle_to_gimbal_quat_filt(),
        filtered_joint_angles(),
        last_vehicle_gyro()
    {
    }

    void    update_target(Vector3f newTarget);
    void    receive_feedback(mavlink_channel_t chan, mavlink_message_t *msg);

    Vector3f getGimbalEstimateEF();

    struct Measurament {
        float delta_time;
        Vector3f delta_angles;
        Vector3f delta_velocity;
        Vector3f joint_angles;
    } _measurement;

    SmallEKF    _ekf;                   // state of small EKF for gimbal
    const AP_AHRS_NavEKF    &_ahrs;     //  Main EKF
    AP_Gimbal_Parameters &_gimbalParams;

    Vector3f    gimbalRateDemVec;       // degrees/s
    Vector3f    _angle_ef_target_rad;   // desired earth-frame roll, tilt and pan angles in radians

    bool lockedToBody;

private:
    

    // filtered yaw rate from the vehicle
    float vehicleYawRateFilt;

    // circular frequency (rad/sec) constant of filter applied to forward path vehicle yaw rate
    // this frequency must not be larger than the update rate (Hz).
    // reducing it makes the gimbal yaw less responsive to vehicle yaw
    // increasing it makes the gimbal yawe more responsive to vehicle yaw
    float const yawRateFiltPole;

    // amount of yaw angle that we permit the gimbal to lag the vehicle when operating in slave mode
    // reducing this makes the gimbal respond more to vehicle yaw disturbances
    float const yawErrorLimit;

    static const uint8_t _compid = MAV_COMP_ID_GIMBAL;

    void send_control(mavlink_channel_t chan);
    void update_state();
    void decode_feedback(mavlink_message_t *msg);
    void update_joint_angle_est();

    bool isCopterFlipped();

    // Control loop functions
    Vector3f getGimbalRateDemVecYaw(const Quaternion &quatEst);
    Vector3f getGimbalRateDemVecTilt(const Quaternion &quatEst);
    Vector3f getGimbalRateDemVecForward(const Quaternion &quatEst);
    Vector3f getGimbalRateDemVecGyroBias();
    Vector3f getGimbalRateBodyLock();

    void gimbal_ang_vel_to_joint_rates(const Vector3f& ang_vel, Vector3f& joint_rates);
    void joint_rates_to_gimbal_ang_vel(const Vector3f& joint_rates, Vector3f& ang_vel);

    // joint angle filter states
    Vector3f vehicle_delta_angles;

    Quaternion vehicle_to_gimbal_quat;
    Quaternion vehicle_to_gimbal_quat_filt;
    Vector3f filtered_joint_angles;
    Vector3f last_vehicle_gyro;
};

#endif // AP_AHRS_NAVEKF_AVAILABLE

#endif // __AP_MOUNT_H__
