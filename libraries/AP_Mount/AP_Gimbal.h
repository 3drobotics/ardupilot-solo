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
#include <AP_AccelCal.h>

enum gimbal_mode_t {
    GIMBAL_MODE_IDLE=0,
    GIMBAL_MODE_POS_HOLD,
    GIMBAL_MODE_POS_HOLD_FF,
    GIMBAL_MODE_STABILIZE
};

class AP_Gimbal : AP_AccelCal_Client
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
        _max_torque(5000.0f)
    {
        ahrs.get_ins().get_acal().register_client(this);
    }

    void    update_target(Vector3f newTarget);
    void    receive_feedback(mavlink_channel_t chan, mavlink_message_t *msg);

    void update_fast();

    bool present();

    Vector3f getGimbalEstimateEF();

    struct {
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
    gimbal_mode_t _mode;

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
    void extract_feedback(const mavlink_gimbal_report_t& report_msg);
    void update_joint_angle_est();

    void update_mode();

    bool isCopterFlipped();
    bool joints_near_limits();

    // Control loop functions
    Vector3f getGimbalRateDemVecYaw(const Quaternion &quatEst);
    Vector3f getGimbalRateDemVecTilt(const Quaternion &quatEst);
    Vector3f getGimbalRateDemVecForward(const Quaternion &quatEst);
    Vector3f getGimbalRateDemVecGyroBias();
    Vector3f getGimbalRateBodyLock();

    void gimbal_ang_vel_to_joint_rates(const Vector3f& ang_vel, Vector3f& joint_rates);
    void joint_rates_to_gimbal_ang_vel(const Vector3f& joint_rates, Vector3f& ang_vel);

    void readVehicleDeltaAngle(uint8_t ins_index, Vector3f &dAng);

    // joint angle filter states
    Vector3f vehicle_delta_angles;

    Quaternion vehicle_to_gimbal_quat;
    Quaternion vehicle_to_gimbal_quat_filt;
    Vector3f filtered_joint_angles;

    uint32_t _last_report_msg_ms;

    float _max_torque;

    float _ang_vel_mag_filt;

    AccelCalibrator _calibrator;
    void _acal_save_calibrations();
    bool _acal_ready_to_sample();
    AccelCalibrator* _acal_get_calibrator(uint8_t instance);

    mavlink_channel_t _chan;
};

#endif // AP_AHRS_NAVEKF_AVAILABLE

#endif // __AP_MOUNT_H__
