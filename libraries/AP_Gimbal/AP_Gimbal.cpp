// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Gimbal.h>
#include <GCS.h>
#include <GCS_MAVLink.h>
#include <AP_SmallEKF.h>

const AP_Param::GroupInfo AP_Gimbal::var_info[] PROGMEM = {
    AP_GROUPEND
};

uint16_t feedback_error_count;
static float K_gimbalRate = 0.1f;
static float angRateLimit = 0.5f;

void AP_Gimbal::receive_feedback(mavlink_message_t *msg)
{
    update_targets_from_rc();
    decode_feedback(msg);
    update_state();
    if (_ekf.getStatus()){
        send_control();
    }
}
    

void AP_Gimbal::decode_feedback(mavlink_message_t *msg)
{
    mavlink_gimbal_report_t report_msg;
    mavlink_msg_gimbal_report_decode(msg, &report_msg);

    /*
    if(expected_id !=_measurament.id){
        feedback_error_count++;
        ::printf("error count: %d\n", feedback_error_count);
    }
    */
    _measurament.delta_time = report_msg.delta_time;
    _measurament.delta_angles.x = report_msg.delta_angle_x;
    _measurament.delta_angles.y = report_msg.delta_angle_y,
    _measurament.delta_angles.z = report_msg.delta_angle_z;
    _measurament.delta_velocity.x = report_msg.delta_velocity_x,
    _measurament.delta_velocity.y = report_msg.delta_velocity_y,
    _measurament.delta_velocity.z = report_msg.delta_velocity_z;   
    _measurament.joint_angles.x = report_msg.joint_roll;
    _measurament.joint_angles.y = report_msg.joint_el,
    _measurament.joint_angles.z = report_msg.joint_az;

    //apply joint angle compensation
    _measurament.joint_angles -= _joint_offsets;
}

void AP_Gimbal::update_state()
{
    // Run the gimbal attitude and gyro bias estimator
    _ekf.RunEKF(_measurament.delta_time, _measurament.delta_angles, _measurament.delta_velocity, _measurament.joint_angles);

    // get the gimbal quaternion estimate
    Quaternion quatEst;
    _ekf.getQuat(quatEst);
 
    // Add the control rate vectors
    gimbalRateDemVec.zero();
    gimbalRateDemVec += getGimbalRateDemVecYaw(quatEst);
    gimbalRateDemVec += getGimbalRateDemVecTilt(quatEst);
    //gimbalRateDemVec += getGimbalRateDemVecForward(quatEst);
    gimbalRateDemVec += getGimbalRateDemVecGyroBias();
}

Vector3f AP_Gimbal::getGimbalRateDemVecYaw(Quaternion quatEst)
{
        // Define rotation from vehicle to gimbal using a 312 rotation sequence
        Matrix3f Tvg;
        float cosPhi = cosf(_measurament.joint_angles.x);
        float cosTheta = cosf(_measurament.joint_angles.y);
        float sinPhi = sinf(_measurament.joint_angles.x);
        float sinTheta = sinf(_measurament.joint_angles.y);
        float sinPsi = sinf(_measurament.joint_angles.z);
        float cosPsi = cosf(_measurament.joint_angles.z);
        Tvg[0][0] = cosTheta*cosPsi-sinPsi*sinPhi*sinTheta;
        Tvg[1][0] = -sinPsi*cosPhi;
        Tvg[2][0] = cosPsi*sinTheta+cosTheta*sinPsi*sinPhi;
        Tvg[0][1] = cosTheta*sinPsi+cosPsi*sinPhi*sinTheta;
        Tvg[1][1] = cosPsi*cosPhi;
        Tvg[2][1] = sinPsi*sinTheta-cosTheta*cosPsi*sinPhi;
        Tvg[0][2] = -sinTheta*cosPhi;
        Tvg[1][2] = sinPhi;
        Tvg[2][2] = cosTheta*cosPhi;

        // multiply the yaw joint angle by a gain to calculate a demanded vehicle frame relative rate vector required to keep the yaw joint centred
        Vector3f gimbalRateDemVecYaw;
        gimbalRateDemVecYaw.z = - K_gimbalRate * _measurament.joint_angles.z;

        // Get filtered vehicle turn rate in earth frame
        vehicleYawRateFilt = (1.0f - yawRateFiltPole * _measurament.delta_time) * vehicleYawRateFilt + yawRateFiltPole * _measurament.delta_time * _ahrs.get_yaw_rate_earth();
        Vector3f vehicle_rate_ef(0,0,vehicleYawRateFilt);

         // calculate the maximum steady state rate error corresponding to the maximum permitted yaw angle error
        float maxRate = K_gimbalRate * yawErrorLimit;
        float vehicle_rate_mag_ef = vehicle_rate_ef.length();
        float excess_rate_correction = fabs(vehicle_rate_mag_ef) - maxRate; 
        if (vehicle_rate_mag_ef > maxRate) {
            if (vehicle_rate_ef.z>0.0f){
                gimbalRateDemVecYaw += _ahrs.get_dcm_matrix().transposed()*Vector3f(0,0,excess_rate_correction);    
            }else{
                gimbalRateDemVecYaw -= _ahrs.get_dcm_matrix().transposed()*Vector3f(0,0,excess_rate_correction);    
            }            
        }        

        // rotate into gimbal frame to calculate the gimbal rate vector required to keep the yaw gimbal centred
        gimbalRateDemVecYaw = Tvg * gimbalRateDemVecYaw;
        return gimbalRateDemVecYaw;
}

Vector3f AP_Gimbal::getGimbalRateDemVecTilt(Quaternion quatEst)
{
        // Calculate the gimbal 321 Euler angle estimates relative to earth frame
        Vector3f eulerEst;
        quatEst.to_euler(eulerEst.x, eulerEst.y, eulerEst.z);

        // Calculate a demanded quaternion using the demanded roll and pitch and estimated yaw (yaw is slaved to the vehicle)
        Quaternion quatDem;
        //TODO receive target from AP_Mount
        quatDem.from_euler(0, _angle_ef_target_rad.y, eulerEst.z);

       //divide the demanded quaternion by the estimated to get the error
        Quaternion quatErr = quatDem / quatEst;

        // multiply the angle error vector by a gain to calculate a demanded gimbal rate required to control tilt
        Vector3f vectorError;
        float scaler = 1.0f-quatErr[0]*quatErr[0];
        if (scaler > 1e-12) {
            scaler = 1.0f/sqrtf(scaler);
            if (quatErr[0] < 0.0f) {
                scaler *= -1.0f;
            }
            vectorError.x = quatErr[1] * scaler;
            vectorError.y = quatErr[2] * scaler;
            vectorError.z = quatErr[3] * scaler;
        } else {
            vectorError.zero();
        }

        Vector3f gimbalRateDemVecTilt = vectorError * K_gimbalRate;
        return gimbalRateDemVecTilt;
}

Vector3f AP_Gimbal::getGimbalRateDemVecForward(Quaternion quatEst)
{
        // quaternion demanded at the previous time step
        static Quaternion lastQuatDem;

        // calculate the delta rotation from the last to the current demand where the demand does not incorporate the copters yaw rotation
        Quaternion quatDemForward;
        quatDemForward.from_euler(0, _angle_ef_target_rad.y, 0);
        Quaternion deltaQuat = quatDemForward / lastQuatDem;
        lastQuatDem = quatDemForward;

        // convert to a rotation vector and divide by delta time to obtain a forward path rate demand
        Vector3f deltaVector;
        float scaler = 1.0f-deltaQuat[0]*deltaQuat[0];
        if (scaler > 1e-12) {
            scaler = 1.0f/sqrtf(scaler);
            if (deltaQuat[0] < 0.0f) {
                scaler *= -1.0f;
            }
            deltaVector.x = deltaQuat[1] * scaler;
            deltaVector.y = deltaQuat[2] * scaler;
            deltaVector.z = deltaQuat[3] * scaler;
        } else {
            deltaVector.zero();
        }     

        Vector3f gimbalRateDemVecForward = deltaVector * (1.0f / _measurament.delta_time);
        return gimbalRateDemVecForward;
}

Vector3f AP_Gimbal::getGimbalRateDemVecGyroBias()
{
    Vector3f gyroBias;
    _ekf.getGyroBias(gyroBias);
    return gyroBias;
}

void AP_Gimbal::send_control()
{
    mavlink_message_t msg;
    mavlink_gimbal_control_t control;
    control.target_system = _sysid;
    control.target_component = _compid;

    control.demanded_rate_x = gimbalRateDemVec.x;
    control.demanded_rate_y = gimbalRateDemVec.y;
    control.demanded_rate_z = gimbalRateDemVec.z;

    mavlink_msg_gimbal_control_encode(1, 1, &msg, &control);
    GCS_MAVLINK::routing.forward(&msg);
}

// returns the angle (degrees*100) that the RC_Channel input is receiving
int32_t angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return (rc->get_reverse() ? -1 : 1) * (rc->radio_in - rc->radio_min) * (int32_t)(angle_max - angle_min) / (rc->radio_max - rc->radio_min) + (rc->get_reverse() ? angle_max : angle_min);
}

// returns the angle (radians) that the RC_Channel input is receiving
float angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return radians(angle_input(rc, angle_min, angle_max)*0.01f);
}

// update_targets_from_rc - updates angle targets using input from receiver
void AP_Gimbal::update_targets_from_rc()
{
    float tilt = angle_input_rad(RC_Channel::rc_channel(tilt_rc_in-1), _tilt_angle_min, _tilt_angle_max);
    float rate = (tilt - _angle_ef_target_rad.y) / _measurament.delta_time;
    if(rate > _max_tilt_rate){
        _angle_ef_target_rad.y += _measurament.delta_time*_max_tilt_rate;
    }else if(rate < -_max_tilt_rate){
        _angle_ef_target_rad.y -= _measurament.delta_time*_max_tilt_rate;
    }else{
        _angle_ef_target_rad.y = tilt;
    }
}
