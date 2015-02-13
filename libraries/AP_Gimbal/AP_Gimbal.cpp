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
    decode_feedback(msg);
    update_state();
    if (_ekf.getStatus()){
        send_control();        
 
        float tilt;
        Vector3f velocity, euler, gyroBias;
        _ekf.getDebug(tilt, velocity, euler, gyroBias);
        
        /*
        ::printf("euler=(%.2f, %.2f, %.2f) \t rates=(%.2f, %.2f, %.2f)\n", 
        degrees(euler.x), degrees(euler.y), degrees(euler.z),
        degrees(gimbalRateDemVec.x), degrees(gimbalRateDemVec.y), degrees(gimbalRateDemVec.z));
        */
        

        ::printf("(%+.1f, %+.1f, %+.1f)\t(%+.1f, %+.1f, %+.1f)\n", 
        degrees(euler.x), degrees(euler.y), degrees(euler.z), 
        degrees(gimbalRateDemVec.x), degrees(gimbalRateDemVec.y), degrees(gimbalRateDemVec.z));
    }
}
    

void AP_Gimbal::decode_feedback(mavlink_message_t *msg)
{
    mavlink_gimbal_feedback_t feedback_msg;
    uint8_t expected_id = _measurament.id +1;
    mavlink_msg_gimbal_feedback_decode(msg, &feedback_msg);
    _measurament.id = feedback_msg.id;

    if(expected_id !=_measurament.id){
        feedback_error_count++;
        ::printf("error count: %d\n", feedback_error_count);
    }

    _measurament.delta_angles.x = feedback_msg.gyrox;
    _measurament.delta_angles.y = feedback_msg.gyroy,
    _measurament.delta_angles.z = feedback_msg.gyroz;
    _measurament.delta_velocity.x = feedback_msg.accx,
    _measurament.delta_velocity.y = feedback_msg.accy,
    _measurament.delta_velocity.z = feedback_msg.accz;   
    _measurament.joint_angles.x = feedback_msg.joint_roll;
    _measurament.joint_angles.y = feedback_msg.joint_el,
    _measurament.joint_angles.z = feedback_msg.joint_az;
}


void AP_Gimbal::update_state()
{
    // Run the gimbal attitude and gyro bias estimator
    _ekf.RunEKF(delta_time, _measurament.delta_angles, _measurament.delta_velocity, _measurament.joint_angles);


    // get the gimbal quaternion estimate
    Quaternion quatEst;
    _ekf.getQuat(quatEst);


    float tilt;
    Vector3f velocity, euler, gyroBias;
    _ekf.getDebug(tilt, velocity, euler, gyroBias);
/*
    ::printf("tilt=%.2f euler=(%.2f, %.2f, %.2f) bias=(%.1f, %.1f, %.1f) status = %d\n",
             tilt,
             degrees(euler.x), degrees(euler.y), degrees(euler.z),
             degrees(gyroBias.x), degrees(gyroBias.y), degrees(gyroBias.z),
             (int) _ekf.getStatus());
*/

 
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

        // constrain the vehicle relative yaw rate demand
        gimbalRateDemVecYaw.z = constrain_float(gimbalRateDemVecYaw.z, -angRateLimit, angRateLimit);

        // Add the vehicle yaw rate after filtering and scaling
        // scaling is applied as a function of yaw rate such that the steady state error does not exceed the limit set
        vehicleYawRateFilt = (1.0f - yawRateFiltPole * delta_time) * vehicleYawRateFilt + yawRateFiltPole * delta_time * _ahrs.get_gyro().z;
        // calculate the maximum steady state rate error corresponding to the maximum permitted yaw angle error
        float maxRate = K_gimbalRate * yawErrorLimit;
        // compare max steady state rate error with vehicle yaw rate magnitude
        float excessRateMag = fabs(vehicleYawRateFilt) - maxRate;
        // if the difference is positive, then we need to use some forward rate demand to reduce steady state error
        if (excessRateMag > 0.0f) {
            if (vehicleYawRateFilt >= 0.0f) {
                gimbalRateDemVecYaw.z += excessRateMag;
            } else {
                gimbalRateDemVecYaw.z -= excessRateMag;
            }
        }

        // rotate into gimbal frame to calculate the gimbal rate vector required to keep the yaw gimbal centred
        gimbalRateDemVecYaw = Tvg * gimbalRateDemVecYaw;

        // Calculate the gimbal 321 Euler angle estimates relative to earth frame
        Vector3f eulerEst;
        quatEst.to_euler(eulerEst.x, eulerEst.y, eulerEst.z);

        // Calculate a demanded quaternion using the demanded roll and pitch and estimated yaw (yaw is slaved to the vehicle)
        Quaternion quatDem;
        quatDem.from_euler(0, 0, eulerEst.z);

        //divide the demanded quaternion by the estimated to get the error
        Quaternion quatErr = quatDem / quatEst;

        // convert the quaternion to an angle error vector
        Vector3f deltaAngErr;
        float scaler = 1.0f-quatErr[0]*quatErr[0];
        if (scaler > 1e-12) {
            scaler = 1.0f/sqrtf(scaler);
            if (quatErr[0] < 0.0f) {
                scaler *= -1.0f;
            }
            deltaAngErr.x = quatErr[1] * scaler;
            deltaAngErr.y = quatErr[2] * scaler;
            deltaAngErr.z = quatErr[3] * scaler;
        } else {
            deltaAngErr.zero();
        }

        // multiply the angle error vector by a gain to calculate a demanded gimbal rate required to control tilt
        Vector3f gimbalRateDemVecTilt = deltaAngErr * K_gimbalRate;

        // Constrain the tilt correction rate vector
        float length = gimbalRateDemVecTilt.length();
        if (length > angRateLimit) {
            gimbalRateDemVecTilt = gimbalRateDemVecTilt * (angRateLimit / length);
        }

        // Add the yaw and tilt control rate vectors
        gimbalRateDemVec = gimbalRateDemVecTilt + gimbalRateDemVecYaw;

        // the copter should not be using the gimbal yaw rate demand in this mode of operation, so we set it to zero
        vehicleYawRateDem = 0.0f;



        //Compensate for gyro bias
        //TODO send the gyro bias to the gimbal
        gimbalRateDemVec+= gyroBias;

}

void AP_Gimbal::send_control()
{
    mavlink_message_t msg;
    mavlink_gimbal_control_t control;
    control.target_system = _sysid;
    control.target_component = _compid;
    control.id = _measurament.id;

    control.ratex = gimbalRateDemVec.x;
    control.ratey = gimbalRateDemVec.y;
    control.ratez = gimbalRateDemVec.z;

    mavlink_msg_gimbal_control_encode(1, 1, &msg, &control);
    GCS_MAVLINK::routing.forward(&msg);


}