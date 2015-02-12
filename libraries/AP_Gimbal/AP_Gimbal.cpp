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
static float K_gimbalRate = 1.0f;
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
        

        ::printf("euler=(%.2f, %.2f, %.2f)\n", 
        degrees(euler.x), degrees(euler.y), degrees(euler.z));
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
    _ekf.RunEKF(1.0/100.0, _measurament.delta_angles, _measurament.delta_velocity, _measurament.joint_angles);


    // get the gimbal quaternion estimate
    Quaternion quatEst;
    _ekf.getQuat(quatEst);

/*
    float tilt;
    Vector3f velocity, euler, gyroBias;
    _ekf.getDebug(tilt, velocity, euler, gyroBias);
    ::printf("tilt=%.2f euler=(%.2f, %.2f, %.2f) bias=(%.1f, %.1f, %.1f) status = %d\n",
             tilt,
             degrees(euler.x), degrees(euler.y), degrees(euler.z),
             degrees(gyroBias.x), degrees(gyroBias.y), degrees(gyroBias.z),
             (int) _ekf.getStatus());
*/

        // Calculate the demanded quaternion orientation for the gimbal
        // set the demanded quaternion using demanded 321 Euler angles wrt earth frame
        Quaternion quatDem;
        quatDem.from_euler(0,0,0);

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

        // multiply the angle error vector by a gain to calculate a demanded gimbal rate
        gimbalRateDemVec = deltaAngErr * K_gimbalRate;

        // Constrain the demanded rate
        float length = gimbalRateDemVec.length();
        if (length > angRateLimit) {
            gimbalRateDemVec = gimbalRateDemVec * (angRateLimit / length);
        }

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