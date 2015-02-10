// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Gimbal.h>
#include <GCS.h>
#include <GCS_MAVLink.h>

const AP_Param::GroupInfo AP_Gimbal::var_info[] PROGMEM = {
    AP_GROUPEND
};

uint16_t feedback_error_count;
static float K_gimbalRate = 2.0f;
static float angRateLimit = 0.5f;

void AP_Gimbal::receive_feedback(mavlink_message_t *msg)
{
    decode_feedback(msg);
    update_state();
    send_control();
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

#define USE_JOINT_ONLY

void AP_Gimbal::update_state()
{
    #ifdef USE_JOINT_ONLY
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

    //Get rotation from earth to vehicle
    Matrix3f Tev; 
    Tev = _ahrs.get_dcm_matrix().transposed();

    //Get rotation from earth to gimbal 
    Matrix3f Teg; 
    Teg = Tvg*Tev;

    // convert vehicle to gimbal rotation matrix to a rotation vector using small angle approximation
    Vector3f deltaAngErr;
    deltaAngErr.x = (Teg[2][1] - Teg[1][2]) * 0.5f;
    deltaAngErr.y = (Teg[0][2] - Teg[2][0]) * 0.5f;
    deltaAngErr.z = (Teg[1][0] - Teg[0][1]) * 0.5f;

    // Zeroing the yaw demand in earth frame
    Vector3f deltaAngErrNED;
    deltaAngErrNED = Teg.transposed() * deltaAngErr;
    deltaAngErrNED.z = -_measurament.joint_angles.z * K_gimbalRate;
    deltaAngErr = Teg * deltaAngErrNED;

    // multiply the rotation vector by an error gain to calculate a demanded   vehicle frame relative rate vector
    gimbalRateDemVec = deltaAngErr * K_gimbalRate;

    // constrain the vehicle relative rate vector length
    float length = gimbalRateDemVec.length();
    if (length > angRateLimit) {
        gimbalRateDemVec = gimbalRateDemVec * (angRateLimit / length);
    }
    
    /*
    // rotate the earth relative vehicle angular rate vector into gimbal axes and add to obtain the demanded gimbal angular rate
    Vector3f forwardPathRateDem = Tvg * _ahrs.get_gyro();
    gimbalRateDemVec += forwardPathRateDem;
    */

    //::printf("joint \t%1.2f\t%1.2f\t%1.2f\t gyro \t%1.4f\t%1.4f\t%1.4f\t rate \t%1.2f\t%1.2f\t%1.2f\t\n",_state.feedback_msg.joint_roll,_state.feedback_msg.joint_el,_state.feedback_msg.joint_az,_state.feedback_msg.gyrox,_state.feedback_msg.gyroy,_state.feedback_msg.gyroz, _state.target_rate[X], _state.target_rate[Y], _state.target_rate[Z]);

    #else

    #endif
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