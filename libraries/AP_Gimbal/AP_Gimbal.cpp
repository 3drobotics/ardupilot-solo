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
    uint8_t expected_id = _state.measuraments.id +1;
    mavlink_msg_gimbal_feedback_decode(msg, &_state.measuraments);
    if(expected_id !=_state.measuraments.id){
        feedback_error_count++;

        ::printf("error count: %d\t %d\t %d\n", feedback_error_count,expected_id,_state.measuraments.id);
    }


    update_state();
    send_control();
}



void AP_Gimbal::update_state()
{
    /*
    _state.measuraments.accx; ///< X acceleration m/s/s
    _state.measuraments.accy; ///< Y acceleration m/s/s
    _state.measuraments.accz; ///< Z acceleration m/s/s
    
    _state.measuraments.gyrox; ///< Angular speed around X axis rad/s
    _state.measuraments.gyroy; ///< Angular speed around Y axis rad/s
    _state.measuraments.gyroz; ///< Angular speed around Z axis rad/s
    _state.measuraments.joint_az; ///<  Azimuth joint angle (-10000,10000)
    _state.measuraments.joint_roll; ///<  Roll joint angle (-10000,10000)
    _state.measuraments.joint_el; ///<  Elevation joint angle (-10000,10000)

    _ahrs.get_dcm_matrix();
    */

    // TODO add EKF code here


    Vector3f gimbalRateDemVec;
    Vector3f joint_angles(_state.measuraments.joint_roll,
                          _state.measuraments.joint_el,
                          _state.measuraments.joint_az);

   // Define rotation from vehicle to gimbal using a 312 rotation sequence
    Matrix3f Tvg;
    float cosPhi = cosf(joint_angles.x);
    float cosTheta = cosf(joint_angles.y);
    float sinPhi = sinf(joint_angles.x);
    float sinTheta = sinf(joint_angles.y);
    float sinPsi = sinf(joint_angles.z);
    float cosPsi = cosf(joint_angles.z);
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
    deltaAngErrNED.z = -joint_angles.z * K_gimbalRate;
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

    _state.target_rate[X] = gimbalRateDemVec.x;
    _state.target_rate[Y] = gimbalRateDemVec.y;
    _state.target_rate[Z] = gimbalRateDemVec.z;

    //::printf("joint \t%1.2f\t%1.2f\t%1.2f\t gyro \t%1.4f\t%1.4f\t%1.4f\t rate \t%1.2f\t%1.2f\t%1.2f\t\n",_state.measuraments.joint_roll,_state.measuraments.joint_el,_state.measuraments.joint_az,_state.measuraments.gyrox,_state.measuraments.gyroy,_state.measuraments.gyroz, _state.target_rate[X], _state.target_rate[Y], _state.target_rate[Z]);
}

void AP_Gimbal::send_control()
{
    mavlink_message_t msg;
    mavlink_gimbal_control_t control;
    control.ratex = _state.target_rate[X];
    control.ratey = _state.target_rate[Y];
    control.ratez = _state.target_rate[Z];
    /*
    control.ratex = 0;
    control.ratey = 0;
    control.ratez = 0;
    */

    control.target_system = _state.sysid;
    control.target_component = _state.compid;
    control.id = _state.measuraments.id;

    mavlink_msg_gimbal_control_encode(1, 1, &msg, &control);

    GCS_MAVLINK::routing.forward(&msg);
}


void AP_Gimbal::status_msg(mavlink_channel_t chan)
{
    mavlink_msg_mount_status_send(chan,0,0, _state.target_angles[EL]*100, _state.target_angles[ROLL]*100, _state.target_angles[AZ]*100);
}

/// This one should be called periodically
void AP_Gimbal::update_position()
{
}