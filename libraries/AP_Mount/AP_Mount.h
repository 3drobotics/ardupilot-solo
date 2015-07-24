// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/************************************************************
* AP_mount -- library to control a 2 or 3 axis mount.		*
*															*
* Author:  Joe Holdsworth;									*
*		   Ritchie Wilson;									*
*		   Amilcar Lucas;									*
*		   Gregory Fletcher;								*
*          heavily modified by Randy Mackay                 *
*															*
* Purpose:  Move a 2 or 3 axis mount attached to vehicle,	*
*			Used for mount to track targets or stabilise	*
*			camera plus	other modes.						*
*															*
* Usage:	Use in main code to control	mounts attached to	*
*			vehicle.										*
*															*
* Comments: All angles in degrees * 100, distances in meters*
*			unless otherwise stated.						*
************************************************************/
#ifndef __AP_MOUNT_H__
#define __AP_MOUNT_H__

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <GCS_MAVLink.h>
#include <DataFlash.h>
#include <AP_Gimbal_Parameters.h>
#include "../RC_Channel/RC_Channel.h"
#include "../AP_SerialManager/AP_SerialManager.h"

// maximum number of mounts
#define AP_MOUNT_MAX_INSTANCES          1

// declare backend classes
class AP_Mount_Backend;
class AP_Mount_Servo;
class AP_Mount_MAVLink;
class AP_Mount_Alexmos;
class AP_Mount_SToRM32;
/*
  This is a workaround to allow the MAVLink backend access to the
  SmallEKF. It would be nice to find a neater solution to this
 */

class AP_Mount
{
    // declare backends as friends
    friend class AP_Mount_Backend;
    friend class AP_Mount_Servo;
    friend class AP_Mount_MAVLink;
    friend class AP_Mount_Alexmos;
    friend class AP_Mount_SToRM32;

public:

    // Enums
    enum MountType {
        Mount_Type_None = 0,            /// no mount
        Mount_Type_Servo = 1,           /// servo controlled mount
        Mount_Type_MAVLink = 2,         /// MAVLink controlled mount
        Mount_Type_Alexmos = 3,         /// Alexmos mount
        Mount_Type_SToRM32 = 4          /// SToRM32 mount
    };

    // Constructor
    AP_Mount(const AP_AHRS_TYPE &ahrs, const struct Location &current_loc);

    // init - detect and initialise all mounts
    void init(DataFlash_Class *dataflash ,const AP_SerialManager& serial_manager);

    // update - give mount opportunity to update servos.  should be called at 10hz or higher
    void update();

    // get_mount_type - returns the type of mount
    AP_Mount::MountType get_mount_type() const { return get_mount_type(_primary); }
    AP_Mount::MountType get_mount_type(uint8_t instance) const;

    // has_pan_control - returns true if the mount has yaw control (required for copters)
    bool has_pan_control() const { return has_pan_control(_primary); }
    bool has_pan_control(uint8_t instance) const;

    // get_mode - returns current mode of mount (i.e. Retracted, Neutral, RC_Targeting, GPS Point)
    enum MAV_MOUNT_MODE get_mode() const { return get_mode(_primary); }
    enum MAV_MOUNT_MODE get_mode(uint8_t instance) const;

    // set_mode - sets mount's mode
    //  returns true if mode is successfully set
    void set_mode(enum MAV_MOUNT_MODE mode) { return set_mode(_primary, mode); }
    void set_mode(uint8_t instance, enum MAV_MOUNT_MODE mode);

    void update_fast();

    // set_mode_to_default - restores the mode to it's default mode held in the MNT_DEFLT_MODE parameter
    //      this operation requires 230us on an APM2, 60us on a Pixhawk/PX4
    void set_mode_to_default() { set_mode_to_default(_primary); }
    void set_mode_to_default(uint8_t instance);

    // set_angle_targets - sets angle targets in degrees
    void set_angle_targets(float roll, float tilt, float pan) { set_angle_targets(_primary, roll, tilt, pan); }
    void set_angle_targets(uint8_t instance, float roll, float tilt, float pan);

    // set_roi_target - sets target location that mount should attempt to point towards
    void set_roi_target(const struct Location &target_loc) { set_roi_target(_primary,target_loc); }
    void set_roi_target(uint8_t instance, const struct Location &target_loc);

    // configure_msg - process MOUNT_CONFIGURE messages received from GCS
    void configure_msg(mavlink_message_t* msg) { configure_msg(_primary, msg); }
    void configure_msg(uint8_t instance, mavlink_message_t* msg);

    // control_msg - process MOUNT_CONTROL messages received from GCS
    void control_msg(mavlink_message_t* msg) { control_msg(_primary, msg); }
    void control_msg(uint8_t instance, mavlink_message_t* msg);

    // handle a GIMBAL_REPORT message
    void handle_gimbal_report(mavlink_channel_t chan, mavlink_message_t *msg);
    void handle_gimbal_torque_report(mavlink_channel_t chan, mavlink_message_t *msg);

    // send a GIMBAL_REPORT message to GCS
    void send_gimbal_report(mavlink_channel_t chan);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    void status_msg(mavlink_channel_t chan);

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

    AP_Gimbal_Parameters _externalParameters;

protected:

    // private members
    const AP_AHRS_TYPE     &_ahrs;
    const struct Location   &_current_loc;  // reference to the vehicle's current location

    // frontend parameters
    AP_Int8             _joystick_speed;    // joystick gain

    // front end members
    uint8_t             _num_instances;     // number of mounts instantiated
    uint8_t             _primary;           // primary mount
    AP_Mount_Backend    *_backends[AP_MOUNT_MAX_INSTANCES];         // pointers to instantiated mounts

    DataFlash_Class *_dataflash;

    // backend state including parameters
    struct mount_state {
        // Parameters
        AP_Int8         _type;              // mount type (None, Servo or MAVLink, see MountType enum)
        AP_Int8         _default_mode;      // default mode on startup and when control is returned from autopilot
        AP_Int8         _stab_roll;         // 1 = mount should stabilize earth-frame roll axis, 0 = no stabilization
        AP_Int8         _stab_tilt;         // 1 = mount should stabilize earth-frame pitch axis
        AP_Int8         _stab_pan;          // 1 = mount should stabilize earth-frame yaw axis

        // RC input channels from receiver used for direct angular input from pilot
        AP_Int8         _roll_rc_in;        // pilot provides roll input on this channel
        AP_Int8         _tilt_rc_in;        // pilot provides tilt input on this channel
        AP_Int8         _pan_rc_in;         // pilot provides pan input on this channel

        // Mount's physical limits
        AP_Int16        _roll_angle_min;    // min roll in 0.01 degree units
        AP_Int16        _roll_angle_max;    // max roll in 0.01 degree units
        AP_Int16        _tilt_angle_min;    // min tilt in 0.01 degree units
        AP_Int16        _tilt_angle_max;    // max tilt in 0.01 degree units
        AP_Int16        _pan_angle_min;     // min pan in 0.01 degree units
        AP_Int16        _pan_angle_max;     // max pan in 0.01 degree units

        AP_Vector3f     _retract_angles;    // retracted position for mount, vector.x = roll vector.y = tilt, vector.z=pan
        AP_Vector3f     _neutral_angles;    // neutral position for mount, vector.x = roll vector.y = tilt, vector.z=pan

        AP_Float        _roll_stb_lead;     // roll lead control gain
        AP_Float        _pitch_stb_lead;    // pitch lead control gain

        MAV_MOUNT_MODE  _mode;              // current mode (see MAV_MOUNT_MODE enum)
        struct Location _roi_target;        // roi target location
        
    } state[AP_MOUNT_MAX_INSTANCES];
};
#endif // __AP_MOUNT_H__
