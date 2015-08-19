/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AP_Progmem.h>
#include "Compass.h"
#include <AP_Vehicle.h>

extern AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define COMPASS_LEARN_DEFAULT 0
#else
#define COMPASS_LEARN_DEFAULT 1
#endif

const AP_Param::GroupInfo Compass::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: OFS_X
    // @DisplayName: Compass offsets on the X axis
    // @Description: Offset to be added to the compass x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS_Y
    // @DisplayName: Compass offsets on the Y axis
    // @Description: Offset to be added to the compass y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS_Z
    // @DisplayName: Compass offsets on the Z axis
    // @Description: Offset to be added to the compass z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1
    AP_GROUPINFO("OFS",    1, Compass, _state[0].offset, 0),

    // @Param: DEC
    // @DisplayName: Compass declination
    // @Description: An angle to compensate between the true north and magnetic north
    // @Range: -3.142 3.142
    // @Units: Radians
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DEC",    2, Compass, _declination, 0),

    // @Param: LEARN
    // @DisplayName: Learn compass offsets automatically
    // @Description: Enable or disable the automatic learning of compass offsets
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LEARN",  3, Compass, _learn, COMPASS_LEARN_DEFAULT),

    // @Param: USE
    // @DisplayName: Use compass for yaw
    // @Description: Enable or disable the use of the compass (instead of the GPS) for determining heading
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE",    4, Compass, _state[0].use_for_yaw, 1), // true if used for DCM yaw

#if !defined( __AVR_ATmega1280__ )
    // @Param: AUTODEC
    // @DisplayName: Auto Declination
    // @Description: Enable or disable the automatic calculation of the declination based on gps location
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTODEC",5, Compass, _auto_declination, 1),
#endif

    // @Param: MOTCT
    // @DisplayName: Motor interference compensation type
    // @Description: Set motor interference compensation type to disabled, throttle or current.  Do not change manually.
    // @Values: 0:Disabled,1:Use Throttle,2:Use Current
    // @Increment: 1
    AP_GROUPINFO("MOTCT",    6, Compass, _motor_comp_type, AP_COMPASS_MOT_COMP_DISABLED),

    // @Param: MOT_X
    // @DisplayName: Motor interference compensation for body frame X axis
    // @Description: Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT_Y
    // @DisplayName: Motor interference compensation for body frame Y axis
    // @Description: Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT_Z
    // @DisplayName: Motor interference compensation for body frame Z axis
    // @Description: Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1
    AP_GROUPINFO("MOT",    7, Compass, _state[0].motor_compensation, 0),

    // @Param: ORIENT
    // @DisplayName: Compass orientation
    // @Description: The orientation of the compass relative to the autopilot board. This will default to the right value for each board type, but can be changed if you have an external compass. See the documentation for your external compass for the right value. The correct orientation should give the X axis forward, the Y axis to the right and the Z axis down. So if your aircraft is pointing west it should show a positive value for the Y axis, and a value close to zero for the X axis. On a PX4 or Pixhawk with an external compass the correct value is zero if the compass is correctly oriented. NOTE: This orientation is combined with any AHRS_ORIENTATION setting.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll90
    AP_GROUPINFO("ORIENT", 8, Compass, _state[0].orientation, ROTATION_NONE),

    // @Param: EXTERNAL
    // @DisplayName: Compass is attached via an external cable
    // @Description: Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk, but must be set correctly on an APM2. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option
    // @Values: 0:Internal,1:External
    // @User: Advanced
    AP_GROUPINFO("EXTERNAL", 9, Compass, _state[0].external, 0),

#if COMPASS_MAX_INSTANCES > 1
    // @Param: OFS2_X
    // @DisplayName: Compass2 offsets on the X axis
    // @Description: Offset to be added to compass2's x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS2_Y
    // @DisplayName: Compass2 offsets on the Y axis
    // @Description: Offset to be added to compass2's y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS2_Z
    // @DisplayName: Compass2 offsets on the Z axis
    // @Description: Offset to be added to compass2's z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1
    AP_GROUPINFO("OFS2",    10, Compass, _state[1].offset, 0),

    // @Param: MOT2_X
    // @DisplayName: Motor interference compensation to compass2 for body frame X axis
    // @Description: Multiplied by the current throttle and added to compass2's x-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT2_Y
    // @DisplayName: Motor interference compensation to compass2 for body frame Y axis
    // @Description: Multiplied by the current throttle and added to compass2's y-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT2_Z
    // @DisplayName: Motor interference compensation to compass2 for body frame Z axis
    // @Description: Multiplied by the current throttle and added to compass2's z-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1
    AP_GROUPINFO("MOT2",    11, Compass, _state[1].motor_compensation, 0),

    // @Param: PRIMARY
    // @DisplayName: Choose primary compass
    // @Description: If more than one compass is available this selects which compass is the primary. Normally 0=External, 1=Internal. If no External compass is attached this parameter is ignored
    // @Values: 0:FirstCompass,1:SecondCompass
    // @User: Advanced
    AP_GROUPINFO("PRIMARY", 12, Compass, _primary, 0),
#endif

#if COMPASS_MAX_INSTANCES > 2
    // @Param: OFS3_X
    // @DisplayName: Compass3 offsets on the X axis
    // @Description: Offset to be added to compass3's x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS3_Y
    // @DisplayName: Compass3 offsets on the Y axis
    // @Description: Offset to be added to compass3's y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS3_Z
    // @DisplayName: Compass3 offsets on the Z axis
    // @Description: Offset to be added to compass3's z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1
    AP_GROUPINFO("OFS3",    13, Compass, _state[2].offset, 0),

    // @Param: MOT3_X
    // @DisplayName: Motor interference compensation to compass3 for body frame X axis
    // @Description: Multiplied by the current throttle and added to compass3's x-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT3_Y
    // @DisplayName: Motor interference compensation to compass3 for body frame Y axis
    // @Description: Multiplied by the current throttle and added to compass3's y-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT3_Z
    // @DisplayName: Motor interference compensation to compass3 for body frame Z axis
    // @Description: Multiplied by the current throttle and added to compass3's z-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1
    AP_GROUPINFO("MOT3",    14, Compass, _state[2].motor_compensation, 0),
#endif

#if COMPASS_MAX_INSTANCES > 1
    // @Param: DEV_ID
    // @DisplayName: Compass device id
    // @Description: Compass device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID",  15, Compass, _state[0].dev_id, 0),

    // @Param: DEV_ID2
    // @DisplayName: Compass2 device id
    // @Description: Second compass's device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID2", 16, Compass, _state[1].dev_id, 0),
#endif

#if COMPASS_MAX_INSTANCES > 2
    // @Param: DEV_ID3
    // @DisplayName: Compass3 device id
    // @Description: Third compass's device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID3", 17, Compass, _state[2].dev_id, 0),
#endif

#if COMPASS_MAX_INSTANCES > 1
    // @Param: USE2
    // @DisplayName: Compass2 used for yaw
    // @Description: Enable or disable the second compass for determining heading.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE2",    18, Compass, _state[1].use_for_yaw, 1),

    // @Param: ORIENT2
    // @DisplayName: Compass2 orientation
    // @Description: The orientation of the second compass relative to the frame (if external) or autopilot board (if internal).
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll90
    AP_GROUPINFO("ORIENT2", 19, Compass, _state[1].orientation, ROTATION_NONE),

    // @Param: EXTERN2
    // @DisplayName: Compass2 is attached via an external cable
    // @Description: Configure second compass so it is attached externally. This is auto-detected on PX4 and Pixhawk.
    // @Values: 0:Internal,1:External
    // @User: Advanced
    AP_GROUPINFO("EXTERN2",20, Compass, _state[1].external, 0),
#endif

#if COMPASS_MAX_INSTANCES > 2
    // @Param: USE3
    // @DisplayName: Compass3 used for yaw
    // @Description: Enable or disable the third compass for determining heading.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE3",    21, Compass, _state[2].use_for_yaw, 1),

    // @Param: ORIENT3
    // @DisplayName: Compass3 orientation
    // @Description: The orientation of the third compass relative to the frame (if external) or autopilot board (if internal).
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll90
    AP_GROUPINFO("ORIENT3", 22, Compass, _state[2].orientation, ROTATION_NONE),

    // @Param: EXTERN3
    // @DisplayName: Compass3 is attached via an external cable
    // @Description: Configure third compass so it is attached externally. This is auto-detected on PX4 and Pixhawk.
    // @Values: 0:Internal,1:External
    // @User: Advanced
    AP_GROUPINFO("EXTERN3",23, Compass, _state[2].external, 0),
#endif

    // @Param: DIA_X
    // @DisplayName: Compass soft-iron diagonal X component
    // @Description: DIA_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA_Y
    // @DisplayName: Compass soft-iron diagonal Y component
    // @Description: DIA_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA_Z
    // @DisplayName: Compass soft-iron diagonal Z component
    // @Description: DIA_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA",    24, Compass, _state[0].diagonals, 0),

    // @Param: ODI_X
    // @DisplayName: Compass soft-iron off-diagonal X component
    // @Description: ODI_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI_Y
    // @DisplayName: Compass soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI_Z
    // @DisplayName: Compass soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI",    25, Compass, _state[0].offdiagonals, 0),

#if COMPASS_MAX_INSTANCES > 1
    // @Param: DIA2_X
    // @DisplayName: Compass2 soft-iron diagonal X component
    // @Description: DIA_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA2_Y
    // @DisplayName: Compass2 soft-iron diagonal Y component
    // @Description: DIA_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA2_Z
    // @DisplayName: Compass2 soft-iron diagonal Z component
    // @Description: DIA_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA2",    26, Compass, _state[1].diagonals, 0),

    // @Param: ODI2_X
    // @DisplayName: Compass2 soft-iron off-diagonal X component
    // @Description: ODI_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI2_Y
    // @DisplayName: Compass2 soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI2_Z
    // @DisplayName: Compass2 soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI2",    27, Compass, _state[1].offdiagonals, 0),
#endif

#if COMPASS_MAX_INSTANCES > 2
    // @Param: DIA3_X
    // @DisplayName: Compass3 soft-iron diagonal X component
    // @Description: DIA_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA3_Y
    // @DisplayName: Compass3 soft-iron diagonal Y component
    // @Description: DIA_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA3_Z
    // @DisplayName: Compass3 soft-iron diagonal Z component
    // @Description: DIA_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA3",    28, Compass, _state[2].diagonals, 0),

    // @Param: ODI3_X
    // @DisplayName: Compass3 soft-iron off-diagonal X component
    // @Description: ODI_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI3_Y
    // @DisplayName: Compass3 soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI3_Z
    // @DisplayName: Compass3 soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI3",    29, Compass, _state[2].offdiagonals, 0),
#endif

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
Compass::Compass(void) :
    _last_update_usec(0),
    _null_init_done(false),
    _backend_count(0),
    _compass_count(0),
    _board_orientation(ROTATION_NONE),
    _thr_or_curr(0.0f),
    _hil_mode(false)
{
    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i=0; i<COMPASS_MAX_BACKEND; i++) {
        _backends[i] = NULL;
    }    

#if COMPASS_MAX_INSTANCES > 1
    // default device ids to zero.  init() method will overwrite with the actual device ids
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        _state[i].dev_id = 0;
    }
#endif
}

// Default init method
//
bool
Compass::init()
{
    if (_compass_count == 0) {
        // detect available backends. Only called once
        _detect_backends();
    }
    if (_compass_count != 0) {
        // get initial health status
        hal.scheduler->delay(100);
        read();
    }
    return true;
}

//  Register a new compass instance
//
uint8_t Compass::register_compass(void)
{
    if (_compass_count == COMPASS_MAX_INSTANCES) {
        hal.scheduler->panic(PSTR("Too many compass instances"));
    }
    return _compass_count++;
}

/*
  try to load a backend
 */
void 
Compass::_add_backend(AP_Compass_Backend *(detect)(Compass &))
{
    if (_backend_count == COMPASS_MAX_BACKEND) {
        hal.scheduler->panic(PSTR("Too many compass backends"));
    }
    _backends[_backend_count] = detect(*this);
    if (_backends[_backend_count] != NULL) {
        _backend_count++;
    }
}

/*
  detect available backends for this board
 */
void 
Compass::_detect_backends(void)
{
    if (_hil_mode) {
        _add_backend(AP_Compass_HIL::detect);
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_NONE
    _add_backend(AP_Compass_HMC5843::detect);
    _add_backend(AP_Compass_AK8963_MPU9250::detect);
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_HIL
    _add_backend(AP_Compass_HIL::detect);
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_HMC5843
    _add_backend(AP_Compass_HMC5843::detect);
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_PX4 || HAL_COMPASS_DEFAULT == HAL_COMPASS_VRBRAIN
    _add_backend(AP_Compass_PX4::detect);
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_AK8963_MPU9250
    _add_backend(AP_Compass_AK8963_MPU9250::detect);
#else
    #error Unrecognised HAL_COMPASS_TYPE setting
#endif

    if (_backend_count == 0 ||
        _compass_count == 0) {
        hal.scheduler->panic(PSTR("No Compass backends available"));
    }
}

void 
Compass::accumulate(void)
{    
    for (uint8_t i=0; i< _backend_count; i++) {
        // call accumulate on each of the backend
        _backends[i]->accumulate();
    }
}

bool 
Compass::read(void)
{
    for (uint8_t i=0; i< _backend_count; i++) {
        // call read on each of the backend. This call updates field[i]
        _backends[i]->read();
    }    
    for (uint8_t i=0; i < COMPASS_MAX_INSTANCES; i++) {
        _state[i].healthy = (hal.scheduler->millis() - _state[i].last_update_ms < 500);
    }
    return healthy();
}

uint8_t
Compass::get_healthy_mask() const
{
    uint8_t healthy_mask = 0;
    for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if(healthy(i)) {
            healthy_mask |= 1 << i;
        }
    }
    return healthy_mask;
}

void
Compass::set_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offset.set(offsets);
    }
}

void
Compass::set_and_save_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offset.set(offsets);
        save_offsets(i);
    }
}

void
Compass::set_and_save_diagonals(uint8_t i, const Vector3f &diagonals)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].diagonals.set_and_save(diagonals);
    }
}

void
Compass::set_and_save_offdiagonals(uint8_t i, const Vector3f &offdiagonals)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offdiagonals.set_and_save(offdiagonals);
    }
}

void
Compass::save_offsets(uint8_t i)
{
    _state[i].offset.save();  // save offsets
#if COMPASS_MAX_INSTANCES > 1
    _state[i].dev_id.save();  // save device id corresponding to these offsets
#endif
}

void
Compass::save_offsets(void)
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        save_offsets(i);
    }
}

void
Compass::set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor)
{
    _state[i].motor_compensation.set(motor_comp_factor);
}

void
Compass::save_motor_compensation()
{
    _motor_comp_type.save();
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        _state[k].motor_compensation.save();
    }
}

void
Compass::set_initial_location(int32_t latitude, int32_t longitude)
{
    // if automatic declination is configured, then compute
    // the declination based on the initial GPS fix
    if (_auto_declination) {
        // Set the declination based on the lat/lng from GPS
        _declination.set(radians(
                AP_Declination::get_declination(
                    (float)latitude / 10000000,
                    (float)longitude / 10000000)));
    }
}

/// return true if the compass should be used for yaw calculations
bool
Compass::use_for_yaw(void) const
{
    uint8_t prim = get_primary();
    return healthy(prim) && use_for_yaw(prim);
}

/// return true if the specified compass can be used for yaw calculations
bool
Compass::use_for_yaw(uint8_t i) const
{
    return _state[i].use_for_yaw;
}

void
Compass::set_declination(float radians, bool save_to_eeprom)
{
    if (save_to_eeprom) {
        _declination.set_and_save(radians);
    }else{
        _declination.set(radians);
    }
}

float
Compass::get_declination() const
{
    return _declination.get();
}

/*
  calculate a compass heading given the attitude from DCM and the mag vector
 */
float
Compass::calculate_heading(const Matrix3f &dcm_matrix) const
{
    float cos_pitch_sq = 1.0f-(dcm_matrix.c.x*dcm_matrix.c.x);

    // Tilt compensated magnetic field Y component:
    const Vector3f &field = get_field();

    float headY = field.y * dcm_matrix.c.z - field.z * dcm_matrix.c.y;

    // Tilt compensated magnetic field X component:
    float headX = field.x * cos_pitch_sq - dcm_matrix.c.x * (field.y * dcm_matrix.c.y + field.z * dcm_matrix.c.z);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    float heading = constrain_float(atan2f(-headY,headX), -3.15f, 3.15f);

    // Declination correction (if supplied)
    if( fabsf(_declination) > 0.0f )
    {
        heading = heading + _declination;
        if (heading > PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0f * PI);
        else if (heading < -PI)
            heading += (2.0f * PI);
    }

    return heading;
}

/// Returns True if the compasses have been configured (i.e. offsets saved)
///
/// @returns                    True if compass has been configured
///
bool Compass::configured(uint8_t i)
{
    // exit immediately if instance is beyond the number of compasses we have available
    if (i > get_count()) {
        return false;
    }

    // exit immediately if all offsets are zero
    if (get_offsets(i).length() == 0.0f) {
        return false;
    }

#if COMPASS_MAX_INSTANCES > 1
    // backup detected dev_id
    int32_t dev_id_orig = _state[i].dev_id;

    // load dev_id from eeprom
    _state[i].dev_id.load();

    // if different then the device has not been configured
    if (_state[i].dev_id != dev_id_orig) {
        // restore device id
        _state[i].dev_id = dev_id_orig;
        // return failure
        return false;
    }
#endif

    // if we got here then it must be configured
    return true;
}

bool Compass::configured(void)
{
    bool all_configured = true;
    for(uint8_t i=0; i<get_count(); i++) {
        all_configured = all_configured && (!use_for_yaw(i) || configured(i));
    }
    return all_configured;
}

// Update raw magnetometer values from HIL data
//
void Compass::setHIL(float roll, float pitch, float yaw)
{
    Matrix3f R;

    // create a rotation matrix for the given attitude
    R.from_euler(roll, pitch, yaw);

    if (_hil.last_declination != get_declination()) {
        _setup_earth_field();
        _hil.last_declination = get_declination();
    }

    // convert the earth frame magnetic vector to body frame, and
    // apply the offsets
    _hil.field = R.mul_transpose(_hil.Bearth);

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _hil.field.rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _hil.field.rotate((enum Rotation)_state[0].orientation.get());

    if (!_state[0].external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        _hil.field.rotate(_board_orientation);
    }
}

// Update raw magnetometer values from HIL mag vector
//
void Compass::setHIL(const Vector3f &mag)
{
    _hil.field = mag;
    _last_update_usec = hal.scheduler->micros();
}

const Vector3f& Compass::getHIL() const {
    return _hil.field;
}

// setup _Bearth
void Compass::_setup_earth_field(void)
{
    // assume a earth field strength of 400
    _hil.Bearth(400, 0, 0);
    
    // rotate _Bearth for inclination and declination. -66 degrees
    // is the inclination in Canberra, Australia
    Matrix3f R;
    R.from_euler(0, ToRad(66), get_declination());
    _hil.Bearth = R * _hil.Bearth;
}

/*
  set the type of motor compensation to use
 */
void Compass::motor_compensation_type(const uint8_t comp_type)
{
    if (_motor_comp_type <= AP_COMPASS_MOT_COMP_CURRENT && _motor_comp_type != (int8_t)comp_type) {
        _motor_comp_type = (int8_t)comp_type;
        _thr_or_curr = 0;                               // set current current or throttle to zero
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            set_motor_compensation(i, Vector3f(0,0,0)); // clear out invalid compensation vectors
        }
    }
}

uint8_t Compass::get_use_mask() const
{
    uint8_t ret = 0;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (use_for_yaw(i)) {
            ret |= 1 << i;
        }
    }
    return ret;
}

bool Compass::consistent(uint8_t mask) const
{
    Vector3f primary_mag_vec = get_field();
    Vector3f primary_mag_vec_norm = primary_mag_vec;
    primary_mag_vec_norm.normalize();

    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if ((mask&(1<<i)) != 0) {
            // get next compass
            Vector3f mag_vec = get_field(i);
            Vector3f mag_vec_norm = mag_vec;
            mag_vec_norm.normalize();
            float xyz_ang_diff = acosf(mag_vec_norm * primary_mag_vec_norm);
            float xy_ang_diff  = acosf(mag_vec_norm.x*primary_mag_vec_norm.x + mag_vec_norm.y*primary_mag_vec_norm.y);
            float xy_len_diff  = sqrtf(sq(primary_mag_vec.x - mag_vec.x) + sq(primary_mag_vec.y - mag_vec.y));

            // check for gross misalignment on all axes
            bool xyz_ang_diff_large = xyz_ang_diff > AP_COMPASS_MAX_XYZ_ANG_DIFF;

            // check for an unacceptable angle difference on the xy plane
            bool xy_ang_diff_large = fabsf(xy_ang_diff) > AP_COMPASS_MAX_XY_ANG_DIFF;

            // check for an unacceptable length difference on the xy plane
            bool xy_length_diff_large = xy_len_diff > AP_COMPASS_MAX_XY_LENGTH_DIFF;

            // check for inconsistency in the XY plane
            if (xyz_ang_diff_large || xy_ang_diff_large || xy_length_diff_large) {
                return false;
            }
        }
    }
    return true;
}
