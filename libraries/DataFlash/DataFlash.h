/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include "../AP_Airspeed/AP_Airspeed.h"
#include "../AP_BattMonitor/AP_BattMonitor.h"
#include <stdint.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <uORB/topics/esc_status.h>
#endif


#if HAL_CPU_CLASS < HAL_CPU_CLASS_75
#define DATAFLASH_NO_CLI
#endif

class DataFlash_Class
{
public:
    // initialisation
    virtual void Init(const struct LogStructure *structure, uint8_t num_types);
    virtual bool CardInserted(void) = 0;

    // erase handling
    virtual bool NeedErase(void) = 0;
    virtual void EraseAll() = 0;

    /* Write a block of data at current offset */
    virtual void WriteBlock(const void *pBuffer, uint16_t size) = 0;

    // high level interface
    virtual uint16_t find_last_log(void) = 0;
    virtual void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) = 0;
    virtual void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) = 0;
    virtual int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    virtual uint16_t get_num_logs(void) = 0;
#ifndef DATAFLASH_NO_CLI
    virtual void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page, 
                                void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
                                AP_HAL::BetterStream *port) = 0;
    virtual void DumpPageInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ShowDeviceInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ListAvailableLogs(AP_HAL::BetterStream *port) = 0;
#endif // DATAFLASH_NO_CLI

    /* logging methods common to all vehicles */
    uint16_t StartNewLog(void);
    void AddLogFormats(const struct LogStructure *structures, uint8_t num_types);
    void EnableWrites(bool enable) { _writes_enabled = enable; }
    void Log_Write_Format(const struct LogStructure *structure);
    void Log_Write_Parameter(const char *name, float value);
    void Log_Write_GPS(const AP_GPS &gps, uint8_t instance, int32_t relative_alt);
    void Log_Write_IMU(const AP_InertialSensor &ins);
    void Log_Write_RCIN(void);
    void Log_Write_RCOUT(void);
    void Log_Write_Baro(AP_Baro &baro);
    void Log_Write_Power(void);
    void Log_Write_AHRS2(AP_AHRS &ahrs);
#if AP_AHRS_NAVEKF_AVAILABLE
    void Log_Write_EKF(AP_AHRS_NavEKF &ahrs, bool optFlowEnabled);
#endif
    void Log_Write_MavCmd(uint16_t cmd_total, const mavlink_mission_item_t& mav_cmd);
    void Log_Write_Radio(const mavlink_radio_t &packet);
    void Log_Write_Message(const char *message);
    void Log_Write_Message_P(const prog_char_t *message);
    void Log_Write_Camera(const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc);
    void Log_Write_ESC(void);
    void Log_Write_Airspeed(AP_Airspeed &airspeed);
    void Log_Write_Attitude(AP_AHRS &ahrs, const Vector3f &targets);
	void Log_Write_Current(const AP_BattMonitor &battery, int16_t throttle);
    void Log_Write_Compass(const Compass &compass);
    void Log_Write_Mode(uint8_t mode);

    bool logging_started(void) const { return _logging_started; }

	/*
      every logged packet starts with 3 bytes
    */
    struct log_Header {
        uint8_t head1, head2, msgid;
    };

protected:
    /*
    read and print a log entry using the format strings from the given structure
    */
    void _print_log_entry(uint8_t msg_type, 
                          void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                          AP_HAL::BetterStream *port);
    
    void Log_Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);
    void Log_Write_Parameter(const AP_Param *ap, const AP_Param::ParamToken &token, 
                             enum ap_var_type type);
    void Log_Write_Parameters(void);
    virtual uint16_t start_new_log(void) = 0;

    const struct LogStructure *_structures;
    uint8_t _num_types;
    bool _writes_enabled;
    bool _logging_started;

    /*
      read a block
    */
    virtual void ReadBlock(void *pkt, uint16_t size) = 0;
    bool is_critical_block;
};

/*
  unfortunately these need to be macros because of a limitation of
  named member structure initialisation in g++
 */
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id

// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

// structure used to define logging format
struct LogStructure {
    uint8_t msg_type;
    uint8_t msg_len;
    const char name[5];
    const char format[16];
    const char labels[64];
};

/*
  log structures common to all vehicle types
 */
struct PACKED log_Format {
    LOG_PACKET_HEADER;
    uint8_t type;
    uint8_t length;
    char name[4];
    char format[16];
    char labels[64];
};

struct PACKED log_Parameter {
    LOG_PACKET_HEADER;
    char name[16];
    float value;
};

struct PACKED log_GPS {
    LOG_PACKET_HEADER;
    uint8_t  status;
    uint32_t gps_week_ms;
    uint16_t gps_week;
    uint8_t  num_sats;
    uint16_t hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  rel_altitude;
    int32_t  altitude;
    uint32_t ground_speed;
    int32_t  ground_course;
    float    vel_z;
    uint32_t apm_time;
};

struct PACKED log_GPS2 {
    LOG_PACKET_HEADER;
    uint8_t  status;
    uint32_t gps_week_ms;
    uint16_t gps_week;
    uint8_t  num_sats;
    uint16_t hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    uint32_t ground_speed;
    int32_t  ground_course;
    float    vel_z;
    uint32_t apm_time;
    uint8_t  dgps_numch;
    uint32_t dgps_age;
};

struct PACKED log_Message {
    LOG_PACKET_HEADER;
    char msg[64];
};

struct PACKED log_IMU {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    uint32_t gyro_error, accel_error;
    float temperature;
};

struct PACKED log_Gimbal1 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float delta_time;
    float delta_angles_x;
    float delta_angles_y;
    float delta_angles_z;
    float delta_velocity_x;
    float delta_velocity_y;
    float delta_velocity_z;
    float joint_angles_x;
    float joint_angles_y;
    float joint_angles_z;
};

struct PACKED log_Gimbal2 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t  est_sta;
    float est_x;
    float est_y;  
    float est_z;  
    float rate_x;  
    float rate_y; 
    float rate_z; 
    float target_x;
    float target_y;
    float target_z;
};

struct PACKED log_Gimbal3 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t rl_torque_cmd;
    int16_t el_torque_cmd;
    int16_t az_torque_cmd;
};

struct PACKED log_RCIN {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
    uint16_t chan9;
    uint16_t chan10;
    uint16_t chan11;
    uint16_t chan12;
    uint16_t chan13;
    uint16_t chan14;
};

struct PACKED log_RCOUT {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
};

struct PACKED log_BARO {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float   altitude;
    float   pressure;
    int16_t temperature;
    float   climbrate;
};

struct PACKED log_AHRS {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float alt;
    int32_t lat;
    int32_t lng;
};

struct PACKED log_POWR {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint16_t Vcc;
    uint16_t Vservo;
    uint16_t flags;
};

struct PACKED log_EKF1 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float velN;
    float velE;
    float velD;
    float posN;
    float posE;
    float posD;
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;
};

struct PACKED log_EKF2 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int8_t Ratio;
    int8_t AZ1bias;
    int8_t AZ2bias;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
};

struct PACKED log_EKF3 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t innovVN;
    int16_t innovVE;
    int16_t innovVD;
    int16_t innovPN;
    int16_t innovPE;
    int16_t innovPD;
    int16_t innovMX;
    int16_t innovMY;
    int16_t innovMZ;
    int16_t innovVT;
};

struct PACKED log_EKF4 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t sqrtvarV;
    int16_t sqrtvarP;
    int16_t sqrtvarH;
    int16_t sqrtvarMX;
    int16_t sqrtvarMY;
    int16_t sqrtvarMZ;
    int16_t sqrtvarVT;
    int8_t  offsetNorth;
    int8_t  offsetEast;
    uint8_t faults;
    uint8_t timeouts;
    uint16_t solution;
};

struct PACKED log_EKF5 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t normInnov;
    int16_t FIX;
    int16_t FIY;
    int16_t AFI;
    int16_t HAGL;
    int16_t offset;
    int16_t RI;
    uint16_t meaRng;
    uint16_t errHAGL;
};

struct PACKED log_Cmd {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint16_t command_total;
    uint16_t sequence;
    uint16_t command;
    float param1;
    float param2;
    float param3;
    float param4;
    float latitude;
    float longitude;
    float altitude;
};

struct PACKED log_Radio {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t rssi;
    uint8_t remrssi;
    uint8_t txbuf;
    uint8_t noise;
    uint8_t remnoise;
    uint16_t rxerrors;
    uint16_t fixed;
};

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    uint16_t gps_week;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int32_t  altitude_rel;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  control_roll;
    int16_t  roll;
    int16_t  control_pitch;
    int16_t  pitch;
    uint16_t control_yaw;
    uint16_t yaw;
    uint16_t error_rp;
    uint16_t error_yaw;
};

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  throttle;
    int16_t  battery_voltage;
    int16_t  current_amps;
    uint16_t board_voltage;
    float    current_total;
	int16_t  battery2_voltage;
};

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int16_t  offset_x;
    int16_t  offset_y;
    int16_t  offset_z;
    int16_t  motor_offset_x;
    int16_t  motor_offset_y;
    int16_t  motor_offset_z;
    uint8_t  health;
};

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t mode;
    uint8_t mode_num;
};

/*
  terrain log structure
 */
struct PACKED log_TERRAIN {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t status;
    int32_t lat;
    int32_t lng;
    uint16_t spacing;
    float terrain_height;
    float current_height;
    uint16_t pending;
    uint16_t loaded;
};

/*
  UBlox logging
 */
struct PACKED log_Ubx1 {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint8_t  instance;
    uint16_t noisePerMS;
    uint8_t  jamInd;
    uint8_t  aPower;
    uint16_t agcCnt;
};

struct PACKED log_Ubx2 {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint8_t  instance;
    int8_t   ofsI;
    uint8_t  magI;
    int8_t   ofsQ;
    uint8_t  magQ;
};

struct PACKED log_Ubx3 {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint8_t  instance;
    float hAcc;
    float vAcc;
    float sAcc;
};

struct PACKED log_Uack {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t instance;
    uint8_t clsID;
    uint8_t msgID;
};

struct PACKED log_Ustg {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t instance;
    uint8_t nav_engine;
    int8_t min_elev;
};

struct PACKED log_Esc {
    LOG_PACKET_HEADER;
    uint32_t time_ms;     
    int16_t rpm;
    int16_t voltage;
    int16_t current;
    int16_t temperature;
};

struct PACKED log_AIRSPEED {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float   airspeed;
    float   diffpressure;
    int16_t temperature;
    float   rawpressure;
    float   offset;
};

struct PACKED log_ACCEL {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint32_t timestamp_us;
    float AccX, AccY, AccZ;
};

struct PACKED log_GYRO {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint32_t timestamp_us;
    float GyrX, GyrY, GyrZ;
};

/*
Format characters in the format string for binary log messages
  b   : int8_t
  B   : uint8_t
  h   : int16_t
  H   : uint16_t
  i   : int32_t
  I   : uint32_t
  f   : float
  n   : char[4]
  N   : char[16]
  Z   : char[64]
  c   : int16_t * 100
  C   : uint16_t * 100
  e   : int32_t * 100
  E   : uint32_t * 100
  L   : int32_t latitude/longitude
  M   : uint8_t flight mode
 */

// messages for all boards
#define LOG_BASE_STRUCTURES \
    { LOG_FORMAT_MSG, sizeof(log_Format), \
      "FMT", "BBnNZ",      "Type,Length,Name,Format,Columns" },    \
    { LOG_PARAMETER_MSG, sizeof(log_Parameter), \
      "PARM", "Nf",        "Name,Value" },    \
    { LOG_GPS_MSG, sizeof(log_GPS), \
      "GPS",  "BIHBcLLeeEefI", "Status,TimeMS,Week,NSats,HDop,Lat,Lng,RelAlt,Alt,Spd,GCrs,VZ,T" }, \
    { LOG_IMU_MSG, sizeof(log_IMU), \
      "IMU",  "IffffffIIf",     "TimeMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,ErrG,ErrA,Temp" }, \
    { LOG_MESSAGE_MSG, sizeof(log_Message), \
      "MSG",  "Z",     "Message"}, \
    { LOG_RCIN_MSG, sizeof(log_RCIN), \
      "RCIN",  "Ihhhhhhhhhhhhhh",     "TimeMS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14" }, \
    { LOG_RCOUT_MSG, sizeof(log_RCOUT), \
      "RCOU",  "Ihhhh",     "TimeMS,Ch1,Ch2,Ch3,Ch4" }, \
    { LOG_BARO_MSG, sizeof(log_BARO), \
      "BARO",  "Iffcf", "TimeMS,Alt,Press,Temp,CRt" }, \
    { LOG_BAR2_MSG, sizeof(log_BARO), \
      "BAR2",  "Iffcf", "TimeMS,Alt,Press,Temp,CRt" }, \
    { LOG_POWR_MSG, sizeof(log_POWR), \
      "POWR","ICCH","TimeMS,Vcc,VServo,Flags" },  \
    { LOG_CMD_MSG, sizeof(log_Cmd), \
      "CMD", "IHHHfffffff","TimeMS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt" }, \
    { LOG_RADIO_MSG, sizeof(log_Radio), \
      "RAD", "IBBBBBHH", "TimeMS,RSSI,RemRSSI,TxBuf,Noise,RemNoise,RxErrors,Fixed" }, \
    { LOG_CAMERA_MSG, sizeof(log_Camera), \
      "CAM", "IHLLeeccC","GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,Roll,Pitch,Yaw" }, \
    { LOG_ARSP_MSG, sizeof(log_AIRSPEED), \
      "ARSP",  "Iffcff",   "TimeMS,Airspeed,DiffPress,Temp,RawPress,Offset" }, \
    { LOG_CURRENT_MSG, sizeof(log_Current), \
      "CURR", "IhhhHfh","TimeMS,Throttle,Volt,Curr,Vcc,CurrTot,Volt2" },\
	{ LOG_ATTITUDE_MSG, sizeof(log_Attitude),\
      "ATT", "IccccCCCC", "TimeMS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw" }, \
    { LOG_COMPASS_MSG, sizeof(log_Compass), \
      "MAG", "IhhhhhhhhhB",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health" }, \
    { LOG_MODE_MSG, sizeof(log_Mode), \
      "MODE", "IMB",         "TimeMS,Mode,ModeNum" }

// messages for more advanced boards
#define LOG_EXTRA_STRUCTURES \
    { LOG_GPS2_MSG, sizeof(log_GPS2), \
      "GPS2",  "BIHBcLLeEefIBI", "Status,TimeMS,Week,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,T,DSc,DAg" }, \
    { LOG_IMU2_MSG, sizeof(log_IMU), \
      "IMU2",  "IffffffIIf",     "TimeMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,ErrG,ErrA,Temp" }, \
    { LOG_IMU3_MSG, sizeof(log_IMU), \
      "IMU3",  "IffffffIIf",     "TimeMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,ErrG,ErrA,Temp" }, \
    { LOG_AHR2_MSG, sizeof(log_AHRS), \
      "AHR2","IccCfLL","TimeMS,Roll,Pitch,Yaw,Alt,Lat,Lng" }, \
    { LOG_SIMSTATE_MSG, sizeof(log_AHRS), \
      "SIM","IccCfLL","TimeMS,Roll,Pitch,Yaw,Alt,Lat,Lng" }, \
    { LOG_EKF1_MSG, sizeof(log_EKF1), \
      "EKF1","IccCffffffccc","TimeMS,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,GX,GY,GZ" }, \
    { LOG_EKF2_MSG, sizeof(log_EKF2), \
      "EKF2","Ibbbcchhhhhh","TimeMS,Ratio,AZ1bias,AZ2bias,VWN,VWE,MN,ME,MD,MX,MY,MZ" }, \
    { LOG_EKF3_MSG, sizeof(log_EKF3), \
      "EKF3","Icccccchhhc","TimeMS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IVT" }, \
    { LOG_EKF4_MSG, sizeof(log_EKF4), \
      "EKF4","IcccccccbbBBH","TimeMS,SV,SP,SH,SMX,SMY,SMZ,SVT,OFN,EFE,FS,TS,SS" }, \
    { LOG_TERRAIN_MSG, sizeof(log_TERRAIN), \
      "TERR","IBLLHffHH","TimeMS,Status,Lat,Lng,Spacing,TerrH,CHeight,Pending,Loaded" }, \
    { LOG_UBX1_MSG, sizeof(log_Ubx1), \
      "UBX1", "IBHBBH",  "TimeMS,Instance,noisePerMS,jamInd,aPower,agcCnt" }, \
    { LOG_UBX2_MSG, sizeof(log_Ubx2), \
      "UBX2", "IBbBbB", "TimeMS,Instance,ofsI,magI,ofsQ,magQ" }, \
    { LOG_UBX3_MSG, sizeof(log_Ubx3), \
      "UBX3", "IBfff", "TimeMS,Instance,hAcc,vAcc,sAcc" }, \
    { LOG_UACK_MSG, sizeof(log_Uack), \
      "UACK", "IBBB", "TimeMS,Instance,clsID,msgID" }, \
    { LOG_UNAK_MSG, sizeof(log_Uack), \
      "UNAK", "IBBB", "TimeMS,Instance,clsID,msgID" }, \
    { LOG_USTG_MSG, sizeof(log_Ustg), \
      "USTG", "IBBb", "TimeMS,Instance,navEng,minElev" }, \
    { LOG_ESC1_MSG, sizeof(log_Esc), \
      "ESC1",  "Icccc", "TimeMS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC2_MSG, sizeof(log_Esc), \
      "ESC2",  "Icccc", "TimeMS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC3_MSG, sizeof(log_Esc), \
      "ESC3",  "Icccc", "TimeMS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC4_MSG, sizeof(log_Esc), \
      "ESC4",  "Icccc", "TimeMS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC5_MSG, sizeof(log_Esc), \
      "ESC5",  "Icccc", "TimeMS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC6_MSG, sizeof(log_Esc), \
      "ESC6",  "Icccc", "TimeMS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC7_MSG, sizeof(log_Esc), \
      "ESC7",  "Icccc", "TimeMS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC8_MSG, sizeof(log_Esc), \
      "ESC8",  "Icccc", "TimeMS,RPM,Volt,Curr,Temp" }, \
    { LOG_EKF5_MSG, sizeof(log_EKF5), \
      "EKF5","IBhhhcccCC","TimeMS,normInnov,FIX,FIY,AFI,HAGL,offset,RI,meaRng,errHAGL" }, \
    { LOG_COMPASS2_MSG, sizeof(log_Compass), \
      "MAG2","IhhhhhhhhhB",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health" }, \
    { LOG_COMPASS3_MSG, sizeof(log_Compass), \
      "MAG3","IhhhhhhhhhB",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health" }, \
    { LOG_GIMBAL1_MSG, sizeof(log_Gimbal1), \
      "GMB1", "Iffffffffff", "TimeMS,dt,dax,day,daz,dvx,dvy,dvz,jx,jy,jz" }, \
    { LOG_GIMBAL2_MSG, sizeof(log_Gimbal2), \
      "GMB2", "IBfffffffff", "TimeMS,es,ex,ey,ez,rx,ry,rz,tx,ty,tz" }, \
    { LOG_GIMBAL3_MSG, sizeof(log_Gimbal3), \
      "GMB3", "Ihhh", "TimeMS,rl_torque_cmd,el_torque_cmd,az_torque_cmd" }, \
    { LOG_ACC1_MSG, sizeof(log_ACCEL), \
      "ACC1", "IIfff",        "TimeMS,TimeUS,AccX,AccY,AccZ" }, \
    { LOG_ACC2_MSG, sizeof(log_ACCEL), \
      "ACC2", "IIfff",        "TimeMS,TimeUS,AccX,AccY,AccZ" }, \
    { LOG_ACC3_MSG, sizeof(log_ACCEL), \
      "ACC3", "IIfff",        "TimeMS,TimeUS,AccX,AccY,AccZ" }, \
    { LOG_GYR1_MSG, sizeof(log_GYRO), \
      "GYR1", "IIfff",        "TimeMS,TimeUS,GyrX,GyrY,GyrZ" }, \
    { LOG_GYR2_MSG, sizeof(log_GYRO), \
      "GYR2", "IIfff",        "TimeMS,TimeUS,GyrX,GyrY,GyrZ" }, \
    { LOG_GYR3_MSG, sizeof(log_GYRO), \
      "GYR3", "IIfff",        "TimeMS,TimeUS,GyrX,GyrY,GyrZ" }

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_75
#define LOG_COMMON_STRUCTURES LOG_BASE_STRUCTURES, LOG_EXTRA_STRUCTURES
#else
#define LOG_COMMON_STRUCTURES LOG_BASE_STRUCTURES
#endif

// message types 0 to 100 reversed for vehicle specific use

// message types for common messages
#define LOG_FORMAT_MSG	  128
#define LOG_PARAMETER_MSG 129
#define LOG_GPS_MSG		  130
#define LOG_IMU_MSG		  131
#define LOG_MESSAGE_MSG	  132
#define LOG_RCIN_MSG      133
#define LOG_RCOUT_MSG     134
#define LOG_IMU2_MSG	  135
#define LOG_BARO_MSG	  136
#define LOG_POWR_MSG	  137
#define LOG_AHR2_MSG	  138
#define LOG_SIMSTATE_MSG  139
#define LOG_EKF1_MSG      140
#define LOG_EKF2_MSG      141
#define LOG_EKF3_MSG      142
#define LOG_EKF4_MSG      143
#define LOG_GPS2_MSG	  144
#define LOG_CMD_MSG       145
#define LOG_RADIO_MSG	  146
#define LOG_ATRP_MSG      147
#define LOG_CAMERA_MSG    148
#define LOG_IMU3_MSG	  149
#define LOG_TERRAIN_MSG   150
#define LOG_UBX1_MSG      151
#define LOG_UBX2_MSG      152
#define LOG_UBX3_MSG      153
#define LOG_ESC1_MSG      154
#define LOG_ESC2_MSG      155
#define LOG_ESC3_MSG      156
#define LOG_ESC4_MSG      157
#define LOG_ESC5_MSG      158
#define LOG_ESC6_MSG      159
#define LOG_ESC7_MSG      160
#define LOG_ESC8_MSG      161
#define LOG_EKF5_MSG      162
#define LOG_BAR2_MSG	  163
#define LOG_ARSP_MSG      164
#define LOG_ATTITUDE_MSG  165
#define LOG_CURRENT_MSG   166
#define LOG_COMPASS_MSG   167
#define LOG_COMPASS2_MSG  168
#define LOG_COMPASS3_MSG  169
#define LOG_MODE_MSG      170
#define LOG_GIMBAL1_MSG   171
#define LOG_GIMBAL2_MSG   172
#define LOG_GIMBAL3_MSG   173
#define LOG_GPS_RAW_MSG   174
#define LOG_ACC1_MSG      175
#define LOG_ACC2_MSG      176
#define LOG_ACC3_MSG      177
#define LOG_GYR1_MSG      178
#define LOG_GYR2_MSG      179
#define LOG_GYR3_MSG      180
#define LOG_UACK_MSG      181
#define LOG_UNAK_MSG      182
#define LOG_USTG_MSG      183

// message types 200 to 210 reversed for GPS driver use
// message types 211 to 220 reversed for autotune use

#include "DataFlash_Block.h"
#include "DataFlash_File.h"
#include "DataFlash_MAVLink.h"

#endif
