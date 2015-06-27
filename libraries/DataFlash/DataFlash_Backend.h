#ifndef DATAFLASH_BACKEND_H
#define DATAFLASH_BACKEND_H

#if HAL_CPU_CLASS < HAL_CPU_CLASS_75
#define DATAFLASH_NO_CLI
#endif

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include <AP_Vehicle.h>
#include "../AP_Airspeed/AP_Airspeed.h"
#include "../AP_BattMonitor/AP_BattMonitor.h"
#include <stdint.h>
#include <GCS_MAVLink.h> // for mavlink_msg_t

class DataFlash_Backend
{
public:

    DataFlash_Backend(DataFlash_Class &front) :
        _front(front),
        _logging_started(false),
        _writes_enabled(false)
        { }

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
                                void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                                AP_HAL::BetterStream *port) = 0;
    virtual void DumpPageInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ShowDeviceInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ListAvailableLogs(AP_HAL::BetterStream *port) = 0;
#endif // DATAFLASH_NO_CLI

    void EnableWrites(bool enable) { _writes_enabled = enable; }
    bool logging_started(void) const { return _logging_started; }

    // initialisation this really shouldn't take structure and
    // num_types, however the CLI LogReadProcess function requires it.
    // That function needs to be split.
    virtual void Init(const struct LogStructure *structure, uint8_t num_types) {
        _writes_enabled = true;
        _num_types = num_types;
        _structures = structure;
    }

    virtual uint16_t start_new_log(void) = 0;
    bool _logging_started;

    void Log_Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);

    // for Dataflash_MAVlink
    virtual void handle_ack(uint32_t block_num) { };
    virtual void handle_retry(uint32_t block_num) { };
    virtual void remote_log_block_status_msg(mavlink_message_t* msg) { }
    // end for Dataflash_MAVlink

protected:
    DataFlash_Class &_front;

    /*
    read and print a log entry using the format strings from the given structure
    */
    void _print_log_entry(uint8_t msg_type,
                          void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                          AP_HAL::BetterStream *port);

    const struct LogStructure *_structures;
    uint8_t _num_types;
    bool _writes_enabled;

    /*
      read a block
    */
    virtual void ReadBlock(void *pkt, uint16_t size) = 0;

};

#endif
