
#ifndef __AP_HAL_UTIL_H__
#define __AP_HAL_UTIL_H__

#include <stdarg.h>
#include "AP_HAL_Namespace.h"
#include <AP_Progmem.h>

class AP_HAL::Util {
public:
    // soft_armed starts out true
    Util() : soft_armed(true),
             in_test_mode(false) {}

    int snprintf(char* str, size_t size,
                 const char *format, ...);

    int snprintf_P(char* str, size_t size,
                   const prog_char_t *format, ...);

    int vsnprintf(char* str, size_t size,
                  const char *format, va_list ap);

    int vsnprintf_P(char* str, size_t size,
                    const prog_char_t *format, va_list ap);

    void set_soft_armed(const bool b) { soft_armed = b; }
    bool get_soft_armed() const { return soft_armed; }

    // set and get funcs for autopilot testing/diagnostic mode
    void set_test_mode(bool b) { in_test_mode = b; }
    bool get_test_mode() const { return in_test_mode; }

    // run a debug shall on the given stream if possible. This is used
    // to support dropping into a debug shell to run firmware upgrade
    // commands
    virtual bool run_debug_shell(AP_HAL::BetterStream *stream) = 0;

    enum safety_state {
        SAFETY_NONE, SAFETY_DISARMED, SAFETY_ARMED
    };

    /*
      return state of safety switch, if applicable
     */
    virtual enum safety_state safety_switch_state(void) { return SAFETY_NONE; }

    /*
      set system clock in UTC microseconds
     */
    virtual void set_system_clock(uint64_t time_utc_usec) {}

    /*
      get system identifier (eg. serial number)
      return false if a system identifier is not available

      Buf should be filled with a printable string and must be null
      terminated
     */
    virtual bool get_system_id(char buf[40]) { return false; }

    /**
       how much free memory do we have in bytes. If more than 0xFFFF
       then return 0xFFFF. If unknown return 4096
     */
    virtual uint16_t available_memory(void) { return 4096; }

    /**
       return commandline arguments, if available
     */
    virtual void commandline_arguments(uint8_t &argc, char * const *&argv) { argc = 0; }
    
    /*
        ToneAlarm Driver
    */
    virtual bool toneAlarm_init() { return false;}
    virtual void toneAlarm_set_tune(uint8_t tune) {}
    virtual void _toneAlarm_timer_tick() {}

protected:
    bool soft_armed;
    bool in_test_mode;
};

#endif // __AP_HAL_UTIL_H__

