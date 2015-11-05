#include "DataFlash_Backend.h"

extern const AP_HAL::HAL& hal;

void DataFlash_Backend::periodic_10Hz(const uint32_t now)
{
}
void DataFlash_Backend::periodic_1Hz(const uint32_t now)
{
}
void DataFlash_Backend::periodic_fullrate(const uint32_t now)
{
}

#include <stdio.h>

void DataFlash_Backend::periodic_tasks()
{
    uint32_t now = hal.scheduler->millis();
    if (now - _last_periodic_1Hz > 1000) {
        periodic_1Hz(now);
        _last_periodic_1Hz = now;
    }
    if (now - _last_periodic_10Hz > 100) {
        periodic_10Hz(now);
        _last_periodic_10Hz = now;
    }
    periodic_fullrate(now);
}

void DataFlash_Backend::internal_error() {
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    abort();
#endif
}

void DataFlash_Backend::write_more_preface_messages()
{
    if (! _front._blockwriter_running) {
        return;
    }
    if (_front._blockwriter == NULL) {
        internal_error();
        return;
    }

    // don't allow the blockwriter to produce more than <n>
    // messages - just to prevent any infinite loop.  At time of
    // writing, startup seems to be ~600 messages(!)
    const uint16_t limit = 1024;
    uint16_t i = 0;
    // 300 bytes should fit any message.  Possibly we need a tristate
    // instead of a boolean; 0 == all done, -1 ==call me again,
    // insufficient space, 1== call me again, more messages
    while (bufferspace_available() > 300) {
        if (i++ >= limit) {
            internal_error();
            _front._blockwriter_running = false;
            break;
        }
        if (_front._blockwriter(_front._blockwriter_state)) {
            _front._blockwriter_running = false;
            break;
        }
    }
}

