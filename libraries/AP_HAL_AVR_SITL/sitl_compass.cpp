/*
  SITL handling

  This simulates a compass

  Andrew Tridgell November 2011
 */

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"
#include "HAL_AVR_SITL_Class.h"

#include <AP_Math.h>
#include "../AP_Compass/AP_Compass.h"
#include "../AP_Declination/AP_Declination.h"
#include "../SITL/SITL.h"

extern const AP_HAL::HAL& hal;

using namespace AVR_SITL;

/*
  setup the compass with new input
  all inputs are in degrees
 */
void SITL_State::_update_compass(float rollDeg, float pitchDeg, float yawDeg)
{
    static uint32_t last_update;

    if (_compass == NULL) {
        // no compass in this sketch
        return;
    }
    yawDeg += _sitl->mag_error;
    if (yawDeg > 180.0f) {
        yawDeg -= 360.0f;
    }
    if (yawDeg < -180.0f) {
        yawDeg += 360.0f;
    }
    _compass->setHIL(radians(rollDeg), radians(pitchDeg), radians(yawDeg));
    Vector3f noise = _rand_vec3f() * _sitl->mag_noise;
    Vector3f motor = _sitl->mag_mot.get() * _current;
    Vector3f new_mag_data = _compass->getHIL() + noise + motor;

    // 100Hz, to match the real APM2 compass
    uint32_t now = hal.scheduler->millis();
    if ((now - last_update) < 10) {
        return;
    }
    last_update = now;

    // add delay
    uint32_t best_time_delta_mag = 1000; // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index_mag = 0; // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now - last_store_time_mag >= 10) { // store data every 10 ms.
        last_store_time_mag = now;
        if (store_index_mag > mag_buffer_length-1) { // reset buffer index if index greater than size of buffer
            store_index_mag = 0;
        }
        buffer_mag[store_index_mag].data = new_mag_data; // add data to current index
        buffer_mag[store_index_mag].time = last_store_time_mag; // add time to current index
        store_index_mag = store_index_mag + 1; // increment index
    }

    // return delayed measurement
    delayed_time_mag = now - _sitl->mag_delay; // get time corresponding to delay
    // find data corresponding to delayed time in buffer
    for (uint8_t i=0; i<=mag_buffer_length-1; i++) {
        time_delta_mag = abs(delayed_time_mag - buffer_mag[i].time); // find difference between delayed time and time stamp in buffer
        // if this difference is smaller than last delta, store this time
        if (time_delta_mag < best_time_delta_mag) {
            best_index_mag = i;
            best_time_delta_mag = time_delta_mag;
        }
    }
    if (best_time_delta_mag < 1000) { // only output stored state if < 1 sec retrieval error
        new_mag_data = buffer_mag[best_index_mag].data;
    }

    new_mag_data -= _sitl->mag_ofs.get();

    _compass->setHIL(new_mag_data);
}

#endif
