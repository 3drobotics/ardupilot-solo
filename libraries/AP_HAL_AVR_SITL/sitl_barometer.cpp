/*
  SITL handling

  This simulates a barometer

  Andrew Tridgell November 2011
 */

#include <AP_HAL.h>
#include <AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include "AP_HAL_AVR_SITL.h"

using namespace AVR_SITL;

extern const AP_HAL::HAL& hal;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/*
  setup the barometer with new input
  altitude is in meters
 */
void SITL_State::_update_barometer(float altitude)
{
    static uint32_t last_update;
    static float sim_alt_avg;

    float sim_alt = altitude;

    if (_barometer == NULL) {
        // this sketch doesn't use a barometer
        return;
    }

    if (_sitl->baro_disable) {
        // barometer is disabled
        return;
    }

    uint32_t now = hal.scheduler->millis();

    sim_alt += _sitl->baro_drift * now / 1000;
    sim_alt += _sitl->baro_noise * _rand_float();

    // add baro glitch
    sim_alt += _sitl->baro_glitch;

    // add delay
    uint32_t best_time_delta_baro = 200; // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index_baro = 0; // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now - last_store_time_baro >= 10) { // store data every 10 ms.
            last_store_time_baro = now;
            if (store_index_baro > baro_buffer_length-1) { // reset buffer index if index greater than size of buffer
                store_index_baro = 0;
            }
            buffer_baro[store_index_baro].data = sim_alt; // add data to current index
            buffer_baro[store_index_baro].time = last_store_time_baro; // add time_stamp_baro to current index
            store_index_baro = store_index_baro + 1; // increment index
    }

    // return delayed measurement
    delayed_time_baro = now - _sitl->baro_delay; // get time corresponding to delay

    // find data corresponding to delayed time in buffer
    for (uint8_t i=0; i<=baro_buffer_length-1; i++) {
        time_delta_baro = abs(delayed_time_baro - buffer_baro[i].time); // find difference between delayed time and time stamp in buffer
        // if this difference is smaller than last delta, store this time
        if (time_delta_baro < best_time_delta_baro) {
            best_index_baro = i;
            best_time_delta_baro = time_delta_baro;
        }
    }
    if (best_time_delta_baro < 200) { // only output stored state if < 200 msec retrieval error
        sim_alt = buffer_baro[best_index_baro].data;
    }

    //1000 sample rolling average
    if (sim_alt_avg == 0){
        sim_alt_avg = sim_alt;
    }
    else{
        sim_alt_avg -= sim_alt_avg / 1000.0;
        sim_alt_avg += sim_alt / 1000.0;
    }

    // 10Hz reporting rate
    if ((now - last_update) < 100) {
        return;
    }
    _barometer->setHIL(sim_alt_avg);  
    last_update = now;
}

#endif
