/*
  OreoLED PX4 driver
*/
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "OreoLED_PX4.h"
#include "AP_Notify.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <drivers/drv_oreoled.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

#define OREOLED_BACKLEFT                0       // back left led instance number
#define OREOLED_BACKRIGHT               1       // back right led instance number
#define OREOLED_FRONTRIGHT              2       // front right led instance number
#define OREOLED_FRONTLEFT               3       // front left led instance number

// constructor
OreoLED_PX4::OreoLED_PX4() : NotifyDevice(),
    _overall_health(false),
    _oreoled_fd(-1),
    _send_required(false),
    _state_desired_semaphore(false),
    _pattern_override(0)
{
    // initialise desired and sent state
    memset(_state_desired,0,sizeof(_state_desired));
    memset(_state_sent,0,sizeof(_state_sent));
}

// init - initialised the device
bool OreoLED_PX4::init()
{
    // open the device
    _oreoled_fd = open(OREOLED0_DEVICE_PATH, O_RDWR);
    if (_oreoled_fd == -1) {
        hal.console->printf("Unable to open " OREOLED0_DEVICE_PATH);
        _overall_health = false;
    } else {
        // set overall health
        _overall_health = true;
        // register timer
        hal.scheduler->register_io_process(AP_HAL_MEMBERPROC(&OreoLED_PX4::update_timer));
    }

    // return health
    return _overall_health;
}

// update - updates device according to timed_updated.  Should be
// called at 50Hz
void OreoLED_PX4::update()
{
    static uint8_t counter = 0;     // counter to reduce rate from 50hz to 10hz
    static uint8_t step = 0;        // step to control pattern
    static uint8_t last_stage = 0;  // unique id of the last messages sent to the LED, used to reduce resends which disrupt some patterns
    static uint8_t initialization_done = 0;   // Keep track if initialization has begun.  There is a period when the driver
                                              // is running but initialization has not yet begun -- this prevents post-initialization
                                              // LED patterns from displaying before initialization has completed.
    uint8_t brightness = OREOLED_BRIGHT;

    // return immediately if not healthy
    if (!_overall_health) {
        return;
    }

    // handle firmware update event
    if (AP_Notify::flags.firmware_update) {
        set_macro(OREOLED_INSTANCE_ALL, OREOLED_PARAM_MACRO_COLOUR_CYCLE);
        return;
    }

    // return immediately if custom pattern has been sent
    if (OreoLED_PX4::_pattern_override != 0) {
        // reset stage so patterns will be resent once override clears
        last_stage = 0;
        return;
    }

    // slow rate from 50Hz to 10hz
    counter++;
    if (counter < 5) {
        return;
    }
    counter = 0;

    // use dim light when connected through USB
    if (hal.gpio->usb_connected()) {
        brightness = OREOLED_DIM;
    }

    // move forward one step
    step++;
    if (step >= 10) {
        step = 0;
    }

    // Pre-initialization pattern is all solid green
    if (last_stage == 0  &&  !initialization_done) {
        uint8_t oreoled_fadein_green[] = {5, 0, 0, 3, 0, 1, 125, 4, 125, 2, 0, 5, 0, 8, 0, 0, 6, 7, 208, 7, 1};
        send_bytes(0, (uint8_t) sizeof(oreoled_fadein_green), oreoled_fadein_green);
        send_bytes(1, (uint8_t) sizeof(oreoled_fadein_green), oreoled_fadein_green);
        send_bytes(2, (uint8_t) sizeof(oreoled_fadein_green), oreoled_fadein_green);
        send_bytes(3, (uint8_t) sizeof(oreoled_fadein_green), oreoled_fadein_green);
        last_stage = 1;   // record stage
    }

    // initialising pattern
    if (AP_Notify::flags.initialising) {
        initialization_done = 1;  // Record initialization has begun
        last_stage = 1;   // record stage

        // exit so no other status modify this pattern
        return;
    }

    // save trim and esc calibration pattern
    if (AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration) {
        switch(step) {
            case 0:
            case 3:
            case 6:
                // red
                set_rgb(OREOLED_INSTANCE_ALL, brightness, 0, 0);
                break;

            case 1:
            case 4:
            case 7:
                // blue
                set_rgb(OREOLED_INSTANCE_ALL, 0, 0, brightness);
                break;

            case 2:
            case 5:
            case 8:
                // green on
                set_rgb(OREOLED_INSTANCE_ALL, 0, brightness, 0);
                break;

            case 9:
                // all off
                set_rgb(OREOLED_INSTANCE_ALL, 0, 0, 0);
                break;
        }
        // record stage
        last_stage = 2;
        // exit so no other status modify this pattern
        return;
    }

    // radio failsafe pattern: Alternate between front red/rear black and front black/rear red
    if (AP_Notify::flags.failsafe_radio) {
        switch(step) {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
                // Front red/rear black
                set_rgb(OREOLED_FRONTLEFT, brightness, 0, 0);
                set_rgb(OREOLED_FRONTRIGHT, brightness, 0, 0);
                set_rgb(OREOLED_BACKLEFT, 0, 0, 0);
                set_rgb(OREOLED_BACKRIGHT, 0, 0, 0);
                break;
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                // Front black/rear red
                set_rgb(OREOLED_FRONTLEFT, 0, 0, 0);
                set_rgb(OREOLED_FRONTRIGHT, 0, 0, 0);
                set_rgb(OREOLED_BACKLEFT, brightness, 0, 0);
                set_rgb(OREOLED_BACKRIGHT, brightness, 0, 0);
                break;
        }
        // record stage
        last_stage = 3;
        // exit so no other status modify this pattern
        return;
    }

    // send colours (later we will set macro if required)
    if (last_stage < 10) {
        if (initialization_done) {
            set_macro(OREOLED_FRONTLEFT, OREOLED_PARAM_MACRO_WHITE); // white
            set_macro(OREOLED_FRONTRIGHT, OREOLED_PARAM_MACRO_WHITE); // white
            set_macro(OREOLED_BACKLEFT, OREOLED_PARAM_MACRO_RED);    // red
            set_macro(OREOLED_BACKRIGHT, OREOLED_PARAM_MACRO_RED);   // red
        }
        last_stage = 10;
    } else if (last_stage >= 10) {
        static uint8_t previous_autopilot_mode = -1;
        if (previous_autopilot_mode != AP_Notify::flags.autopilot_mode) {
            if (AP_Notify::flags.autopilot_mode) {
                // autopilot flight modes start breathing macro
                uint8_t oreoled_red_sine[]    = {1, 0, 125, 3, 125, 1, 0, 4, 0, 2, 0, 5, 0, 8, 0, 0, 6, 7, 208};
                uint8_t oreoled_white_sine[]  = {1, 0, 125, 3, 125, 1, 125, 4, 125, 2, 125, 5, 125, 8, 0, 0, 6, 7, 208};
                send_bytes(0, (uint8_t) 1, oreoled_red_sine);
                send_bytes(1, (uint8_t) 1, oreoled_red_sine);
                send_bytes(2, (uint8_t) 1, oreoled_white_sine);
                send_bytes(3, (uint8_t) 1, oreoled_white_sine);
            } else {
                // Manual flight modes solid color
                uint8_t oreoled_red_solid[]   = {2, 0, 125, 3, 125, 1, 0, 4, 0, 2, 0, 5, 0};
                uint8_t oreoled_white_solid[] = {2, 0, 125, 3, 125, 1, 125, 4, 125, 2, 125, 5, 125};
                send_bytes(0, (uint8_t) 1, oreoled_red_solid);
                send_bytes(1, (uint8_t) 1, oreoled_red_solid);
                send_bytes(2, (uint8_t) 1, oreoled_white_solid);
                send_bytes(3, (uint8_t) 1, oreoled_white_solid);
            }

            // record we have processed this change
            previous_autopilot_mode = AP_Notify::flags.autopilot_mode;
        }
        last_stage = 11;
    }
}

// set_rgb - set color as a combination of red, green and blue values for one or all LEDs
void OreoLED_PX4::set_rgb(uint8_t instance, uint8_t red, uint8_t green, uint8_t blue)
{
    // return immediately if no healty leds
    if (!_overall_health) {
        return;
    }

    // get semaphore
    _state_desired_semaphore = true;

    // check for all instances
    if (instance == OREOLED_INSTANCE_ALL) {
        // store desired rgb for all LEDs
        for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
            if (_state_desired[i].mode != OREOLED_MODE_RGB || _state_desired[i].red != red || _state_desired[i].green != green || _state_desired[i].blue != blue) {
                _state_desired[i].mode = OREOLED_MODE_RGB;
                _state_desired[i].red = red;
                _state_desired[i].green = green;
                _state_desired[i].blue = blue;
                _send_required = true;
            }
        }
    } else if (instance < OREOLED_NUM_LEDS) {
        // store desired rgb for one LED
        if (_state_desired[instance].mode != OREOLED_MODE_RGB || _state_desired[instance].red != red || _state_desired[instance].green != green || _state_desired[instance].blue != blue) {
            _state_desired[instance].mode = OREOLED_MODE_RGB;
            _state_desired[instance].red = red;
            _state_desired[instance].green = green;
            _state_desired[instance].blue = blue;
            _send_required = true;
        }
    }

    // release semaphore
    _state_desired_semaphore = false;
}

// set_macro - set macro for one or all LEDs
void OreoLED_PX4::set_macro(uint8_t instance, oreoled_macro macro)
{
    // return immediately if no healthy leds
    if (!_overall_health) {
        return;
    }

    // set semaphore
    _state_desired_semaphore = true;

    // check for all instances
    if (instance == OREOLED_INSTANCE_ALL) {
        // store desired macro for all LEDs
        for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
            if (_state_desired[i].mode != OREOLED_MODE_MACRO || _state_desired[i].macro != macro) {
                _state_desired[i].mode = OREOLED_MODE_MACRO;
                _state_desired[i].macro = macro;
                _send_required = true;
            }
        }
    } else if (instance < OREOLED_NUM_LEDS) {
        // store desired macro for one LED
        if (_state_desired[instance].mode != OREOLED_MODE_MACRO || _state_desired[instance].macro != macro) {
            _state_desired[instance].mode = OREOLED_MODE_MACRO;
            _state_desired[instance].macro = macro;
            _send_required = true;
        }
    }

    // release semaphore
    _state_desired_semaphore = false;
}

// send_bytes - send bytes to one or all LEDs
void OreoLED_PX4::send_bytes(uint8_t instance, uint8_t num_bytes, uint8_t bytes[OREOLED_CMD_LENGTH_MAX])
{
    // return immediately if no healthy leds
    if (!_overall_health) {
        return;
    }

    // set semaphore
    _state_desired_semaphore = true;

    // send bytes to some or all leds
    oreoled_cmd_t new_cmd;
    new_cmd.led_num = instance;
    new_cmd.num_bytes = num_bytes;
    memcpy(new_cmd.buff, bytes, OREOLED_CMD_LENGTH_MAX);
    ioctl(_oreoled_fd, OREOLED_SEND_BYTES, (unsigned long)&new_cmd);

    // release semaphore
    _state_desired_semaphore = false;
}

// update_timer - called by scheduler and updates PX4 driver with commands
void OreoLED_PX4::update_timer(void)
{
    // exit immediately if unhealthy
    if (!_overall_health) {
        return;
    }

    // exit immediately if send not required, or state is being updated
    if (!_send_required || _state_desired_semaphore) {
        return;
    }

    // for each LED
    for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {

        // check for state change
        if (!(_state_desired[i] == _state_sent[i])) {
            switch (_state_desired[i].mode) {
                case OREOLED_MODE_PATTERN:
                    // not yet supported
                    break;
                case OREOLED_MODE_MACRO:
                    {
                    oreoled_macrorun_t macro_run = {i, _state_desired[i].macro};
                    ioctl(_oreoled_fd, OREOLED_RUN_MACRO, (unsigned long)&macro_run);
                    }
                    break;
                case OREOLED_MODE_RGB:
                    {
                    oreoled_rgbset_t rgb_set = {i, OREOLED_PATTERN_SOLID, _state_desired[i].red, _state_desired[i].green, _state_desired[i].blue};
                    ioctl(_oreoled_fd, OREOLED_SET_RGB, (unsigned long)&rgb_set);
                    }
                    break;
            }
            // save state change
            _state_sent[i] = _state_desired[i];
        }
    }

    // flag updates sent
    _send_required = false;
}

// handle a LED_CONTROL message
void OreoLED_PX4::handle_led_control(mavlink_message_t *msg)
{
    // exit immediately if unhealthy
    if (!_overall_health) {
        return;
    }

    // decode mavlink message
    mavlink_led_control_t packet;
    mavlink_msg_led_control_decode(msg, &packet);

    // exit immediately if instance is invalid
    if (packet.instance >= OREOLED_NUM_LEDS && packet.instance != OREOLED_INSTANCE_ALL) {
        return;
    }

    // if pattern is OFF, we clear pattern override so normal lighting should resume
    if (packet.pattern == LED_CONTROL_PATTERN_OFF) {
        _pattern_override = 0;
        return;
    }

    // custom pattern
    if (packet.pattern == LED_CONTROL_PATTERN_CUSTOM) {
        // sanity check length
        if (packet.custom_len <= OREOLED_CMD_LENGTH_MAX) {
            // send bytes
            send_bytes(packet.instance, packet.custom_len, packet.custom_bytes);
            _pattern_override = packet.pattern;
        }
        return;
    }

    // other patterns sent as macro
    set_macro(packet.instance, (oreoled_macro)packet.pattern);
    _pattern_override = packet.pattern;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
