// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/// @file	GCS_MAVLink.cpp

/*
This provides some support code and variables for MAVLink enabled sketches

*/

#include <AP_HAL.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include <GCS.h>
#include <AP_GPS.h>

#ifdef MAVLINK_SEPARATE_HELPERS
#include "include/mavlink/v1.0/mavlink_helpers.h"
#endif

AP_HAL::UARTDriver	*mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];

mavlink_system_t mavlink_system = {7,1};

// mask of serial ports disabled to allow for SERIAL_CONTROL
static uint8_t mavlink_locked_mask;

// routing table
MAVLink_routing GCS_MAVLINK::routing;

// snoop function for vehicle types that want to see messages for
// other targets
void (*GCS_MAVLINK::msg_snoop)(const mavlink_message_t* msg) = NULL;

/*
  lock a channel, preventing use by MAVLink
 */
void GCS_MAVLINK::lock_channel(mavlink_channel_t _chan, bool lock)
{
    if (_chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return;
    }
    if (lock) {
        mavlink_locked_mask |= (1U<<(unsigned)_chan);
    } else {
        mavlink_locked_mask &= ~(1U<<(unsigned)_chan);
    }
}

// return a MAVLink variable type given a AP_Param type
uint8_t mav_var_type(enum ap_var_type t)
{
    if (t == AP_PARAM_INT8) {
	    return MAVLINK_TYPE_INT8_T;
    }
    if (t == AP_PARAM_INT16) {
	    return MAVLINK_TYPE_INT16_T;
    }
    if (t == AP_PARAM_INT32) {
	    return MAVLINK_TYPE_INT32_T;
    }
    // treat any others as float
    return MAVLINK_TYPE_FLOAT;
}


/// Read a byte from the nominated MAVLink channel
///
/// @param chan		Channel to receive on
/// @returns		Byte read
///
uint8_t comm_receive_ch(mavlink_channel_t chan)
{
    // sanity check chan
    if (chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return 0;
    }

    return (uint8_t)mavlink_comm_port[chan]->read();
}

/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan)
{
    // sanity check chan
    if (chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return 0;
    }
    if ((1U<<chan) & mavlink_locked_mask) {
        return 0;
    }
	int16_t ret = mavlink_comm_port[chan]->txspace();
	if (ret < 0) {
		ret = 0;
	}
    return (uint16_t)ret;
}

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_available(mavlink_channel_t chan)
{
    // sanity check chan
    if (chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return 0;
    }
    if ((1U<<chan) & mavlink_locked_mask) {
        return 0;
    }
    int16_t bytes = mavlink_comm_port[chan]->available();
	if (bytes == -1) {
		return 0;
	}
    return (uint16_t)bytes;
}

/*
  send a buffer out a MAVLink channel
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    // sanity check chan
    if (chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return;
    }
    mavlink_comm_port[chan]->write(buf, len);
}

static const uint8_t mavlink_message_crc_progmem[256] PROGMEM = MAVLINK_MESSAGE_CRCS;

// return CRC byte for a mavlink message ID
uint8_t mavlink_get_message_crc(uint8_t msgid)
{
	return pgm_read_byte(&mavlink_message_crc_progmem[msgid]);
}

extern const AP_HAL::HAL& hal;

/*
  return true if the MAVLink parser is idle, so there is no partly parsed
  MAVLink message being processed
 */
bool comm_is_idle(mavlink_channel_t chan)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	return status == NULL || status->parse_state <= MAVLINK_PARSE_STATE_IDLE;
}
