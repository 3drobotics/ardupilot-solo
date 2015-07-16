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

/// @file	MAVLink_routing.h
/// @brief	handle routing of MAVLink packets by sysid/componentid

#include <stdio.h>
#include <AP_HAL.h>
#include <AP_Common.h>
#include <GCS.h>
#include <MAVLink_routing.h>

extern const AP_HAL::HAL& hal;

#define ROUTING_DEBUG 0

// constructor
MAVLink_routing::MAVLink_routing(void) : num_routes(0) {}

/*
  forward a MAVLink message to the right port. This also
  automatically learns the route for the sender if it is not
  already known.
  
  This returns true if the message should be processed locally

  Theory of MAVLink routing:

  When a flight controller receives a message it should process it
  locally if any of these conditions hold:

    1a) the message has no target_system field

    1b) the message has a target_system of zero

    1c) the message has the flight controllers target system and has no
       target_component field

    1d) the message has the flight controllers target system and has
       the flight controllers target_component 

    1e) the message has the flight controllers target system and the
        flight controller has not seen any messages on any of its links
        from a system that has the messages
        target_system/target_component combination

  When a flight controller receives a message it should forward it
  onto another different link if any of these conditions hold for that
  link: 

    2a) the message has no target_system field

    2b) the message has a target_system of zero

    2c) the message does not have the flight controllers target_system
        and the flight controller has seen a message from the messages
        target_system on the link

    2d) the message has the flight controllers target_system and has a
        target_component field and the flight controllers has seen a
        message from the target_system/target_component combination on
        the link

Note: This proposal assumes that ground stations will not send command
packets to a non-broadcast destination (sysid/compid combination)
until they have received at least one package from that destination
over the link. This is essential to prevent a flight controller from
acting on a message that is not meant for it. For example, a PARAM_SET
cannot be sent to a specific sysid/compid combination until the GCS
has seen a packet from that sysid/compid combination on the link. 

The GCS must also reset what sysid/compid combinations it has seen on
a link when it sees a SYSTEM_TIME message with a decrease in
time_boot_ms from a particular sysid/compid. That is essential to
detect a reset of the flight controller, which implies a reset of its
routing table.

*/
bool MAVLink_routing::check_and_forward(mavlink_channel_t in_channel, const mavlink_message_t* msg)
{
    // handle the case of loopback of our own messages, due to
    // incorrect serial configuration.
    if (msg->sysid == mavlink_system.sysid && 
        msg->compid == mavlink_system.compid) {
        return true;
    }

    // learn new routes
    learn_route(in_channel, msg);

    if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        // heartbeat needs special handling
        handle_heartbeat(in_channel, msg);
        return true;
    }

    // extract the targets for this packet
    int16_t target_system = -1;
    int16_t target_component = -1;
    get_targets(msg, target_system, target_component);

    bool broadcast_system = (target_system == 0 || target_system == -1);
    bool broadcast_component = (target_component == 0 || target_component == -1);
    bool match_system = broadcast_system || (target_system == mavlink_system.sysid);
    bool match_component = match_system && (broadcast_component || 
                                            (target_component == mavlink_system.compid));
    bool process_locally = match_system && match_component;

    if (process_locally && !broadcast_system && !broadcast_component) {
        // nothing more to do - it can only be for us
        return true;
    }

    // forward on any channels matching the targets
    bool forwarded = false;
    for (int8_t interface = MAVLINK_COMM_0; interface <= MAVLINK_COMM_3; ++interface){
        if (interface == in_channel){
            continue; // skip the receiving interface
        }

        for (uint8_t i=0; i<num_routes; i++) {
            if (interface == routes[i].channel) {
                if(broadcast_system || (target_system == routes[i].sysid && 
                        (broadcast_component || target_component == routes[i].compid))){
                    if (comm_get_txspace(routes[i].channel) >=
                            ((uint16_t)msg->len) + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
                        _mavlink_resend_uart(routes[i].channel, msg);
                        #if ROUTING_DEBUG
                        ::printf("fwd msg %u from chan %u on chan %u sysid=%i compid=%i\n",
                                 msg->msgid,
                                 (unsigned)in_channel,
                                 (unsigned)routes[i].channel,
                                 target_system,
                                 target_component);
                        #endif
                    }
                    forwarded = true;
                    break; // only forward msg once per interface
                }
            }
        }
    }
    if (!forwarded && match_system) {
        process_locally = true;
    }

    return process_locally;
}

/*
  send a MAVLink message to all components with this vehicle's system id

  This is a no-op if no routes to components have been learned
*/
void MAVLink_routing::send_to_components(const mavlink_message_t* msg)
{
    bool sent_to_chan[MAVLINK_COMM_NUM_BUFFERS];
    memset(sent_to_chan, 0, sizeof(sent_to_chan));

    // check learned routes
    for (uint8_t i=0; i<num_routes; i++) {
        if ((routes[i].sysid == mavlink_system.sysid) && !sent_to_chan[routes[i].channel]) {
            if (comm_get_txspace(routes[i].channel) >= ((uint16_t)msg->len) + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
#if ROUTING_DEBUG
                ::printf("send msg %u on chan %u sysid=%u compid=%u\n",
                         msg->msgid,
                         (unsigned)routes[i].channel,
                         (unsigned)routes[i].sysid,
                         (unsigned)routes[i].compid);
#endif
                _mavlink_resend_uart(routes[i].channel, msg);
                sent_to_chan[routes[i].channel] = true;
            }
        }
    }
}

/*
  see if the message is for a new route and learn it
*/
void MAVLink_routing::learn_route(mavlink_channel_t in_channel, const mavlink_message_t* msg)
{
    uint8_t i;
    if (msg->sysid == 0 || 
        (msg->sysid == mavlink_system.sysid && 
         msg->compid == mavlink_system.compid)) {
        return;
    }
    for (i=0; i<num_routes; i++) {
        if (routes[i].sysid == msg->sysid && 
            routes[i].compid == msg->compid &&
            routes[i].channel == in_channel) {
            break;
        }
    }
    if (i == num_routes && i<MAVLINK_MAX_ROUTES) {
        routes[i].sysid = msg->sysid;
        routes[i].compid = msg->compid;
        routes[i].channel = in_channel;
        num_routes++;
#if ROUTING_DEBUG
        ::printf("learned route %u %u via %u\n",
                 (unsigned)msg->sysid, 
                 (unsigned)msg->compid,
                 (unsigned)in_channel);
#endif
    }
}


/*
  special handling for heartbeat messages. To ensure routing
  propogation heartbeat messages need to be forwarded on all channels
  except channels where the sysid/compid of the heartbeat could come from
*/
void MAVLink_routing::handle_heartbeat(mavlink_channel_t in_channel, const mavlink_message_t* msg)
{
    if (msg->compid == MAV_COMP_ID_GIMBAL)
    {
        //Mask out gimbal messages, since those are causing problems for the controller
        return;
    }

    uint16_t mask = GCS_MAVLINK::active_channel_mask();

    // don't send on the incoming channel. This should only matter if
    // the routing table is full
    mask &= ~(1U<<(in_channel-MAVLINK_COMM_0));

    // mask out channels that are known sources for this sysid/compid
    for (uint8_t i=0; i<num_routes; i++) {
        if (routes[i].sysid == msg->sysid && routes[i].compid == msg->compid) {
            mask &= ~(1U<<((unsigned)(routes[i].channel-MAVLINK_COMM_0)));
        }
    }

    if (mask == 0) {
        // nothing to send to
        return;
    }

    // send on the remaining channels
    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        if (mask & (1U<<i)) {
            mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
            if (comm_get_txspace(channel) >= ((uint16_t)msg->len) + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
#if ROUTING_DEBUG
                ::printf("fwd HB from chan %u on chan %u from sysid=%u compid=%u\n",
                         (unsigned)in_channel,
                         (unsigned)channel,
                         (unsigned)msg->sysid,
                         (unsigned)msg->compid);
#endif
                _mavlink_resend_uart(channel, msg);
            }
        }
    }
}


/*
  extract target sysid and compid from a message. int16_t is used so
  that the caller can set them to -1 and know when a sysid or compid
  target is found in the message
*/
void MAVLink_routing::get_targets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid)
{
    // unfortunately the targets are not in a consistent position in
    // the packets, so we need a switch. Using the single element
    // extraction functions (which are inline) makes this a bit faster
    // than it would otherwise be.
    // This list of messages below was extracted using:
    //
    // cat ardupilotmega/*h common/*h|egrep
    // 'get_target_system|get_target_component' |grep inline | cut
    // -d'(' -f1 | cut -d' ' -f4 > /tmp/targets.txt
    //
    // TODO: we should write a python script to extract this list
    // properly
    
    switch (msg->msgid) {
        // these messages only have a target system
    case MAVLINK_MSG_ID_CAMERA_FEEDBACK:
        sysid = mavlink_msg_camera_feedback_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_CAMERA_STATUS:
        sysid = mavlink_msg_camera_status_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
        sysid = mavlink_msg_change_operator_control_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_SET_MODE:
        sysid = mavlink_msg_set_mode_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
        sysid = mavlink_msg_set_gps_global_origin_get_target_system(msg);
        break;

    // these support both target system and target component
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
        sysid  = mavlink_msg_digicam_configure_get_target_system(msg);
        compid = mavlink_msg_digicam_configure_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
        sysid  = mavlink_msg_digicam_control_get_target_system(msg);
        compid = mavlink_msg_digicam_control_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
        sysid  = mavlink_msg_fence_fetch_point_get_target_system(msg);
        compid = mavlink_msg_fence_fetch_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_FENCE_POINT:
        sysid  = mavlink_msg_fence_point_get_target_system(msg);
        compid = mavlink_msg_fence_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
        sysid  = mavlink_msg_mount_configure_get_target_system(msg);
        compid = mavlink_msg_mount_configure_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        sysid  = mavlink_msg_mount_control_get_target_system(msg);
        compid = mavlink_msg_mount_control_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_STATUS:
        sysid  = mavlink_msg_mount_status_get_target_system(msg);
        compid = mavlink_msg_mount_status_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT:
        sysid  = mavlink_msg_rally_fetch_point_get_target_system(msg);
        compid = mavlink_msg_rally_fetch_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_RALLY_POINT:
        sysid  = mavlink_msg_rally_point_get_target_system(msg);
        compid = mavlink_msg_rally_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
        sysid  = mavlink_msg_set_mag_offsets_get_target_system(msg);
        compid = mavlink_msg_set_mag_offsets_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_COMMAND_INT:
        sysid  = mavlink_msg_command_int_get_target_system(msg);
        compid = mavlink_msg_command_int_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        sysid  = mavlink_msg_command_long_get_target_system(msg);
        compid = mavlink_msg_command_long_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        sysid  = mavlink_msg_file_transfer_protocol_get_target_system(msg);
        compid = mavlink_msg_file_transfer_protocol_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        sysid  = mavlink_msg_gps_inject_data_get_target_system(msg);
        compid = mavlink_msg_gps_inject_data_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_ERASE:
        sysid  = mavlink_msg_log_erase_get_target_system(msg);
        compid = mavlink_msg_log_erase_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        sysid  = mavlink_msg_log_request_data_get_target_system(msg);
        compid = mavlink_msg_log_request_data_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        sysid  = mavlink_msg_log_request_end_get_target_system(msg);
        compid = mavlink_msg_log_request_end_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        sysid  = mavlink_msg_log_request_list_get_target_system(msg);
        compid = mavlink_msg_log_request_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ACK:
        sysid  = mavlink_msg_mission_ack_get_target_system(msg);
        compid = mavlink_msg_mission_ack_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        sysid  = mavlink_msg_mission_clear_all_get_target_system(msg);
        compid = mavlink_msg_mission_clear_all_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_COUNT:
        sysid  = mavlink_msg_mission_count_get_target_system(msg);
        compid = mavlink_msg_mission_count_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ITEM:
        sysid  = mavlink_msg_mission_item_get_target_system(msg);
        compid = mavlink_msg_mission_item_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        sysid  = mavlink_msg_mission_item_int_get_target_system(msg);
        compid = mavlink_msg_mission_item_int_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST:
        sysid  = mavlink_msg_mission_request_get_target_system(msg);
        compid = mavlink_msg_mission_request_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        sysid  = mavlink_msg_mission_request_list_get_target_system(msg);
        compid = mavlink_msg_mission_request_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
        sysid  = mavlink_msg_mission_request_partial_list_get_target_system(msg);
        compid = mavlink_msg_mission_request_partial_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        sysid  = mavlink_msg_mission_set_current_get_target_system(msg);
        compid = mavlink_msg_mission_set_current_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
        sysid  = mavlink_msg_mission_write_partial_list_get_target_system(msg);
        compid = mavlink_msg_mission_write_partial_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        sysid  = mavlink_msg_param_request_list_get_target_system(msg);
        compid = mavlink_msg_param_request_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        sysid  = mavlink_msg_param_request_read_get_target_system(msg);
        compid = mavlink_msg_param_request_read_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_SET:
        sysid  = mavlink_msg_param_set_get_target_system(msg);
        compid = mavlink_msg_param_set_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PING:
        sysid  = mavlink_msg_ping_get_target_system(msg);
        compid = mavlink_msg_ping_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        sysid  = mavlink_msg_rc_channels_override_get_target_system(msg);
        compid = mavlink_msg_rc_channels_override_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        sysid  = mavlink_msg_request_data_stream_get_target_system(msg);
        compid = mavlink_msg_request_data_stream_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA:
        sysid  = mavlink_msg_safety_set_allowed_area_get_target_system(msg);
        compid = mavlink_msg_safety_set_allowed_area_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        sysid  = mavlink_msg_set_attitude_target_get_target_system(msg);
        compid = mavlink_msg_set_attitude_target_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        sysid  = mavlink_msg_set_position_target_global_int_get_target_system(msg);
        compid = mavlink_msg_set_position_target_global_int_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        sysid  = mavlink_msg_set_position_target_local_ned_get_target_system(msg);
        compid = mavlink_msg_set_position_target_local_ned_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_V2_EXTENSION:
        sysid  = mavlink_msg_v2_extension_get_target_system(msg);
        compid = mavlink_msg_v2_extension_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_REPORT:
        sysid  = mavlink_msg_gimbal_report_get_target_system(msg);
        compid = mavlink_msg_gimbal_report_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_CONTROL:
        sysid  = mavlink_msg_gimbal_control_get_target_system(msg);
        compid = mavlink_msg_gimbal_control_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK:
        sysid  = mavlink_msg_remote_log_data_block_get_target_system(msg);
        compid = mavlink_msg_remote_log_data_block_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        sysid  = mavlink_msg_remote_log_block_status_get_target_system(msg);
        compid = mavlink_msg_remote_log_block_status_get_target_component(msg);
        break;
    }
}

