// MESSAGE MAG_CAL_ACK PACKING

#define MAVLINK_MSG_ID_MAG_CAL_ACK 193

typedef struct __mavlink_mag_cal_ack_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t compass_id; ///< Compass ID
 uint8_t accept_cal; ///< 0=Discard the calibration, 1=Save the calibration
} mavlink_mag_cal_ack_t;

#define MAVLINK_MSG_ID_MAG_CAL_ACK_LEN 4
#define MAVLINK_MSG_ID_193_LEN 4

#define MAVLINK_MSG_ID_MAG_CAL_ACK_CRC 133
#define MAVLINK_MSG_ID_193_CRC 133



#define MAVLINK_MESSAGE_INFO_MAG_CAL_ACK { \
	"MAG_CAL_ACK", \
	4, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mag_cal_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mag_cal_ack_t, target_component) }, \
         { "compass_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mag_cal_ack_t, compass_id) }, \
         { "accept_cal", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mag_cal_ack_t, accept_cal) }, \
         } \
}


/**
 * @brief Pack a mag_cal_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param compass_id Compass ID
 * @param accept_cal 0=Discard the calibration, 1=Save the calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mag_cal_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t compass_id, uint8_t accept_cal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAG_CAL_ACK_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, compass_id);
	_mav_put_uint8_t(buf, 3, accept_cal);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#else
	mavlink_mag_cal_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.compass_id = compass_id;
	packet.accept_cal = accept_cal;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAG_CAL_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN, MAVLINK_MSG_ID_MAG_CAL_ACK_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#endif
}

/**
 * @brief Pack a mag_cal_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param compass_id Compass ID
 * @param accept_cal 0=Discard the calibration, 1=Save the calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mag_cal_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t compass_id,uint8_t accept_cal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAG_CAL_ACK_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, compass_id);
	_mav_put_uint8_t(buf, 3, accept_cal);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#else
	mavlink_mag_cal_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.compass_id = compass_id;
	packet.accept_cal = accept_cal;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAG_CAL_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN, MAVLINK_MSG_ID_MAG_CAL_ACK_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#endif
}

/**
 * @brief Encode a mag_cal_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mag_cal_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mag_cal_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mag_cal_ack_t* mag_cal_ack)
{
	return mavlink_msg_mag_cal_ack_pack(system_id, component_id, msg, mag_cal_ack->target_system, mag_cal_ack->target_component, mag_cal_ack->compass_id, mag_cal_ack->accept_cal);
}

/**
 * @brief Encode a mag_cal_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mag_cal_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mag_cal_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mag_cal_ack_t* mag_cal_ack)
{
	return mavlink_msg_mag_cal_ack_pack_chan(system_id, component_id, chan, msg, mag_cal_ack->target_system, mag_cal_ack->target_component, mag_cal_ack->compass_id, mag_cal_ack->accept_cal);
}

/**
 * @brief Send a mag_cal_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param compass_id Compass ID
 * @param accept_cal 0=Discard the calibration, 1=Save the calibration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mag_cal_ack_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t compass_id, uint8_t accept_cal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAG_CAL_ACK_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, compass_id);
	_mav_put_uint8_t(buf, 3, accept_cal);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_ACK, buf, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN, MAVLINK_MSG_ID_MAG_CAL_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_ACK, buf, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#endif
#else
	mavlink_mag_cal_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.compass_id = compass_id;
	packet.accept_cal = accept_cal;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_ACK, (const char *)&packet, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN, MAVLINK_MSG_ID_MAG_CAL_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_ACK, (const char *)&packet, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MAG_CAL_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mag_cal_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t compass_id, uint8_t accept_cal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, compass_id);
	_mav_put_uint8_t(buf, 3, accept_cal);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_ACK, buf, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN, MAVLINK_MSG_ID_MAG_CAL_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_ACK, buf, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#endif
#else
	mavlink_mag_cal_ack_t *packet = (mavlink_mag_cal_ack_t *)msgbuf;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->compass_id = compass_id;
	packet->accept_cal = accept_cal;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_ACK, (const char *)packet, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN, MAVLINK_MSG_ID_MAG_CAL_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_ACK, (const char *)packet, MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MAG_CAL_ACK UNPACKING


/**
 * @brief Get field target_system from mag_cal_ack message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mag_cal_ack_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from mag_cal_ack message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_mag_cal_ack_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field compass_id from mag_cal_ack message
 *
 * @return Compass ID
 */
static inline uint8_t mavlink_msg_mag_cal_ack_get_compass_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field accept_cal from mag_cal_ack message
 *
 * @return 0=Discard the calibration, 1=Save the calibration
 */
static inline uint8_t mavlink_msg_mag_cal_ack_get_accept_cal(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a mag_cal_ack message into a struct
 *
 * @param msg The message to decode
 * @param mag_cal_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_mag_cal_ack_decode(const mavlink_message_t* msg, mavlink_mag_cal_ack_t* mag_cal_ack)
{
#if MAVLINK_NEED_BYTE_SWAP
	mag_cal_ack->target_system = mavlink_msg_mag_cal_ack_get_target_system(msg);
	mag_cal_ack->target_component = mavlink_msg_mag_cal_ack_get_target_component(msg);
	mag_cal_ack->compass_id = mavlink_msg_mag_cal_ack_get_compass_id(msg);
	mag_cal_ack->accept_cal = mavlink_msg_mag_cal_ack_get_accept_cal(msg);
#else
	memcpy(mag_cal_ack, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MAG_CAL_ACK_LEN);
#endif
}
