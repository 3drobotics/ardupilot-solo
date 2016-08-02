// MESSAGE R10C_GIMBAL_UPDATE PACKING

#define MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE 223

typedef struct __mavlink_r10c_gimbal_update_t
{
 float roll; /*< copter roll (rad)*/
 float pitch; /*< copter pitch (rad)*/
 float yaw; /*< copter yaw (rad)*/
 float pitch_ref; /*< body-frame pitch reference (cd)*/
} mavlink_r10c_gimbal_update_t;

#define MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN 16
#define MAVLINK_MSG_ID_223_LEN 16

#define MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_CRC 68
#define MAVLINK_MSG_ID_223_CRC 68



#define MAVLINK_MESSAGE_INFO_R10C_GIMBAL_UPDATE { \
	"R10C_GIMBAL_UPDATE", \
	4, \
	{  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_r10c_gimbal_update_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_r10c_gimbal_update_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_r10c_gimbal_update_t, yaw) }, \
         { "pitch_ref", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_r10c_gimbal_update_t, pitch_ref) }, \
         } \
}


/**
 * @brief Pack a r10c_gimbal_update message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll copter roll (rad)
 * @param pitch copter pitch (rad)
 * @param yaw copter yaw (rad)
 * @param pitch_ref body-frame pitch reference (cd)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_r10c_gimbal_update_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float roll, float pitch, float yaw, float pitch_ref)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, pitch_ref);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#else
	mavlink_r10c_gimbal_update_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.pitch_ref = pitch_ref;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#endif
}

/**
 * @brief Pack a r10c_gimbal_update message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll copter roll (rad)
 * @param pitch copter pitch (rad)
 * @param yaw copter yaw (rad)
 * @param pitch_ref body-frame pitch reference (cd)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_r10c_gimbal_update_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float roll,float pitch,float yaw,float pitch_ref)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, pitch_ref);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#else
	mavlink_r10c_gimbal_update_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.pitch_ref = pitch_ref;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#endif
}

/**
 * @brief Encode a r10c_gimbal_update struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param r10c_gimbal_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_r10c_gimbal_update_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_r10c_gimbal_update_t* r10c_gimbal_update)
{
	return mavlink_msg_r10c_gimbal_update_pack(system_id, component_id, msg, r10c_gimbal_update->roll, r10c_gimbal_update->pitch, r10c_gimbal_update->yaw, r10c_gimbal_update->pitch_ref);
}

/**
 * @brief Encode a r10c_gimbal_update struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param r10c_gimbal_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_r10c_gimbal_update_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_r10c_gimbal_update_t* r10c_gimbal_update)
{
	return mavlink_msg_r10c_gimbal_update_pack_chan(system_id, component_id, chan, msg, r10c_gimbal_update->roll, r10c_gimbal_update->pitch, r10c_gimbal_update->yaw, r10c_gimbal_update->pitch_ref);
}

/**
 * @brief Send a r10c_gimbal_update message
 * @param chan MAVLink channel to send the message
 *
 * @param roll copter roll (rad)
 * @param pitch copter pitch (rad)
 * @param yaw copter yaw (rad)
 * @param pitch_ref body-frame pitch reference (cd)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_r10c_gimbal_update_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float pitch_ref)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, pitch_ref);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE, buf, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE, buf, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#endif
#else
	mavlink_r10c_gimbal_update_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.pitch_ref = pitch_ref;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE, (const char *)&packet, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE, (const char *)&packet, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_r10c_gimbal_update_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw, float pitch_ref)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, pitch_ref);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE, buf, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE, buf, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#endif
#else
	mavlink_r10c_gimbal_update_t *packet = (mavlink_r10c_gimbal_update_t *)msgbuf;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->pitch_ref = pitch_ref;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE, (const char *)packet, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE, (const char *)packet, MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE R10C_GIMBAL_UPDATE UNPACKING


/**
 * @brief Get field roll from r10c_gimbal_update message
 *
 * @return copter roll (rad)
 */
static inline float mavlink_msg_r10c_gimbal_update_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from r10c_gimbal_update message
 *
 * @return copter pitch (rad)
 */
static inline float mavlink_msg_r10c_gimbal_update_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from r10c_gimbal_update message
 *
 * @return copter yaw (rad)
 */
static inline float mavlink_msg_r10c_gimbal_update_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch_ref from r10c_gimbal_update message
 *
 * @return body-frame pitch reference (cd)
 */
static inline float mavlink_msg_r10c_gimbal_update_get_pitch_ref(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a r10c_gimbal_update message into a struct
 *
 * @param msg The message to decode
 * @param r10c_gimbal_update C-struct to decode the message contents into
 */
static inline void mavlink_msg_r10c_gimbal_update_decode(const mavlink_message_t* msg, mavlink_r10c_gimbal_update_t* r10c_gimbal_update)
{
#if MAVLINK_NEED_BYTE_SWAP
	r10c_gimbal_update->roll = mavlink_msg_r10c_gimbal_update_get_roll(msg);
	r10c_gimbal_update->pitch = mavlink_msg_r10c_gimbal_update_get_pitch(msg);
	r10c_gimbal_update->yaw = mavlink_msg_r10c_gimbal_update_get_yaw(msg);
	r10c_gimbal_update->pitch_ref = mavlink_msg_r10c_gimbal_update_get_pitch_ref(msg);
#else
	memcpy(r10c_gimbal_update, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_R10C_GIMBAL_UPDATE_LEN);
#endif
}
