// MESSAGE R10C_GIMBAL_REPORT PACKING

#define MAVLINK_MSG_ID_R10C_GIMBAL_REPORT 224

typedef struct __mavlink_r10c_gimbal_report_t
{
 int32_t pitch_ref; /*< pitch reference (radians*1000)*/
 int32_t roll_out; /*< roll angle command (radians*1000)*/
 int32_t pitch_out; /*< pitch angle command (radians*1000)*/
 uint32_t roll_pwm; /*< roll pwm output*/
 uint32_t pitch_pwm; /*< pitch pwm output*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
} mavlink_r10c_gimbal_report_t;

#define MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN 22
#define MAVLINK_MSG_ID_224_LEN 22

#define MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_CRC 133
#define MAVLINK_MSG_ID_224_CRC 133



#define MAVLINK_MESSAGE_INFO_R10C_GIMBAL_REPORT { \
	"R10C_GIMBAL_REPORT", \
	7, \
	{  { "pitch_ref", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_r10c_gimbal_report_t, pitch_ref) }, \
         { "roll_out", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_r10c_gimbal_report_t, roll_out) }, \
         { "pitch_out", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_r10c_gimbal_report_t, pitch_out) }, \
         { "roll_pwm", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_r10c_gimbal_report_t, roll_pwm) }, \
         { "pitch_pwm", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_r10c_gimbal_report_t, pitch_pwm) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_r10c_gimbal_report_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_r10c_gimbal_report_t, target_component) }, \
         } \
}


/**
 * @brief Pack a r10c_gimbal_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param pitch_ref pitch reference (radians*1000)
 * @param roll_out roll angle command (radians*1000)
 * @param pitch_out pitch angle command (radians*1000)
 * @param roll_pwm roll pwm output
 * @param pitch_pwm pitch pwm output
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_r10c_gimbal_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, int32_t pitch_ref, int32_t roll_out, int32_t pitch_out, uint32_t roll_pwm, uint32_t pitch_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN];
	_mav_put_int32_t(buf, 0, pitch_ref);
	_mav_put_int32_t(buf, 4, roll_out);
	_mav_put_int32_t(buf, 8, pitch_out);
	_mav_put_uint32_t(buf, 12, roll_pwm);
	_mav_put_uint32_t(buf, 16, pitch_pwm);
	_mav_put_uint8_t(buf, 20, target_system);
	_mav_put_uint8_t(buf, 21, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#else
	mavlink_r10c_gimbal_report_t packet;
	packet.pitch_ref = pitch_ref;
	packet.roll_out = roll_out;
	packet.pitch_out = pitch_out;
	packet.roll_pwm = roll_pwm;
	packet.pitch_pwm = pitch_pwm;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_R10C_GIMBAL_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#endif
}

/**
 * @brief Pack a r10c_gimbal_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param pitch_ref pitch reference (radians*1000)
 * @param roll_out roll angle command (radians*1000)
 * @param pitch_out pitch angle command (radians*1000)
 * @param roll_pwm roll pwm output
 * @param pitch_pwm pitch pwm output
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_r10c_gimbal_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,int32_t pitch_ref,int32_t roll_out,int32_t pitch_out,uint32_t roll_pwm,uint32_t pitch_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN];
	_mav_put_int32_t(buf, 0, pitch_ref);
	_mav_put_int32_t(buf, 4, roll_out);
	_mav_put_int32_t(buf, 8, pitch_out);
	_mav_put_uint32_t(buf, 12, roll_pwm);
	_mav_put_uint32_t(buf, 16, pitch_pwm);
	_mav_put_uint8_t(buf, 20, target_system);
	_mav_put_uint8_t(buf, 21, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#else
	mavlink_r10c_gimbal_report_t packet;
	packet.pitch_ref = pitch_ref;
	packet.roll_out = roll_out;
	packet.pitch_out = pitch_out;
	packet.roll_pwm = roll_pwm;
	packet.pitch_pwm = pitch_pwm;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_R10C_GIMBAL_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#endif
}

/**
 * @brief Encode a r10c_gimbal_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param r10c_gimbal_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_r10c_gimbal_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_r10c_gimbal_report_t* r10c_gimbal_report)
{
	return mavlink_msg_r10c_gimbal_report_pack(system_id, component_id, msg, r10c_gimbal_report->target_system, r10c_gimbal_report->target_component, r10c_gimbal_report->pitch_ref, r10c_gimbal_report->roll_out, r10c_gimbal_report->pitch_out, r10c_gimbal_report->roll_pwm, r10c_gimbal_report->pitch_pwm);
}

/**
 * @brief Encode a r10c_gimbal_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param r10c_gimbal_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_r10c_gimbal_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_r10c_gimbal_report_t* r10c_gimbal_report)
{
	return mavlink_msg_r10c_gimbal_report_pack_chan(system_id, component_id, chan, msg, r10c_gimbal_report->target_system, r10c_gimbal_report->target_component, r10c_gimbal_report->pitch_ref, r10c_gimbal_report->roll_out, r10c_gimbal_report->pitch_out, r10c_gimbal_report->roll_pwm, r10c_gimbal_report->pitch_pwm);
}

/**
 * @brief Send a r10c_gimbal_report message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param pitch_ref pitch reference (radians*1000)
 * @param roll_out roll angle command (radians*1000)
 * @param pitch_out pitch angle command (radians*1000)
 * @param roll_pwm roll pwm output
 * @param pitch_pwm pitch pwm output
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_r10c_gimbal_report_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int32_t pitch_ref, int32_t roll_out, int32_t pitch_out, uint32_t roll_pwm, uint32_t pitch_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN];
	_mav_put_int32_t(buf, 0, pitch_ref);
	_mav_put_int32_t(buf, 4, roll_out);
	_mav_put_int32_t(buf, 8, pitch_out);
	_mav_put_uint32_t(buf, 12, roll_pwm);
	_mav_put_uint32_t(buf, 16, pitch_pwm);
	_mav_put_uint8_t(buf, 20, target_system);
	_mav_put_uint8_t(buf, 21, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT, buf, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT, buf, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#endif
#else
	mavlink_r10c_gimbal_report_t packet;
	packet.pitch_ref = pitch_ref;
	packet.roll_out = roll_out;
	packet.pitch_out = pitch_out;
	packet.roll_pwm = roll_pwm;
	packet.pitch_pwm = pitch_pwm;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT, (const char *)&packet, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT, (const char *)&packet, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_r10c_gimbal_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, int32_t pitch_ref, int32_t roll_out, int32_t pitch_out, uint32_t roll_pwm, uint32_t pitch_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, pitch_ref);
	_mav_put_int32_t(buf, 4, roll_out);
	_mav_put_int32_t(buf, 8, pitch_out);
	_mav_put_uint32_t(buf, 12, roll_pwm);
	_mav_put_uint32_t(buf, 16, pitch_pwm);
	_mav_put_uint8_t(buf, 20, target_system);
	_mav_put_uint8_t(buf, 21, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT, buf, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT, buf, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#endif
#else
	mavlink_r10c_gimbal_report_t *packet = (mavlink_r10c_gimbal_report_t *)msgbuf;
	packet->pitch_ref = pitch_ref;
	packet->roll_out = roll_out;
	packet->pitch_out = pitch_out;
	packet->roll_pwm = roll_pwm;
	packet->pitch_pwm = pitch_pwm;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT, (const char *)packet, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT, (const char *)packet, MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE R10C_GIMBAL_REPORT UNPACKING


/**
 * @brief Get field target_system from r10c_gimbal_report message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_r10c_gimbal_report_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field target_component from r10c_gimbal_report message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_r10c_gimbal_report_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field pitch_ref from r10c_gimbal_report message
 *
 * @return pitch reference (radians*1000)
 */
static inline int32_t mavlink_msg_r10c_gimbal_report_get_pitch_ref(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field roll_out from r10c_gimbal_report message
 *
 * @return roll angle command (radians*1000)
 */
static inline int32_t mavlink_msg_r10c_gimbal_report_get_roll_out(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field pitch_out from r10c_gimbal_report message
 *
 * @return pitch angle command (radians*1000)
 */
static inline int32_t mavlink_msg_r10c_gimbal_report_get_pitch_out(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field roll_pwm from r10c_gimbal_report message
 *
 * @return roll pwm output
 */
static inline uint32_t mavlink_msg_r10c_gimbal_report_get_roll_pwm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field pitch_pwm from r10c_gimbal_report message
 *
 * @return pitch pwm output
 */
static inline uint32_t mavlink_msg_r10c_gimbal_report_get_pitch_pwm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Decode a r10c_gimbal_report message into a struct
 *
 * @param msg The message to decode
 * @param r10c_gimbal_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_r10c_gimbal_report_decode(const mavlink_message_t* msg, mavlink_r10c_gimbal_report_t* r10c_gimbal_report)
{
#if MAVLINK_NEED_BYTE_SWAP
	r10c_gimbal_report->pitch_ref = mavlink_msg_r10c_gimbal_report_get_pitch_ref(msg);
	r10c_gimbal_report->roll_out = mavlink_msg_r10c_gimbal_report_get_roll_out(msg);
	r10c_gimbal_report->pitch_out = mavlink_msg_r10c_gimbal_report_get_pitch_out(msg);
	r10c_gimbal_report->roll_pwm = mavlink_msg_r10c_gimbal_report_get_roll_pwm(msg);
	r10c_gimbal_report->pitch_pwm = mavlink_msg_r10c_gimbal_report_get_pitch_pwm(msg);
	r10c_gimbal_report->target_system = mavlink_msg_r10c_gimbal_report_get_target_system(msg);
	r10c_gimbal_report->target_component = mavlink_msg_r10c_gimbal_report_get_target_component(msg);
#else
	memcpy(r10c_gimbal_report, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_R10C_GIMBAL_REPORT_LEN);
#endif
}
