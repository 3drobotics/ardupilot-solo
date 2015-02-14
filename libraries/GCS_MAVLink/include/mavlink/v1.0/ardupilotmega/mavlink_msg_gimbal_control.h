// MESSAGE GIMBAL_CONTROL PACKING

#define MAVLINK_MSG_ID_GIMBAL_CONTROL 184

typedef struct __mavlink_gimbal_control_t
{
 float ratex; ///< Desired angular speed around X axis rad/s
 float ratey; ///< Desired angular speed around Y axis rad/s
 float ratez; ///< Desired angular speed around Z axis rad/s
 float gyro_cal_x; ///< gyro X calibration
 float gyro_cal_y; ///< gyro Y calibration
 float gyro_cal_z; ///< gyro Z calibration
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t id; ///< Message identifier
} mavlink_gimbal_control_t;

#define MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN 27
#define MAVLINK_MSG_ID_184_LEN 27

#define MAVLINK_MSG_ID_GIMBAL_CONTROL_CRC 162
#define MAVLINK_MSG_ID_184_CRC 162



#define MAVLINK_MESSAGE_INFO_GIMBAL_CONTROL { \
	"GIMBAL_CONTROL", \
	9, \
	{  { "ratex", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_control_t, ratex) }, \
         { "ratey", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_control_t, ratey) }, \
         { "ratez", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_control_t, ratez) }, \
         { "gyro_cal_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_control_t, gyro_cal_x) }, \
         { "gyro_cal_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gimbal_control_t, gyro_cal_y) }, \
         { "gyro_cal_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_control_t, gyro_cal_z) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_gimbal_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_gimbal_control_t, target_component) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_gimbal_control_t, id) }, \
         } \
}


/**
 * @brief Pack a gimbal_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param id Message identifier
 * @param ratex Desired angular speed around X axis rad/s
 * @param ratey Desired angular speed around Y axis rad/s
 * @param ratez Desired angular speed around Z axis rad/s
 * @param gyro_cal_x gyro X calibration
 * @param gyro_cal_y gyro Y calibration
 * @param gyro_cal_z gyro Z calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t id, float ratex, float ratey, float ratez, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN];
	_mav_put_float(buf, 0, ratex);
	_mav_put_float(buf, 4, ratey);
	_mav_put_float(buf, 8, ratez);
	_mav_put_float(buf, 12, gyro_cal_x);
	_mav_put_float(buf, 16, gyro_cal_y);
	_mav_put_float(buf, 20, gyro_cal_z);
	_mav_put_uint8_t(buf, 24, target_system);
	_mav_put_uint8_t(buf, 25, target_component);
	_mav_put_uint8_t(buf, 26, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#else
	mavlink_gimbal_control_t packet;
	packet.ratex = ratex;
	packet.ratey = ratey;
	packet.ratez = ratez;
	packet.gyro_cal_x = gyro_cal_x;
	packet.gyro_cal_y = gyro_cal_y;
	packet.gyro_cal_z = gyro_cal_z;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN, MAVLINK_MSG_ID_GIMBAL_CONTROL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a gimbal_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param id Message identifier
 * @param ratex Desired angular speed around X axis rad/s
 * @param ratey Desired angular speed around Y axis rad/s
 * @param ratez Desired angular speed around Z axis rad/s
 * @param gyro_cal_x gyro X calibration
 * @param gyro_cal_y gyro Y calibration
 * @param gyro_cal_z gyro Z calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t id,float ratex,float ratey,float ratez,float gyro_cal_x,float gyro_cal_y,float gyro_cal_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN];
	_mav_put_float(buf, 0, ratex);
	_mav_put_float(buf, 4, ratey);
	_mav_put_float(buf, 8, ratez);
	_mav_put_float(buf, 12, gyro_cal_x);
	_mav_put_float(buf, 16, gyro_cal_y);
	_mav_put_float(buf, 20, gyro_cal_z);
	_mav_put_uint8_t(buf, 24, target_system);
	_mav_put_uint8_t(buf, 25, target_component);
	_mav_put_uint8_t(buf, 26, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#else
	mavlink_gimbal_control_t packet;
	packet.ratex = ratex;
	packet.ratey = ratey;
	packet.ratez = ratez;
	packet.gyro_cal_x = gyro_cal_x;
	packet.gyro_cal_y = gyro_cal_y;
	packet.gyro_cal_z = gyro_cal_z;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN, MAVLINK_MSG_ID_GIMBAL_CONTROL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#endif
}

/**
 * @brief Encode a gimbal_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_control_t* gimbal_control)
{
	return mavlink_msg_gimbal_control_pack(system_id, component_id, msg, gimbal_control->target_system, gimbal_control->target_component, gimbal_control->id, gimbal_control->ratex, gimbal_control->ratey, gimbal_control->ratez, gimbal_control->gyro_cal_x, gimbal_control->gyro_cal_y, gimbal_control->gyro_cal_z);
}

/**
 * @brief Encode a gimbal_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_control_t* gimbal_control)
{
	return mavlink_msg_gimbal_control_pack_chan(system_id, component_id, chan, msg, gimbal_control->target_system, gimbal_control->target_component, gimbal_control->id, gimbal_control->ratex, gimbal_control->ratey, gimbal_control->ratez, gimbal_control->gyro_cal_x, gimbal_control->gyro_cal_y, gimbal_control->gyro_cal_z);
}

/**
 * @brief Send a gimbal_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param id Message identifier
 * @param ratex Desired angular speed around X axis rad/s
 * @param ratey Desired angular speed around Y axis rad/s
 * @param ratez Desired angular speed around Z axis rad/s
 * @param gyro_cal_x gyro X calibration
 * @param gyro_cal_y gyro Y calibration
 * @param gyro_cal_z gyro Z calibration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_control_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t id, float ratex, float ratey, float ratez, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN];
	_mav_put_float(buf, 0, ratex);
	_mav_put_float(buf, 4, ratey);
	_mav_put_float(buf, 8, ratez);
	_mav_put_float(buf, 12, gyro_cal_x);
	_mav_put_float(buf, 16, gyro_cal_y);
	_mav_put_float(buf, 20, gyro_cal_z);
	_mav_put_uint8_t(buf, 24, target_system);
	_mav_put_uint8_t(buf, 25, target_component);
	_mav_put_uint8_t(buf, 26, id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CONTROL, buf, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN, MAVLINK_MSG_ID_GIMBAL_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CONTROL, buf, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#endif
#else
	mavlink_gimbal_control_t packet;
	packet.ratex = ratex;
	packet.ratey = ratey;
	packet.ratez = ratez;
	packet.gyro_cal_x = gyro_cal_x;
	packet.gyro_cal_y = gyro_cal_y;
	packet.gyro_cal_z = gyro_cal_z;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.id = id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN, MAVLINK_MSG_ID_GIMBAL_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t id, float ratex, float ratey, float ratez, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, ratex);
	_mav_put_float(buf, 4, ratey);
	_mav_put_float(buf, 8, ratez);
	_mav_put_float(buf, 12, gyro_cal_x);
	_mav_put_float(buf, 16, gyro_cal_y);
	_mav_put_float(buf, 20, gyro_cal_z);
	_mav_put_uint8_t(buf, 24, target_system);
	_mav_put_uint8_t(buf, 25, target_component);
	_mav_put_uint8_t(buf, 26, id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CONTROL, buf, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN, MAVLINK_MSG_ID_GIMBAL_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CONTROL, buf, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#endif
#else
	mavlink_gimbal_control_t *packet = (mavlink_gimbal_control_t *)msgbuf;
	packet->ratex = ratex;
	packet->ratey = ratey;
	packet->ratez = ratez;
	packet->gyro_cal_x = gyro_cal_x;
	packet->gyro_cal_y = gyro_cal_y;
	packet->gyro_cal_z = gyro_cal_z;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->id = id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CONTROL, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN, MAVLINK_MSG_ID_GIMBAL_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CONTROL, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GIMBAL_CONTROL UNPACKING


/**
 * @brief Get field target_system from gimbal_control message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gimbal_control_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field target_component from gimbal_control message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gimbal_control_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field id from gimbal_control message
 *
 * @return Message identifier
 */
static inline uint8_t mavlink_msg_gimbal_control_get_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field ratex from gimbal_control message
 *
 * @return Desired angular speed around X axis rad/s
 */
static inline float mavlink_msg_gimbal_control_get_ratex(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ratey from gimbal_control message
 *
 * @return Desired angular speed around Y axis rad/s
 */
static inline float mavlink_msg_gimbal_control_get_ratey(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field ratez from gimbal_control message
 *
 * @return Desired angular speed around Z axis rad/s
 */
static inline float mavlink_msg_gimbal_control_get_ratez(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gyro_cal_x from gimbal_control message
 *
 * @return gyro X calibration
 */
static inline float mavlink_msg_gimbal_control_get_gyro_cal_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyro_cal_y from gimbal_control message
 *
 * @return gyro Y calibration
 */
static inline float mavlink_msg_gimbal_control_get_gyro_cal_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyro_cal_z from gimbal_control message
 *
 * @return gyro Z calibration
 */
static inline float mavlink_msg_gimbal_control_get_gyro_cal_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a gimbal_control message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_control_decode(const mavlink_message_t* msg, mavlink_gimbal_control_t* gimbal_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	gimbal_control->ratex = mavlink_msg_gimbal_control_get_ratex(msg);
	gimbal_control->ratey = mavlink_msg_gimbal_control_get_ratey(msg);
	gimbal_control->ratez = mavlink_msg_gimbal_control_get_ratez(msg);
	gimbal_control->gyro_cal_x = mavlink_msg_gimbal_control_get_gyro_cal_x(msg);
	gimbal_control->gyro_cal_y = mavlink_msg_gimbal_control_get_gyro_cal_y(msg);
	gimbal_control->gyro_cal_z = mavlink_msg_gimbal_control_get_gyro_cal_z(msg);
	gimbal_control->target_system = mavlink_msg_gimbal_control_get_target_system(msg);
	gimbal_control->target_component = mavlink_msg_gimbal_control_get_target_component(msg);
	gimbal_control->id = mavlink_msg_gimbal_control_get_id(msg);
#else
	memcpy(gimbal_control, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN);
#endif
}
