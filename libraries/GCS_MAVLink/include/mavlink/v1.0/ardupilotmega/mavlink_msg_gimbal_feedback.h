// MESSAGE GIMBAL_FEEDBACK PACKING

#if MAVLINK_C2000
#include "protocol_c2000.h"
#endif

#define MAVLINK_MSG_ID_GIMBAL_FEEDBACK 183

typedef struct __mavlink_gimbal_feedback_t
{
 float accx; ///< Delta velocity X, m/s
 float accy; ///< Delta velocity Y, m/s
 float accz; ///< Delta velocity Z, m/s
 float gyrox; ///< Delta angle X, radians
 float gyroy; ///< Delta angle Y, radians
 float gyroz; ///< Delta angle X, radians
 float joint_az; ///<  Joint AZ, radians
 float joint_roll; ///<  Joint ROLL, radians
 float joint_el; ///<  Joint EL, radians
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t id; ///< Message identifier
} mavlink_gimbal_feedback_t;

#define MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN 39
#define MAVLINK_MSG_ID_183_LEN 39

#define MAVLINK_MSG_ID_GIMBAL_FEEDBACK_CRC 27
#define MAVLINK_MSG_ID_183_CRC 27



#define MAVLINK_MESSAGE_INFO_GIMBAL_FEEDBACK { \
	"GIMBAL_FEEDBACK", \
	12, \
	{  { "accx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_feedback_t, accx) }, \
         { "accy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_feedback_t, accy) }, \
         { "accz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_feedback_t, accz) }, \
         { "gyrox", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_feedback_t, gyrox) }, \
         { "gyroy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gimbal_feedback_t, gyroy) }, \
         { "gyroz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_feedback_t, gyroz) }, \
         { "joint_az", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_feedback_t, joint_az) }, \
         { "joint_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gimbal_feedback_t, joint_roll) }, \
         { "joint_el", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gimbal_feedback_t, joint_el) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_gimbal_feedback_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_gimbal_feedback_t, target_component) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_gimbal_feedback_t, id) }, \
         } \
}


/**
 * @brief Pack a gimbal_feedback message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param id Message identifier
 * @param accx Delta velocity X, m/s
 * @param accy Delta velocity Y, m/s
 * @param accz Delta velocity Z, m/s
 * @param gyrox Delta angle X, radians
 * @param gyroy Delta angle Y, radians
 * @param gyroz Delta angle X, radians
 * @param joint_az  Joint AZ, radians
 * @param joint_roll  Joint ROLL, radians
 * @param joint_el  Joint EL, radians
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_feedback_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t id, float accx, float accy, float accz, float gyrox, float gyroy, float gyroz, float joint_az, float joint_roll, float joint_el)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN];
	_mav_put_float(buf, 0, accx);
	_mav_put_float(buf, 4, accy);
	_mav_put_float(buf, 8, accz);
	_mav_put_float(buf, 12, gyrox);
	_mav_put_float(buf, 16, gyroy);
	_mav_put_float(buf, 20, gyroz);
	_mav_put_float(buf, 24, joint_az);
	_mav_put_float(buf, 28, joint_roll);
	_mav_put_float(buf, 32, joint_el);
	_mav_put_uint8_t(buf, 36, target_system);
	_mav_put_uint8_t(buf, 37, target_component);
	_mav_put_uint8_t(buf, 38, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#elif MAVLINK_C2000
		mav_put_float_c2000(&(msg->payload64[0]), 0, accx);
		mav_put_float_c2000(&(msg->payload64[0]), 4, accy);
		mav_put_float_c2000(&(msg->payload64[0]), 8, accz);
		mav_put_float_c2000(&(msg->payload64[0]), 12, gyrox);
		mav_put_float_c2000(&(msg->payload64[0]), 16, gyroy);
		mav_put_float_c2000(&(msg->payload64[0]), 20, gyroz);
		mav_put_float_c2000(&(msg->payload64[0]), 24, joint_az);
		mav_put_float_c2000(&(msg->payload64[0]), 28, joint_roll);
		mav_put_float_c2000(&(msg->payload64[0]), 32, joint_el);
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 36, target_system);
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 37, target_component);
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 38, id);
	
	
#else
	mavlink_gimbal_feedback_t packet;
	packet.accx = accx;
	packet.accy = accy;
	packet.accz = accz;
	packet.gyrox = gyrox;
	packet.gyroy = gyroy;
	packet.gyroz = gyroz;
	packet.joint_az = joint_az;
	packet.joint_roll = joint_roll;
	packet.joint_el = joint_el;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_FEEDBACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#endif
}

/**
 * @brief Pack a gimbal_feedback message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param id Message identifier
 * @param accx Delta velocity X, m/s
 * @param accy Delta velocity Y, m/s
 * @param accz Delta velocity Z, m/s
 * @param gyrox Delta angle X, radians
 * @param gyroy Delta angle Y, radians
 * @param gyroz Delta angle X, radians
 * @param joint_az  Joint AZ, radians
 * @param joint_roll  Joint ROLL, radians
 * @param joint_el  Joint EL, radians
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_feedback_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t id,float accx,float accy,float accz,float gyrox,float gyroy,float gyroz,float joint_az,float joint_roll,float joint_el)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN];
	_mav_put_float(buf, 0, accx);
	_mav_put_float(buf, 4, accy);
	_mav_put_float(buf, 8, accz);
	_mav_put_float(buf, 12, gyrox);
	_mav_put_float(buf, 16, gyroy);
	_mav_put_float(buf, 20, gyroz);
	_mav_put_float(buf, 24, joint_az);
	_mav_put_float(buf, 28, joint_roll);
	_mav_put_float(buf, 32, joint_el);
	_mav_put_uint8_t(buf, 36, target_system);
	_mav_put_uint8_t(buf, 37, target_component);
	_mav_put_uint8_t(buf, 38, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#else
	mavlink_gimbal_feedback_t packet;
	packet.accx = accx;
	packet.accy = accy;
	packet.accz = accz;
	packet.gyrox = gyrox;
	packet.gyroy = gyroy;
	packet.gyroz = gyroz;
	packet.joint_az = joint_az;
	packet.joint_roll = joint_roll;
	packet.joint_el = joint_el;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_FEEDBACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#endif
}

/**
 * @brief Encode a gimbal_feedback struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_feedback_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_feedback_t* gimbal_feedback)
{
	return mavlink_msg_gimbal_feedback_pack(system_id, component_id, msg, gimbal_feedback->target_system, gimbal_feedback->target_component, gimbal_feedback->id, gimbal_feedback->accx, gimbal_feedback->accy, gimbal_feedback->accz, gimbal_feedback->gyrox, gimbal_feedback->gyroy, gimbal_feedback->gyroz, gimbal_feedback->joint_az, gimbal_feedback->joint_roll, gimbal_feedback->joint_el);
}

/**
 * @brief Encode a gimbal_feedback struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_feedback_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_feedback_t* gimbal_feedback)
{
	return mavlink_msg_gimbal_feedback_pack_chan(system_id, component_id, chan, msg, gimbal_feedback->target_system, gimbal_feedback->target_component, gimbal_feedback->id, gimbal_feedback->accx, gimbal_feedback->accy, gimbal_feedback->accz, gimbal_feedback->gyrox, gimbal_feedback->gyroy, gimbal_feedback->gyroz, gimbal_feedback->joint_az, gimbal_feedback->joint_roll, gimbal_feedback->joint_el);
}

/**
 * @brief Send a gimbal_feedback message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param id Message identifier
 * @param accx Delta velocity X, m/s
 * @param accy Delta velocity Y, m/s
 * @param accz Delta velocity Z, m/s
 * @param gyrox Delta angle X, radians
 * @param gyroy Delta angle Y, radians
 * @param gyroz Delta angle X, radians
 * @param joint_az  Joint AZ, radians
 * @param joint_roll  Joint ROLL, radians
 * @param joint_el  Joint EL, radians
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_feedback_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t id, float accx, float accy, float accz, float gyrox, float gyroy, float gyroz, float joint_az, float joint_roll, float joint_el)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN];
	_mav_put_float(buf, 0, accx);
	_mav_put_float(buf, 4, accy);
	_mav_put_float(buf, 8, accz);
	_mav_put_float(buf, 12, gyrox);
	_mav_put_float(buf, 16, gyroy);
	_mav_put_float(buf, 20, gyroz);
	_mav_put_float(buf, 24, joint_az);
	_mav_put_float(buf, 28, joint_roll);
	_mav_put_float(buf, 32, joint_el);
	_mav_put_uint8_t(buf, 36, target_system);
	_mav_put_uint8_t(buf, 37, target_component);
	_mav_put_uint8_t(buf, 38, id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FEEDBACK, buf, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FEEDBACK, buf, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#endif
#else
	mavlink_gimbal_feedback_t packet;
	packet.accx = accx;
	packet.accy = accy;
	packet.accz = accz;
	packet.gyrox = gyrox;
	packet.gyroy = gyroy;
	packet.gyroz = gyroz;
	packet.joint_az = joint_az;
	packet.joint_roll = joint_roll;
	packet.joint_el = joint_el;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.id = id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FEEDBACK, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FEEDBACK, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_feedback_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t id, float accx, float accy, float accz, float gyrox, float gyroy, float gyroz, float joint_az, float joint_roll, float joint_el)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, accx);
	_mav_put_float(buf, 4, accy);
	_mav_put_float(buf, 8, accz);
	_mav_put_float(buf, 12, gyrox);
	_mav_put_float(buf, 16, gyroy);
	_mav_put_float(buf, 20, gyroz);
	_mav_put_float(buf, 24, joint_az);
	_mav_put_float(buf, 28, joint_roll);
	_mav_put_float(buf, 32, joint_el);
	_mav_put_uint8_t(buf, 36, target_system);
	_mav_put_uint8_t(buf, 37, target_component);
	_mav_put_uint8_t(buf, 38, id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FEEDBACK, buf, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FEEDBACK, buf, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#endif
#else
	mavlink_gimbal_feedback_t *packet = (mavlink_gimbal_feedback_t *)msgbuf;
	packet->accx = accx;
	packet->accy = accy;
	packet->accz = accz;
	packet->gyrox = gyrox;
	packet->gyroy = gyroy;
	packet->gyroz = gyroz;
	packet->joint_az = joint_az;
	packet->joint_roll = joint_roll;
	packet->joint_el = joint_el;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->id = id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FEEDBACK, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FEEDBACK, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GIMBAL_FEEDBACK UNPACKING


/**
 * @brief Get field target_system from gimbal_feedback message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gimbal_feedback_get_target_system(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  36);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  36);
#endif
}

/**
 * @brief Get field target_component from gimbal_feedback message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gimbal_feedback_get_target_component(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  37);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  37);
#endif
}

/**
 * @brief Get field id from gimbal_feedback message
 *
 * @return Message identifier
 */
static inline uint8_t mavlink_msg_gimbal_feedback_get_id(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  38);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  38);
#endif
}

/**
 * @brief Get field accx from gimbal_feedback message
 *
 * @return Delta velocity X, m/s
 */
static inline float mavlink_msg_gimbal_feedback_get_accx(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_float(msg,  0);
#else
	return mav_get_float_c2000(&(msg->payload64[0]),  0);
#endif
}

/**
 * @brief Get field accy from gimbal_feedback message
 *
 * @return Delta velocity Y, m/s
 */
static inline float mavlink_msg_gimbal_feedback_get_accy(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_float(msg,  4);
#else
	return mav_get_float_c2000(&(msg->payload64[0]),  4);
#endif
}

/**
 * @brief Get field accz from gimbal_feedback message
 *
 * @return Delta velocity Z, m/s
 */
static inline float mavlink_msg_gimbal_feedback_get_accz(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_float(msg,  8);
#else
	return mav_get_float_c2000(&(msg->payload64[0]),  8);
#endif
}

/**
 * @brief Get field gyrox from gimbal_feedback message
 *
 * @return Delta angle X, radians
 */
static inline float mavlink_msg_gimbal_feedback_get_gyrox(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_float(msg,  12);
#else
	return mav_get_float_c2000(&(msg->payload64[0]),  12);
#endif
}

/**
 * @brief Get field gyroy from gimbal_feedback message
 *
 * @return Delta angle Y, radians
 */
static inline float mavlink_msg_gimbal_feedback_get_gyroy(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_float(msg,  16);
#else
	return mav_get_float_c2000(&(msg->payload64[0]),  16);
#endif
}

/**
 * @brief Get field gyroz from gimbal_feedback message
 *
 * @return Delta angle X, radians
 */
static inline float mavlink_msg_gimbal_feedback_get_gyroz(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_float(msg,  20);
#else
	return mav_get_float_c2000(&(msg->payload64[0]),  20);
#endif
}

/**
 * @brief Get field joint_az from gimbal_feedback message
 *
 * @return  Joint AZ, radians
 */
static inline float mavlink_msg_gimbal_feedback_get_joint_az(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_float(msg,  24);
#else
	return mav_get_float_c2000(&(msg->payload64[0]),  24);
#endif
}

/**
 * @brief Get field joint_roll from gimbal_feedback message
 *
 * @return  Joint ROLL, radians
 */
static inline float mavlink_msg_gimbal_feedback_get_joint_roll(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_float(msg,  28);
#else
	return mav_get_float_c2000(&(msg->payload64[0]),  28);
#endif
}

/**
 * @brief Get field joint_el from gimbal_feedback message
 *
 * @return  Joint EL, radians
 */
static inline float mavlink_msg_gimbal_feedback_get_joint_el(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_float(msg,  32);
#else
	return mav_get_float_c2000(&(msg->payload64[0]),  32);
#endif
}

/**
 * @brief Decode a gimbal_feedback message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_feedback C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_feedback_decode(const mavlink_message_t* msg, mavlink_gimbal_feedback_t* gimbal_feedback)
{
#if MAVLINK_NEED_BYTE_SWAP || MAVLINK_C2000
	gimbal_feedback->accx = mavlink_msg_gimbal_feedback_get_accx(msg);
	gimbal_feedback->accy = mavlink_msg_gimbal_feedback_get_accy(msg);
	gimbal_feedback->accz = mavlink_msg_gimbal_feedback_get_accz(msg);
	gimbal_feedback->gyrox = mavlink_msg_gimbal_feedback_get_gyrox(msg);
	gimbal_feedback->gyroy = mavlink_msg_gimbal_feedback_get_gyroy(msg);
	gimbal_feedback->gyroz = mavlink_msg_gimbal_feedback_get_gyroz(msg);
	gimbal_feedback->joint_az = mavlink_msg_gimbal_feedback_get_joint_az(msg);
	gimbal_feedback->joint_roll = mavlink_msg_gimbal_feedback_get_joint_roll(msg);
	gimbal_feedback->joint_el = mavlink_msg_gimbal_feedback_get_joint_el(msg);
	gimbal_feedback->target_system = mavlink_msg_gimbal_feedback_get_target_system(msg);
	gimbal_feedback->target_component = mavlink_msg_gimbal_feedback_get_target_component(msg);
	gimbal_feedback->id = mavlink_msg_gimbal_feedback_get_id(msg);
#else
	memcpy(gimbal_feedback, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GIMBAL_FEEDBACK_LEN);
#endif
}
