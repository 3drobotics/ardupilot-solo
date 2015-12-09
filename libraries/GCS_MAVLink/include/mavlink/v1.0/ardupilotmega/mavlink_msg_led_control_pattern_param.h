// MESSAGE LED_CONTROL_PATTERN_PARAM PACKING

#define MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM 187

typedef struct __mavlink_led_control_pattern_param_t
{
 uint16_t param_value; /*< Parameter value*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t instance; /*< Instance (see LED_POSITIONS)*/
 uint8_t param_type; /*< Parameter type (see LED_CONTROL_PARAMS)*/
} mavlink_led_control_pattern_param_t;

#define MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN 6
#define MAVLINK_MSG_ID_187_LEN 6

#define MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_CRC 101
#define MAVLINK_MSG_ID_187_CRC 101



#define MAVLINK_MESSAGE_INFO_LED_CONTROL_PATTERN_PARAM { \
	"LED_CONTROL_PATTERN_PARAM", \
	5, \
	{  { "param_value", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_led_control_pattern_param_t, param_value) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_led_control_pattern_param_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_led_control_pattern_param_t, target_component) }, \
         { "instance", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_led_control_pattern_param_t, instance) }, \
         { "param_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_led_control_pattern_param_t, param_type) }, \
         } \
}


/**
 * @brief Pack a led_control_pattern_param message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param instance Instance (see LED_POSITIONS)
 * @param param_type Parameter type (see LED_CONTROL_PARAMS)
 * @param param_value Parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_led_control_pattern_param_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t param_type, uint16_t param_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN];
	_mav_put_uint16_t(buf, 0, param_value);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, instance);
	_mav_put_uint8_t(buf, 5, param_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#else
	mavlink_led_control_pattern_param_t packet;
	packet.param_value = param_value;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.instance = instance;
	packet.param_type = param_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#endif
}

/**
 * @brief Pack a led_control_pattern_param message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param instance Instance (see LED_POSITIONS)
 * @param param_type Parameter type (see LED_CONTROL_PARAMS)
 * @param param_value Parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_led_control_pattern_param_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t instance,uint8_t param_type,uint16_t param_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN];
	_mav_put_uint16_t(buf, 0, param_value);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, instance);
	_mav_put_uint8_t(buf, 5, param_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#else
	mavlink_led_control_pattern_param_t packet;
	packet.param_value = param_value;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.instance = instance;
	packet.param_type = param_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#endif
}

/**
 * @brief Encode a led_control_pattern_param struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param led_control_pattern_param C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_led_control_pattern_param_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_led_control_pattern_param_t* led_control_pattern_param)
{
	return mavlink_msg_led_control_pattern_param_pack(system_id, component_id, msg, led_control_pattern_param->target_system, led_control_pattern_param->target_component, led_control_pattern_param->instance, led_control_pattern_param->param_type, led_control_pattern_param->param_value);
}

/**
 * @brief Encode a led_control_pattern_param struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param led_control_pattern_param C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_led_control_pattern_param_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_led_control_pattern_param_t* led_control_pattern_param)
{
	return mavlink_msg_led_control_pattern_param_pack_chan(system_id, component_id, chan, msg, led_control_pattern_param->target_system, led_control_pattern_param->target_component, led_control_pattern_param->instance, led_control_pattern_param->param_type, led_control_pattern_param->param_value);
}

/**
 * @brief Send a led_control_pattern_param message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param instance Instance (see LED_POSITIONS)
 * @param param_type Parameter type (see LED_CONTROL_PARAMS)
 * @param param_value Parameter value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_led_control_pattern_param_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t param_type, uint16_t param_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN];
	_mav_put_uint16_t(buf, 0, param_value);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, instance);
	_mav_put_uint8_t(buf, 5, param_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM, buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM, buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#endif
#else
	mavlink_led_control_pattern_param_t packet;
	packet.param_value = param_value;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.instance = instance;
	packet.param_type = param_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM, (const char *)&packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM, (const char *)&packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_led_control_pattern_param_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t param_type, uint16_t param_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, param_value);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, instance);
	_mav_put_uint8_t(buf, 5, param_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM, buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM, buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#endif
#else
	mavlink_led_control_pattern_param_t *packet = (mavlink_led_control_pattern_param_t *)msgbuf;
	packet->param_value = param_value;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->instance = instance;
	packet->param_type = param_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM, (const char *)packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM, (const char *)packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE LED_CONTROL_PATTERN_PARAM UNPACKING


/**
 * @brief Get field target_system from led_control_pattern_param message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_led_control_pattern_param_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field target_component from led_control_pattern_param message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_led_control_pattern_param_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field instance from led_control_pattern_param message
 *
 * @return Instance (see LED_POSITIONS)
 */
static inline uint8_t mavlink_msg_led_control_pattern_param_get_instance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field param_type from led_control_pattern_param message
 *
 * @return Parameter type (see LED_CONTROL_PARAMS)
 */
static inline uint8_t mavlink_msg_led_control_pattern_param_get_param_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field param_value from led_control_pattern_param message
 *
 * @return Parameter value
 */
static inline uint16_t mavlink_msg_led_control_pattern_param_get_param_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a led_control_pattern_param message into a struct
 *
 * @param msg The message to decode
 * @param led_control_pattern_param C-struct to decode the message contents into
 */
static inline void mavlink_msg_led_control_pattern_param_decode(const mavlink_message_t* msg, mavlink_led_control_pattern_param_t* led_control_pattern_param)
{
#if MAVLINK_NEED_BYTE_SWAP
	led_control_pattern_param->param_value = mavlink_msg_led_control_pattern_param_get_param_value(msg);
	led_control_pattern_param->target_system = mavlink_msg_led_control_pattern_param_get_target_system(msg);
	led_control_pattern_param->target_component = mavlink_msg_led_control_pattern_param_get_target_component(msg);
	led_control_pattern_param->instance = mavlink_msg_led_control_pattern_param_get_instance(msg);
	led_control_pattern_param->param_type = mavlink_msg_led_control_pattern_param_get_param_type(msg);
#else
	memcpy(led_control_pattern_param, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_LED_CONTROL_PATTERN_PARAM_LEN);
#endif
}
