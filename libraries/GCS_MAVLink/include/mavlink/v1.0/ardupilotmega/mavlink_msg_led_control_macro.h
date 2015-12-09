// MESSAGE LED_CONTROL_MACRO PACKING

#define MAVLINK_MSG_ID_LED_CONTROL_MACRO 188

typedef struct __mavlink_led_control_macro_t
{
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t instance; /*< Instance (see LED_POSITIONS)*/
 uint8_t macro; /*< Pattern (see LED_CONTROL_MACROS)*/
} mavlink_led_control_macro_t;

#define MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN 4
#define MAVLINK_MSG_ID_188_LEN 4

#define MAVLINK_MSG_ID_LED_CONTROL_MACRO_CRC 39
#define MAVLINK_MSG_ID_188_CRC 39



#define MAVLINK_MESSAGE_INFO_LED_CONTROL_MACRO { \
	"LED_CONTROL_MACRO", \
	4, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_led_control_macro_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_led_control_macro_t, target_component) }, \
         { "instance", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_led_control_macro_t, instance) }, \
         { "macro", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_led_control_macro_t, macro) }, \
         } \
}


/**
 * @brief Pack a led_control_macro message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param instance Instance (see LED_POSITIONS)
 * @param macro Pattern (see LED_CONTROL_MACROS)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_led_control_macro_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t macro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, instance);
	_mav_put_uint8_t(buf, 3, macro);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#else
	mavlink_led_control_macro_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.instance = instance;
	packet.macro = macro;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LED_CONTROL_MACRO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN, MAVLINK_MSG_ID_LED_CONTROL_MACRO_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#endif
}

/**
 * @brief Pack a led_control_macro message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param instance Instance (see LED_POSITIONS)
 * @param macro Pattern (see LED_CONTROL_MACROS)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_led_control_macro_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t instance,uint8_t macro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, instance);
	_mav_put_uint8_t(buf, 3, macro);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#else
	mavlink_led_control_macro_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.instance = instance;
	packet.macro = macro;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LED_CONTROL_MACRO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN, MAVLINK_MSG_ID_LED_CONTROL_MACRO_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#endif
}

/**
 * @brief Encode a led_control_macro struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param led_control_macro C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_led_control_macro_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_led_control_macro_t* led_control_macro)
{
	return mavlink_msg_led_control_macro_pack(system_id, component_id, msg, led_control_macro->target_system, led_control_macro->target_component, led_control_macro->instance, led_control_macro->macro);
}

/**
 * @brief Encode a led_control_macro struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param led_control_macro C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_led_control_macro_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_led_control_macro_t* led_control_macro)
{
	return mavlink_msg_led_control_macro_pack_chan(system_id, component_id, chan, msg, led_control_macro->target_system, led_control_macro->target_component, led_control_macro->instance, led_control_macro->macro);
}

/**
 * @brief Send a led_control_macro message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param instance Instance (see LED_POSITIONS)
 * @param macro Pattern (see LED_CONTROL_MACROS)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_led_control_macro_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t macro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, instance);
	_mav_put_uint8_t(buf, 3, macro);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_MACRO, buf, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN, MAVLINK_MSG_ID_LED_CONTROL_MACRO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_MACRO, buf, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#endif
#else
	mavlink_led_control_macro_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.instance = instance;
	packet.macro = macro;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_MACRO, (const char *)&packet, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN, MAVLINK_MSG_ID_LED_CONTROL_MACRO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_MACRO, (const char *)&packet, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_led_control_macro_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t macro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, instance);
	_mav_put_uint8_t(buf, 3, macro);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_MACRO, buf, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN, MAVLINK_MSG_ID_LED_CONTROL_MACRO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_MACRO, buf, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#endif
#else
	mavlink_led_control_macro_t *packet = (mavlink_led_control_macro_t *)msgbuf;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->instance = instance;
	packet->macro = macro;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_MACRO, (const char *)packet, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN, MAVLINK_MSG_ID_LED_CONTROL_MACRO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_MACRO, (const char *)packet, MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE LED_CONTROL_MACRO UNPACKING


/**
 * @brief Get field target_system from led_control_macro message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_led_control_macro_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from led_control_macro message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_led_control_macro_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field instance from led_control_macro message
 *
 * @return Instance (see LED_POSITIONS)
 */
static inline uint8_t mavlink_msg_led_control_macro_get_instance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field macro from led_control_macro message
 *
 * @return Pattern (see LED_CONTROL_MACROS)
 */
static inline uint8_t mavlink_msg_led_control_macro_get_macro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a led_control_macro message into a struct
 *
 * @param msg The message to decode
 * @param led_control_macro C-struct to decode the message contents into
 */
static inline void mavlink_msg_led_control_macro_decode(const mavlink_message_t* msg, mavlink_led_control_macro_t* led_control_macro)
{
#if MAVLINK_NEED_BYTE_SWAP
	led_control_macro->target_system = mavlink_msg_led_control_macro_get_target_system(msg);
	led_control_macro->target_component = mavlink_msg_led_control_macro_get_target_component(msg);
	led_control_macro->instance = mavlink_msg_led_control_macro_get_instance(msg);
	led_control_macro->macro = mavlink_msg_led_control_macro_get_macro(msg);
#else
	memcpy(led_control_macro, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_LED_CONTROL_MACRO_LEN);
#endif
}
