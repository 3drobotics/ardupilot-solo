// MESSAGE LED_CONTROL_PATTERN PACKING

#define MAVLINK_MSG_ID_LED_CONTROL_PATTERN 186

typedef struct __mavlink_led_control_pattern_t
{
 uint16_t period; /*< Pattern period (milliseconds)*/
 uint16_t phase_offset; /*< Pattern phase-offset (degrees)*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t instance; /*< Instance (see LED_POSITIONS)*/
 uint8_t pattern; /*< Pattern (see LED_CONTROL_PATTERNS)*/
 uint8_t bias_red; /*< Bias of the red LED brightness*/
 uint8_t bias_green; /*< Bias of the green LED brightness*/
 uint8_t bias_blue; /*< Bias of the blue LED brightness*/
 uint8_t amplitude_red; /*< Amplitude of the red LED brightness*/
 uint8_t amplitude_green; /*< Amplitude of the green LED brightness*/
 uint8_t amplitude_blue; /*< Amplitude of the blue LED brightness*/
 int8_t repeat; /*< Numer of pattern cycles to run*/
} mavlink_led_control_pattern_t;

#define MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN 15
#define MAVLINK_MSG_ID_186_LEN 15

#define MAVLINK_MSG_ID_LED_CONTROL_PATTERN_CRC 87
#define MAVLINK_MSG_ID_186_CRC 87



#define MAVLINK_MESSAGE_INFO_LED_CONTROL_PATTERN { \
	"LED_CONTROL_PATTERN", \
	13, \
	{  { "period", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_led_control_pattern_t, period) }, \
         { "phase_offset", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_led_control_pattern_t, phase_offset) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_led_control_pattern_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_led_control_pattern_t, target_component) }, \
         { "instance", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_led_control_pattern_t, instance) }, \
         { "pattern", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_led_control_pattern_t, pattern) }, \
         { "bias_red", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_led_control_pattern_t, bias_red) }, \
         { "bias_green", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_led_control_pattern_t, bias_green) }, \
         { "bias_blue", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_led_control_pattern_t, bias_blue) }, \
         { "amplitude_red", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_led_control_pattern_t, amplitude_red) }, \
         { "amplitude_green", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_led_control_pattern_t, amplitude_green) }, \
         { "amplitude_blue", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_led_control_pattern_t, amplitude_blue) }, \
         { "repeat", NULL, MAVLINK_TYPE_INT8_T, 0, 14, offsetof(mavlink_led_control_pattern_t, repeat) }, \
         } \
}


/**
 * @brief Pack a led_control_pattern message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param instance Instance (see LED_POSITIONS)
 * @param pattern Pattern (see LED_CONTROL_PATTERNS)
 * @param bias_red Bias of the red LED brightness
 * @param bias_green Bias of the green LED brightness
 * @param bias_blue Bias of the blue LED brightness
 * @param amplitude_red Amplitude of the red LED brightness
 * @param amplitude_green Amplitude of the green LED brightness
 * @param amplitude_blue Amplitude of the blue LED brightness
 * @param period Pattern period (milliseconds)
 * @param repeat Numer of pattern cycles to run
 * @param phase_offset Pattern phase-offset (degrees)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_led_control_pattern_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t pattern, uint8_t bias_red, uint8_t bias_green, uint8_t bias_blue, uint8_t amplitude_red, uint8_t amplitude_green, uint8_t amplitude_blue, uint16_t period, int8_t repeat, uint16_t phase_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN];
	_mav_put_uint16_t(buf, 0, period);
	_mav_put_uint16_t(buf, 2, phase_offset);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);
	_mav_put_uint8_t(buf, 6, instance);
	_mav_put_uint8_t(buf, 7, pattern);
	_mav_put_uint8_t(buf, 8, bias_red);
	_mav_put_uint8_t(buf, 9, bias_green);
	_mav_put_uint8_t(buf, 10, bias_blue);
	_mav_put_uint8_t(buf, 11, amplitude_red);
	_mav_put_uint8_t(buf, 12, amplitude_green);
	_mav_put_uint8_t(buf, 13, amplitude_blue);
	_mav_put_int8_t(buf, 14, repeat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#else
	mavlink_led_control_pattern_t packet;
	packet.period = period;
	packet.phase_offset = phase_offset;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.instance = instance;
	packet.pattern = pattern;
	packet.bias_red = bias_red;
	packet.bias_green = bias_green;
	packet.bias_blue = bias_blue;
	packet.amplitude_red = amplitude_red;
	packet.amplitude_green = amplitude_green;
	packet.amplitude_blue = amplitude_blue;
	packet.repeat = repeat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LED_CONTROL_PATTERN;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#endif
}

/**
 * @brief Pack a led_control_pattern message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param instance Instance (see LED_POSITIONS)
 * @param pattern Pattern (see LED_CONTROL_PATTERNS)
 * @param bias_red Bias of the red LED brightness
 * @param bias_green Bias of the green LED brightness
 * @param bias_blue Bias of the blue LED brightness
 * @param amplitude_red Amplitude of the red LED brightness
 * @param amplitude_green Amplitude of the green LED brightness
 * @param amplitude_blue Amplitude of the blue LED brightness
 * @param period Pattern period (milliseconds)
 * @param repeat Numer of pattern cycles to run
 * @param phase_offset Pattern phase-offset (degrees)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_led_control_pattern_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t instance,uint8_t pattern,uint8_t bias_red,uint8_t bias_green,uint8_t bias_blue,uint8_t amplitude_red,uint8_t amplitude_green,uint8_t amplitude_blue,uint16_t period,int8_t repeat,uint16_t phase_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN];
	_mav_put_uint16_t(buf, 0, period);
	_mav_put_uint16_t(buf, 2, phase_offset);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);
	_mav_put_uint8_t(buf, 6, instance);
	_mav_put_uint8_t(buf, 7, pattern);
	_mav_put_uint8_t(buf, 8, bias_red);
	_mav_put_uint8_t(buf, 9, bias_green);
	_mav_put_uint8_t(buf, 10, bias_blue);
	_mav_put_uint8_t(buf, 11, amplitude_red);
	_mav_put_uint8_t(buf, 12, amplitude_green);
	_mav_put_uint8_t(buf, 13, amplitude_blue);
	_mav_put_int8_t(buf, 14, repeat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#else
	mavlink_led_control_pattern_t packet;
	packet.period = period;
	packet.phase_offset = phase_offset;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.instance = instance;
	packet.pattern = pattern;
	packet.bias_red = bias_red;
	packet.bias_green = bias_green;
	packet.bias_blue = bias_blue;
	packet.amplitude_red = amplitude_red;
	packet.amplitude_green = amplitude_green;
	packet.amplitude_blue = amplitude_blue;
	packet.repeat = repeat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LED_CONTROL_PATTERN;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#endif
}

/**
 * @brief Encode a led_control_pattern struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param led_control_pattern C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_led_control_pattern_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_led_control_pattern_t* led_control_pattern)
{
	return mavlink_msg_led_control_pattern_pack(system_id, component_id, msg, led_control_pattern->target_system, led_control_pattern->target_component, led_control_pattern->instance, led_control_pattern->pattern, led_control_pattern->bias_red, led_control_pattern->bias_green, led_control_pattern->bias_blue, led_control_pattern->amplitude_red, led_control_pattern->amplitude_green, led_control_pattern->amplitude_blue, led_control_pattern->period, led_control_pattern->repeat, led_control_pattern->phase_offset);
}

/**
 * @brief Encode a led_control_pattern struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param led_control_pattern C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_led_control_pattern_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_led_control_pattern_t* led_control_pattern)
{
	return mavlink_msg_led_control_pattern_pack_chan(system_id, component_id, chan, msg, led_control_pattern->target_system, led_control_pattern->target_component, led_control_pattern->instance, led_control_pattern->pattern, led_control_pattern->bias_red, led_control_pattern->bias_green, led_control_pattern->bias_blue, led_control_pattern->amplitude_red, led_control_pattern->amplitude_green, led_control_pattern->amplitude_blue, led_control_pattern->period, led_control_pattern->repeat, led_control_pattern->phase_offset);
}

/**
 * @brief Send a led_control_pattern message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param instance Instance (see LED_POSITIONS)
 * @param pattern Pattern (see LED_CONTROL_PATTERNS)
 * @param bias_red Bias of the red LED brightness
 * @param bias_green Bias of the green LED brightness
 * @param bias_blue Bias of the blue LED brightness
 * @param amplitude_red Amplitude of the red LED brightness
 * @param amplitude_green Amplitude of the green LED brightness
 * @param amplitude_blue Amplitude of the blue LED brightness
 * @param period Pattern period (milliseconds)
 * @param repeat Numer of pattern cycles to run
 * @param phase_offset Pattern phase-offset (degrees)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_led_control_pattern_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t pattern, uint8_t bias_red, uint8_t bias_green, uint8_t bias_blue, uint8_t amplitude_red, uint8_t amplitude_green, uint8_t amplitude_blue, uint16_t period, int8_t repeat, uint16_t phase_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN];
	_mav_put_uint16_t(buf, 0, period);
	_mav_put_uint16_t(buf, 2, phase_offset);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);
	_mav_put_uint8_t(buf, 6, instance);
	_mav_put_uint8_t(buf, 7, pattern);
	_mav_put_uint8_t(buf, 8, bias_red);
	_mav_put_uint8_t(buf, 9, bias_green);
	_mav_put_uint8_t(buf, 10, bias_blue);
	_mav_put_uint8_t(buf, 11, amplitude_red);
	_mav_put_uint8_t(buf, 12, amplitude_green);
	_mav_put_uint8_t(buf, 13, amplitude_blue);
	_mav_put_int8_t(buf, 14, repeat);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN, buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN, buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#endif
#else
	mavlink_led_control_pattern_t packet;
	packet.period = period;
	packet.phase_offset = phase_offset;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.instance = instance;
	packet.pattern = pattern;
	packet.bias_red = bias_red;
	packet.bias_green = bias_green;
	packet.bias_blue = bias_blue;
	packet.amplitude_red = amplitude_red;
	packet.amplitude_green = amplitude_green;
	packet.amplitude_blue = amplitude_blue;
	packet.repeat = repeat;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN, (const char *)&packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN, (const char *)&packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_led_control_pattern_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t pattern, uint8_t bias_red, uint8_t bias_green, uint8_t bias_blue, uint8_t amplitude_red, uint8_t amplitude_green, uint8_t amplitude_blue, uint16_t period, int8_t repeat, uint16_t phase_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, period);
	_mav_put_uint16_t(buf, 2, phase_offset);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);
	_mav_put_uint8_t(buf, 6, instance);
	_mav_put_uint8_t(buf, 7, pattern);
	_mav_put_uint8_t(buf, 8, bias_red);
	_mav_put_uint8_t(buf, 9, bias_green);
	_mav_put_uint8_t(buf, 10, bias_blue);
	_mav_put_uint8_t(buf, 11, amplitude_red);
	_mav_put_uint8_t(buf, 12, amplitude_green);
	_mav_put_uint8_t(buf, 13, amplitude_blue);
	_mav_put_int8_t(buf, 14, repeat);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN, buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN, buf, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#endif
#else
	mavlink_led_control_pattern_t *packet = (mavlink_led_control_pattern_t *)msgbuf;
	packet->period = period;
	packet->phase_offset = phase_offset;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->instance = instance;
	packet->pattern = pattern;
	packet->bias_red = bias_red;
	packet->bias_green = bias_green;
	packet->bias_blue = bias_blue;
	packet->amplitude_red = amplitude_red;
	packet->amplitude_green = amplitude_green;
	packet->amplitude_blue = amplitude_blue;
	packet->repeat = repeat;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN, (const char *)packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL_PATTERN, (const char *)packet, MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE LED_CONTROL_PATTERN UNPACKING


/**
 * @brief Get field target_system from led_control_pattern message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_led_control_pattern_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from led_control_pattern message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_led_control_pattern_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field instance from led_control_pattern message
 *
 * @return Instance (see LED_POSITIONS)
 */
static inline uint8_t mavlink_msg_led_control_pattern_get_instance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field pattern from led_control_pattern message
 *
 * @return Pattern (see LED_CONTROL_PATTERNS)
 */
static inline uint8_t mavlink_msg_led_control_pattern_get_pattern(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field bias_red from led_control_pattern message
 *
 * @return Bias of the red LED brightness
 */
static inline uint8_t mavlink_msg_led_control_pattern_get_bias_red(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field bias_green from led_control_pattern message
 *
 * @return Bias of the green LED brightness
 */
static inline uint8_t mavlink_msg_led_control_pattern_get_bias_green(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field bias_blue from led_control_pattern message
 *
 * @return Bias of the blue LED brightness
 */
static inline uint8_t mavlink_msg_led_control_pattern_get_bias_blue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field amplitude_red from led_control_pattern message
 *
 * @return Amplitude of the red LED brightness
 */
static inline uint8_t mavlink_msg_led_control_pattern_get_amplitude_red(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field amplitude_green from led_control_pattern message
 *
 * @return Amplitude of the green LED brightness
 */
static inline uint8_t mavlink_msg_led_control_pattern_get_amplitude_green(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field amplitude_blue from led_control_pattern message
 *
 * @return Amplitude of the blue LED brightness
 */
static inline uint8_t mavlink_msg_led_control_pattern_get_amplitude_blue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field period from led_control_pattern message
 *
 * @return Pattern period (milliseconds)
 */
static inline uint16_t mavlink_msg_led_control_pattern_get_period(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field repeat from led_control_pattern message
 *
 * @return Numer of pattern cycles to run
 */
static inline int8_t mavlink_msg_led_control_pattern_get_repeat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  14);
}

/**
 * @brief Get field phase_offset from led_control_pattern message
 *
 * @return Pattern phase-offset (degrees)
 */
static inline uint16_t mavlink_msg_led_control_pattern_get_phase_offset(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a led_control_pattern message into a struct
 *
 * @param msg The message to decode
 * @param led_control_pattern C-struct to decode the message contents into
 */
static inline void mavlink_msg_led_control_pattern_decode(const mavlink_message_t* msg, mavlink_led_control_pattern_t* led_control_pattern)
{
#if MAVLINK_NEED_BYTE_SWAP
	led_control_pattern->period = mavlink_msg_led_control_pattern_get_period(msg);
	led_control_pattern->phase_offset = mavlink_msg_led_control_pattern_get_phase_offset(msg);
	led_control_pattern->target_system = mavlink_msg_led_control_pattern_get_target_system(msg);
	led_control_pattern->target_component = mavlink_msg_led_control_pattern_get_target_component(msg);
	led_control_pattern->instance = mavlink_msg_led_control_pattern_get_instance(msg);
	led_control_pattern->pattern = mavlink_msg_led_control_pattern_get_pattern(msg);
	led_control_pattern->bias_red = mavlink_msg_led_control_pattern_get_bias_red(msg);
	led_control_pattern->bias_green = mavlink_msg_led_control_pattern_get_bias_green(msg);
	led_control_pattern->bias_blue = mavlink_msg_led_control_pattern_get_bias_blue(msg);
	led_control_pattern->amplitude_red = mavlink_msg_led_control_pattern_get_amplitude_red(msg);
	led_control_pattern->amplitude_green = mavlink_msg_led_control_pattern_get_amplitude_green(msg);
	led_control_pattern->amplitude_blue = mavlink_msg_led_control_pattern_get_amplitude_blue(msg);
	led_control_pattern->repeat = mavlink_msg_led_control_pattern_get_repeat(msg);
#else
	memcpy(led_control_pattern, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_LED_CONTROL_PATTERN_LEN);
#endif
}
