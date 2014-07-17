// MESSAGE XBEE_HARDRESET PACKING

#define MAVLINK_MSG_ID_XBEE_HARDRESET 201

typedef struct __mavlink_xbee_hardreset_t
{
 uint8_t test; ///< test data
} mavlink_xbee_hardreset_t;

#define MAVLINK_MSG_ID_XBEE_HARDRESET_LEN 1
#define MAVLINK_MSG_ID_201_LEN 1

#define MAVLINK_MSG_ID_XBEE_HARDRESET_CRC 52
#define MAVLINK_MSG_ID_201_CRC 52



#define MAVLINK_MESSAGE_INFO_XBEE_HARDRESET { \
	"XBEE_HARDRESET", \
	1, \
	{  { "test", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_xbee_hardreset_t, test) }, \
         } \
}


/**
 * @brief Pack a xbee_hardreset message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param test test data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xbee_hardreset_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t test)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XBEE_HARDRESET_LEN];
	_mav_put_uint8_t(buf, 0, test);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN);
#else
	mavlink_xbee_hardreset_t packet;
	packet.test = test;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XBEE_HARDRESET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN, MAVLINK_MSG_ID_XBEE_HARDRESET_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN);
#endif
}

/**
 * @brief Pack a xbee_hardreset message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param test test data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xbee_hardreset_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t test)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XBEE_HARDRESET_LEN];
	_mav_put_uint8_t(buf, 0, test);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN);
#else
	mavlink_xbee_hardreset_t packet;
	packet.test = test;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XBEE_HARDRESET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN, MAVLINK_MSG_ID_XBEE_HARDRESET_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN);
#endif
}

/**
 * @brief Encode a xbee_hardreset struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param xbee_hardreset C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xbee_hardreset_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_xbee_hardreset_t* xbee_hardreset)
{
	return mavlink_msg_xbee_hardreset_pack(system_id, component_id, msg, xbee_hardreset->test);
}

/**
 * @brief Encode a xbee_hardreset struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param xbee_hardreset C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xbee_hardreset_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_xbee_hardreset_t* xbee_hardreset)
{
	return mavlink_msg_xbee_hardreset_pack_chan(system_id, component_id, chan, msg, xbee_hardreset->test);
}

/**
 * @brief Send a xbee_hardreset message
 * @param chan MAVLink channel to send the message
 *
 * @param test test data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_xbee_hardreset_send(mavlink_channel_t chan, uint8_t test)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XBEE_HARDRESET_LEN];
	_mav_put_uint8_t(buf, 0, test);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XBEE_HARDRESET, buf, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN, MAVLINK_MSG_ID_XBEE_HARDRESET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XBEE_HARDRESET, buf, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN);
#endif
#else
	mavlink_xbee_hardreset_t packet;
	packet.test = test;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XBEE_HARDRESET, (const char *)&packet, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN, MAVLINK_MSG_ID_XBEE_HARDRESET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XBEE_HARDRESET, (const char *)&packet, MAVLINK_MSG_ID_XBEE_HARDRESET_LEN);
#endif
#endif
}

#endif

// MESSAGE XBEE_HARDRESET UNPACKING


/**
 * @brief Get field test from xbee_hardreset message
 *
 * @return test data
 */
static inline uint8_t mavlink_msg_xbee_hardreset_get_test(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a xbee_hardreset message into a struct
 *
 * @param msg The message to decode
 * @param xbee_hardreset C-struct to decode the message contents into
 */
static inline void mavlink_msg_xbee_hardreset_decode(const mavlink_message_t* msg, mavlink_xbee_hardreset_t* xbee_hardreset)
{
#if MAVLINK_NEED_BYTE_SWAP
	xbee_hardreset->test = mavlink_msg_xbee_hardreset_get_test(msg);
#else
	memcpy(xbee_hardreset, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_XBEE_HARDRESET_LEN);
#endif
}
