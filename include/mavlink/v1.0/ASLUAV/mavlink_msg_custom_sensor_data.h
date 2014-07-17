// MESSAGE CUSTOM_SENSOR_DATA PACKING

#define MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA 202

typedef struct __mavlink_custom_sensor_data_t
{
 uint64_t mppt_timestamp; ///<  MPPT last timestamp 
 float dbaro_pres_pa; ///<  Differential pressure, already temp. comp.
 float dbaro_velo_ms; ///<  Velocity calculation from dpressure sensor	in m/sec
 float amb_temp_celsius; ///<  Ambient temperature in degrees celsius
 float adc121_vspb_volt; ///<  Power board voltage sensor reading in volts
 float adc121_cspb_amp; ///<  Power board current sensor reading in amps
 float adc121_cs1_amp; ///<  Board current sensor 1 reading in amps
 float adc121_cs2_amp; ///<  Board current sensor 2 reading in amps
 float mppt1_volt; ///<  MPPT1 voltage 
 float mppt1_amp; ///<  MPPT1 current 
 float mppt2_volt; ///<  MPPT2 voltage 
 float mppt2_amp; ///<  MPPT2 current 
 float mppt3_volt; ///<  MPPT3 voltage 
 float mppt3_amp; ///<  MPPT3 current 
 uint16_t mppt1_pwm; ///<  MPPT1 pwm 
 uint16_t mppt2_pwm; ///<  MPPT2 pwm 
 uint16_t mppt3_pwm; ///<  MPPT3 pwm 
 uint8_t mppt1_status; ///<  MPPT1 status 
 uint8_t mppt2_status; ///<  MPPT2 status 
 uint8_t mppt3_status; ///<  MPPT3 status 
} mavlink_custom_sensor_data_t;

#define MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN 69
#define MAVLINK_MSG_ID_202_LEN 69

#define MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_CRC 83
#define MAVLINK_MSG_ID_202_CRC 83



#define MAVLINK_MESSAGE_INFO_CUSTOM_SENSOR_DATA { \
	"CUSTOM_SENSOR_DATA", \
	20, \
	{  { "mppt_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_custom_sensor_data_t, mppt_timestamp) }, \
         { "dbaro_pres_pa", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_custom_sensor_data_t, dbaro_pres_pa) }, \
         { "dbaro_velo_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_custom_sensor_data_t, dbaro_velo_ms) }, \
         { "amb_temp_celsius", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_custom_sensor_data_t, amb_temp_celsius) }, \
         { "adc121_vspb_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_custom_sensor_data_t, adc121_vspb_volt) }, \
         { "adc121_cspb_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_custom_sensor_data_t, adc121_cspb_amp) }, \
         { "adc121_cs1_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_custom_sensor_data_t, adc121_cs1_amp) }, \
         { "adc121_cs2_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_custom_sensor_data_t, adc121_cs2_amp) }, \
         { "mppt1_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_custom_sensor_data_t, mppt1_volt) }, \
         { "mppt1_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_custom_sensor_data_t, mppt1_amp) }, \
         { "mppt2_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_custom_sensor_data_t, mppt2_volt) }, \
         { "mppt2_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_custom_sensor_data_t, mppt2_amp) }, \
         { "mppt3_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_custom_sensor_data_t, mppt3_volt) }, \
         { "mppt3_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_custom_sensor_data_t, mppt3_amp) }, \
         { "mppt1_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_custom_sensor_data_t, mppt1_pwm) }, \
         { "mppt2_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 62, offsetof(mavlink_custom_sensor_data_t, mppt2_pwm) }, \
         { "mppt3_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 64, offsetof(mavlink_custom_sensor_data_t, mppt3_pwm) }, \
         { "mppt1_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_custom_sensor_data_t, mppt1_status) }, \
         { "mppt2_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 67, offsetof(mavlink_custom_sensor_data_t, mppt2_status) }, \
         { "mppt3_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_custom_sensor_data_t, mppt3_status) }, \
         } \
}


/**
 * @brief Pack a custom_sensor_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param dbaro_pres_pa  Differential pressure, already temp. comp.
 * @param dbaro_velo_ms  Velocity calculation from dpressure sensor	in m/sec
 * @param amb_temp_celsius  Ambient temperature in degrees celsius
 * @param adc121_vspb_volt  Power board voltage sensor reading in volts
 * @param adc121_cspb_amp  Power board current sensor reading in amps
 * @param adc121_cs1_amp  Board current sensor 1 reading in amps
 * @param adc121_cs2_amp  Board current sensor 2 reading in amps
 * @param mppt_timestamp  MPPT last timestamp 
 * @param mppt1_volt  MPPT1 voltage 
 * @param mppt1_amp  MPPT1 current 
 * @param mppt1_pwm  MPPT1 pwm 
 * @param mppt1_status  MPPT1 status 
 * @param mppt2_volt  MPPT2 voltage 
 * @param mppt2_amp  MPPT2 current 
 * @param mppt2_pwm  MPPT2 pwm 
 * @param mppt2_status  MPPT2 status 
 * @param mppt3_volt  MPPT3 voltage 
 * @param mppt3_amp  MPPT3 current 
 * @param mppt3_pwm  MPPT3 pwm 
 * @param mppt3_status  MPPT3 status 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_custom_sensor_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float dbaro_pres_pa, float dbaro_velo_ms, float amb_temp_celsius, float adc121_vspb_volt, float adc121_cspb_amp, float adc121_cs1_amp, float adc121_cs2_amp, uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN];
	_mav_put_uint64_t(buf, 0, mppt_timestamp);
	_mav_put_float(buf, 8, dbaro_pres_pa);
	_mav_put_float(buf, 12, dbaro_velo_ms);
	_mav_put_float(buf, 16, amb_temp_celsius);
	_mav_put_float(buf, 20, adc121_vspb_volt);
	_mav_put_float(buf, 24, adc121_cspb_amp);
	_mav_put_float(buf, 28, adc121_cs1_amp);
	_mav_put_float(buf, 32, adc121_cs2_amp);
	_mav_put_float(buf, 36, mppt1_volt);
	_mav_put_float(buf, 40, mppt1_amp);
	_mav_put_float(buf, 44, mppt2_volt);
	_mav_put_float(buf, 48, mppt2_amp);
	_mav_put_float(buf, 52, mppt3_volt);
	_mav_put_float(buf, 56, mppt3_amp);
	_mav_put_uint16_t(buf, 60, mppt1_pwm);
	_mav_put_uint16_t(buf, 62, mppt2_pwm);
	_mav_put_uint16_t(buf, 64, mppt3_pwm);
	_mav_put_uint8_t(buf, 66, mppt1_status);
	_mav_put_uint8_t(buf, 67, mppt2_status);
	_mav_put_uint8_t(buf, 68, mppt3_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN);
#else
	mavlink_custom_sensor_data_t packet;
	packet.mppt_timestamp = mppt_timestamp;
	packet.dbaro_pres_pa = dbaro_pres_pa;
	packet.dbaro_velo_ms = dbaro_velo_ms;
	packet.amb_temp_celsius = amb_temp_celsius;
	packet.adc121_vspb_volt = adc121_vspb_volt;
	packet.adc121_cspb_amp = adc121_cspb_amp;
	packet.adc121_cs1_amp = adc121_cs1_amp;
	packet.adc121_cs2_amp = adc121_cs2_amp;
	packet.mppt1_volt = mppt1_volt;
	packet.mppt1_amp = mppt1_amp;
	packet.mppt2_volt = mppt2_volt;
	packet.mppt2_amp = mppt2_amp;
	packet.mppt3_volt = mppt3_volt;
	packet.mppt3_amp = mppt3_amp;
	packet.mppt1_pwm = mppt1_pwm;
	packet.mppt2_pwm = mppt2_pwm;
	packet.mppt3_pwm = mppt3_pwm;
	packet.mppt1_status = mppt1_status;
	packet.mppt2_status = mppt2_status;
	packet.mppt3_status = mppt3_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN);
#endif
}

/**
 * @brief Pack a custom_sensor_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dbaro_pres_pa  Differential pressure, already temp. comp.
 * @param dbaro_velo_ms  Velocity calculation from dpressure sensor	in m/sec
 * @param amb_temp_celsius  Ambient temperature in degrees celsius
 * @param adc121_vspb_volt  Power board voltage sensor reading in volts
 * @param adc121_cspb_amp  Power board current sensor reading in amps
 * @param adc121_cs1_amp  Board current sensor 1 reading in amps
 * @param adc121_cs2_amp  Board current sensor 2 reading in amps
 * @param mppt_timestamp  MPPT last timestamp 
 * @param mppt1_volt  MPPT1 voltage 
 * @param mppt1_amp  MPPT1 current 
 * @param mppt1_pwm  MPPT1 pwm 
 * @param mppt1_status  MPPT1 status 
 * @param mppt2_volt  MPPT2 voltage 
 * @param mppt2_amp  MPPT2 current 
 * @param mppt2_pwm  MPPT2 pwm 
 * @param mppt2_status  MPPT2 status 
 * @param mppt3_volt  MPPT3 voltage 
 * @param mppt3_amp  MPPT3 current 
 * @param mppt3_pwm  MPPT3 pwm 
 * @param mppt3_status  MPPT3 status 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_custom_sensor_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float dbaro_pres_pa,float dbaro_velo_ms,float amb_temp_celsius,float adc121_vspb_volt,float adc121_cspb_amp,float adc121_cs1_amp,float adc121_cs2_amp,uint64_t mppt_timestamp,float mppt1_volt,float mppt1_amp,uint16_t mppt1_pwm,uint8_t mppt1_status,float mppt2_volt,float mppt2_amp,uint16_t mppt2_pwm,uint8_t mppt2_status,float mppt3_volt,float mppt3_amp,uint16_t mppt3_pwm,uint8_t mppt3_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN];
	_mav_put_uint64_t(buf, 0, mppt_timestamp);
	_mav_put_float(buf, 8, dbaro_pres_pa);
	_mav_put_float(buf, 12, dbaro_velo_ms);
	_mav_put_float(buf, 16, amb_temp_celsius);
	_mav_put_float(buf, 20, adc121_vspb_volt);
	_mav_put_float(buf, 24, adc121_cspb_amp);
	_mav_put_float(buf, 28, adc121_cs1_amp);
	_mav_put_float(buf, 32, adc121_cs2_amp);
	_mav_put_float(buf, 36, mppt1_volt);
	_mav_put_float(buf, 40, mppt1_amp);
	_mav_put_float(buf, 44, mppt2_volt);
	_mav_put_float(buf, 48, mppt2_amp);
	_mav_put_float(buf, 52, mppt3_volt);
	_mav_put_float(buf, 56, mppt3_amp);
	_mav_put_uint16_t(buf, 60, mppt1_pwm);
	_mav_put_uint16_t(buf, 62, mppt2_pwm);
	_mav_put_uint16_t(buf, 64, mppt3_pwm);
	_mav_put_uint8_t(buf, 66, mppt1_status);
	_mav_put_uint8_t(buf, 67, mppt2_status);
	_mav_put_uint8_t(buf, 68, mppt3_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN);
#else
	mavlink_custom_sensor_data_t packet;
	packet.mppt_timestamp = mppt_timestamp;
	packet.dbaro_pres_pa = dbaro_pres_pa;
	packet.dbaro_velo_ms = dbaro_velo_ms;
	packet.amb_temp_celsius = amb_temp_celsius;
	packet.adc121_vspb_volt = adc121_vspb_volt;
	packet.adc121_cspb_amp = adc121_cspb_amp;
	packet.adc121_cs1_amp = adc121_cs1_amp;
	packet.adc121_cs2_amp = adc121_cs2_amp;
	packet.mppt1_volt = mppt1_volt;
	packet.mppt1_amp = mppt1_amp;
	packet.mppt2_volt = mppt2_volt;
	packet.mppt2_amp = mppt2_amp;
	packet.mppt3_volt = mppt3_volt;
	packet.mppt3_amp = mppt3_amp;
	packet.mppt1_pwm = mppt1_pwm;
	packet.mppt2_pwm = mppt2_pwm;
	packet.mppt3_pwm = mppt3_pwm;
	packet.mppt1_status = mppt1_status;
	packet.mppt2_status = mppt2_status;
	packet.mppt3_status = mppt3_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN);
#endif
}

/**
 * @brief Encode a custom_sensor_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param custom_sensor_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_custom_sensor_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_custom_sensor_data_t* custom_sensor_data)
{
	return mavlink_msg_custom_sensor_data_pack(system_id, component_id, msg, custom_sensor_data->dbaro_pres_pa, custom_sensor_data->dbaro_velo_ms, custom_sensor_data->amb_temp_celsius, custom_sensor_data->adc121_vspb_volt, custom_sensor_data->adc121_cspb_amp, custom_sensor_data->adc121_cs1_amp, custom_sensor_data->adc121_cs2_amp, custom_sensor_data->mppt_timestamp, custom_sensor_data->mppt1_volt, custom_sensor_data->mppt1_amp, custom_sensor_data->mppt1_pwm, custom_sensor_data->mppt1_status, custom_sensor_data->mppt2_volt, custom_sensor_data->mppt2_amp, custom_sensor_data->mppt2_pwm, custom_sensor_data->mppt2_status, custom_sensor_data->mppt3_volt, custom_sensor_data->mppt3_amp, custom_sensor_data->mppt3_pwm, custom_sensor_data->mppt3_status);
}

/**
 * @brief Encode a custom_sensor_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param custom_sensor_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_custom_sensor_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_custom_sensor_data_t* custom_sensor_data)
{
	return mavlink_msg_custom_sensor_data_pack_chan(system_id, component_id, chan, msg, custom_sensor_data->dbaro_pres_pa, custom_sensor_data->dbaro_velo_ms, custom_sensor_data->amb_temp_celsius, custom_sensor_data->adc121_vspb_volt, custom_sensor_data->adc121_cspb_amp, custom_sensor_data->adc121_cs1_amp, custom_sensor_data->adc121_cs2_amp, custom_sensor_data->mppt_timestamp, custom_sensor_data->mppt1_volt, custom_sensor_data->mppt1_amp, custom_sensor_data->mppt1_pwm, custom_sensor_data->mppt1_status, custom_sensor_data->mppt2_volt, custom_sensor_data->mppt2_amp, custom_sensor_data->mppt2_pwm, custom_sensor_data->mppt2_status, custom_sensor_data->mppt3_volt, custom_sensor_data->mppt3_amp, custom_sensor_data->mppt3_pwm, custom_sensor_data->mppt3_status);
}

/**
 * @brief Send a custom_sensor_data message
 * @param chan MAVLink channel to send the message
 *
 * @param dbaro_pres_pa  Differential pressure, already temp. comp.
 * @param dbaro_velo_ms  Velocity calculation from dpressure sensor	in m/sec
 * @param amb_temp_celsius  Ambient temperature in degrees celsius
 * @param adc121_vspb_volt  Power board voltage sensor reading in volts
 * @param adc121_cspb_amp  Power board current sensor reading in amps
 * @param adc121_cs1_amp  Board current sensor 1 reading in amps
 * @param adc121_cs2_amp  Board current sensor 2 reading in amps
 * @param mppt_timestamp  MPPT last timestamp 
 * @param mppt1_volt  MPPT1 voltage 
 * @param mppt1_amp  MPPT1 current 
 * @param mppt1_pwm  MPPT1 pwm 
 * @param mppt1_status  MPPT1 status 
 * @param mppt2_volt  MPPT2 voltage 
 * @param mppt2_amp  MPPT2 current 
 * @param mppt2_pwm  MPPT2 pwm 
 * @param mppt2_status  MPPT2 status 
 * @param mppt3_volt  MPPT3 voltage 
 * @param mppt3_amp  MPPT3 current 
 * @param mppt3_pwm  MPPT3 pwm 
 * @param mppt3_status  MPPT3 status 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_custom_sensor_data_send(mavlink_channel_t chan, float dbaro_pres_pa, float dbaro_velo_ms, float amb_temp_celsius, float adc121_vspb_volt, float adc121_cspb_amp, float adc121_cs1_amp, float adc121_cs2_amp, uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN];
	_mav_put_uint64_t(buf, 0, mppt_timestamp);
	_mav_put_float(buf, 8, dbaro_pres_pa);
	_mav_put_float(buf, 12, dbaro_velo_ms);
	_mav_put_float(buf, 16, amb_temp_celsius);
	_mav_put_float(buf, 20, adc121_vspb_volt);
	_mav_put_float(buf, 24, adc121_cspb_amp);
	_mav_put_float(buf, 28, adc121_cs1_amp);
	_mav_put_float(buf, 32, adc121_cs2_amp);
	_mav_put_float(buf, 36, mppt1_volt);
	_mav_put_float(buf, 40, mppt1_amp);
	_mav_put_float(buf, 44, mppt2_volt);
	_mav_put_float(buf, 48, mppt2_amp);
	_mav_put_float(buf, 52, mppt3_volt);
	_mav_put_float(buf, 56, mppt3_amp);
	_mav_put_uint16_t(buf, 60, mppt1_pwm);
	_mav_put_uint16_t(buf, 62, mppt2_pwm);
	_mav_put_uint16_t(buf, 64, mppt3_pwm);
	_mav_put_uint8_t(buf, 66, mppt1_status);
	_mav_put_uint8_t(buf, 67, mppt2_status);
	_mav_put_uint8_t(buf, 68, mppt3_status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA, buf, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA, buf, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN);
#endif
#else
	mavlink_custom_sensor_data_t packet;
	packet.mppt_timestamp = mppt_timestamp;
	packet.dbaro_pres_pa = dbaro_pres_pa;
	packet.dbaro_velo_ms = dbaro_velo_ms;
	packet.amb_temp_celsius = amb_temp_celsius;
	packet.adc121_vspb_volt = adc121_vspb_volt;
	packet.adc121_cspb_amp = adc121_cspb_amp;
	packet.adc121_cs1_amp = adc121_cs1_amp;
	packet.adc121_cs2_amp = adc121_cs2_amp;
	packet.mppt1_volt = mppt1_volt;
	packet.mppt1_amp = mppt1_amp;
	packet.mppt2_volt = mppt2_volt;
	packet.mppt2_amp = mppt2_amp;
	packet.mppt3_volt = mppt3_volt;
	packet.mppt3_amp = mppt3_amp;
	packet.mppt1_pwm = mppt1_pwm;
	packet.mppt2_pwm = mppt2_pwm;
	packet.mppt3_pwm = mppt3_pwm;
	packet.mppt1_status = mppt1_status;
	packet.mppt2_status = mppt2_status;
	packet.mppt3_status = mppt3_status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA, (const char *)&packet, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA, (const char *)&packet, MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN);
#endif
#endif
}

#endif

// MESSAGE CUSTOM_SENSOR_DATA UNPACKING


/**
 * @brief Get field dbaro_pres_pa from custom_sensor_data message
 *
 * @return  Differential pressure, already temp. comp.
 */
static inline float mavlink_msg_custom_sensor_data_get_dbaro_pres_pa(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field dbaro_velo_ms from custom_sensor_data message
 *
 * @return  Velocity calculation from dpressure sensor	in m/sec
 */
static inline float mavlink_msg_custom_sensor_data_get_dbaro_velo_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field amb_temp_celsius from custom_sensor_data message
 *
 * @return  Ambient temperature in degrees celsius
 */
static inline float mavlink_msg_custom_sensor_data_get_amb_temp_celsius(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field adc121_vspb_volt from custom_sensor_data message
 *
 * @return  Power board voltage sensor reading in volts
 */
static inline float mavlink_msg_custom_sensor_data_get_adc121_vspb_volt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field adc121_cspb_amp from custom_sensor_data message
 *
 * @return  Power board current sensor reading in amps
 */
static inline float mavlink_msg_custom_sensor_data_get_adc121_cspb_amp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field adc121_cs1_amp from custom_sensor_data message
 *
 * @return  Board current sensor 1 reading in amps
 */
static inline float mavlink_msg_custom_sensor_data_get_adc121_cs1_amp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field adc121_cs2_amp from custom_sensor_data message
 *
 * @return  Board current sensor 2 reading in amps
 */
static inline float mavlink_msg_custom_sensor_data_get_adc121_cs2_amp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field mppt_timestamp from custom_sensor_data message
 *
 * @return  MPPT last timestamp 
 */
static inline uint64_t mavlink_msg_custom_sensor_data_get_mppt_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field mppt1_volt from custom_sensor_data message
 *
 * @return  MPPT1 voltage 
 */
static inline float mavlink_msg_custom_sensor_data_get_mppt1_volt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field mppt1_amp from custom_sensor_data message
 *
 * @return  MPPT1 current 
 */
static inline float mavlink_msg_custom_sensor_data_get_mppt1_amp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field mppt1_pwm from custom_sensor_data message
 *
 * @return  MPPT1 pwm 
 */
static inline uint16_t mavlink_msg_custom_sensor_data_get_mppt1_pwm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  60);
}

/**
 * @brief Get field mppt1_status from custom_sensor_data message
 *
 * @return  MPPT1 status 
 */
static inline uint8_t mavlink_msg_custom_sensor_data_get_mppt1_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  66);
}

/**
 * @brief Get field mppt2_volt from custom_sensor_data message
 *
 * @return  MPPT2 voltage 
 */
static inline float mavlink_msg_custom_sensor_data_get_mppt2_volt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field mppt2_amp from custom_sensor_data message
 *
 * @return  MPPT2 current 
 */
static inline float mavlink_msg_custom_sensor_data_get_mppt2_amp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field mppt2_pwm from custom_sensor_data message
 *
 * @return  MPPT2 pwm 
 */
static inline uint16_t mavlink_msg_custom_sensor_data_get_mppt2_pwm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  62);
}

/**
 * @brief Get field mppt2_status from custom_sensor_data message
 *
 * @return  MPPT2 status 
 */
static inline uint8_t mavlink_msg_custom_sensor_data_get_mppt2_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  67);
}

/**
 * @brief Get field mppt3_volt from custom_sensor_data message
 *
 * @return  MPPT3 voltage 
 */
static inline float mavlink_msg_custom_sensor_data_get_mppt3_volt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field mppt3_amp from custom_sensor_data message
 *
 * @return  MPPT3 current 
 */
static inline float mavlink_msg_custom_sensor_data_get_mppt3_amp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field mppt3_pwm from custom_sensor_data message
 *
 * @return  MPPT3 pwm 
 */
static inline uint16_t mavlink_msg_custom_sensor_data_get_mppt3_pwm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  64);
}

/**
 * @brief Get field mppt3_status from custom_sensor_data message
 *
 * @return  MPPT3 status 
 */
static inline uint8_t mavlink_msg_custom_sensor_data_get_mppt3_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  68);
}

/**
 * @brief Decode a custom_sensor_data message into a struct
 *
 * @param msg The message to decode
 * @param custom_sensor_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_custom_sensor_data_decode(const mavlink_message_t* msg, mavlink_custom_sensor_data_t* custom_sensor_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	custom_sensor_data->mppt_timestamp = mavlink_msg_custom_sensor_data_get_mppt_timestamp(msg);
	custom_sensor_data->dbaro_pres_pa = mavlink_msg_custom_sensor_data_get_dbaro_pres_pa(msg);
	custom_sensor_data->dbaro_velo_ms = mavlink_msg_custom_sensor_data_get_dbaro_velo_ms(msg);
	custom_sensor_data->amb_temp_celsius = mavlink_msg_custom_sensor_data_get_amb_temp_celsius(msg);
	custom_sensor_data->adc121_vspb_volt = mavlink_msg_custom_sensor_data_get_adc121_vspb_volt(msg);
	custom_sensor_data->adc121_cspb_amp = mavlink_msg_custom_sensor_data_get_adc121_cspb_amp(msg);
	custom_sensor_data->adc121_cs1_amp = mavlink_msg_custom_sensor_data_get_adc121_cs1_amp(msg);
	custom_sensor_data->adc121_cs2_amp = mavlink_msg_custom_sensor_data_get_adc121_cs2_amp(msg);
	custom_sensor_data->mppt1_volt = mavlink_msg_custom_sensor_data_get_mppt1_volt(msg);
	custom_sensor_data->mppt1_amp = mavlink_msg_custom_sensor_data_get_mppt1_amp(msg);
	custom_sensor_data->mppt2_volt = mavlink_msg_custom_sensor_data_get_mppt2_volt(msg);
	custom_sensor_data->mppt2_amp = mavlink_msg_custom_sensor_data_get_mppt2_amp(msg);
	custom_sensor_data->mppt3_volt = mavlink_msg_custom_sensor_data_get_mppt3_volt(msg);
	custom_sensor_data->mppt3_amp = mavlink_msg_custom_sensor_data_get_mppt3_amp(msg);
	custom_sensor_data->mppt1_pwm = mavlink_msg_custom_sensor_data_get_mppt1_pwm(msg);
	custom_sensor_data->mppt2_pwm = mavlink_msg_custom_sensor_data_get_mppt2_pwm(msg);
	custom_sensor_data->mppt3_pwm = mavlink_msg_custom_sensor_data_get_mppt3_pwm(msg);
	custom_sensor_data->mppt1_status = mavlink_msg_custom_sensor_data_get_mppt1_status(msg);
	custom_sensor_data->mppt2_status = mavlink_msg_custom_sensor_data_get_mppt2_status(msg);
	custom_sensor_data->mppt3_status = mavlink_msg_custom_sensor_data_get_mppt3_status(msg);
#else
	memcpy(custom_sensor_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN);
#endif
}
