/** @file
 *	@brief MAVLink comm protocol testsuite generated from ASLUAV.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef ASLUAV_TESTSUITE_H
#define ASLUAV_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_pixhawk(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ASLUAV(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_pixhawk(system_id, component_id, last_msg);
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_ASLUAV(system_id, component_id, last_msg);
}
#endif

#include "../pixhawk/testsuite.h"
#include "../common/testsuite.h"


static void mavlink_test_xbee_hardreset(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_xbee_hardreset_t packet_in = {
		5,
	};
	mavlink_xbee_hardreset_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.test = packet_in.test;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_xbee_hardreset_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_xbee_hardreset_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_xbee_hardreset_pack(system_id, component_id, &msg , packet1.test );
	mavlink_msg_xbee_hardreset_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_xbee_hardreset_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.test );
	mavlink_msg_xbee_hardreset_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_xbee_hardreset_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_xbee_hardreset_send(MAVLINK_COMM_1 , packet1.test );
	mavlink_msg_xbee_hardreset_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_custom_sensor_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_custom_sensor_data_t packet_in = {
		93372036854775807ULL,
	}73.0,
	}101.0,
	}129.0,
	}157.0,
	}185.0,
	}213.0,
	}241.0,
	}269.0,
	}297.0,
	}325.0,
	}353.0,
	}381.0,
	}409.0,
	}20355,
	}20459,
	}20563,
	}75,
	}142,
	}209,
	};
	mavlink_custom_sensor_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.mppt_timestamp = packet_in.mppt_timestamp;
        	packet1.dbaro_pres_pa = packet_in.dbaro_pres_pa;
        	packet1.dbaro_velo_ms = packet_in.dbaro_velo_ms;
        	packet1.amb_temp_celsius = packet_in.amb_temp_celsius;
        	packet1.adc121_vspb_volt = packet_in.adc121_vspb_volt;
        	packet1.adc121_cspb_amp = packet_in.adc121_cspb_amp;
        	packet1.adc121_cs1_amp = packet_in.adc121_cs1_amp;
        	packet1.adc121_cs2_amp = packet_in.adc121_cs2_amp;
        	packet1.mppt1_volt = packet_in.mppt1_volt;
        	packet1.mppt1_amp = packet_in.mppt1_amp;
        	packet1.mppt2_volt = packet_in.mppt2_volt;
        	packet1.mppt2_amp = packet_in.mppt2_amp;
        	packet1.mppt3_volt = packet_in.mppt3_volt;
        	packet1.mppt3_amp = packet_in.mppt3_amp;
        	packet1.mppt1_pwm = packet_in.mppt1_pwm;
        	packet1.mppt2_pwm = packet_in.mppt2_pwm;
        	packet1.mppt3_pwm = packet_in.mppt3_pwm;
        	packet1.mppt1_status = packet_in.mppt1_status;
        	packet1.mppt2_status = packet_in.mppt2_status;
        	packet1.mppt3_status = packet_in.mppt3_status;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_custom_sensor_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_custom_sensor_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_custom_sensor_data_pack(system_id, component_id, &msg , packet1.dbaro_pres_pa , packet1.dbaro_velo_ms , packet1.amb_temp_celsius , packet1.adc121_vspb_volt , packet1.adc121_cspb_amp , packet1.adc121_cs1_amp , packet1.adc121_cs2_amp , packet1.mppt_timestamp , packet1.mppt1_volt , packet1.mppt1_amp , packet1.mppt1_pwm , packet1.mppt1_status , packet1.mppt2_volt , packet1.mppt2_amp , packet1.mppt2_pwm , packet1.mppt2_status , packet1.mppt3_volt , packet1.mppt3_amp , packet1.mppt3_pwm , packet1.mppt3_status );
	mavlink_msg_custom_sensor_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_custom_sensor_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.dbaro_pres_pa , packet1.dbaro_velo_ms , packet1.amb_temp_celsius , packet1.adc121_vspb_volt , packet1.adc121_cspb_amp , packet1.adc121_cs1_amp , packet1.adc121_cs2_amp , packet1.mppt_timestamp , packet1.mppt1_volt , packet1.mppt1_amp , packet1.mppt1_pwm , packet1.mppt1_status , packet1.mppt2_volt , packet1.mppt2_amp , packet1.mppt2_pwm , packet1.mppt2_status , packet1.mppt3_volt , packet1.mppt3_amp , packet1.mppt3_pwm , packet1.mppt3_status );
	mavlink_msg_custom_sensor_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_custom_sensor_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_custom_sensor_data_send(MAVLINK_COMM_1 , packet1.dbaro_pres_pa , packet1.dbaro_velo_ms , packet1.amb_temp_celsius , packet1.adc121_vspb_volt , packet1.adc121_cspb_amp , packet1.adc121_cs1_amp , packet1.adc121_cs2_amp , packet1.mppt_timestamp , packet1.mppt1_volt , packet1.mppt1_amp , packet1.mppt1_pwm , packet1.mppt1_status , packet1.mppt2_volt , packet1.mppt2_amp , packet1.mppt2_pwm , packet1.mppt2_status , packet1.mppt3_volt , packet1.mppt3_amp , packet1.mppt3_pwm , packet1.mppt3_status );
	mavlink_msg_custom_sensor_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_aslctrl_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_aslctrl_data_t packet_in = {
		93372036854775807ULL,
	}73.0,
	}101.0,
	}129.0,
	}157.0,
	}185.0,
	}213.0,
	}241.0,
	}269.0,
	}297.0,
	}325.0,
	}353.0,
	}381.0,
	}409.0,
	}437.0,
	}465.0,
	}493.0,
	}521.0,
	}549.0,
	}577.0,
	}605.0,
	}633.0,
	}25,
	};
	mavlink_aslctrl_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.h = packet_in.h;
        	packet1.hRef = packet_in.hRef;
        	packet1.hRef_t = packet_in.hRef_t;
        	packet1.PitchAngle = packet_in.PitchAngle;
        	packet1.PitchAngleRef = packet_in.PitchAngleRef;
        	packet1.q = packet_in.q;
        	packet1.qRef = packet_in.qRef;
        	packet1.uElev = packet_in.uElev;
        	packet1.uThrot = packet_in.uThrot;
        	packet1.uThrot2 = packet_in.uThrot2;
        	packet1.aZ = packet_in.aZ;
        	packet1.YawAngle = packet_in.YawAngle;
        	packet1.YawAngleRef = packet_in.YawAngleRef;
        	packet1.RollAngle = packet_in.RollAngle;
        	packet1.RollAngleRef = packet_in.RollAngleRef;
        	packet1.p = packet_in.p;
        	packet1.pRef = packet_in.pRef;
        	packet1.r = packet_in.r;
        	packet1.rRef = packet_in.rRef;
        	packet1.uAil = packet_in.uAil;
        	packet1.uRud = packet_in.uRud;
        	packet1.aslctrl_mode = packet_in.aslctrl_mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_aslctrl_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_data_pack(system_id, component_id, &msg , packet1.timestamp , packet1.aslctrl_mode , packet1.h , packet1.hRef , packet1.hRef_t , packet1.PitchAngle , packet1.PitchAngleRef , packet1.q , packet1.qRef , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.aZ , packet1.YawAngle , packet1.YawAngleRef , packet1.RollAngle , packet1.RollAngleRef , packet1.p , packet1.pRef , packet1.r , packet1.rRef , packet1.uAil , packet1.uRud );
	mavlink_msg_aslctrl_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.aslctrl_mode , packet1.h , packet1.hRef , packet1.hRef_t , packet1.PitchAngle , packet1.PitchAngleRef , packet1.q , packet1.qRef , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.aZ , packet1.YawAngle , packet1.YawAngleRef , packet1.RollAngle , packet1.RollAngleRef , packet1.p , packet1.pRef , packet1.r , packet1.rRef , packet1.uAil , packet1.uRud );
	mavlink_msg_aslctrl_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_aslctrl_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_aslctrl_data_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.aslctrl_mode , packet1.h , packet1.hRef , packet1.hRef_t , packet1.PitchAngle , packet1.PitchAngleRef , packet1.q , packet1.qRef , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.aZ , packet1.YawAngle , packet1.YawAngleRef , packet1.RollAngle , packet1.RollAngleRef , packet1.p , packet1.pRef , packet1.r , packet1.rRef , packet1.uAil , packet1.uRud );
	mavlink_msg_aslctrl_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ASLUAV(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_xbee_hardreset(system_id, component_id, last_msg);
	mavlink_test_custom_sensor_data(system_id, component_id, last_msg);
	mavlink_test_aslctrl_data(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ASLUAV_TESTSUITE_H
