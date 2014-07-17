/*=====================================================================
 
 MAVCONN Micro Air Vehicle Flying Robotics Toolkit
 
 (c) 2009, 2010 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>
 
 This file is part of the MAVCONN project
 
 MAVCONN is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 MAVCONN is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.
 
 ======================================================================*/

/**
 * @file
 *   @brief The serial interface process
 *
 *   This process connects any external MAVLink UART device to ROS
 *   Haha, Amir, you will get the support e-mails ;-P
 *
 *   @author Lorenz Meier, <mavteam@student.ethz.ch>
 *   @author Stefan Leutenegger, <amir.melzer@mavt.ethz.ch>
 *
 */

#include "ros/ros.h"

#include "mavlink_ros/Mavlink.h"
#include "mavlink_ros/AslctrlData.h"
#include "mavlink_ros/AslctrlDebug.h"
#include "mavlink_ros/CustomSensorData.h"
#include "mavlink_ros/ServoOutput.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "gps_common/GPSFix.h"

#include "mavlink/v1.0/ASLUAV/mavlink.h"
#include <glib.h>

// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>

// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

using std::string;
using namespace std;

struct timeval tv;		  ///< System time

int baud = 230400;                 ///< The serial baud rate

// Settings
int sysid = 42;  ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 110;
int serial_compid = 0;
std::string port = "/dev/ttyUSB0";  ///< The serial port name, e.g. /dev/ttyUSB0
bool silent = false;              ///< Wether console output should be enabled
bool verbose = false;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output
bool pc2serial = true;	///< Enable PC to serial push mode (send more stuff from pc over serial)
int fd;

/**
 * Grabs all mavlink-messages from the ROS-Topic "mavlink" and publishes them on ROS
 */

ros::Subscriber mavlink_sub;
ros::Publisher mavlink_pub;
ros::Publisher imu_pub;
ros::Publisher imu_raw_pub;
ros::Publisher gps_pub;
ros::Publisher mag_pub;
ros::Publisher static_pressure_pub;
ros::Publisher dynamic_pressure_pub;
ros::Publisher ambient_temperature_pub;
ros::Publisher custom_sensor_data_pub;
ros::Publisher aslctrl_data_pub;
ros::Publisher aslctrl_debug_pub;
ros::Publisher servo_output_pub;

mavlink_highres_imu_t imu_raw;
mavlink_gps_raw_int_t gps_raw;
mavlink_custom_sensor_data_t cust_raw;
mavlink_aslctrl_data_t aslctrl_data_raw;
mavlink_aslctrl_debug_t aslctrl_debug_raw;
mavlink_servo_output_raw_t servo_output_raw;
std::string frame_id("px4");

// from asctec_hl_interface
inline void angle2quaternion(const double &roll, const double &pitch,
                             const double &yaw, double *w, double *x, double *y,
                             double *z) {
  double sR2, cR2, sP2, cP2, sY2, cY2;
  sincos(roll * 0.5, &sR2, &cR2);
  sincos(pitch * 0.5, &sP2, &cP2);
  sincos(yaw * 0.5, &sY2, &cY2);

  // TODO: change rotation order
  // this follows AscTec's pre- 2012 firmware rotation order: Rz*Rx*Ry
//  *w = cP2 * cR2 * cY2 - sP2 * sR2 * sY2;
//  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
//  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
//  *z = cP2 * cR2 * sY2 + cY2 * sP2 * sR2;

  // Rz*Ry*Rx for 2012 firmware on the:
  *w = cP2 * cR2 * cY2 + sP2 * sR2 * sY2;
  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
  *z = cP2 * cR2 * sY2 - cY2 * sP2 * sR2;
}

/**
 *
 *
 * Returns the file descriptor on success or -1 on error.
 */

int open_port(std::string& port) {
  int fd; /* File descriptor for the port */

  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    /* Could not open the port. */
    return (-1);
  } else {
    fcntl(fd, F_SETFL, 0);
  }

  return (fd);
}

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity,
                bool hardware_control) {
  //struct termios options;

  struct termios config;
  if (!isatty(fd)) {
    fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
    return false;
  }
  if (tcgetattr(fd, &config) < 0) {
    fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
    return false;
  }
  //
  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
  config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP
      | IXON);
  //
  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  //
  config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

#ifdef OLCUC
  config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
  config.c_oflag &= ~ONOEOT;
#endif

  //
  // No line processing:
  // echo off, echo newline off, canonical mode off,
  // extended input processing off, signal chars off
  //
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  //
  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  //
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  //
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  config.c_cc[VMIN] = 1;
  config.c_cc[VTIME] = 10;  // was 0

  // Get the current options for the port
  //tcgetattr(fd, &options);

  switch (baud) {
    case 1200:
      if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0) {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n",
                baud);
        return false;
      }
      break;
    case 1800:
      cfsetispeed(&config, B1800);
      cfsetospeed(&config, B1800);
      break;
    case 9600:
      cfsetispeed(&config, B9600);
      cfsetospeed(&config, B9600);
      break;
    case 19200:
      cfsetispeed(&config, B19200);
      cfsetospeed(&config, B19200);
      break;
    case 38400:
      if (cfsetispeed(&config, B38400) < 0
          || cfsetospeed(&config, B38400) < 0) {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n",
                baud);
        return false;
      }
      break;
    case 57600:
      if (cfsetispeed(&config, B57600) < 0
          || cfsetospeed(&config, B57600) < 0) {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n",
                baud);
        return false;
      }
      break;
    case 115200:
      if (cfsetispeed(&config, B115200) < 0
          || cfsetospeed(&config, B115200) < 0) {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n",
                baud);
        return false;
      }
      break;

      // lestefan added: I believe, this should work as well...
    case 230400:
      if (cfsetispeed(&config, B230400) < 0
          || cfsetospeed(&config, B230400) < 0) {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n",
                baud);
        return false;
      }
      break;

      // These two non-standard (by the 70'ties ) rates are fully supported on
      // current Debian and Mac OS versions (tested since 2010).
    case 460800:
      if (cfsetispeed(&config, 460800) < 0
          || cfsetospeed(&config, 460800) < 0) {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n",
                baud);
        return false;
      }
      break;
    case 921600:
      if (cfsetispeed(&config, 921600) < 0
          || cfsetospeed(&config, 921600) < 0) {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n",
                baud);
        return false;
      }
      break;
    default:
      fprintf(stderr,
              "ERROR: Desired baud rate %d could not be set, aborting.\n",
              baud);
      return false;

      break;
  }

  //
  // Finally, apply the configuration
  //
  if (tcsetattr(fd, TCSAFLUSH, &config) < 0) {
    fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
    return false;
  }
  return true;
}

void close_port(int fd) {
  close(fd);
}

/**
 * @brief Serial function
 *
 * This function blocks waiting for serial data in it's own thread
 * and forwards the data once received.
 */
void* serial_wait(void* serial_ptr) {
  int fd = *((int*) serial_ptr);

  mavlink_status_t lastStatus;
  lastStatus.packet_rx_drop_count = 0;

  // Blocking wait for new data
  while (1) {
    //if (debug) printf("Checking for new data on serial port\n");
    // Block until data is available, read only one byte to be able to continue immediately
    //char buf[MAVLINK_MAX_PACKET_LEN];
    uint8_t cp;
    mavlink_message_t message;
    mavlink_status_t status;
    uint8_t msgReceived = false;
    //tcflush(fd, TCIFLUSH);
    if (read(fd, &cp, 1) > 0) {
      //fprintf(stderr, "%02x ", cp);
      // Check if a message could be decoded, return the message in case yes
      msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
      if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) {
        if (verbose || debug)
          printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
        if (debug) {
          unsigned char v = cp;
          fprintf(stderr, "%02x ", v);
        }
      }
      lastStatus = status;
    } else {
      if (!silent)
        fprintf(stderr, "ERROR: Could not read from port %s\n", port.c_str());
    }

    // If a message could be decoded, handle it
    if (msgReceived) {
      //if (verbose || debug) std::cout << std::dec << "Received and forwarded serial port message with id " << static_cast<unsigned int>(message.msgid) << " from system " << static_cast<int>(message.sysid) << std::endl;

      // Do not send images over serial port

      // DEBUG output
      if (debug) {
        fprintf(stderr, "Forwarding SERIAL -> ROS: ");
        unsigned int i;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        unsigned int messageLength = mavlink_msg_to_send_buffer(buffer,
                                                                &message);
        if (messageLength > MAVLINK_MAX_PACKET_LEN) {
          fprintf(stderr,
                  "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
        } else {
          for (i = 0; i < messageLength; i++) {
            unsigned char v = buffer[i];
            fprintf(stderr, "%02x ", v);
          }
          fprintf(stderr, "\n");
        }
      }

      if (verbose || debug)
        ROS_INFO("Received message from serial with ID #%d (sys:%d|comp:%d):\n",
                 message.msgid, message.sysid, message.compid);

      /**
       * Serialize the Mavlink-ROS-message
       */
      mavlink_ros::Mavlink rosmavlink_msg;

      rosmavlink_msg.len = message.len;
      rosmavlink_msg.seq = message.seq;
      rosmavlink_msg.sysid = message.sysid;
      rosmavlink_msg.compid = message.compid;
      rosmavlink_msg.msgid = message.msgid;

      for (int i = 0; i < message.len / 8; i++) {
        (rosmavlink_msg.payload64).push_back(message.payload64[i]);
      }

      /**
       * Send the received MAVLink message to ROS (topic: mavlink, see main())
       */
      //mavlink_pub.publish(rosmavlink_msg);
      switch (message.msgid) {
        /*
         * Message specs (xxx: soon mavlink.org):
         * https://pixhawk.ethz.ch/mavlink/#ATTITUDE
         */
        case MAVLINK_MSG_ID_ATTITUDE: {
          if (imu_pub.getNumSubscribers() > 0) {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&message, &att);

            sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);

            angle2quaternion(att.roll, -att.pitch, -att.yaw,
                             &(imu_msg->orientation.w),
                             &(imu_msg->orientation.x),
                             &(imu_msg->orientation.y),
                             &(imu_msg->orientation.z));

            // TODO: check/verify that these are body-fixed
            imu_msg->angular_velocity.x = att.rollspeed;
            imu_msg->angular_velocity.y = att.pitchspeed;
            imu_msg->angular_velocity.z = att.yawspeed;

            // take this from imu high res message, this is supposed to arrive before this one and should pretty much be in sync then
            imu_msg->linear_acceleration.x = imu_raw.xacc;
            imu_msg->linear_acceleration.y = imu_raw.yacc;
            imu_msg->linear_acceleration.z = imu_raw.zacc;

            // TODO: can we fill in the covariance here from a parameter that we set from the specs/experience?
            for (sensor_msgs::Imu::_orientation_covariance_type::iterator it =
                imu_msg->orientation_covariance.begin();
                it != imu_msg->orientation_covariance.end(); ++it)
              *it = 0;

            for (sensor_msgs::Imu::_angular_velocity_covariance_type::iterator it =
                imu_msg->angular_velocity_covariance.begin();
                it != imu_msg->angular_velocity_covariance.end(); ++it)
              *it = 0;

            for (sensor_msgs::Imu::_linear_acceleration_covariance_type::iterator it =
                imu_msg->linear_acceleration_covariance.begin();
                it != imu_msg->linear_acceleration_covariance.end(); ++it)
              *it = 0;

            imu_msg->header.frame_id = frame_id;
            imu_msg->header.seq = imu_raw.time_usec / 1000;
            imu_msg->header.stamp = ros::Time::now();

            imu_pub.publish(imu_msg);
          }
        }
          break;

          /*
           * Message specs (xxx: soon mavlink.org):
           * https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU
           */
        case MAVLINK_MSG_ID_HIGHRES_IMU: {
          /* decode message */
          mavlink_msg_highres_imu_decode(&message, &imu_raw);

          std_msgs::Header header;
          header.stamp = ros::Time::now();
          header.seq = imu_raw.time_usec / 1000;
          header.frame_id = frame_id;

          if (imu_raw_pub.getNumSubscribers() > 0) {

            sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);

            imu_msg->angular_velocity.x = imu_raw.xgyro;
            imu_msg->angular_velocity.y = imu_raw.ygyro;
            imu_msg->angular_velocity.z = imu_raw.zgyro;

            imu_msg->linear_acceleration.x = imu_raw.xacc;
            imu_msg->linear_acceleration.y = imu_raw.yacc;
            imu_msg->linear_acceleration.z = imu_raw.zacc;

            // TODO: can we fill in the covariance here from a parameter that we set from the specs/experience?
            for (sensor_msgs::Imu::_angular_velocity_covariance_type::iterator it =
                imu_msg->angular_velocity_covariance.begin();
                it != imu_msg->angular_velocity_covariance.end(); ++it)
              *it = 0;

            for (sensor_msgs::Imu::_linear_acceleration_covariance_type::iterator it =
                imu_msg->linear_acceleration_covariance.begin();
                it != imu_msg->linear_acceleration_covariance.end(); ++it)
              *it = 0;

            imu_msg->orientation_covariance[0] = -1;

            imu_msg->header = header;

            imu_raw_pub.publish(imu_msg);

            if (verbose)
              ROS_INFO_THROTTLE(1, "Published IMU message (sys:%d|comp:%d):\n",
                                message.sysid, message.compid);
          }
          if (mag_pub.getNumSubscribers() > 0) {
            const double gauss_to_tesla = 1.0e-4;
            sensor_msgs::MagneticFieldPtr mag_msg(
                new sensor_msgs::MagneticField);

            mag_msg->magnetic_field.x = imu_raw.xmag * gauss_to_tesla;
            mag_msg->magnetic_field.y = imu_raw.ymag * gauss_to_tesla;
            mag_msg->magnetic_field.z = imu_raw.zmag * gauss_to_tesla;

            // TODO: again covariance
            for (sensor_msgs::MagneticField::_magnetic_field_covariance_type::iterator it =
                mag_msg->magnetic_field_covariance.begin();
                it != mag_msg->magnetic_field_covariance.end(); ++it)
              *it = 0;

            mag_msg->header = header;
            mag_pub.publish(mag_msg);
          }

          // static pressure
          if (static_pressure_pub.getNumSubscribers() > 0) {
            sensor_msgs::FluidPressurePtr static_pressure_msg(
                new sensor_msgs::FluidPressure);

            static_pressure_msg->fluid_pressure = imu_raw.abs_pressure * 100;  // convert to Pa
            static_pressure_msg->header = header;
            static_pressure_pub.publish(static_pressure_msg);

            if (verbose)
              ROS_INFO_THROTTLE(1, "Published IMU message (sys:%d|comp:%d):\n",
                                message.sysid, message.compid);

          }
        }
          break;

        case MAVLINK_MSG_ID_GPS_RAW_INT: {

          // decode message
          mavlink_msg_gps_raw_int_decode(&message, &gps_raw);

          std_msgs::Header header;

          // TODO->Amir: find out, if this is actually the gps time...
          header.stamp = ros::Time::now();  //ros::Time(double(gps_raw.time_usec)*1.0e6);
          header.seq = gps_raw.time_usec / 1000;
          header.frame_id = frame_id;

          if (gps_pub.getNumSubscribers() > 0) {

            gps_common::GPSFixPtr gps_msg(new gps_common::GPSFix);

            gps_msg->header = header;

            // populate message fields
            gps_msg->latitude = double(gps_raw.lat) * 1.0e-7;  // deg
            gps_msg->longitude = double(gps_raw.lon) * 1.0e-7;  // deg
            gps_msg->altitude = double(gps_raw.alt) * 1.0e-3;  // deg
            gps_msg->hdop = double(gps_raw.eph) * 1.0e-2; // m
            gps_msg->vdop = double(gps_raw.epv) * 1.0e-2; // m
            gps_msg->speed = double(gps_raw.vel) * 1.0e-2; // m/s

            // TODO: Add other fields

            // publish message
            gps_pub.publish(gps_msg);

            if (verbose)
              ROS_INFO_THROTTLE(1, "Published IMU message (sys:%d|comp:%d):\n",
                                message.sysid, message.compid);
          }

        }
          break;



        /*case MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA: {

          mavlink_msg_custom_sensor_data_decode(&message, &cust_raw);

          std_msgs::Header header;
          // TODO->Amir: find out, if this is actually the gps time...
          header.stamp = ros::Time::now();  //ros::Time(double(gps_raw.time_usec)*1.0e6);
          header.seq = cust_raw.mppt_timestamp / 1000;
          header.frame_id = frame_id;

          // publish dynamic pressure
          if (dynamic_pressure_pub.getNumSubscribers() > 0) {

            sensor_msgs::FluidPressurePtr dynamic_pressure_msg(
                new sensor_msgs::FluidPressure);
            dynamic_pressure_msg->fluid_pressure = cust_raw.dbaro_pres_pa;
            dynamic_pressure_msg->header = header;
            dynamic_pressure_pub.publish(dynamic_pressure_msg);

            if (verbose)
              ROS_INFO_THROTTLE(1, "Published IMU message (sys:%d|comp:%d):\n",
                                message.sysid, message.compid);
          }

          // publish ambient temperature
          if (ambient_temperature_pub.getNumSubscribers() > 0) {

            sensor_msgs::TemperaturePtr ambient_temperature_msg(
                new sensor_msgs::Temperature);
            ambient_temperature_msg->temperature = cust_raw.amb_temp_celsius;  // deg Celsius
            ambient_temperature_msg->header = header;
            ambient_temperature_pub.publish(ambient_temperature_msg);

            if (verbose)
              ROS_INFO_THROTTLE(1, "Published IMU message (sys:%d|comp:%d):\n",
                                message.sysid, message.compid);
          }

        }
          break;*/

        case MAVLINK_MSG_ID_ASLCTRL_DATA: {

          // decode message
          mavlink_msg_aslctrl_data_decode(&message, &aslctrl_data_raw);

          std_msgs::Header header;

          // TODO->Amir: find out, if this is actually the gps time...
          header.stamp = ros::Time::now();  //ros::Time(double(gps_raw.time_usec)*1.0e6);
          header.seq = cust_raw.mppt_timestamp / 1000;
          header.frame_id = frame_id;

          if (aslctrl_data_pub.getNumSubscribers() > 0) {

            mavlink_ros::AslctrlDataPtr aslctrl_data_msg(
                new mavlink_ros::AslctrlData);

            aslctrl_data_msg->header = header;

            // populate message fields
            aslctrl_data_msg->h = aslctrl_data_raw.h;
            aslctrl_data_msg->hRef = aslctrl_data_raw.hRef;
            aslctrl_data_msg->hRef_t = aslctrl_data_raw.hRef_t;
            aslctrl_data_msg->PitchAngle = aslctrl_data_raw.PitchAngle;
            aslctrl_data_msg->PitchAngleRef = aslctrl_data_raw.PitchAngleRef;
            aslctrl_data_msg->q = aslctrl_data_raw.q;
            aslctrl_data_msg->qRef = aslctrl_data_raw.qRef;
            aslctrl_data_msg->uElev = aslctrl_data_raw.uElev;
            aslctrl_data_msg->uThrot = aslctrl_data_raw.uThrot;
            aslctrl_data_msg->uThrot2 = aslctrl_data_raw.uThrot2;
            aslctrl_data_msg->aZ = aslctrl_data_raw.aZ;
            aslctrl_data_msg->YawAngle = aslctrl_data_raw.YawAngle;
            aslctrl_data_msg->YawAngleRef = aslctrl_data_raw.YawAngleRef;
            aslctrl_data_msg->RollAngle = aslctrl_data_raw.RollAngle;
            aslctrl_data_msg->p = aslctrl_data_raw.p;
            aslctrl_data_msg->pRef = aslctrl_data_raw.pRef;
            aslctrl_data_msg->r = aslctrl_data_raw.r;
            aslctrl_data_msg->rRef = aslctrl_data_raw.rRef;
            aslctrl_data_msg->uAil = aslctrl_data_raw.uAil;
            aslctrl_data_msg->uRud = aslctrl_data_raw.uRud;
            aslctrl_data_msg->aslctrl_mode = aslctrl_data_raw.aslctrl_mode;

            // publish message
            aslctrl_data_pub.publish(aslctrl_data_msg);

            if (verbose)
              ROS_INFO_THROTTLE(1, "Published AslctrlData message (sys:%d|comp:%d):\n",
                                message.sysid, message.compid);
          }

        }
          break;

        case MAVLINK_MSG_ID_ASLCTRL_DEBUG: {

          // decode message
          mavlink_msg_aslctrl_debug_decode(&message, &aslctrl_debug_raw);

          std_msgs::Header header;

          // TODO->Amir: find out, if this is actually the gps time...
          header.stamp = ros::Time::now();  //ros::Time(double(gps_raw.time_usec)*1.0e6);
          header.seq = cust_raw.mppt_timestamp / 1000;
          header.frame_id = frame_id;

          if (aslctrl_debug_pub.getNumSubscribers() > 0) {

            mavlink_ros::AslctrlDebugPtr aslctrl_debug_msg(
                new mavlink_ros::AslctrlDebug);

            aslctrl_debug_msg->header = header;

            // populate message fields
            aslctrl_debug_msg->i32_1 = aslctrl_debug_raw.i32_1;
            aslctrl_debug_msg->f_1 = aslctrl_debug_raw.f_1;
            aslctrl_debug_msg->f_2 = aslctrl_debug_raw.f_2;
            aslctrl_debug_msg->f_3 = aslctrl_debug_raw.f_3;
            aslctrl_debug_msg->f_4 = aslctrl_debug_raw.f_4;
            aslctrl_debug_msg->f_5 = aslctrl_debug_raw.f_5;
            aslctrl_debug_msg->f_6 = aslctrl_debug_raw.f_6;
            aslctrl_debug_msg->f_7 = aslctrl_debug_raw.f_7;
            aslctrl_debug_msg->f_8 = aslctrl_debug_raw.f_8;
            aslctrl_debug_msg->i8_1 = aslctrl_debug_raw.i8_1;
            aslctrl_debug_msg->i8_2 = aslctrl_debug_raw.i8_2;

            // publish message
            aslctrl_debug_pub.publish(aslctrl_debug_msg);

            if (verbose)
              ROS_INFO_THROTTLE(1, "Published AslctrlDebug message (sys:%d|comp:%d):\n",
                                message.sysid, message.compid);
          }

        }
          break;

        case MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA: {

          // decode message
          mavlink_msg_custom_sensor_data_decode(&message, &cust_raw);

          std_msgs::Header header;

          // TODO->Amir: find out, if this is actually the gps time...
          header.stamp = ros::Time::now();  //ros::Time(double(gps_raw.time_usec)*1.0e6);
          header.seq = cust_raw.mppt_timestamp / 1000;
          header.frame_id = frame_id;

          if (custom_sensor_data_pub.getNumSubscribers() > 0) {

            mavlink_ros::CustomSensorDataPtr custom_sensor_data_msg(
                new mavlink_ros::CustomSensorData);

            custom_sensor_data_msg->header = header;

            // populate message fields
            custom_sensor_data_msg->dbaro_pres_pa = cust_raw.dbaro_pres_pa;
            custom_sensor_data_msg->dbaro_velo_ms = cust_raw.dbaro_velo_ms;
            custom_sensor_data_msg->amb_temp_celsius = cust_raw.amb_temp_celsius;
            custom_sensor_data_msg->adc121_vspb_volt = cust_raw.adc121_vspb_volt;
            custom_sensor_data_msg->adc121_cspb_amp = cust_raw.adc121_cspb_amp;
            custom_sensor_data_msg->adc121_cs1_amp = cust_raw.adc121_cs1_amp;
            custom_sensor_data_msg->adc121_cs2_amp = cust_raw.adc121_cs2_amp;
            custom_sensor_data_msg->mppt1_volt = cust_raw.mppt1_volt;
            custom_sensor_data_msg->mppt1_amp = cust_raw.mppt1_amp;
            custom_sensor_data_msg->mppt2_volt = cust_raw.mppt2_volt;
            custom_sensor_data_msg->mppt2_amp = cust_raw.mppt2_amp;
            custom_sensor_data_msg->mppt3_volt = cust_raw.mppt3_volt;
            custom_sensor_data_msg->mppt3_amp = cust_raw.mppt3_amp;
            custom_sensor_data_msg->mppt1_pwm = cust_raw.mppt1_pwm;
            custom_sensor_data_msg->mppt2_pwm = cust_raw.mppt2_pwm;
            custom_sensor_data_msg->mppt3_pwm = cust_raw.mppt3_pwm;
            custom_sensor_data_msg->mppt1_status = cust_raw.mppt1_status;
            custom_sensor_data_msg->mppt2_status = cust_raw.mppt2_status;
            custom_sensor_data_msg->mppt3_status = cust_raw.mppt3_status;

            // publish message
            custom_sensor_data_pub.publish(custom_sensor_data_msg);

            if (verbose)
              ROS_INFO_THROTTLE(1, "Published CustomSensorData message (sys:%d|comp:%d):\n",
                                message.sysid, message.compid);
          }

        }
          break;

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {

          // decode message
          mavlink_msg_servo_output_raw_decode(&message, &servo_output_raw);

          std_msgs::Header header;

          // TODO->Amir: find out, if this is actually the gps time...
          header.stamp = ros::Time::now();  //ros::Time(double(gps_raw.time_usec)*1.0e6);
          header.seq = cust_raw.mppt_timestamp / 1000;
          header.frame_id = frame_id;

          if (servo_output_pub.getNumSubscribers() > 0) {

            mavlink_ros::ServoOutputPtr servo_output_msg(
                new mavlink_ros::ServoOutput);

            servo_output_msg->header = header;

            // populate message fields
            servo_output_msg->time_usec = servo_output_raw.time_usec;

            // publish message
            servo_output_pub.publish(servo_output_msg);

            if (verbose)
              ROS_INFO_THROTTLE(1, "Published ServoOutput message (sys:%d|comp:%d):\n",
                                message.sysid, message.compid);
          }

        }
          break;

      }
    }
  }
  return NULL;
}

void mavlinkCallback(const mavlink_ros::Mavlink &mavlink_ros_msg) {

  /**
   * Convert mavlink_ros::Mavlink to mavlink_message_t
   */
  mavlink_message_t msg;
  msg.msgid = mavlink_ros_msg.msgid;

  static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;

  //Copy payload from mavlink_msg (from ROS) to the new "real" mavlink message
  copy(mavlink_ros_msg.payload64.begin(), mavlink_ros_msg.payload64.end(),
       msg.payload64);

  mavlink_finalize_message_chan(&msg, mavlink_ros_msg.sysid,
                                mavlink_ros_msg.compid, MAVLINK_COMM_0,
                                mavlink_ros_msg.len, mavlink_crcs[msg.msgid]);

  /**
   * Send mavlink_message to UART
   */
  if (verbose)
    ROS_INFO("Sent Mavlink from ROS to UART, Message-ID: [%i]",
             mavlink_ros_msg.msgid);

  // Send message over serial port
  static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int messageLength = mavlink_msg_to_send_buffer(buffer, &msg);
  if (debug)
    printf("Writing %d bytes\n", messageLength);
  int written = write(fd, (char*) buffer, messageLength);
  tcflush(fd, TCOFLUSH);
  if (messageLength != written)
    fprintf(stderr, "ERROR: Wrote %d bytes but should have written %d\n",
            written, messageLength);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mavlink_ros_serial");

  // Handling Program options
  static GOptionEntry entries[] = { { "portname", 'p', 0, G_OPTION_ARG_STRING,
      &port, "Serial port name", port.c_str() }, { "baudrate", 'b', 0,
      G_OPTION_ARG_INT, &baud, "Baudrate", "115200" }, { "verbose", 'v', 0,
      G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL }, { "debug", 'd', 0,
      G_OPTION_ARG_NONE, &debug, "Debug output", NULL }, { "sysid", 'a', 0,
      G_OPTION_ARG_INT, &sysid, "ID of this system, 1-255", "42" }, { "compid",
      'c', 0, G_OPTION_ARG_INT, &compid, "ID of this component, 1-255", "199" },
      { NULL, 0, 0, G_OPTION_ARG_NONE, NULL, NULL, 0 } };

  GError *error = NULL;
  GOptionContext *context;

  context = g_option_context_new(
      "- translate MAVLink messages between ROS to serial port");
  g_option_context_add_main_entries(context, entries, NULL);
  //g_option_context_add_group (context, NULL);
  if (!g_option_context_parse(context, &argc, &argv, &error)) {
    g_print("Option parsing failed: %s\n", error->message);
    exit(1);
  }

  // SETUP SERIAL PORT

  // Exit if opening port failed
  // Open the serial port.
  if (!silent)
    printf("Trying to connect to %s.. ", port.c_str());
  fd = open_port(port);
  if (fd == -1) {
    if (!silent)
      fprintf(stderr, "failure, could not open port.\n");
    exit(EXIT_FAILURE);
  } else {
    if (!silent)
      printf("success.\n");
  }
  if (!silent)
    printf("Trying to configure %s.. ", port.c_str());
  bool setup = setup_port(fd, baud, 8, 1, false, false);
  if (!setup) {
    if (!silent)
      fprintf(stderr, "failure, could not configure port.\n");
    exit(EXIT_FAILURE);
  } else {
    if (!silent)
      printf("success.\n");
  }
  int* fd_ptr = &fd;

  // SETUP ROS
  ros::NodeHandle mavlink_nh("/mavlink");  // always root namespace since I assume it's somewhat "broadcast"
  mavlink_sub = mavlink_nh.subscribe("to", 1000, mavlinkCallback);
  mavlink_pub = mavlink_nh.advertise<mavlink_ros::Mavlink>("from", 1000);

  ros::NodeHandle nh("px4");
  imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);

  ros::NodeHandle raw_nh("px4/raw");
  imu_raw_pub = raw_nh.advertise<sensor_msgs::Imu>("imu", 10);
  mag_pub = raw_nh.advertise<sensor_msgs::MagneticField>("mag", 10);
  gps_pub = raw_nh.advertise<gps_common::GPSFix>("gps", 10);
  static_pressure_pub = raw_nh.advertise<sensor_msgs::FluidPressure>(
      "static_pressure", 10);
  dynamic_pressure_pub = raw_nh.advertise<sensor_msgs::FluidPressure>(
      "dynamic_pressure", 10);
  ambient_temperature_pub = raw_nh.advertise<sensor_msgs::Temperature>(
      "ambient_temperature", 10);
  custom_sensor_data_pub = raw_nh.advertise<mavlink_ros::CustomSensorData>(
      "custom_sensor_data", 10);
  aslctrl_data_pub = raw_nh.advertise<mavlink_ros::AslctrlData>(
      "aslctrl_data", 10);
  aslctrl_debug_pub = raw_nh.advertise<mavlink_ros::AslctrlDebug>(
      "aslctrl_debug", 10);
  servo_output_pub = raw_nh.advertise<mavlink_ros::ServoOutput>(
      "servo_output_raw", 10);

  GThread* serial_thread;
  GError* err;
  if (!g_thread_supported()) {
    g_thread_init(NULL);
    // Only initialize g thread if not already done
  }

  // Run indefinitely while the ROS and serial threads handle the data
  if (!silent)
    printf("\nREADY, waiting for serial/ROS data.\n");

  if ((serial_thread = g_thread_create((GThreadFunc) serial_wait,
                                       (void *) fd_ptr, TRUE, &err)) == NULL) {
    printf("Failed to create serial handling thread: %s!!\n", err->message);
    g_error_free(err);
  }

  int noErrors = 0;
  if (fd == -1 || fd == 0) {
    if (!silent)
      fprintf(
          stderr,
          "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n",
          port.c_str(), baud);
    exit(EXIT_FAILURE);
  } else {
    if (!silent)
      fprintf(
          stderr,
          "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n",
          port.c_str(), baud);
  }

  // FIXME ADD MORE CONNECTION ATTEMPTS

  if (fd == -1 || fd == 0) {
    exit(noErrors);
  }

  // Ready to roll
  printf(
      "\nMAVLINK SERIAL TO ROS BRIDGE STARTED ON MAV %d (COMPONENT ID:%d) - RUNNING..\n\n",
      sysid, compid);

  /**
   * Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
   */
  ros::spin();

  close_port(fd);

  //g_thread_join(serial_thread);
  //exit(0);

  return 0;
}
