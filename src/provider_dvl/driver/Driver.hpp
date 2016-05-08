/**
 * \file	driver.hpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DVL_TELEDYNE_DRIVER_HPP
#define DVL_TELEDYNE_DRIVER_HPP

#include <ros/forwards.h>
#include <ros/service_server.h>
#include <ros/node_handle.h>
#include <sonia_msgs/SendDvlConfigFile.h>
#include <sonia_msgs/SendDvlConfigCommand.h>
#include <provider_dvl/base/Driver.hpp>
#include <provider_dvl/driver/PD0Parser.hpp>

#define M_SONIA_BAUDRATE 115200

namespace dvl_teledyne {

class Driver : public iodrivers_base::Driver, public PD0Parser {
 public:
  Driver(const ros::NodeHandlePtr &nh);

  void PrintDeviceInfos() const;

  /**
   * Tries to access the DVL at the provided URI
   *
   * For now, only a serial port can be provided. It is assumed that the
   * DVL is using 9600 bauds (the manufacturer's default)
   */
  void open(std::string const& uri);

  /**
   * Once open using the baudrate specified in the URI, configures the
   * device to output at a different baud rate, and modifies the driver's
   * configuration accordingly
   */
  void setDesiredBaudrate(int rate);

  /**
   * Configures the output coordinate system
   */
  void setOutputConfiguration(PD0Message::OutputConfiguration conf);

  /** Sends a text file that contains commands to the device
   *
   * The device is guaranteed to be in configuration mode afterwards
   * (regardless of whether the configuration file contains a CS
   * command). Use startAcquisition() to put it in acquisition mode
   */
  bool sendConfigurationFile(std::string const& file_name);

  /**
   * Sets the device into configuration mode (and make it stop pinging)
   */
  void setConfigurationMode();

  /**
   * Start acquisition
   *
   * Since the driver relies on receiving PD0 message frames, this method
   * requires the DVL to send into this format, and then starts pinging
   */
  void startAcquisition();

  /**
   * Read available packets on the I/O
   */
  void read();

  /**
   * Verifies that the DVL acked a configuration command
   * Throws std::runtime_error if an error is reported by the device
   */
  void readConfigurationAck(
      base::Time const& timeout = base::Time::fromSeconds(1));

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  int extractPacket(uint8_t const* buffer, size_t buffer_size) const;


  /**
   * Tells the DVL to switch to the desired rate
   */
  void setDeviceBaudrate(int rate);

  bool SendConfigFileSrv(sonia_msgs::SendDvlConfigFile::Request &req,
                         sonia_msgs::SendDvlConfigFile::Response &res);

  bool SendConfigCommandSrv(sonia_msgs::SendDvlConfigCommand::Request &req,
                            sonia_msgs::SendDvlConfigCommand::Response &res);

  //============================================================================
  // P R I V A T E   M E M B E R S

  bool mConfMode;

  int mDesiredBaudrate;

  ros::NodeHandlePtr nh_;

  ros::ServiceServer send_config_file_srv_;

  ros::ServiceServer send_config_command_srv_;

  std::vector<uint8_t> buffer;
};

}  // namespace dvl_teledyne

#endif
