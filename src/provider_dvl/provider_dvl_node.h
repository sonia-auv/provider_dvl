/**
 * \file	provider_dvl_node.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	07/05/2017
 *
 * \copyright Copyright (c) 2017 S.O.N.I.A. All rights reserved.
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

#ifndef PROVIDER_DVL_NODE_H
#define PROVIDER_DVL_NODE_H

#include <string>

#include <ros/node_handle.h>
#include <sonia_common/BodyVelocityDVL.h>
#include <sonia_common/AttitudeDVL.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include "../driver/ethernet_socket.h"
#include "dvl_data.h"

#define ANGLE_LSD 0.01

namespace provider_dvl {


class ProviderDvlNode {
  public:
      //==========================================================================
      // P U B L I C   C / D T O R S

      ProviderDvlNode(const ros::NodeHandlePtr &nh);
      ~ProviderDvlNode();

      //==========================================================================
      // P U B L I C   M E T H O D S

      void Spin();
      

  private:
    //==========================================================================
    // P R I V A T E   M E T H O D S

      void FillVelocityMessage(ros::Time timestamp);
      void FillAttitudeDVLMessage(ros::Time timestamp);
      void LeakSensorMessage();
      uint16_t calculateChecksum(uint8_t *dvlData);
      bool confirmChecksum(uint8_t *dvlData);
      std::string verifyFrameId(uint8_t systemId);

      void enableDisablePingCallback(const std_msgs::Bool& msg);
      void setAnglesCallback(const geometry_msgs::Vector3& msg);
      void setDepthCallback(const std_msgs::Float64& msg);

      //==========================================================================
      // P R I V A T E   M E M B E R S

      ros::NodeHandlePtr nh_;
      EthernetSocket socketUDP_, socketTCP_;

      DVLformat21_t dvl_data_;

      ros::Time timestamp_;
      ros::Publisher dvl_velocity_publisher_;
      ros::Publisher dvl_position_publisher_;
      ros::Publisher dvl_leak_sensor_publisher_;
      
      ros::Subscriber enableDisablePingSub;
      ros::Subscriber setAnglesSub;
      ros::Subscriber setDepthSub;
  };

} // namespace provider_dvl

#endif //PROVIDER_DVL_NODE_H
