/**
 * \file	provider_dvl_node.cc
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

#include "provider_dvl/provider_dvl_node.h"

namespace provider_dvl {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ProviderDvlNode::ProviderDvlNode(const ros::NodeHandlePtr &nh) :
    nh_(nh),
    socket_()
{
  std::string hostname = "192.168.0.240";
  socket_.Connect(hostname, 9002);

  dvl_twist_publisher_ = nh_->advertise<geometry_msgs::TwistStamped>
      ("/provider_dvl/dvl_twist", 1000);
  dvl_fluid_pressure_publisher_ = nh_->advertise<sensor_msgs::FluidPressure>
      ("/provider_dvl/dvl_pressure", 1000);
}

//------------------------------------------------------------------------------
//
ProviderDvlNode::~ProviderDvlNode() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ProviderDvlNode::Spin() {
  ros::Rate r(15);  // 15 hz

  while (ros::ok()) {
    ros::spinOnce();

    timestamp_ = ros::Time::now();
    FillTwistMessage(timestamp_);
    FillFluidPressureMessage(timestamp_);

    r.sleep();
  }
}

//------------------------------------------------------------------------------
//
void ProviderDvlNode::FillTwistMessage(ros::Time timestamp) {
  geometry_msgs::TwistStamped message;

  message.header.stamp = timestamp;
  message.header.frame_id = "/base_link";
  message.twist.linear = linear_velocity_;

  dvl_twist_publisher_.publish(message);
}

//------------------------------------------------------------------------------
//
void ProviderDvlNode::FillFluidPressureMessage(ros::Time timestamp) {
  sensor_msgs::FluidPressure message;

  message.header.stamp = timestamp;
  message.header.frame_id = "/base_link";
  message.fluid_pressure = pressure_;

  dvl_fluid_pressure_publisher_.publish(message);
}

} // namespace provider_dvl
