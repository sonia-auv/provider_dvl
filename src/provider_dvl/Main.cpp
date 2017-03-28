/**
 * \file	observer.h
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

#include <ros/ros.h>
#include <AcquisitionConfiguration.h>
#include <BottomTracking.h>
#include <BottomTrackingConfiguration.h>
#include <CellReading.h>
#include <CellReadings.h>
#include <DeviceInfo.h>
#include <OutputConfiguration.h>
#include <PD0Packet.h>
#include <fstream>
#include <provider_dvl/driver/Driver.hpp>
#include <provider_dvl/message_builder.hpp>
#include <provider_dvl/base/Float.hpp>

using namespace dvl_teledyne;

int main(int argc, char *argv[]) {
  std::string node_name("provider_dvl");

  ros::init(argc, argv, node_name);
  ros::NodeHandlePtr n(new ros::NodeHandle("~"));

  ros::Publisher pd0_pub_;
  ros::Publisher acquisition_conf_pub_;
  ros::Publisher output_conf_pub_;
  ros::Publisher status_pub_;
  ros::Publisher cell_readings_pub_;
  ros::Publisher bottom_tracking_conf_pub_;
  ros::Publisher bottom_tracking_pub_;
  ros::Publisher twist_pub_;

  twist_pub_ = n->advertise<geometry_msgs::TwistWithCovarianceStamped>("twist", 100);
  pd0_pub_ = n->advertise<msg::PD0Packet>("pd0_packet", 100);
  acquisition_conf_pub_ = n->advertise<msg::AcquisitionConfiguration>("acquisition_conf", 100);
  output_conf_pub_ = n->advertise<msg::OutputConfiguration>("output_conf", 100);
  status_pub_ = n->advertise<msg::Status>("status", 100);
  cell_readings_pub_ = n->advertise<msg::CellReadings>("cell_readings", 100);
  bottom_tracking_conf_pub_ = n->advertise<msg::BottomTrackingConfiguration>("bottom_tracking_conf", 100);
  bottom_tracking_pub_ =n->advertise<msg::BottomTracking>("bottom_tracking", 100);

  dvl_teledyne::Driver driver(n);

  if (argc < 2) {
    ROS_INFO(
        "No device specified. Using default value:    "
        "serial:///dev/ttyS0:115200");
    driver.open("serial:///dev/ttyS0:115200");
  } else {
    driver.open(argv[1]);
  }

  driver.setReadTimeout(base::Time::fromSeconds(5));
  driver.read();
  driver.PrintDeviceInfos();

  while (ros::ok()) {
    driver.read();
    msg::PD0Packet pd0Packet;
    msg::Sensors sensors;
    msg::DeviceInfo deviceInfo =
        provider_dvl::msg_builder::BuildDeviceInfo(driver.deviceInfo);
    msg::AcquisitionConfiguration acquisitionConfiguration =
        provider_dvl::msg_builder::BuildAcquisitionConfiguration(
            driver.acqConf);
    msg::OutputConfiguration outputConfiguration =
        provider_dvl::msg_builder::BuildOutputConfiguration(driver.outputConf);
    msg::Status status =
        provider_dvl::msg_builder::BuildStatus(driver.status);
    msg::CellReadings cellReadings =
        provider_dvl::msg_builder::BuildCellReadings(driver.cellReadings);
    msg::BottomTrackingConfiguration bottomTrackingConfiguration =
        provider_dvl::msg_builder::BuildBottomTrackingConfiguration(
            driver.bottomTrackingConf);
    msg::BottomTracking bottomTracking =
        provider_dvl::msg_builder::BuildBottomTracking(driver.bottomTracking);
    geometry_msgs::TwistWithCovarianceStamped twistWithCovarianceStamped =
        provider_dvl::msg_builder::BuildTwistWithCovariance(
            driver.bottomTracking);

    //  PD0Packet msg;
    pd0Packet.header.stamp = ros::Time::now();
    pd0Packet.device_info = deviceInfo;
    pd0Packet.acquisition_conf = acquisitionConfiguration;
    pd0Packet.output_conf = outputConfiguration;
    pd0Packet.status = status;
    pd0Packet.cell_readings = cellReadings;
    pd0Packet.bottom_tracking_conf = bottomTrackingConfiguration;
    pd0Packet.bottom_tracking = bottomTracking;

    pd0_pub_.publish(pd0Packet);

    acquisition_conf_pub_.publish(acquisitionConfiguration);
    output_conf_pub_.publish(outputConfiguration);
    status_pub_.publish(status);
    cell_readings_pub_.publish(cellReadings);
    bottom_tracking_conf_pub_.publish(bottomTrackingConfiguration);
    bottom_tracking_pub_.publish(bottomTracking);

    // test if a number is different from NaN. A NaN number is not equal to
    // itself.
    if (twistWithCovarianceStamped.twist.twist.linear.x ==
            twistWithCovarianceStamped.twist.twist.linear.x &&
        twistWithCovarianceStamped.twist.twist.linear.y ==
            twistWithCovarianceStamped.twist.twist.linear.y &&
        twistWithCovarianceStamped.twist.twist.linear.z ==
            twistWithCovarianceStamped.twist.twist.linear.z)
      twist_pub_.publish(twistWithCovarianceStamped);

    ros::spinOnce();
  }
}
