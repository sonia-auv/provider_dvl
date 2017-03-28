/**
 * \file	message_builder.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	16/12/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
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

#ifndef PROVIDER_DVL_MESSAGE_BUILDER_H_
#define PROVIDER_DVL_MESSAGE_BUILDER_H_

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <provider_dvl/msg/AcquisitionConfiguration.h>
#include <BottomTracking.h>
#include <BottomTrackingConfiguration.h>
#include <CellReading.h>
#include <CellReadings.h>
#include <DeviceInfo.h>
#include <OutputConfiguration.h>
#include <PD0Packet.h>
#include <Sensors.h>
#include <Status.h>
#include "provider_dvl/driver/PD0Messages.hpp"

namespace provider_dvl {

/**
 * This namespace regroup some functions that will build the ROS message with
 * a given pd0 packet.
 *
 * Each of the functions will take a a struct and will return a ROS message (
 * that can be either a standard message or a message from provider_dvl)
 */
namespace msg_builder {

// PD0Packet BuildPD0(const pd0_msg::PD0Packet &pd0_packet);

msg::Sensors BuildSensors(const PD0Message::Sensors &sensors);

msg::CellReading BuildCellReading(
    const PD0Message::CellReading &reading);

msg::DeviceInfo BuildDeviceInfo(
    const PD0Message::DeviceInfo &device_info);

msg::AcquisitionConfiguration BuildAcquisitionConfiguration(
    const PD0Message::AcquisitionConfiguration &acquisition_conf);

msg::OutputConfiguration BuildOutputConfiguration(
    const PD0Message::OutputConfiguration &output_conf);

msg::Status BuildStatus(const PD0Message::Status &status);

msg::CellReadings BuildCellReadings(
    const PD0Message::CellReadings &cell_readings);

msg::BottomTrackingConfiguration BuildBottomTrackingConfiguration(
    const PD0Message::BottomTrackingConfiguration &bottom_tracking_conf);

msg::BottomTracking BuildBottomTracking(
    const PD0Message::BottomTracking &bottom_tracking);

geometry_msgs::TwistWithCovarianceStamped BuildTwistWithCovariance(
    const PD0Message::BottomTracking &bottom_tracking);

}  // namespace msg_builder

}  // namespace provider_dvl

#endif  // PROVIDER_DVL_MESSAGE_BUILDER_H_
