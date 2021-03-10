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
//#include <geometry_msgs/TwistStamped.h>
//#include <sensor_msgs/FluidPressure.h>
#include <sonia_common/RelativeVelocityDVL.h>

#include "../driver/ethernet_socket.h"
#include "dvl_data.h"

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

    //void FillTwistMessage(ros::Time timestamp);
    void FillVelocityMessage(ros::Time timestamp);
    //void FillFluidPressureMessage(ros::Time timestamp);
    //void FillBottomTracking(ros::Time timestamp);
    uint16_t calculateChecksum(uint8_t *data);
    bool confirmChecksum(DVLformat21_t *pData);

    //==========================================================================
    // P R I V A T E   M E M B E R S

    ros::NodeHandlePtr nh_;
    EthernetSocket socket_;

    DVLformat21_t dvl_data_;

    ros::Time timestamp_;
    //ros::Publisher dvl_twist_publisher_;
    ros::Publisher dvl_velocity_publisher_;
    //ros::Publisher dvl_fluid_pressure_publisher_;
    //ros::Publisher dvl_bottom_tracking_publisher_;
};

} // namespace provider_dvl

#endif //PROVIDER_DVL_NODE_H
