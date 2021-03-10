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
#include "dvl_data.h"

namespace provider_dvl {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
    ProviderDvlNode::ProviderDvlNode(const ros::NodeHandlePtr &nh) :
        nh_(nh),
        socket_()
    {
        std::string hostname = "192.168.1.100"; // To change
        socket_.Connect(hostname, 1033);

        dvl_velocity_publisher_ = nh_->advertise<sonia_common::RelativeVelocityDVL>("/provider_dvl/dvl_velocity", 100);
        dvl_position_publisher_ = nh_->advertise<sonia_common::PositionDVL>("/provider_dvl/dvl_position", 100);
        dvl_leak_sensor_publisher_ = nh_->advertise<std_msgs::Bool>("/provider_dvl/dvl_leak_sensor", 100);
    }

    //------------------------------------------------------------------------------
    //
    ProviderDvlNode::~ProviderDvlNode() {}

    //==============================================================================
    // M E T H O D   S E C T I O N

    //------------------------------------------------------------------------------
    //
    void ProviderDvlNode::Spin() {
        ros::Rate r(30);  // 30 hz

        while (ros::ok())
        {
            ros::spinOnce();

            socket_.Receive();

            dvl_data_ = *((DVLformat21_t*)(socket_.GetRawData()));

            if (dvl_data_.pd4.pathfinderDataId == 0x7D)
            {
                if(confirmChecksum(&dvl_data_))
                {
                    timestamp_ = ros::Time::now();
                    FillVelocityMessage(timestamp_);
                    FillPositionDVLMessage(timestamp_);
                    LeakSensorMessage();
                }
            }
            r.sleep();
        }
    }

    void ProviderDvlNode::FillVelocityMessage(ros::Time timestamp)
    {
        sonia_common::RelativeVelocityDVL message;

        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";

        message.xVelBtm = dvl_data_.pd4.xVelBtm;
        message.yVelBtm = dvl_data_.pd4.yVelBtm;
        message.zVelBtm = dvl_data_.pd4.zVelBtm;
        message.eVelBtm = dvl_data_.pd4.eVelBtm;

        dvl_velocity_publisher_.publish(message);
    }

    void ProviderDvlNode::FillPositionDVLMessage(ros::Time timestamp)
    {
        sonia_common::PositionDVL message;

        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";

        message.position.Z = dvl_data_.pd5.depth;
        message.position.ROLL = dvl_data_.pd5.roll;
        message.position.PITCH = dvl_data_.pd5.pitch;
        message.position.YAW = dvl_data_.pd5.heading;

        dvl_position_publisher_.publish(message);
    }

    void ProviderDvlNode::LeakSensorMessage()
    {
        std_msgs::Bool leakDetected;

        if(dvl_data_.pd4.statusLeakSensors == 0b01 || dvl_data_.pd4.statusLeakSensors == 0b0100 || dvl_data_.pd4.statusLeakSensors == 0b0101)
        {
            ROS_INFO("Leak detected in the pathfinder!!!!!!");
            leakDetected.data = true;
        }
        else
        {
            leakDetected.data = false;
        }

        dvl_leak_sensor_publisher_.publish(leakDetected);
    }

    uint16_t ProviderDvlNode::calculateChecksum(uint8_t *data)
    {
        float_t checksum = 0, wholeChecksum, decimal;

        for(uint8_t i=0; i < (sizeof(data)/sizeof(uint8_t))-2; ++i) //Removing checksum value from array
        {
            checksum += data[i];
        }
        
        wholeChecksum = ceil(checksum/65536);
        decimal = wholeChecksum-checksum/65536;
        checksum = (1-decimal)*65536;

        return (uint16_t)ceil(checksum);
    }

    bool ProviderDvlNode::confirmChecksum(DVLformat21_t *dvlData)
    {
        uint16_t calculatedChecksum = calculateChecksum((uint8_t*) &dvlData);
        return dvlData->pd5.checksum == calculatedChecksum;
    }

} // namespace provider_dvl
