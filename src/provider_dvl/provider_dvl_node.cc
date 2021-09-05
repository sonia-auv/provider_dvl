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
        std::string hostname = "192.168.0.32";
        socket_.ConnectUDP(1034);
        socket_.ConnectTCP(hostname, 1033);

        dvl_velocity_publisher_ = nh_->advertise<sonia_common::BodyVelocityDVL>("/provider_dvl/dvl_velocity", 100);
        dvl_position_publisher_ = nh_->advertise<sonia_common::AttitudeDVL>("/provider_dvl/dvl_attitude", 100);
        dvl_leak_sensor_publisher_ = nh_->advertise<std_msgs::Bool>("/provider_dvl/dvl_leak_sensor", 100);

        enableDisablePingSub = nh->subscribe("/provider_dvl/enable_disable_ping", 100, &ProviderDvlNode::enableDisablePingCallback, this);
        setAnglesSub = nh->subscribe("/provider_dvl/set_angles", 100, &ProviderDvlNode::setAnglesCallback, this);
        setDepthSub = nh->subscribe("/provider_dvl/set_depth", 100, &ProviderDvlNode::setDepthCallback, this);
    }

    //------------------------------------------------------------------------------
    //
    ProviderDvlNode::~ProviderDvlNode() 
    {
        socket_.~EthernetSocket();
    }

    //==============================================================================
    // M E T H O D   S E C T I O N

    //------------------------------------------------------------------------------
    //
    void ProviderDvlNode::Spin() {
        ros::Rate r(20);  // 20 hz

        while (ros::ok())
        {
            ros::spinOnce();

            socket_.Receive();

            ROS_DEBUG("Data received");

            dvl_data_ = *((DVLformat21_t*)(socket_.GetRawData()));

            ROS_DEBUG("Data obtained");
            
            if (dvl_data_.pd4.pathfinderDataId == 0x7D)
            {
                ROS_DEBUG("ID correct");
                if(confirmChecksum((uint8_t *)&dvl_data_))
                {
                    ROS_DEBUG("Checksum passed");
                    timestamp_ = ros::Time::now();
                    FillVelocityMessage(timestamp_);
                    FillAttitudeDVLMessage(timestamp_);
                    LeakSensorMessage();
                }
                else
                {
                    ROS_INFO_STREAM("Checksum failed");
                }
            }
            else
            {
                ROS_INFO("Pathfinder ID didn't egal in the data obtained : %d", 0x7D);
            }
            r.sleep();
        }
    }

    void ProviderDvlNode::FillVelocityMessage(ros::Time timestamp)
    {
        sonia_common::BodyVelocityDVL message;

        message.header.stamp = timestamp;
        message.header.frame_id = verifyFrameId(dvl_data_.pd4.systemConfig);

        message.xVelBtm = ((double_t)dvl_data_.pd4.xVelBtm)/1000.0;
        message.yVelBtm = ((double_t)dvl_data_.pd4.yVelBtm)/1000.0;
        message.zVelBtm = ((double_t)dvl_data_.pd4.zVelBtm)/1000.0;
        message.eVelBtm = ((double_t)dvl_data_.pd4.eVelBtm)/1000.0;

        message.velocity1 = ((double_t)dvl_data_.pd4.velocity1)/1000.0;
        message.velocity2 = ((double_t)dvl_data_.pd4.velocity2)/1000.0;
        message.velocity3 = ((double_t)dvl_data_.pd4.velocity3)/1000.0;
        message.velocity4 = ((double_t)dvl_data_.pd4.velocity4)/1000.0;

        dvl_velocity_publisher_.publish(message);
    }

    void ProviderDvlNode::FillAttitudeDVLMessage(ros::Time timestamp)
    {
        sonia_common::AttitudeDVL message;

        message.header.stamp = timestamp;
        message.header.frame_id = verifyFrameId(dvl_data_.pd4.systemConfig);
        // DEPTH
        if(dvl_data_.pd5.depth >= 9999)
            message.position.Z = 9999.0/10.0;
        else if(dvl_data_.pd5.depth <= 1)
            message.position.Z = 1.0/10.0;
        else
            message.position.Z = dvl_data_.pd5.depth/10.0;
        // ROLL
        if(dvl_data_.pd5.roll*ANGLE_LSD >= 20.0)
            message.position.ROLL = 20.0;
        else if(dvl_data_.pd5.roll*ANGLE_LSD <= -20.0)
            message.position.ROLL = -20.0;
        else
            message.position.ROLL = dvl_data_.pd5.roll*ANGLE_LSD;
        // PITCH
        if(dvl_data_.pd5.pitch*ANGLE_LSD >= 20.0)
            message.position.PITCH = 20.0;
        else if(dvl_data_.pd5.pitch*ANGLE_LSD <= -20.0)
            message.position.PITCH = -20.0;
        else
            message.position.PITCH = dvl_data_.pd5.pitch*ANGLE_LSD;
        // YAW
        if(dvl_data_.pd5.heading*ANGLE_LSD >= 360.0)
            message.position.YAW = 360.0;
        else if(dvl_data_.pd5.heading*ANGLE_LSD <= 0.0)
            message.position.YAW = 0.0;
        else
            message.position.YAW = dvl_data_.pd5.heading*ANGLE_LSD;

        dvl_position_publisher_.publish(message);
    }

    void ProviderDvlNode::LeakSensorMessage()
    {
        std_msgs::Bool leakDetected;

        if(dvl_data_.pd4.statusLeakSensors == 0b01 || dvl_data_.pd4.statusLeakSensors == 0b0100 || dvl_data_.pd4.statusLeakSensors == 0b0101)
        {
            ROS_INFO_STREAM("Leak detected in the pathfinder!!!!!!");
            leakDetected.data = true;
        }
        else
        {
            leakDetected.data = false;
        }

        dvl_leak_sensor_publisher_.publish(leakDetected);
    }

    uint16_t ProviderDvlNode::calculateChecksum(uint8_t *dataDVL)
    {
        float_t checksum = 0, wholeChecksum, decimal;
        uint8_t sizeTotal = (sizeof(DVLformat21_t)/sizeof(uint8_t))-2; //Removing checksum value from array (-2)

        for(uint8_t i=0; i < sizeTotal; ++i)
        {
            checksum += dataDVL[i];
        }
        
        wholeChecksum = ceil(checksum/65536);
        decimal = wholeChecksum-checksum/65536;
        checksum = (1-decimal)*65536;

        return (uint16_t)ceil(checksum);
    }

    bool ProviderDvlNode::confirmChecksum(uint8_t *dvlData)
    {
        uint16_t calculatedChecksum = calculateChecksum(dvlData);
        return dvl_data_.pd5.checksum == calculatedChecksum;
    } 

    std::string ProviderDvlNode::verifyFrameId(uint8_t systemId)
    {
        uint8_t frameId = (systemId) >> 6;
        switch (frameId)
        {
        case 0:
            return "/NoTF";
            break;
        case 1:
            return "/Instrument";
            break;
        case 2:
            return "/Vessel";
            break;
        case 3:
            return "/ENU";
            break;
        default:
            return "/Error";
            break;
        }
    }


    void ProviderDvlNode::enableDisablePingCallback(const std_msgs::Bool& msg)
    {
        std::string str;
        
        if(msg.data == true)
        {   
            str = "CS";
            socket_.Send(&str[0]);
        }
        else if(msg.data == false)
        {
            str = "===";
            socket_.Send(&str[0]);
        }
        else
        {
            ROS_WARN_STREAM("Message isn't boolean");
        }
    }

    void ProviderDvlNode::setAnglesCallback(const geometry_msgs::Vector3& msg)
    {
        // Do something
    }

    void ProviderDvlNode::setDepthCallback(const std_msgs::Float64& msg)
    {
        // Do something
    }
} // namespace provider_dvl
