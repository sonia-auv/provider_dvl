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
        std::string hostname = "192.168.0.240"; // To change
        socket_.Connect(hostname, 9002);

        //dvl_twist_publisher_ = nh_->advertise<geometry_msgs::TwistStamped>("/provider_dvl/dvl_twist", 1000);
        dvl_velocity_publisher_ = nh_->advertise<sonia_common::RelativeVelocityDVL>("/provider_dvl/dvl_velocity", 100);
        //dvl_fluid_pressure_publisher_ = nh_->advertise<sensor_msgs::FluidPressure>("/provider_dvl/dvl_pressure", 1000);
        //dvl_bottom_tracking_publisher_ = nh_->advertise<sonia_common::BottomTracking>("/provider_dvl/dvl_data", 1000);
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
                //if (calculateChecksum((uint32_t *) &dvl_data_) == dvl_data_.pd5.checksum)
                if(confirmChecksum(&dvl_data_))
                {
                    timestamp_ = ros::Time::now();
                    FillVelocityMessage(timestamp_);
                    //FillTwistMessage(timestamp_);
                    //FillFluidPressureMessage(timestamp_);
                    //FillBottomTracking(timestamp_);
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

    /*//------------------------------------------------------------------------------
    //
    void ProviderDvlNode::FillTwistMessage(ros::Time timestamp) {
        geometry_msgs::TwistStamped message;

        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";

        if(dvl_data_.data.status.bit_field.xVelValid == 0)
        {
            message.twist.linear.x = 0;
        }
        else
        {
            message.twist.linear.x = dvl_data_.data.velX;
        }
        if(dvl_data_.data.status.bit_field.yVelValid == 0)
        {
            message.twist.linear.y = 0;
        }
        else
        {
            message.twist.linear.y = dvl_data_.data.velY;
        }
        if(dvl_data_.data.status.bit_field.z1VelValid == 0)
        {
            message.twist.linear.z = 0;
        }
        else
        {
            message.twist.linear.z = dvl_data_.data.velZ1;
        }

        message.twist.angular.x = dvl_data_.data.timeVelEstX;
        message.twist.angular.y = dvl_data_.data.timeVelEstY;

        dvl_twist_publisher_.publish(message);
    }

    //------------------------------------------------------------------------------
    //
    void ProviderDvlNode::FillFluidPressureMessage(ros::Time timestamp)
    {
        sensor_msgs::FluidPressure message;

        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";
        message.fluid_pressure = dvl_data_.data.pressure;

        dvl_fluid_pressure_publisher_.publish(message);
    }

    void ProviderDvlNode::FillBottomTracking(ros::Time timestamp)
    {
        sonia_common::BottomTracking message;

        message.header.stamp = timestamp;
        message.header.frame_id = "/ENU";

        message.sync = dvl_data_.header.sync;
        message.hdrSize = dvl_data_.header.hdrSize;
        message.ID = dvl_data_.header.ID;
        message.family = dvl_data_.header.family;
        message.datasize = dvl_data_.header.dataSize;
        message.dataChecksum = dvl_data_.header.dataChecksum;
        message.hdrCecksum = dvl_data_.header.hdrChecksum;


        message.version = dvl_data_.data.version;
        message.offsetOfData = dvl_data_.data.offsetOfData;
        message.serial_number = dvl_data_.data.serialNumber;
        message.year = dvl_data_.data.year;
        message.month = dvl_data_.data.month;
        message.day = dvl_data_.data.day;
        message.hour = dvl_data_.data.hour;
        message.minutes = dvl_data_.data.minute;
        message.seconds = dvl_data_.data.seconds;
        message.microSeconds100 = dvl_data_.data.microSeconds100;
        message.nbBeams = dvl_data_.data.nBeams;
        message.error = dvl_data_.data.error;
        message.status = dvl_data_.data.status.integer;
        message.sound_speed = dvl_data_.data.soundSpeed;
        message.temperature  = dvl_data_.data.temperature;
        message.pressure  = dvl_data_.data.pressure;
        message.velBeam1  = dvl_data_.data.velBeam[0];
        message.velBeam2  = dvl_data_.data.velBeam[1];
        message.velBeam3  = dvl_data_.data.velBeam[2];
        message.velBeam4  = dvl_data_.data.velBeam[3];
        message.distBeam1  = dvl_data_.data.distBeam[0];
        message.distBeam2 = dvl_data_.data.distBeam[1];
        message.distBeam3 = dvl_data_.data.distBeam[2];
        message.distBeam4 = dvl_data_.data.distBeam[3];
        message.fomBeam1  = dvl_data_.data.fomBeam[0];
        message.fomBeam2  = dvl_data_.data.fomBeam[1];
        message.fomBeam3  = dvl_data_.data.fomBeam[2];
        message.fomBeam4  = dvl_data_.data.fomBeam[3];
        message.timeDiff1Beam1  = dvl_data_.data.timeDiff1Beam[0];
        message.timeDiff1Beam2  = dvl_data_.data.timeDiff1Beam[1];
        message.timeDiff1Beam3  = dvl_data_.data.timeDiff1Beam[2];
        message.timeDiff1Beam4  = dvl_data_.data.timeDiff1Beam[3];
        message.timeDiff2Beam1  = dvl_data_.data.timeDiff2Beam[0];
        message.timeDiff2Beam2  = dvl_data_.data.timeDiff2Beam[1];
        message.timeDiff2Beam3  = dvl_data_.data.timeDiff2Beam[2];
        message.timeDiff2Beam4  = dvl_data_.data.timeDiff2Beam[3];
        message.timeVelEstBeam1  = dvl_data_.data.timeVelEstBeam[0];
        message.timeVelEstBeam2  = dvl_data_.data.timeVelEstBeam[1];
        message.timeVelEstBeam3  = dvl_data_.data.timeVelEstBeam[2];
        message.timeVelEstBeam4  = dvl_data_.data.timeVelEstBeam[3];
        message.velX = dvl_data_.data.velX;
        message.velY = dvl_data_.data.velY;
        message.velZ1 = dvl_data_.data.velZ1;
        message.velZ2 = dvl_data_.data.velZ2;
        message.fomX = dvl_data_.data.fomX;
        message.fomY = dvl_data_.data.fomY;
        message.fomZ1 = dvl_data_.data.fomZ1;
        message.fomZ2 = dvl_data_.data.fomZ2;
        message.timeDiff1X = dvl_data_.data.timeDiff1X;
        message.timeDiff1Y = dvl_data_.data.timeDiff1Y;
        message.timeDiff1Z1 = dvl_data_.data.timeDiff1Z1;
        message.timeDiff1Z2 = dvl_data_.data.timeDiff1Z2;
        message.timeDiff2X = dvl_data_.data.timeDiff2X;
        message.timeDiff2Y = dvl_data_.data.timeDiff2Y;
        message.timeDiff2Z1 = dvl_data_.data.timeDiff2Z1;
        message.timeDiff2Z2 = dvl_data_.data.timeDiff2Z2;
        message.timeVelEstX = dvl_data_.data.timeVelEstX;
        message.timeVelEstY = dvl_data_.data.timeVelEstY;
        message.timeVelEstZ1 = dvl_data_.data.timeVelEstZ1;
        message.timeVelEstZ2 = dvl_data_.data.timeVelEstZ2;

        dvl_bottom_tracking_publisher_.publish(message);
    }*/

    uint16_t ProviderDvlNode::calculateChecksum(uint8_t *data)
    {
        uint16_t checksum = 0;

        for(uint8_t i=0; i < data.size()-2; ++i) //Removing checksum value from array
        {
            checksum += data[i];
        }
        return checksum;
    }

    bool ProviderDvlNode::confirmChecksum(DVLformat21_t *dvlData)
    {
        uint16_t calculatedChecksum = calculateChecksum(*(uint8_t) dvlData);
        return dvlData.pd5.checksum == calculatedChecksum;
    }

} // namespace provider_dvl
