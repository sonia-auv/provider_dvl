#include "provider_dvl/provider_dvl_node.h"


//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
PathfinderDvl::PathfinderDvl(const ros::NodeHandlePtr & nh, std::string hostName, size_t pUDP, size_t pTCP, size_t dataSize)
: ProviderDvl(nh,hostName,pUDP, pTCP,dataSize)
{
  Connect();
  SetupROSCommunication();
  setRate(20);
  mSendReceivedMessage = std::thread(std::bind(&PathfinderDvl::SendReceivedMessageThread, this));
}
//------------------------------------------------------------------------------
//

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//


void PathfinderDvl::SetupROSCommunication() {

  dvl_velocity_publisher_ = nh()->advertise<sonia_common::BodyVelocityDVL>( "/provider_dvl/dvl_velocity", 100);
  dvl_leak_sensor_publisher_ = nh()->advertise<std_msgs::Bool>("/provider_dvl/dvl_leak_sensor", 100);
  enableDisablePingSub = nh()->subscribe("/provider_dvl/enable_disable_ping", 100, &PathfinderDvl::EnableDisablePingCallback, this);
}

void PathfinderDvl::EnableDisablePingCallback(const std_msgs::Bool& msg)
{
    std::string str, cmd;
    
    if(msg.data == true)
    {   
        str = "===\n";
        socket().Send(&str[0]);
        ros::Duration(5).sleep();
        cmd = "CS\n";
        socket().Send(&cmd[0]);
        ROS_INFO("Start Ping Sent");
    }
    else if(msg.data == false)
    {
        str = "===\n";
        socket().Send(&str[0]);
        ROS_INFO("Stop Ping Sent");
    }
    else
    {
        ROS_WARN_STREAM("Message isn't boolean");
    }
}

void PathfinderDvl::SendReceivedMessageThread()
{
    ros::Rate r(rate()); // 20 Hz

    while(!ros::isShuttingDown())
    {
        mSocket.Receive();

        ROS_DEBUG("Data received");

        getData<DVLformat21_t>(mDvl_data);

        ROS_DEBUG("Data obtained");

        if(mDvl_data.pd4.pathfinderDataId == 0x7D)
        {
            ROS_DEBUG("ID correct");
            if(mDvl_data.pd4.checksum == CalculateChecksum<DVLformat21_t>(reinterpret_cast<uint8_t*>(mSocket.GetRawData())))
            {
                sonia_common::BodyVelocityDVL message;

                message.header.stamp.sec = mDvl_data.pd4.secondFirstPing;
                message.header.stamp.nsec = mDvl_data.pd4.hundredthFirstPing;
                message.header.frame_id = "/ENU"; //PD4

                message.xVelBtm = ((double_t)mDvl_data.pd4.xVelBtm)/1000.0;
                message.yVelBtm = ((double_t)mDvl_data.pd4.yVelBtm)/1000.0;
                message.zVelBtm = ((double_t)mDvl_data.pd4.zVelBtm)/1000.0;
                message.eVelBtm = ((double_t)mDvl_data.pd4.eVelBtm)/1000.0;

                message.velocity1 = ((double_t)mDvl_data.pd4.velocity1)/1000.0;
                message.velocity2 = ((double_t)mDvl_data.pd4.velocity2)/1000.0;
                message.velocity3 = ((double_t)mDvl_data.pd4.velocity3)/1000.0;
                message.velocity4 = ((double_t)mDvl_data.pd4.velocity4)/1000.0;

                dvl_velocity_publisher_.publish(message);
            }
        }
        else
        {
            ROS_INFO("Pathfinder ID mismatch : %d", 0x7D);
        }
        r.sleep();
    }
}



//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
NortekDvl::NortekDvl(const ros::NodeHandlePtr & nh, std::string hostName, size_t pTCP, size_t dataSize)
: ProviderDvl(nh,hostName,0, pTCP,dataSize)
{
  Connect();
  SetupROSCommunication();
  setRate(20);
  mSendReceivedMessage = std::thread(std::bind(&NortekDvl::SendReceivedMessageThread, this));
}
//------------------------------------------------------------------------------
//

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//

void NortekDvl::SetupROSCommunication() 
{
    dvl_speed_publisher_ = nh()->advertise<sonia_common::BodyVelocityDVL>("/provider_dvl/dvl_velocity", 100);
    dvl_fluid_pressure_publisher_ = nh()->advertise<sensor_msgs::FluidPressure>("/provider_dvl/dvl_pressure", 100);
    dvl_bottom_tracking_publisher_ = nh()->advertise<sonia_common::BottomTracking>("/provider_dvl/dvl_data", 100);
}

void NortekDvl::SendReceivedMessageThread()
{
    ros::Rate r(rate()); // 20 Hz

    while (ros::ok())
        {
            mSocket.Receive();

            ROS_DEBUG("Data received");

            getData<DVLformat21_t>(mDvl_data);

            ROS_DEBUG("Data obtained");

            // if (dvl_data_.header.sync == 0xA5)
            // {
            //     if (calculateChecksum((unsigned short *) &dvl_data_.data, dvl_data_.header.dataSize) == dvl_data_.header.dataChecksum)
            //     {
            //         timestamp_ = ros::Time::now();
            //         FillTwistMessage(timestamp_);
            //         FillFluidPressureMessage(timestamp_);
            //         FillBottomTracking(timestamp_);
            //     }
            // }
            r.sleep();
        }
}
