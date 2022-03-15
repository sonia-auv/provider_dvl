#include "provider_dvl/provider_dvl_node.h"


//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
PathfinderDvl::PathfinderDvl(const ros::NodeHandlePtr & nh, std::string hostName, size_t pUDP, size_t pTCP, size_t dataSize)
: ProviderDvl(nh,hostName,pUDP, pTCP,dataSize)
{

//   ProviderDvl::nh() = nh;
//   ProviderDvl::portUDP() = pUDP;
//   ProviderDvl::portTCP() = pTCP;
//   ProviderDvl::rate() = rate;
//   ProviderDvl::hostName() = hostName;
  PathfinderDvl::Connect();
  PathfinderDvl::SetupROSCommunication();
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

  dvl_velocity_publisher_ = ProviderDvl::nh()->advertise<sonia_common::BodyVelocityDVL>( "/provider_dvl/dvl_velocity", 100);
  dvl_leak_sensor_publisher_ = ProviderDvl::nh()->advertise<std_msgs::Bool>("/provider_dvl/dvl_leak_sensor", 100);
  enableDisablePingSub = ProviderDvl::nh()->subscribe("/provider_dvl/enable_disable_ping", 100, &PathfinderDvl::EnableDisablePingCallback, this);
}

void PathfinderDvl::Connect() {
  socket().ConnectUDP(portUDP());
  socket().ConnectTCP(hostName(),  portTCP());
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
