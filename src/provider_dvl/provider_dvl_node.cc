#include "provider_dvl/provider_dvl_node.h"


//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
PathfinderDvl::PathfinderDvl(const ros::NodeHandlePtr & nh, std::string hostName, size_t pUDP, size_t pTCP,  size_t rate, size_t dataSize)
: ProviderDvl(nh,hostName,pUDP, pTCP, rate,dataSize)
{

//   ProviderDvl::nh() = nh;
//   ProviderDvl::portUDP() = pUDP;
//   ProviderDvl::portTCP() = pTCP;
//   ProviderDvl::rate() = rate;
//   ProviderDvl::hostName() = hostName;
  PathfinderDvl::connect();
  PathfinderDvl::setupROSCommunication();
  //mSendReceivedMessage = std::thread(std::bind(&PathfinderDvl::SendReceivedMessageThread, this));
}
//------------------------------------------------------------------------------
//
PathfinderDvl::~PathfinderDvl() { }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//


void PathfinderDvl::setupROSCommunication() {

  dvl_velocity_publisher_ = ProviderDvl::nh()->advertise<sonia_common::BodyVelocityDVL>( "/provider_dvl/dvl_velocity", 100);
  dvl_leak_sensor_publisher_ = ProviderDvl::nh()->advertise<std_msgs::Bool>("/provider_dvl/dvl_leak_sensor", 100);
  enableDisablePingSub = ProviderDvl::nh()->subscribe("/provider_dvl/enable_disable_ping", 100, &PathfinderDvl::enableDisablePingCallback, this);
}

void PathfinderDvl::connect() {
  socket().ConnectUDP(portUDP());
  socket().ConnectTCP(hostName(),  portTCP());
}

void PathfinderDvl::enableDisablePingCallback(const std_msgs::Bool& msg)
{
    std::string str, cmd;
    
    if(msg.data == true)
    {   
        str = "===\n";
        socket().Send(&str[0]);
        ros::Duration(5).sleep();
        cmd = "CS\n";
        socket().Send(&cmd[0]);
    }
    else if(msg.data == false)
    {
        str = "===\n";
        socket().Send(&str[0]);
    }
    else
    {
        ROS_WARN_STREAM("Message isn't boolean");
    }
}
