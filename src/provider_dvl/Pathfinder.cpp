#include "dvl_data.h"
#include "provider_dvl/provider_dvl_node.h"
#include "Pathfinder.h"


//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ProviderDvlNode::ProviderDvlNode(const ros::NodeHandlePtr &nh, size_t pUDP, size_t pTCP, size_t rate )
    : mNh(nh), mSocket()
{
  ProviderDvl::portUDP() = pUDP;
  ProviderDvl::portTCP() = pTCP;
  ProviderDvl::rate() = rate;
  PathfinderDvl::connect();
  PathfinderDvl::setupROSCommunication(nh);

  sendReceivedMessage =
      std::thread(std::bind(&ProviderDvlNode::SendReceivedMessageThread, this));
}

//------------------------------------------------------------------------------
//
ProviderDvlNode::~ProviderDvlNode() { socket_.~EthernetSocket(); }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//

void PathfinderDvl::setupROSCommunication(const ros::NodeHandlePtr &nh) {

  dvl_velocity_publisher_ = nh_->advertise<sonia_common::BodyVelocityDVL>(
      "/provider_dvl/dvl_velocity", 100);
  dvl_position_publisher_ = nh_->advertise<sonia_common::AttitudeDVL>(
      "/provider_dvl/dvl_attitude", 100);
  dvl_leak_sensor_publisher_ =
      nh_->advertise<std_msgs::Bool>("/provider_dvl/dvl_leak_sensor", 100);

  enableDisablePingSub =
      nh->subscribe("/provider_dvl/enable_disable_ping", 100,
                    &ProviderDvlNode::enableDisablePingCallback, this);
  setAnglesSub = nh->subscribe("/provider_dvl/set_angles", 100,
                               &ProviderDvlNode::setAnglesCallback, this);
  setDepthSub = nh->subscribe("/provider_dvl/set_depth", 100,
                              &ProviderDvlNode::setDepthCallback, this);

}

void PathfinderDvl::connect() {

  socket_.ConnectUDP(this->portUDP());
  socket_.ConnectTCP(this->hostName(), this->portTCP);

}
