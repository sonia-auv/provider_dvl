#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <string>
#include <vector>

#include <std_msgs/Bool.h>
#include <ros/node_handle.h>
#include <sonia_common/BodyVelocityDVL.h>
#include "../driver/ethernet_socket.h"
#include "dvl_data.h"
#include <thread>

// class ProviderDvlFactory {
//  public:
//   ProviderDvlFactory();
//   virtual PathfinderDvl* MakePathfinder(const ros::NodeHandlePtr& nh) const { return new PathfinderDvl(nh); }
// };

class ProviderDvl {
 public:
  // enum class frameId { "/NoTF" = 0, "/Instrument", "/Vessel", "/ENU", "/Error" };
  virtual void connect() = 0; // For TCP or Serial
  virtual void Spin()  {
    ros::Rate r(mRate);

    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }
  };

  virtual void setupROSCommunication() = 0;
  void SendReceivedMessageThread();
  void buildMessage(const std::vector<size_t> & outputDataFormat) 
  {
    
  };
  size_t & portUDP() { return mPortUDP;}
  size_t & portTCP() {return mPortTCP;}
  size_t & rate() {return mRate;}
  std::string & hostName() {return mHostName;}
  ros::NodeHandlePtr & nh() {return mNh;}

  std::thread mSendReceivedMessage;

 private:
    std::string mHostName;
    size_t mPortUDP{};
    size_t mPortTCP{};
    size_t mRate{};

    ros::NodeHandlePtr mNh;
};


class PathfinderDvl :  public ProviderDvl{
    public:
      //==========================================================================
      // P U B L I C   C / D T O R S
      PathfinderDvl() = default;
      PathfinderDvl(const ros::NodeHandlePtr& nh);
      PathfinderDvl(const ros::NodeHandlePtr& nh, size_t ,size_t, size_t);

     ~PathfinderDvl();
      void setupROSCommunication() override;

      //==========================================================================
      // P U B L I C   M E T H O D S
      void connect() override;
      void enableDisablePingCallback(const std_msgs::Bool& msg);

      private:
      EthernetSocket mSocket;
      DVLformat21_t mDvl_data;

      ros::Publisher dvl_velocity_publisher_;
      ros::Publisher dvl_position_publisher_;
      ros::Publisher dvl_leak_sensor_publisher_;
      
      ros::Subscriber enableDisablePingSub;
};



#endif //PATHFINDER_H
