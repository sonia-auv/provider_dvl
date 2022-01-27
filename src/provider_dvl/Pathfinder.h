#pragma once

#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <sonia_common/BodyVelocityDVL.h>
#include "../driver/ethernet_socket.h"
#include "dvl_data.h"

class ProviderDvlFactory {
 public:
  ProviderDvlFactory();
  virtual Pathfinder* MakePathfinder() const { return new Pathfinder; }
};


class PathfinderDvl : ProviderDvl {
    public:
      //==========================================================================
      // P U B L I C   C / D T O R S

      PathfinderDvl(const ros::NodeHandlePtr& nh);
     ~PathfinderDvl() = default;
      void setupROSCommunication() override;

      //==========================================================================
      // P U B L I C   M E T H O D S
      void connect() override;

      private:
      EthernetSocket mSocket;
      DVLformat21_t mDvl_data;
}

virtual class ProviderDvl {
 public:
  enum class frameId { "/NoTF" = 0, "/Instrument", "/Vessel", "/ENU", "/Error" };
  virtual void connect() = 0; // For TCP or Serial
  virtual void Spin() {
    ros::Rate r(mRrate);

    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }
  };

  virtual void setupROSCommunication() = 0;

  void buildMessage(const std::vector<size_t> & outputDataFormat) 
  {
    
  }
  size_t portUDP() { return &mPortUDP};
  size_t portTCP() {return &mPortTCP};
  size_t rate() {return &mRate};
  std::string hostName() const {return &mHostName};
 private:
    std::string mHostName;
  size_t mPortUDP{};
  size_t mPortTCP{};
  size_t mRate{};

  ros::NodeHandlePtr mNh;

     
};
