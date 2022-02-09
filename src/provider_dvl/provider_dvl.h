#ifndef PROVIDER_DVL_H
#define PROVIDER_DVL_H

#include <string>
#include <vector>

#include <std_msgs/Bool.h>
#include <ros/node_handle.h>
#include <sonia_common/BodyVelocityDVL.h>
#include "../driver/ethernet_socket.h"
#include "dvl_data.h"
#include <thread>
#include <ros/ros.h>


enum class FrameId { NoTF, Instrument, Vessel, ENU, Error};

class ProviderDvl {
  public:
    //==========================================================================
    // P U B L I C   C / D T O R S
    ~ProviderDvl(){socket().~EthernetSocket();} // Close connection

    //==========================================================================
    // P U B L I C  V I R T U A L   M E T H O D S
    virtual void connect() = 0; // For TCP or Serial
    virtual void Spin()  {
      ros::Rate r(mRate);

      while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
      }
    };

    virtual void setupROSCommunication() = 0;
    //==========================================================================
    // P U B L I C   M E T H O D S

    void buildMessage(const std::vector<size_t> & outputDataFormat) 
    {
      
    };

    size_t & portUDP() { return mPortUDP;}
    size_t & portTCP() {return mPortTCP;}
    size_t & rate() {return mRate;}
    std::string & hostName() {return mHostName;}
    ros::NodeHandlePtr & nh() {return mNh;}
    EthernetSocket & socket() {return mSocket;}
    FrameId & frameId() {return mFrameId;}

  protected:
    void SendReceivedMessageThread();
    std::thread mSendReceivedMessage;
  private:
      FrameId mFrameId{};
      EthernetSocket mSocket;
      std::string mHostName;
      size_t mPortUDP{};
      size_t mPortTCP{};
      size_t mRate{};

      ros::NodeHandlePtr mNh;
  };

// template<type T>
// void ProviderDvl::updateMyClassValue<short int>(T value) {
//     objMyClass.var2 = value;
//}
  #endif // PROVIDER_DVL_H