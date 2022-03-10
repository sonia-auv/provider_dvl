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


class ProviderDvl {
  public:
    //==========================================================================
    // P U B L I C   C / D T O R S
    ProviderDvl() = delete;
    ProviderDvl(const ros::NodeHandlePtr &nh, std::string hostName, size_t pUDP, size_t pTCP,  size_t rate, size_t dataSize);
    ~ProviderDvl(){mSocket.~EthernetSocket();} // Close connection

    //==========================================================================
    // P U B L I C  V I R T U A L   M E T H O D S
    virtual void Connect() = 0; // For TCP or Serial
    virtual void Spin()  {
      ros::Rate r(mRate);

      while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
      }
    };

    virtual void SendReceivedMessageThread() = 0;
    virtual void SetupROSCommunication() = 0;
    //==========================================================================
    // P U B L I C   M E T H O D S

    size_t portUDP() const { return mPortUDP;}
    size_t portTCP() const {return mPortTCP;}
    size_t rate() const  {return mRate;}
    std::string hostName() const {return mHostName;}
    ros::NodeHandlePtr nh() const {return mNh;}
    EthernetSocket & socket() {return mSocket;}

  protected:
    std::thread mSendReceivedMessage;

    template<class T>
    inline void getData(T & x)
    {
      x = *((T*)(mSocket.GetRawData()));
    }

    template<class T>
    inline uint16_t CalculateChecksum(uint8_t *dataDVL)
    {
        float_t checksum{}, wholeChecksum, decimal;
        uint8_t sizeTotal {(sizeof(T)/sizeof(uint8_t))-2}; //Removing checksum value from array (-2)

        for(uint8_t i{}; i < sizeTotal; ++i)
        {
            checksum += dataDVL[i];
        }
        
        wholeChecksum = ceil(checksum/65536); // sizeof(uint16) = 2^16 = 65536
        decimal = wholeChecksum-checksum/65536;
        checksum = (1-decimal)*65536;

        return (uint16_t)ceil(checksum);
    }

      EthernetSocket mSocket;

  private:
      ros::NodeHandlePtr mNh;
      std::string mHostName;
      size_t mPortUDP;
      size_t mPortTCP;
      size_t mRate;
};
  #endif // PROVIDER_DVL_H