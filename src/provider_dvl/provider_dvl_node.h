#ifndef PROVIDER_DVL_NODE_H
#define PROVIDER_DVL_NODE_H

#include <string>
#include <vector>

#include <std_msgs/Bool.h>
#include <ros/node_handle.h>
#include <sonia_common/BodyVelocityDVL.h>

#include "dvl_data.h"
#include <thread>
#include <ros/ros.h>

#include "provider_dvl/provider_dvl.h"

// class ProviderDvlFactory {
//  public:
//   ProviderDvlFactory();
//   virtual PathfinderDvl* MakePathfinder(const ros::NodeHandlePtr& nh) const { return new PathfinderDvl(nh); }
// };


class PathfinderDvl :  public ProviderDvl{
    public:
      //==========================================================================
      // P U B L I C   C / D T O R S
      PathfinderDvl() = delete;
      PathfinderDvl(const ros::NodeHandlePtr& nh);
      PathfinderDvl(const ros::NodeHandlePtr &nh, std::string hostName, size_t pUDP, size_t pTCP, size_t dataSize);
      ~PathfinderDvl() = default;


      void Spin() override  {
      ros::Rate r(rate());

      while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
      }
    };

      void SetupROSCommunication() override;
      inline void SendReceivedMessageThread() override;
      //==========================================================================
      // P U B L I C   M E T H O D S
      void Connect() override;
      void EnableDisablePingCallback(const std_msgs::Bool& msg);

      private:
      
      DVLformat21_t mDvl_data;

      ros::Publisher dvl_velocity_publisher_;
      ros::Publisher dvl_leak_sensor_publisher_;
      
      ros::Subscriber enableDisablePingSub;
};


#endif //PROVIDER_DVL_NODE_H
