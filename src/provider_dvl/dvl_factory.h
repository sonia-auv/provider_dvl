#include <string>
#include "provider_dvl_node.h"
class ProviderDvl;
class dvl_factory 
{
    public:
      dvl_factory() = delete;
      ~dvl_factory() = default;
      
     static void loadRosParams(ros::NodeHandlePtr &nh)
     {
        if (nh->hasParam("/Provider_dvl/dvl_type")) {
            nh->getParam("/Provider_dvl/dvl_type", dvlType_);
        }
        if (nh->hasParam("/Provider_dvl/ip")) {
            nh->getParam("/Provider_dvl/ip", dvlIp_);
        }
        if (nh->hasParam("/Provider_dvl/TCPPort")) {
            nh->getParam("/Provider_dvl/TCPPort", TCPPort_);
        }
        if (nh->hasParam("/Provider_dvl/UDPPort")) {
            nh->getParam("/Provider_dvl/UDPPort", UDPPort_);
        }
    };

    static ProviderDvl* createDvl(const ros::NodeHandlePtr & nh)
    {
        if(dvlType_ == "Pathfinder")
        {
            return new PathfinderDvl(nh,dvlIp_, UDPPort_, TCPPort_);
        }
        else if(dvlType_ == "Nortek")
        {
            return new NortekDvl(nh,dvlIp_,UDPPort_, TCPPort_);
        }
        return nullptr;
    }

      private:
      
      static std::string dvlType_;
      static std::string dvlIp_;
      static int TCPPort_;
      static int UDPPort_;
      
};
