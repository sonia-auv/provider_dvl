#include "dvl_factory.h"
#include "provider_dvl_node.h"

std::string dvl_factory::dvlType_{""};
std::string dvl_factory::dvlIp_{""};
int dvl_factory::TCPPort_{};
int dvl_factory::UDPPort_{};


void dvl_factory::loadRosParams(ros::NodeHandlePtr &nh)
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

ProviderDvl* dvl_factory::createDvl(ros::NodeHandlePtr & nh)
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