#include "provider_dvl/provider_dvl.h"


void ProviderDvl::SendReceivedMessageThread()
{
    ros::Rate r(mRate); 

    while(!ros::isShuttingDown())
    {
        if(mSocket.Receive()) ROS_DEBUG("Data received");
        
        *((DVLformat21_t*)(mSocket.GetRawData()));
        r.sleep();
    }
}