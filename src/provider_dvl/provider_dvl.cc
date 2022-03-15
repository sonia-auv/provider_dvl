#include "provider_dvl/provider_dvl.h"


ProviderDvl::ProviderDvl(const ros::NodeHandlePtr &nh, std::string hostName, size_t pUDP, size_t pTCP, size_t dataSize)
: mSocket(dataSize),
  mNh(nh),
  mHostName(hostName),
  mPortUDP{pUDP},
  mPortTCP{pTCP}
{
    
}




