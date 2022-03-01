#include "provider_dvl/provider_dvl.h"


ProviderDvl::ProviderDvl(const ros::NodeHandlePtr &nh, std::string hostName, size_t pUDP, size_t pTCP,  size_t rate, size_t dataSize)
: mNh(nh),
  mHostName(hostName),
  mPortUDP{pUDP},
  mPortTCP{pTCP},
  mRate{mRate},
  mFrameId{FrameId::ENU},
  mSocket(dataSize)
{
}




