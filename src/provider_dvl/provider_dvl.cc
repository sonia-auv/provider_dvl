#include "provider_dvl/provider_dvl.h"

const float ProviderDvl::dBAR_TO_METER_OF_WATER{10.1972f};

ProviderDvl::ProviderDvl(const ros::NodeHandlePtr &nh, std::string hostName, size_t pUDP, size_t pTCP, size_t dataSize)
: mSocket(dataSize),
  mNh(nh),
  mHostName(hostName),
  mPortUDP{pUDP},
  mPortTCP{pTCP}
{
    
}

void ProviderDvl::Connect() {

    if(portUDP())
    {
        socket().ConnectUDP(portUDP());
    }
    if(portTCP()) // verif host name todo
    {
        socket().ConnectTCP(hostName(),  portTCP());
    }
}

float ProviderDvl::convertDBarToMeters(float dBarValue)
{
    return dBarValue / dBAR_TO_METER_OF_WATER;
}




