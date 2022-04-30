#ifndef DVL_FACTORY_H
#define DVL_FACTORY_H
#include <string>
#include <ros/node_handle.h>
class ProviderDvl;
class dvl_factory 
{
    public:
      dvl_factory() = delete;
      ~dvl_factory() = default;
      
        static void loadRosParams(ros::NodeHandlePtr &nh);

        static ProviderDvl* createDvl(ros::NodeHandlePtr & nh);

    private:
      
      static std::string dvlType_;
      static std::string dvlIp_;
      static int TCPPort_;
      static int UDPPort_;
      
};
#endif // DVL_FACTORY_H
