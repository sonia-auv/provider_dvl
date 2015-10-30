#include <provider_dvl/driver/Driver.hpp>
#include <ros/ros.h>
#include <provider_dvl/PD0Packet.h>
#include <string>
#include <iostream>
#include <fstream>

using namespace dvl_teledyne;

void usage() { std::cerr << "dvl_teledyne_read DEVICE" << std::endl; }

int main(int argc, char *argv[]) {
  if (argc != 2) {
    usage();
    return 1;
  }

  ros::init(argc, argv, "provider_dvl");
  ros::NodeHandle n;
  ros::Publisher publisher =n.advertise<provider_dvl::PD0Packet>("provider_dvl_PD0Packet", 1000);
  dvl_teledyne::Driver driver;
  driver.open(argv[1]);
  driver.setReadTimeout(base::Time::fromSeconds(5));
  driver.read();

  char const *coord_systems[4] = {"BEAM", "INSTRUMENT", "SHIP", "EARTH"};
  std::cout << "Device outputs its data in the "
      << coord_systems[driver.outputConf.coordinate_system]
      << " coordinate system" << std::endl;

  std::cout << "Time Seq ";
  for (int beam = 0; beam < 4; ++beam)
    std::cout << " range[" << beam << "] velocity[" << beam << "] evaluation["
        << beam << "]";
  std::cout << std::endl;

  DeviceInfo const &info = driver.deviceInfo;
  std::cout << "sys config: " << info.system_configuration <<
      " fw version: " << int(info.fw_version) <<
      " fw revision: " << int(info.fw_revision) << std::endl;

  std::ofstream file;
  file.open("/sonia/DVL_LOG.txt");
  BottomTracking bottomTracking;
  while (ros::ok()) {
    driver.read();
    ros::spinOnce();
    bottomTracking = driver.bottomTracking;
    provider_dvl::PD0Packet packet;
    packet.timestamp = bottomTracking.time.microseconds;
    for( size_t i = 0; i < 4; i++)
    {
      packet.range[i] = bottomTracking.range[i];
      packet.velocity[i] = bottomTracking.velocity[i];
      packet.correlation[i] = bottomTracking.correlation[i];
      packet.evaluation[i] = bottomTracking.evaluation[i];
      packet.good_ping_ratio[i] = bottomTracking.good_ping_ratio[i];
      packet.rssi[i] = bottomTracking.rssi[i];
    }
    publisher.publish(packet);
  }
}
