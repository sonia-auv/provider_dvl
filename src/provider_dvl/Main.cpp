#include <provider_dvl/driver/Driver.hpp>
#include <ros/ros.h>
#include <provider_dvl/message_builder.hpp>
#include <fstream>
#include <provider_dvl/PD0Packet.h>
#include <provider_dvl/DeviceInfo.h>
#include <provider_dvl/AcquisitionConfiguration.h>
#include <provider_dvl/OutputConfiguration.h>
#include <provider_dvl/Status.h>
#include <provider_dvl/CellReadings.h>
#include <provider_dvl/BottomTrackingConfiguration.h>
#include <provider_dvl/BottomTracking.h>
#include <provider_dvl/CellReading.h>
#include <provider_dvl/Sensors.h>

using namespace dvl_teledyne;

void usage() { std::cerr << "dvl_teledyne_read DEVICE" << std::endl; }

int main(int argc, char *argv[]) {
  if (argc != 2) {
    usage();
    return 1;
  }


  ros::init(argc, argv, "provider_dvl");
  ros::NodeHandle n;

  ros::Publisher pd0_pub_;
  ros::Publisher acquisition_conf_pub_;
  ros::Publisher output_conf_pub_;
  ros::Publisher status_pub_;
  ros::Publisher cell_readings_pub_;
  ros::Publisher bottom_tracking_conf_pub_;
  ros::Publisher bottom_tracking_pub_;
  ros::Publisher twist_pub_;

  pd0_pub_ = n.advertise<provider_dvl::PD0Packet>("pd0_packet", 100);
  acquisition_conf_pub_ = n.advertise<provider_dvl::AcquisitionConfiguration>
      ("acquisition_conf", 100);
  output_conf_pub_ = n.advertise<provider_dvl::OutputConfiguration>
      ("output_conf",
       100);
  status_pub_ = n.advertise<provider_dvl::Status>("status",
                                                  100);
  cell_readings_pub_ = n.advertise<provider_dvl::CellReadings>
      ("cell_readings", 100);
  bottom_tracking_conf_pub_ =
      n.advertise<provider_dvl::BottomTrackingConfiguration>
          ("bottom_tracking_conf", 100);
  bottom_tracking_pub_ = n.advertise<provider_dvl::BottomTracking>
      ("bottom_tracking", 100);
  twist_pub_ = n.advertise<provider_dvl::BottomTracking>
      ("twist", 100);

  dvl_teledyne::Driver driver;
  driver.open(argv[1]);
  driver.setReadTimeout(base::Time::fromSeconds(5));
  driver.read();


  while (ros::ok()) {

    driver.read();
    provider_dvl::PD0Packet pd0Packet;
    provider_dvl::Sensors sensors;
    provider_dvl::DeviceInfo deviceInfo = provider_dvl::msg_builder::BuildDeviceInfo(driver.deviceInfo);
    provider_dvl::AcquisitionConfiguration acquisitionConfiguration =
        provider_dvl::msg_builder::BuildAcquisitionConfiguration(driver.acqConf);
    provider_dvl::OutputConfiguration outputConfiguration =
        provider_dvl::msg_builder::BuildOutputConfiguration(driver.outputConf);
    provider_dvl::Status status = provider_dvl::msg_builder::BuildStatus(driver.status);
    provider_dvl::CellReadings cellReadings =
        provider_dvl::msg_builder::BuildCellReadings(driver.cellReadings);
    provider_dvl::BottomTrackingConfiguration bottomTrackingConfiguration =
        provider_dvl::msg_builder::BuildBottomTrackingConfiguration(driver.bottomTrackingConf);
    provider_dvl::BottomTracking bottomTracking =
        provider_dvl::msg_builder::BuildBottomTracking(driver.bottomTracking);
    geometry_msgs::TwistWithCovarianceStamped twistWithCovarianceStamped =
        provider_dvl::msg_builder::BuildTwistWithCovariance(driver.bottomTracking);

    //  PD0Packet msg;
    pd0Packet.header.stamp = ros::Time::now();
    pd0Packet.device_info = deviceInfo;
    pd0Packet.acquisition_conf = acquisitionConfiguration;
    pd0Packet.output_conf = outputConfiguration;
    pd0Packet.status = status;
    pd0Packet.cell_readings = cellReadings;
    pd0Packet.bottom_tracking_conf = bottomTrackingConfiguration;
    pd0Packet.bottom_tracking = bottomTracking;

    pd0_pub_.publish(pd0Packet);
    acquisition_conf_pub_.publish(deviceInfo);
    output_conf_pub_.publish(outputConfiguration);
    status_pub_.publish(status);
    cell_readings_pub_.publish(cellReadings);
    bottom_tracking_conf_pub_.publish(bottomTrackingConfiguration);
    bottom_tracking_pub_.publish(bottomTracking);
    twist_pub_.publish(twistWithCovarianceStamped);
    ros::spinOnce();
  }
}
