#include <ros/ros.h>
#include <sonia_msgs/AcquisitionConfiguration.h>
#include <sonia_msgs/BottomTracking.h>
#include <sonia_msgs/BottomTrackingConfiguration.h>
#include <sonia_msgs/CellReading.h>
#include <sonia_msgs/CellReadings.h>
#include <sonia_msgs/DeviceInfo.h>
#include <sonia_msgs/OutputConfiguration.h>
#include <sonia_msgs/PD0Packet.h>
#include <fstream>
#include <provider_dvl/driver/Driver.hpp>
#include <provider_dvl/message_builder.hpp>
#include <provider_dvl/base/Float.hpp>

using namespace dvl_teledyne;

int main(int argc, char *argv[]) {
  std::string node_name("provider_dvl");

  ros::init(argc, argv, node_name);
  ros::NodeHandlePtr n(new ros::NodeHandle("~"));

  ros::Publisher pd0_pub_;
  ros::Publisher acquisition_conf_pub_;
  ros::Publisher output_conf_pub_;
  ros::Publisher status_pub_;
  ros::Publisher cell_readings_pub_;
  ros::Publisher bottom_tracking_conf_pub_;
  ros::Publisher bottom_tracking_pub_;
  ros::Publisher twist_pub_;

  twist_pub_ =
      n->advertise<geometry_msgs::TwistWithCovarianceStamped>("twist", 100);
  pd0_pub_ = n->advertise<sonia_msgs::PD0Packet>("pd0_packet", 100);
  acquisition_conf_pub_ = n->advertise<sonia_msgs::AcquisitionConfiguration>(
      "acquisition_conf", 100);
  output_conf_pub_ =
      n->advertise<sonia_msgs::OutputConfiguration>("output_conf", 100);
  status_pub_ = n->advertise<sonia_msgs::Status>("status", 100);
  cell_readings_pub_ =
      n->advertise<sonia_msgs::CellReadings>("cell_readings", 100);
  bottom_tracking_conf_pub_ =
      n->advertise<sonia_msgs::BottomTrackingConfiguration>(
          "bottom_tracking_conf", 100);
  bottom_tracking_pub_ =
      n->advertise<sonia_msgs::BottomTracking>("bottom_tracking", 100);

  dvl_teledyne::Driver driver(n);

  if (argc < 2) {
    ROS_WARN(
        "No device specified. Using default value:    "
        "serial:///dev/ttyS0:115200");
    driver.open("serial:///dev/ttyS0:115200");
  } else {
    driver.open(argv[1]);
  }

  driver.setReadTimeout(base::Time::fromSeconds(5));
  driver.read();
  driver.PrintDeviceInfos();

  while (ros::ok()) {
    driver.read();
    sonia_msgs::PD0Packet pd0Packet;
    sonia_msgs::Sensors sensors;
    sonia_msgs::DeviceInfo deviceInfo =
        provider_dvl::msg_builder::BuildDeviceInfo(driver.deviceInfo);
    sonia_msgs::AcquisitionConfiguration acquisitionConfiguration =
        provider_dvl::msg_builder::BuildAcquisitionConfiguration(
            driver.acqConf);
    sonia_msgs::OutputConfiguration outputConfiguration =
        provider_dvl::msg_builder::BuildOutputConfiguration(driver.outputConf);
    sonia_msgs::Status status =
        provider_dvl::msg_builder::BuildStatus(driver.status);
    sonia_msgs::CellReadings cellReadings =
        provider_dvl::msg_builder::BuildCellReadings(driver.cellReadings);
    sonia_msgs::BottomTrackingConfiguration bottomTrackingConfiguration =
        provider_dvl::msg_builder::BuildBottomTrackingConfiguration(
            driver.bottomTrackingConf);
    sonia_msgs::BottomTracking bottomTracking =
        provider_dvl::msg_builder::BuildBottomTracking(driver.bottomTracking);
    geometry_msgs::TwistWithCovarianceStamped twistWithCovarianceStamped =
        provider_dvl::msg_builder::BuildTwistWithCovariance(
            driver.bottomTracking);

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

    acquisition_conf_pub_.publish(acquisitionConfiguration);
    output_conf_pub_.publish(outputConfiguration);
    status_pub_.publish(status);
    cell_readings_pub_.publish(cellReadings);
    bottom_tracking_conf_pub_.publish(bottomTrackingConfiguration);
    bottom_tracking_pub_.publish(bottomTracking);

    // test if a number is different from NaN. A NaN number is not equal to
    // itself.
    if (twistWithCovarianceStamped.twist.twist.linear.x ==
            twistWithCovarianceStamped.twist.twist.linear.x &&
        twistWithCovarianceStamped.twist.twist.linear.y ==
            twistWithCovarianceStamped.twist.twist.linear.y &&
        twistWithCovarianceStamped.twist.twist.linear.z ==
            twistWithCovarianceStamped.twist.twist.linear.z)
      twist_pub_.publish(twistWithCovarianceStamped);

    ros::spinOnce();
  }
}
