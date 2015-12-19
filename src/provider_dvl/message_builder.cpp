/**
 * \file	message_builder.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	16/12/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "message_builder.hpp"

namespace provider_dvl {

namespace msg_builder {

//------------------------------------------------------------------------------
//
//PD0Packet BuildPD0(const PD0Message::PD0Packet &pd0_packet) {
//  PD0Packet msg;
//  msg.header.stamp = ros::Time::now();
//
//  msg.device_info = BuildDeviceInfo(pd0_packet.deviceInfo);
//  msg.acquisition_conf = BuildAcquisitionConfiguration(pd0_packet.acqConf);
//  msg.output_conf = BuildOutputConfiguration(pd0_packet.outputConf);
//  msg.status = BuildStatus(pd0_packet.status);
//  msg.cell_readings = BuildCellReadings(pd0_packet.cellReadings);
//  msg.bottom_tracking_conf = BuildBottomTrackingConfiguration(pd0_packet
//                                                                  .bottomTrackingConf);
//  msg.bottom_tracking = BuildBottomTracking(pd0_packet.bottomTracking);
//
//  return msg;
//}

//------------------------------------------------------------------------------
//
Sensors BuildSensors(const PD0Message::Sensors &sensors) {
  Sensors msg;
  msg.header.stamp = ros::Time::now();

  msg.calculates_speed_of_sound = sensors.calculates_speed_of_sound;
  msg.depth = sensors.depth;
  msg.yaw = sensors.yaw;
  msg.pitch = sensors.pitch;
  msg.roll = sensors.roll;
  msg.salinity = sensors.salinity;
  msg.temperature = sensors.temperature;

  return msg;
}

//------------------------------------------------------------------------------
//
CellReading BuildCellReading(const PD0Message::CellReading &reading) {
  CellReading msg;
  msg.header.stamp = ros::Time::now();

  for(int i = 0; i < 4; ++i) {
    msg.velocity[i] = reading.velocity[i];
    msg.correlation[i] = reading.correlation[i];
    msg.intensity[i] = reading.intensity[i];
    msg.quality[i] = reading.quality[i];
  }

  return msg;
}

//------------------------------------------------------------------------------
//
DeviceInfo BuildDeviceInfo(const PD0Message::DeviceInfo &device_info) {
  DeviceInfo msg;
  msg.header.stamp = ros::Time::now();

  msg.fw_version = device_info.fw_version;
  msg.fw_revision = device_info.fw_version;
  msg.cpu_board_serno = device_info.cpu_board_serno;
  msg.system_configuration = device_info.system_configuration;
  msg.beam_count = device_info.beam_count;
  msg.available_sensors = BuildSensors(device_info.available_sensors);

  return msg;
}

//------------------------------------------------------------------------------
//
AcquisitionConfiguration BuildAcquisitionConfiguration(const PD0Message::AcquisitionConfiguration
                                                       &acquisition_conf) {
  AcquisitionConfiguration msg;
  msg.header.stamp = ros::Time::now();

  msg.used_sensors = BuildSensors(acquisition_conf.used_sensors);
  msg.lag_duration = acquisition_conf.lag_duration;
  msg.cell_count = acquisition_conf.cell_count;
  msg.profiling_mode = acquisition_conf.profiling_mode;
  msg.low_correlation_threshold = acquisition_conf.low_correlation_threshold;
  msg.code_repetition_count = acquisition_conf.code_repetition_count;
  msg.pings_per_ensemble = acquisition_conf.pings_per_ensemble;
  msg.cell_length = acquisition_conf.cell_length;
  msg.blank_after_transmit_distance =
      acquisition_conf.blank_after_transmit_distance;
  msg.water_layer_min_ping_threshold =
      acquisition_conf.water_layer_min_ping_threshold;
  msg.water_layer_velocity_threshold =
      acquisition_conf.water_layer_velocity_threshold;
  msg.time_between_ping_groups = acquisition_conf.time_between_ping_groups.Microseconds;
  msg.yaw_alignment = acquisition_conf.yaw_alignment;
  msg.yaw_bias = acquisition_conf.yaw_bias;
  msg.first_cell_distance = acquisition_conf.first_cell_distance;
  msg.transmit_pulse_length = acquisition_conf.transmit_pulse_length;
  msg.water_layer_start = acquisition_conf.water_layer_start;
  msg.water_layer_end = acquisition_conf.water_layer_end;
  msg.false_target_threshold = acquisition_conf.false_target_threshold;
  msg.low_latency_trigger = acquisition_conf.low_latency_trigger;
  msg.transmit_lag_distance = acquisition_conf.transmit_lag_distance;
  msg.narrow_bandwidth_mode = acquisition_conf.narrow_bandwidth_mode;
  msg.base_frequency_index = acquisition_conf.base_frequency_index;

  return msg;
}

//------------------------------------------------------------------------------
//
OutputConfiguration BuildOutputConfiguration(const
                                             PD0Message::OutputConfiguration
                                             &output_conf) {
  OutputConfiguration msg;
  msg.header.stamp = ros::Time::now();

  msg.coordinate_system = output_conf.coordinate_system;
  msg.use_attitude = output_conf.use_attitude;
  msg.use_3beam_solution = output_conf.use_3beam_solution;
  msg.use_bin_mapping = output_conf.use_bin_mapping;

  return msg;
}

//------------------------------------------------------------------------------
//
Status BuildStatus(const PD0Message::Status &status) {
  Status msg;
  msg.header.stamp = ros::Time::now();

  msg.seq = status.seq;
  msg.time = status.time.microseconds;
  msg.depth = status.depth;
  msg.speed_of_sound = status.speed_of_sound;
  msg.salinity = status.salinity;
  msg.temperature = status.temperature;
  msg.pressure = status.pressure;
  msg.pressure_variance = status.pressure_variance;
  msg.min_preping_wait = status.min_preping_wait.microseconds;
  msg.self_test_result = status.self_test_result;
  msg.status_word = status.status_word;


  msg.stddev_orientation.z = status.stddev_orientation[0];
  msg.stddev_orientation.y = status.stddev_orientation[1];
  msg.stddev_orientation.x = status.stddev_orientation[2];

  msg.orientation.x = status.orientation.x();
  msg.orientation.w = status.orientation.w();
  msg.orientation.y = status.orientation.y();
  msg.orientation.z = status.orientation.z();

  for (int i = 0; i < 8; ++i) {
    msg.adc_channels[i] = status.adc_channels[i];
  }

  return msg;
}

//------------------------------------------------------------------------------
//
CellReadings BuildCellReadings(const PD0Message::CellReadings &cell_readings) {
  CellReadings msg;
  msg.header.stamp = ros::Time::now();

  msg.time = cell_readings.time.microseconds;

  for(const auto & reading : cell_readings.readings) {
    msg.readings.push_back(BuildCellReading(reading));
  }

  return msg;
}

//------------------------------------------------------------------------------
//
BottomTrackingConfiguration BuildBottomTrackingConfiguration(const
                                                             PD0Message::BottomTrackingConfiguration &bottom_tracking_conf) {
  BottomTrackingConfiguration msg;
  msg.header.stamp = ros::Time::now();

  msg.ping_per_ensemble = bottom_tracking_conf.ping_per_ensemble;
  msg.delay_before_reacquiring = bottom_tracking_conf.delay_before_reacquiring;
  msg.correlation_threshold = bottom_tracking_conf.correlation_threshold;
  msg.evaluation_threshold = bottom_tracking_conf.evaluation_threshold;
  msg.good_ping_threshold = bottom_tracking_conf.good_ping_threshold;
  msg.mode = bottom_tracking_conf.mode;
  msg.max_velocity_error = bottom_tracking_conf.max_velocity_error;
  msg.max_tracking_depth = bottom_tracking_conf.max_tracking_depth;
  msg.gain = bottom_tracking_conf.gain;

  return msg;
}

//------------------------------------------------------------------------------
//
BottomTracking BuildBottomTracking(const PD0Message::BottomTracking
                                   &bottom_tracking) {
  BottomTracking msg;
  msg.header.stamp = ros::Time::now();

  msg.time = bottom_tracking.time.microseconds;

  for(int i = 0; i < 4; ++i) {
    msg.range[i] = bottom_tracking.range[i];
    msg.velocity[i] = bottom_tracking.velocity[i];
    msg.correlation[i] = bottom_tracking.correlation[i];
    msg.evaluation[i] = bottom_tracking.evaluation[i];
    msg.good_ping_ratio[i] = bottom_tracking.good_ping_ratio[i];
    msg.rssi[i] = bottom_tracking.rssi[i];
  }

  return msg;
}

//------------------------------------------------------------------------------
//
geometry_msgs::TwistWithCovarianceStamped BuildTwistWithCovariance(const PD0Message::BottomTracking
                                                                   &bottom_tracking) {
  geometry_msgs::TwistWithCovarianceStamped msg;
  msg.header.stamp = ros::Time::now();

  msg.twist.twist.linear.x = bottom_tracking.range[0];
  msg.twist.twist.linear.y = bottom_tracking.range[1];
  msg.twist.twist.linear.z = bottom_tracking.range[2];

  return msg;
}

} // namespace msg_builder

}  // namespace provider_dvl
