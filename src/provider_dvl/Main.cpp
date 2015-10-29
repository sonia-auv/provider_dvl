#include <provider_dvl/driver/Driver.hpp>
#include <iostream>
#include <fstream>

using namespace dvl_teledyne;

void usage() { std::cerr << "dvl_teledyne_read DEVICE" << std::endl; }

int main(int argc, char const *argv[]) {
  if (argc != 2) {
    usage();
    return 1;
  }

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
  while (true) {
    driver.read();
    file << "NEW FRAME" << std::endl;
    DeviceInfo const &info = driver.deviceInfo;
    file << "Device info\n";
    file << "sys config\t" << info.system_configuration << std::endl <<
        " fw version\t" << int(info.fw_version) << std::endl <<
        " fw revision\t" << int(info.fw_revision) << std::endl <<
        " beam count\t" << int(info.beam_count) << std::endl <<
        " CPU Board Number\t" << int(info.cpu_board_serno) << std::endl
        << "\t Available device: " << std::endl <<
        " \tSpeed of sound?\t"
        << int(info.available_sensors.calculates_speed_of_sound) << std::endl <<
        " \tDepth?\t" << int(info.available_sensors.depth) << std::endl <<
        " \tPitch?\t" << int(info.available_sensors.pitch) << std::endl <<
        " \tRoll?\t" << int(info.available_sensors.roll) << std::endl <<
        " \tYaw?\t" << int(info.available_sensors.yaw) << std::endl <<
        " \tTemperature?\t" << int(info.available_sensors.temperature)
        << std::endl <<
        " \tSalinity?\t" << int(info.available_sensors.salinity) << std::endl;
    AcquisitionConfiguration acqConf = driver.acqConf;
    file << "Acquisition configuration" << std::endl;
    file << "\tlag_duration\t" << int(acqConf.lag_duration) << std::endl <<
        "\tcell_count\t" << int(acqConf.cell_count) << std::endl <<
        "\tprofiling_mode\t" << int(acqConf.profiling_mode) << std::endl <<
        "\tlow_correlation_threshold\t"
        << int(acqConf.low_correlation_threshold) << std::endl <<
        "\tcode_repetition_count\t" << int(acqConf.code_repetition_count)
        << std::endl <<
        "\tpings_per_ensemble\t" << int(acqConf.pings_per_ensemble) << std::endl
        <<
            "\tcell_length\t" << acqConf.cell_length << std::endl <<
        "\tblank_after_transmit_distance\t"
        << acqConf.blank_after_transmit_distance << std::endl <<
        "\twater_layer_min_ping_threshold\t"
        << acqConf.water_layer_min_ping_threshold << std::endl <<
        "\twater_layer_velocity_threshold\t"
        << acqConf.water_layer_velocity_threshold << std::endl <<
        "\tyaw_alignment\t" << acqConf.yaw_alignment << std::endl <<
        "\tyaw_bias\t" << acqConf.yaw_bias << std::endl <<
        "\tfirst_cell_distance\t" << acqConf.first_cell_distance << std::endl <<
        "\ttransmit_pulse_length\t" << acqConf.transmit_pulse_length
        << std::endl <<
        "\twater_layer_start\t" << int(acqConf.water_layer_start) << std::endl
        <<
            "\twater_layer_end\t" << int(acqConf.water_layer_end) << std::endl
        <<
            "\tfalse_target_threshold\t" << int(acqConf.false_target_threshold)
        << std::endl <<
        "\tlow_latency_trigger\t" << int(acqConf.low_latency_trigger)
        << std::endl <<
        "\ttransmit_lag_distance\t" << acqConf.transmit_lag_distance
        << std::endl <<
        "\tnarrow_bandwidth_mode\t" << acqConf.narrow_bandwidth_mode
        << std::endl <<
        "\tbase_frequency_index\t" << int(acqConf.base_frequency_index)
        << std::endl;

    OutputConfiguration outputConf = driver.outputConf;
    file << "Output configuration \n";
    file << "\tuse_3beam_solution\t" << outputConf.use_3beam_solution
        << std::endl <<
        "\tuse_attitude\t" << int(outputConf.use_attitude) << std::endl <<
        "\tuse_bin_mapping\t" << int(outputConf.use_bin_mapping) << std::endl <<
        "\tcoordinate_system\t" << int(outputConf.coordinate_system)
        << std::endl;
    
    Status status = driver.status;
    file << "Status" << std::endl;
    file << "\tseq\t" << status.seq << std::endl <<
        file << "\tstddev_orientation\t" << status.stddev_orientation[0]
        << std::endl <<
        file << "\tstddev_orientation\t" << status.stddev_orientation[0]
        << std::endl <<
        file << "\tstddev_orientation\t" << status.stddev_orientation[0]
        << std::endl <<

        file << "\tdepth \t" << status.depth << std::endl <<
        file << "\tspeed_of_sound\t" << status.speed_of_sound << std::endl <<
        file << "\tsalinity \t" << status.salinity << std::endl <<
        file << "\ttemperature \t" << status.temperature << std::endl <<
        file << "\tpressure \t" << status.pressure << std::endl <<
        file << "\tpressure_variance \t" << status.pressure_variance
        << std::endl <<
        file << "\tself_test_result\t" << status.self_test_result << std::endl
        <<
            file << "\tstatus_word \t" << status.status_word << std::endl;

    BottomTrackingConfiguration bottomTrackingConf = driver.bottomTrackingConf;
    file << "Bottom tracking configuration" << std::endl;
    file << "\tping_per_ensemble\t" << bottomTrackingConf.ping_per_ensemble
        << std::endl <<
        "\tdelay_before_reacquiring\t"
        << bottomTrackingConf.delay_before_reacquiring << std::endl <<
        "\tcorrelation_threshold\t" << bottomTrackingConf.correlation_threshold
        << std::endl <<
        "\tevaluation_threshold\t" << bottomTrackingConf.evaluation_threshold
        << std::endl <<
        "\tgood_ping_threshold\t" << bottomTrackingConf.good_ping_threshold
        << std::endl <<
        "\tmode <<\t" << int(bottomTrackingConf.mode) << std::endl <<
        "\tmax_velocity_error\t" << bottomTrackingConf.max_velocity_error
        << std::endl <<
        "\tmax_tracking_depth\t" << bottomTrackingConf.max_tracking_depth
        << std::endl <<
        "\tgain << int(gain)\\t" << std::endl;

    BottomTracking const &tracking = driver.bottomTracking;
    file << "Bottom Tracking" << std::endl;
    for( int i = 0; i < 4; i ++)
    {
      file << "\trange\t" << tracking.range[i] << std::endl <<
          file << "\tvelocity\t" << tracking.velocity[i] << std::endl <<
          file << "\tcorrelation\t" << tracking.correlation[i] << std::endl <<
          file << "\tevaluation\t" << tracking.evaluation[i] << std::endl <<
          file << "\tgood_ping_ratio\t" << tracking.good_ping_ratio[i] << std::endl;
    }
    file << "END OF PACKET" << std::endl;
//    std::cout << tracking.time.toString() << " " << driver.status.seq;
//    for (int beam = 0; beam < 4; ++beam)
//      std::cout << " " << tracking.range[beam] << " " << tracking.velocity[beam]
//          << " " << tracking.evaluation[beam];
//    std::cout << std::endl;
  }
}
