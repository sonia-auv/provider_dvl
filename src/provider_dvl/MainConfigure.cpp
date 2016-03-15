#include <fstream>
#include <iostream>
#include <provider_dvl/driver/Driver.hpp>

using namespace dvl_teledyne;

void usage() {
  std::cerr << "dvl_teledyne_configure DEVICE [FILE]" << std::endl;
}

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    usage();
    return 1;
  }

  dvl_teledyne::Driver driver;
  driver.openSerial(argv[1], 115200);
  driver.setWriteTimeout(base::Time::fromSeconds(5));
  driver.setReadTimeout(base::Time::fromSeconds(5));

  if (argc == 3) driver.sendConfigurationFile(argv[2]);

  // The file might contain a CS command. Make sure we are still in
  // configuration mode
  driver.setConfigurationMode();
  driver.startAcquisition();
}
