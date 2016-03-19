#include <sys/ioctl.h>
#include <termios.h>
#include <fstream>
#include <iostream>
#include <provider_dvl/driver/Driver.hpp>

using namespace dvl_teledyne;

Driver::Driver()
    : iodrivers_base::Driver(1000000),
      mConfMode(false),
      mDesiredBaudrate(M_SONIA_BAUDRATE) {
  buffer.resize(1000000);
}

void Driver::open(std::string const &uri) {
  openURI(uri);
  setConfigurationMode();
  if (mDesiredBaudrate != M_SONIA_BAUDRATE)
    setDesiredBaudrate(mDesiredBaudrate);

  startAcquisition();
}

void Driver::sendConfigurationFile(std::string const &file_name) {
  setConfigurationMode();

  std::ifstream file(file_name.c_str());

  char line_buffer[2000];
  while (!file.eof()) {
    if (!file.getline(line_buffer, 2000) && !file.eof())
      throw std::runtime_error("lines longer than 2000 characters");

    std::string line(line_buffer);
    if (line == "CS") break;

    line += "\n";
    std::cout << iodrivers_base::Driver::printable_com(line) << std::endl;
    writePacket(reinterpret_cast<uint8_t const *>(line.c_str()), line.length());
    readConfigurationAck();
  }
}

void Driver::setDesiredBaudrate(int rate) {
  if (getFileDescriptor() != iodrivers_base::Driver::INVALID_FD)
    setDeviceBaudrate(rate);
  mDesiredBaudrate = rate;
}

void Driver::setDeviceBaudrate(int rate) {
  setConfigurationMode();

  uint8_t code = 0;
  switch (rate) {
    case 300:
      code = 0;
      break;
    case 1200:
      code = 1;
      break;
    case 2400:
      code = 2;
      break;
    case 4800:
      code = 3;
      break;
    case 9600:
      code = 4;
      break;
    case 19200:
      code = 5;
      break;
    case 38400:
      code = 6;
      break;
    case 57600:
      code = 7;
      break;
    case 115200:
      code = 8;
      break;
    default:
      throw std::runtime_error("invalid baud rate specified");
  }
  uint8_t data[7] = {
      static_cast<uint8_t>('C'),        static_cast<uint8_t>('B'),
      static_cast<uint8_t>('0' + code), static_cast<uint8_t>('1'),
      static_cast<uint8_t>('1'),        static_cast<uint8_t>('\n'),
      0};
  writePacket(data, 6, 100);
  readConfigurationAck();
}

void Driver::read() {
  int packet_size = readPacket(&buffer[0], buffer.size());
  if (packet_size) parseEnsemble(&buffer[0], packet_size);
}

int Driver::extractPacket(uint8_t const *buffer, size_t buffer_size) const {
  if (mConfMode) {
    char const *buffer_as_string = reinterpret_cast<char const *>(buffer);
    if (buffer_as_string[0] == '>')
      return 1;
    else if (buffer_as_string[0] == 'E') {
      if (buffer_size > 1 && buffer_as_string[1] != 'R')
        return -1;
      else if (buffer_size > 2 && buffer_as_string[2] != 'R')
        return -1;

      // We have an error. Find \n> and return
      size_t eol = 2;
      for (eol = 2; eol < buffer_size - 1; ++eol) {
        if (buffer_as_string[eol] == '\n' && buffer_as_string[eol + 1] == '>')
          return eol;
      }
      return 0;
    } else
      return -1;
  } else {
    // std::cout << iodrivers_base::Driver::printable_com(buffer, buffer_size)
    // << std::endl;
    return PD0Parser::extractPacket(buffer, buffer_size);
  }
}

void Driver::setConfigurationMode() {
  if (tcsendbreak(getFileDescriptor(), 0))
    throw iodrivers_base::UnixError("failed to set configuration mode");
  mConfMode = true;

  // This is a tricky one. As usual with fiddling with serial lines, the
  // device is inaccessible "for a while" (which is unspecified)
  //
  // Repeatedly write a CR on the line and check for an ack (i.e. a prompt).
  // We do it repeatedly so that we are sure that the CR is not lost.
  clear();
  for (int i = 0; i < 12; ++i) {
    writePacket(reinterpret_cast<uint8_t const *>("\n"), 1, 100);
    try {
      readConfigurationAck(base::Time::fromSeconds(0.1));
      clear();
      break;
    } catch (iodrivers_base::TimeoutError) {
      if (i == 11) throw;
    }
  }
}

void Driver::readConfigurationAck(base::Time const &timeout) {
  if (!mConfMode) throw std::runtime_error("not in configuration mode");
  int packet_size = readPacket(&buffer[0], buffer.size(), timeout);
  if (buffer[0] != '>')
    throw std::runtime_error(
        std::string(reinterpret_cast<char const *>(&buffer[0]), packet_size));
}

/** Configures the output coordinate system */
void Driver::setOutputConfiguration(PD0Message::OutputConfiguration conf) {
  if (!mConfMode) throw std::runtime_error("not in configuration mode");

  uint8_t mode_codes_1[4] = {'0', '0', '1', '1'};
  uint8_t mode_codes_2[4] = {'0', '1', '0', '1'};
  uint8_t const cmd[7] = {'E', 'X', mode_codes_1[conf.coordinate_system],
                          mode_codes_2[conf.coordinate_system],
                          (conf.use_attitude ? static_cast<uint8_t>('1')
                                             : static_cast<uint8_t>('0')),
                          (conf.use_3beam_solution ? static_cast<uint8_t>('1')
                                                   : static_cast<uint8_t>('0')),
                          (conf.use_bin_mapping ? static_cast<uint8_t>('1')
                                                : static_cast<uint8_t>('0'))};

  writePacket(cmd, 7, 500);
}

void Driver::startAcquisition() {
  if (!mConfMode) throw std::logic_error("not in configuration mode");

  // We are sending raw data according to our needs here.
  // We should definitely have a ROS service for this...
  // writePacket(reinterpret_cast<uint8_t const *>("EX11111\n"), 8, 100);
  // readConfigurationAck();
  writePacket(reinterpret_cast<uint8_t const *>("EX01011\n"), 8, 100);
  readConfigurationAck();
  writePacket(reinterpret_cast<uint8_t const *>("EA00000\n"), 8, 100);
  readConfigurationAck();
  writePacket(reinterpret_cast<uint8_t const *>("PD0\n"), 4, 100);
  readConfigurationAck();
  writePacket(reinterpret_cast<uint8_t const *>("CS\n"), 3, 100);
  readConfigurationAck();
  mConfMode = false;
}
