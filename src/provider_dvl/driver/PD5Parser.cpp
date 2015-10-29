//
// Created by jeremie on 10/29/15.
//

#include <stdexcept>
#include "PD5Parser.hpp"

namespace dvl_teledyne {

void PD5Parser::ParsePacket(uint8_t const *buffer, size_t size, PD5Packet &packet) {
  // New packet
  mBufferCounter = 0;

  if (size < packet.MESSAGE_SIZE + 2) {
    throw std::invalid_argument("wrong size packet for PD5");
  }

  if (buffer[0] != 0x7D) {
    std::cout << "Wrong header for ID" << std::endl;
    mBufferCounter++;
    return;
  }
  uint8_t data_struct = buffer[1], nb_bytes = buffer[2];
  // Check if we have a PD4 packet (should not happen)
  if (data_struct == PD5Packet::PD4 && nb_bytes == 45) {
    // Somehow received a PD4 packet (contains 45 bytes + 2
    // bytes of checksum)
    for (int i = 0; i < 47; i++) {
      ReadByte();
    }
    return;
  }
  else if (data_struct == PD5Packet::PD5
      && nb_bytes == PD5Packet::MESSAGE_SIZE) {
    ResetChecksum();

// Read the first 4 bytes which consists of the
    // ExplorerDVL Data ID, Data Structure and the number of
    // bytes
    for (int i = 0; i < 4; i++) {
      ReadSignedByte();
    }

// Read system config
    packet.SetSystemConfig(ReadSignedByte());

// Read bottom velocity
    packet.SetXVelBtm(ReadSigned16BitData());
    packet.SetYVelBtm(ReadSigned16BitData());
    packet.SetZVelBtm(ReadSigned16BitData());
    packet.SetEVelBtm(ReadSigned16BitData());

// Read bottom range
    packet.SetBm1RngBtm(ReadUnsigned16BitData());
    packet.SetBm2RngBtm(ReadUnsigned16BitData());
    packet.SetBm3RngBtm(ReadUnsigned16BitData());
    packet.SetBm4RngBtm(ReadUnsigned16BitData());

// Read bottom status
    packet.SetBottomStatus(ReadSignedByte());

    packet.SetBeam1LowCorrelation(
        (packet.GetBottomStatus() & 0x01) != 0 ? 1 : 0);
    packet.SetBeam1LowEchoAmplitude(
        (packet.GetBottomStatus() & 0x02) != 0 ? 1 : 0);
    packet.SetBeam2LowCorrelation(
        (packet.GetBottomStatus() & 0x04) != 0 ? 1 : 0);
    packet.SetBeam2LowEchoAmplitude(
        (packet.GetBottomStatus() & 0x08) != 0 ? 1 : 0);
    packet.SetBeam3LowCorrelation(
        (packet.GetBottomStatus() & 0x10) != 0 ? 1 : 0);
    packet.SetBeam3LowEchoAmplitude(
        (packet.GetBottomStatus() & 0x20) != 0 ? 1 : 0);
    packet.SetBeam4LowCorrelation(
        (packet.GetBottomStatus() & 0x40) != 0 ? 1 : 0);
    packet.SetBeam4LowEchoAmplitude(
        (packet.GetBottomStatus() & 0x80) != 0 ? 1 : 0);

// Read water mass velocity
    packet.SetXVelRef(ReadSigned16BitData());
    packet.SetYVelRef(ReadSigned16BitData());
    packet.SetZVelRef(ReadSigned16BitData());
    packet.SetEVelRef(ReadSigned16BitData());

// Read water mass position and status
    packet.SetRefLayerStart(ReadUnsigned16BitData());
    packet.SetRefLayerEnd(ReadUnsigned16BitData());
    packet.SetRefLayerStatus(ReadSignedByte());

// Read time of first ping
    packet.SetHour(ReadSignedByte());
    packet.SetMinute(ReadSignedByte());
    packet.SetSecond(ReadSignedByte());
    packet.SetHundredth(ReadSignedByte());

// Read built-in test
    packet.SetBitResult(ReadUnsigned16BitData());

// Read environmental parameters
    packet.SetSpeedOfSound(ReadUnsigned16BitData());
    packet.SetTemperature(ReadSigned16BitData());
    packet.SetSalinity(ReadSignedByte());

// Read DVL transducer position and orientation
    packet.SetDepth(ReadUnsigned16BitData());
    packet.SetPitch(ReadSigned16BitData());
    packet.SetRoll(ReadSigned16BitData());
    packet.SetHeading(ReadUnsigned16BitData());

// Read distance made good(bottom)
    packet.SetDmgBottomEast(ReadSigned32BitData());
    packet.SetDmgBottomNorth(ReadSigned32BitData());
    packet.SetDmgBottomUp(ReadSigned32BitData());
    packet.SetDmgBottomError(ReadSigned32BitData());

// Read distance made good(water mass)
    packet.SetRefBottomEast(ReadSigned32BitData());
    packet.SetRefBottomNorth(ReadSigned32BitData());
    packet.SetRefBottomUp(ReadSigned32BitData());
    packet.SetRefBottomError(ReadSigned32BitData());

// Validate checksum
    ReadAndValidateChecksum();
  } else {
    mBufferCounter++;
  }
}
}