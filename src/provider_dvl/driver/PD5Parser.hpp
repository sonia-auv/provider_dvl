//
// Created by jeremie on 10/29/15.
//

#ifndef PROVIDER_DVL_PD5PARSER_HPP
#define PROVIDER_DVL_PD5PARSER_HPP

#include <cstdlib>
#include <stdint.h>
#include <exception>
#include <iostream>
#include "PD5Packet.hpp"


namespace dvl_teledyne {
class PD5Parser {
 public:
  void ParsePacket(uint8_t const *buffer, size_t size, PD5Packet &packet);
 private:
  uint8_t ReadByte();
  int8_t ReadSignedByte();
  uint16_t ReadUnsigned16BitData();
  int16_t ReadSigned16BitData();
  int32_t ReadSigned32BitData();

  void ResetChecksum(){mChecksum = 0;}
  bool ReadAndValidateChecksum();

  int mChecksum;
  size_t mBufferCounter;
  const uint8_t *mBuffer ;
};

inline uint8_t PD5Parser::ReadByte()
{
  return  mBuffer[mBufferCounter++];
}

inline int8_t PD5Parser::ReadSignedByte() {
  int8_t byteRead = static_cast<int8_t>(ReadByte());

  mChecksum += (byteRead & 0xFF);
  mChecksum %= 65536;

  return byteRead;
}

inline uint16_t PD5Parser::ReadUnsigned16BitData() {
  uint8_t lowByte = ReadSignedByte();
  uint8_t highByte = ReadSignedByte();
  uint16_t value = ((highByte & 0xFF) << 8) + (lowByte & 0xFF);

  return value;
}

inline int16_t PD5Parser::ReadSigned16BitData() {
  uint8_t lowByte = ReadSignedByte();
  uint8_t highByte = ReadSignedByte();

  int16_t value = (highByte << 8) | (lowByte & 0xFF);

  return value;
}

inline int32_t PD5Parser::ReadSigned32BitData() {
  uint16_t lowerNibble = ReadUnsigned16BitData();
  uint16_t higherNibble = ReadUnsigned16BitData();

  uint32_t value = (higherNibble << 16) | (lowerNibble & 0xFFFF);

  return value;
}
bool PD5Parser::ReadAndValidateChecksum() {
  int checksumValue = mChecksum;
  int readChecksum = ReadUnsigned16BitData();
  if (checksumValue != readChecksum) {
    std::cout << "Checksum error on PD5 packet" << std::endl;
  }
  return checksumValue == readChecksum;
}

}

#endif //PROVIDER_DVL_PD5PARSER_HPP
