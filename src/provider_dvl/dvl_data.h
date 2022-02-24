//
// Created by Francis Alonzo on 9/3/21.
//

#ifndef PROVIDER_DVL_DVL_DATA_H
#define PROVIDER_DVL_DVL_DATA_H

#pragma pack(push, 1)

typedef struct
{
  uint8_t pathfinderDataId;
  uint8_t dataStructure;
  uint16_t numberBytes;
  uint8_t systemConfig;
  int16_t xVelBtm;
  int16_t yVelBtm;
  int16_t zVelBtm;
  int16_t eVelBtm;
  uint16_t rngToBottomBm1;
  uint16_t rngToBottomBm2;
  uint16_t rngToBottomBm3;
  uint16_t rngToBottomBm4;
  uint8_t bottomStatus;
  uint16_t velocity1;
  uint16_t velocity2;
  uint16_t velocity3;
  uint16_t velocity4;
  uint16_t refLayerStart;
  uint16_t refLayerEnd;
  uint8_t refLayerStatus;
  uint8_t hourFirstPing;
  uint8_t minuteFirstPing;
  uint8_t secondFirstPing;
  uint8_t hundredthFirstPing;
  uint16_t statusLeakSensors;
  uint16_t speedOfSound;
  uint16_t temperature;
  uint16_t checksum;
} PD4;

typedef struct {
  PD4 pd4;
} DVLformat21_t;

#pragma pack(pop)

#endif //PROVIDER_DVL_DVL_DATA_H
