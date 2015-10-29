//
// Created by jeremie on 10/29/15.
//

#ifndef PROVIDER_DVL_PD5MESSAGES_H
#define PROVIDER_DVL_PD5MESSAGES_H


class PD5Messages {
 static const int32_t MESSAGE_SIZE = 86;

 private:
  // System config
  uint8_t systemConfig;

  // Bottom Velocities
  int32_t xVelBtm;
  int32_t yVelBtm;
  int32_t zVelBtm;
  int32_t eVelBtm;

  // Beam Range
  int32_t bm1RngBtm;
  int32_t bm2RngBtm;
  int32_t bm3RngBtm;
  int32_t bm4RngBtm;

  // Bottom status
  int32_t bottomStatus;

  int32_t beam1LowCorrelation;
  int32_t beam1LowEchoAmplitude;
  int32_t beam2LowCorrelation;
  int32_t beam2LowEchoAmplitude;
  int32_t beam3LowCorrelation;
  int32_t beam3LowEchoAmplitude;
  int32_t beam4LowCorrelation;
  int32_t beam4LowEchoAmplitude;

  // WaterMass Velocities
  int32_t xVelRef;
  int32_t yVelRef;
  int32_t zVelRef;
  int32_t eVelRef;

  // WaterMass Start End Status
  int32_t refLayerStart;
  int32_t refLayerEnd;
  uint8_t refLayerStatus;

  // Time of first ping (TOFP)
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t hundredth;

  // Built-in test result
  int32_t bitResult;

  // Environemental data
  int32_t speedOfSound;
  int32_t temperature;
  uint8_t salinity;

  // Instrument data
  int32_t depth;
  int32_t pitch;
  int32_t roll;
  int32_t heading;

  // Distance made good bottom track
  int32_t dmgBottomEast;
  int32_t dmgBottomNorth;
  int32_t dmgBottomUp;
  int32_t dmgBottomError;

  // Distance made good water mass
  int32_t refBottomEast;
  int32_t refBottomNorth;
  int32_t refBottomUp;
  int32_t refBottomError;
};


#endif //PROVIDER_DVL_PD5MESSAGES_H
