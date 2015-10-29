//
// Created by jeremie on 10/29/15.
//

#ifndef PROVIDER_DVL_PD5MESSAGES_H
#define PROVIDER_DVL_PD5MESSAGES_H

#include <stdint.h>

class PD5Messages {
 public:
  static const uint32_t MESSAGE_SIZE = 86;
  PD5Messages();
  ~PD5Messages();
  uint8_t GetSystemConfig();
  void SetSystemConfig(uint8_t systemConfig);

  uint32_t GetXVelBtm();
  void SetXVelBtm(uint32_t velBtm);

  uint32_t GetYVelBtm();
  void SetYVelBtm(uint32_t velBtm);

  uint32_t GetZVelBtm();
  void SetZVelBtm(uint32_t velBtm);

  uint32_t GetEVelBtm();
  void SetEVelBtm(uint32_t velBtm);

  uint32_t GetBm1RngBtm();
  void SetBm1RngBtm(uint32_t bm1RngBtm);

  uint32_t GetBm2RngBtm();
  void SetBm2RngBtm(uint32_t bm2RngBtm);

  uint32_t GetBm3RngBtm();
  void SetBm3RngBtm(uint32_t bm3RngBtm);

  uint32_t GetBm4RngBtm();
  void SetBm4RngBtm(uint32_t bm4RngBtm);

  uint32_t GetBottomStatus();
  void SetBottomStatus(uint32_t bottomStatus);

  uint32_t GetBeam1LowCorrelation();
  void SetBeam1LowCorrelation(uint32_t beam1LowCorrelation);

  uint32_t GetBeam1LowEchoAmplitude();
  void SetBeam1LowEchoAmplitude(uint32_t beam1LowEchoAmplitude);

  uint32_t GetBeam2LowCorrelation();
  void SetBeam2LowCorrelation(uint32_t beam2LowCorrelation);

  uint32_t GetBeam2LowEchoAmplitude();
  void SetBeam2LowEchoAmplitude(uint32_t beam2LowEchoAmplitude);

  uint32_t GetBeam3LowCorrelation();
  void SetBeam3LowCorrelation(uint32_t beam3LowCorrelation);

  uint32_t GetBeam3LowEchoAmplitude();
  void SetBeam3LowEchoAmplitude(uint32_t beam3LowEchoAmplitude);

  uint32_t GetBeam4LowCorrelation();
  void SetBeam4LowCorrelation(uint32_t beam4LowCorrelation);

  uint32_t GetBeam4LowEchoAmplitude();
  void SetBeam4LowEchoAmplitude(uint32_t beam4LowEchoAmplitude);

  uint32_t GetXVelRef();
  void SetXVelRef(uint32_t velRef);

  uint32_t GetYVelRef();
  void SetYVelRef(uint32_t velRef);

  uint32_t GetZVelRef();
  void SetZVelRef(uint32_t velRef);

  uint32_t GetEVelRef();
  void SetEVelRef(uint32_t velRef);

  uint32_t GetRefLayerStart();
  void SetRefLayerStart(uint32_t refLayerStart);

  uint32_t GetRefLayerEnd();
  void SetRefLayerEnd(uint32_t refLayerEnd);

  uint8_t GetRefLayerStatus();
  void SetRefLayerStatus(uint8_t refLayerStatus);

  uint8_t GetHour();
  void SetHour(uint8_t hour);

  uint8_t GetMinute();
  void SetMinute(uint8_t minute);

  uint8_t GetSecond();
  void SetSecond(uint8_t second);

  uint8_t GetHundredth();
  void SetHundredth(uint8_t hundredth);

  uint32_t GetBitResult();
  void SetBitResult(uint32_t bitResult);

  uint32_t GetSpeedOfSound();
  void SetSpeedOfSound(uint32_t speedOfSound);

  uint32_t GetTemperature();
  void SetTemperature(uint32_t temperature);

  uint8_t GetSalinity();
  void SetSalinity(uint8_t salinity);

  uint32_t GetDepth();
  void SetDepth(uint32_t depth);

  uint32_t GetPitch();
  void SetPitch(uint32_t pitch);

  uint32_t GetRoll();
  void SetRoll(uint32_t roll);

  uint32_t GetHeading();
  void SetHeading(uint32_t heading);

  uint32_t GetDmgBottomEast();
  void SetDmgBottomEast(uint32_t dmgBottomEast);

  uint32_t GetDmgBottomNorth();
  void SetDmgBottomNorth(uint32_t dmgBottomNorth);

  uint32_t GetDmgBottomUp();
  void SetDmgBottomUp(uint32_t dmgBottomUp);

  uint32_t GetDmgBottomError();
  void SetDmgBottomError(uint32_t dmgBottomError);

  uint32_t GetRefBottomEast();
  void SetRefBottomEast(uint32_t refBottomEast);

  uint32_t GetRefBottomNorth();
  void SetRefBottomNorth(uint32_t refBottomNorth);

  uint32_t GetRefBottomUp();
  void SetRefBottomUp(uint32_t refBottomUp);

  uint32_t GetRefBottomError();
  void SetRefBottomError(uint32_t refBottomError);

 private:
  // System config
  uint8_t systemConfig;

  // Bottom Velocities
  uint32_t xVelBtm;
  uint32_t yVelBtm;
  uint32_t zVelBtm;
  uint32_t eVelBtm;

  // Beam Range
  uint32_t bm1RngBtm;
  uint32_t bm2RngBtm;
  uint32_t bm3RngBtm;
  uint32_t bm4RngBtm;

  // Bottom status
  uint32_t bottomStatus;

  uint32_t beam1LowCorrelation;
  uint32_t beam1LowEchoAmplitude;
  uint32_t beam2LowCorrelation;
  uint32_t beam2LowEchoAmplitude;
  uint32_t beam3LowCorrelation;
  uint32_t beam3LowEchoAmplitude;
  uint32_t beam4LowCorrelation;
  uint32_t beam4LowEchoAmplitude;

  // WaterMass Velocities
  uint32_t xVelRef;
  uint32_t yVelRef;
  uint32_t zVelRef;
  uint32_t eVelRef;

  // WaterMass Start End Status
  uint32_t refLayerStart;
  uint32_t refLayerEnd;
  uint8_t refLayerStatus;

  // Time of first ping (TOFP)
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t hundredth;

  // Built-in test result
  uint32_t bitResult;

  // Environemental data
  uint32_t speedOfSound;
  uint32_t temperature;
  uint8_t salinity;

  // Instrument data
  uint32_t depth;
  uint32_t pitch;
  uint32_t roll;
  uint32_t heading;

  // Distance made good bottom track
  uint32_t dmgBottomEast;
  uint32_t dmgBottomNorth;
  uint32_t dmgBottomUp;
  uint32_t dmgBottomError;

  // Distance made good water mass
  uint32_t refBottomEast;
  uint32_t refBottomNorth;
  uint32_t refBottomUp;
  uint32_t refBottomError;
};
//==============================================================================
// INLINE CODE SECTION
//------------------------------------------------------------------------------
//
inline uint8_t PD5Messages::GetSystemConfig() {
  return systemConfig;
}

inline void PD5Messages::SetSystemConfig(uint8_t systemConfig) {
  systemConfig = systemConfig;
}

inline uint32_t PD5Messages::GetXVelBtm() {
  return xVelBtm;
}

inline void PD5Messages::SetXVelBtm(uint32_t velBtm) {
  xVelBtm = velBtm;
}

inline uint32_t PD5Messages::GetYVelBtm() {
  return yVelBtm;
}

inline void PD5Messages::SetYVelBtm(uint32_t velBtm) {
  yVelBtm = velBtm;
}

inline uint32_t PD5Messages::GetZVelBtm() {
  return zVelBtm;
}

inline void PD5Messages::SetZVelBtm(uint32_t velBtm) {
  zVelBtm = velBtm;
}

inline uint32_t PD5Messages::GetEVelBtm() {
  return eVelBtm;
}

inline void PD5Messages::SetEVelBtm(uint32_t velBtm) {
  eVelBtm = velBtm;
}

inline uint32_t PD5Messages::GetBm1RngBtm() {
  return bm1RngBtm;
}

inline void PD5Messages::SetBm1RngBtm(uint32_t bm1RngBtm) {
  bm1RngBtm = bm1RngBtm;
}

inline uint32_t PD5Messages::GetBm2RngBtm() {
  return bm2RngBtm;
}

inline void PD5Messages::SetBm2RngBtm(uint32_t bm2RngBtm) {
  bm2RngBtm = bm2RngBtm;
}

inline uint32_t PD5Messages::GetBm3RngBtm() {
  return bm3RngBtm;
}

inline void PD5Messages::SetBm3RngBtm(uint32_t bm3RngBtm) {
  bm3RngBtm = bm3RngBtm;
}

inline uint32_t PD5Messages::GetBm4RngBtm() {
  return bm4RngBtm;
}

inline void PD5Messages::SetBm4RngBtm(uint32_t bm4RngBtm) {
  bm4RngBtm = bm4RngBtm;
}

inline uint32_t PD5Messages::GetBottomStatus() {
  return bottomStatus;
}

inline void PD5Messages::SetBottomStatus(uint32_t bottomStatus) {
  bottomStatus = bottomStatus;
}

inline uint32_t PD5Messages::GetBeam1LowCorrelation() {
  return beam1LowCorrelation;
}

inline void PD5Messages::SetBeam1LowCorrelation(uint32_t beam1LowCorrelation) {
  beam1LowCorrelation = beam1LowCorrelation;
}

inline uint32_t PD5Messages::GetBeam1LowEchoAmplitude() {
  return beam1LowEchoAmplitude;
}

inline void PD5Messages::SetBeam1LowEchoAmplitude(uint32_t beam1LowEchoAmplitude) {
  beam1LowEchoAmplitude = beam1LowEchoAmplitude;
}

inline uint32_t PD5Messages::GetBeam2LowCorrelation() {
  return beam2LowCorrelation;
}

inline void PD5Messages::SetBeam2LowCorrelation(uint32_t beam2LowCorrelation) {
  beam2LowCorrelation = beam2LowCorrelation;
}

inline uint32_t PD5Messages::GetBeam2LowEchoAmplitude() {
  return beam2LowEchoAmplitude;
}

inline void PD5Messages::SetBeam2LowEchoAmplitude(uint32_t beam2LowEchoAmplitude) {
  beam2LowEchoAmplitude = beam2LowEchoAmplitude;
}

inline uint32_t PD5Messages::GetBeam3LowCorrelation() {
  return beam3LowCorrelation;
}

inline void PD5Messages::SetBeam3LowCorrelation(uint32_t beam3LowCorrelation) {
  beam3LowCorrelation = beam3LowCorrelation;
}

inline uint32_t PD5Messages::GetBeam3LowEchoAmplitude() {
  return beam3LowEchoAmplitude;
}

inline void PD5Messages::SetBeam3LowEchoAmplitude(uint32_t beam3LowEchoAmplitude) {
  beam3LowEchoAmplitude = beam3LowEchoAmplitude;
}

inline uint32_t PD5Messages::GetBeam4LowCorrelation() {
  return beam4LowCorrelation;
}

inline void PD5Messages::SetBeam4LowCorrelation(uint32_t beam4LowCorrelation) {
  beam4LowCorrelation = beam4LowCorrelation;
}

inline uint32_t PD5Messages::GetBeam4LowEchoAmplitude() {
  return beam4LowEchoAmplitude;
}

inline void PD5Messages::SetBeam4LowEchoAmplitude(uint32_t beam4LowEchoAmplitude) {
  beam4LowEchoAmplitude = beam4LowEchoAmplitude;
}

inline uint32_t PD5Messages::GetXVelRef() {
  return xVelRef;
}

inline void PD5Messages::SetXVelRef(uint32_t velRef) {
  xVelRef = velRef;
}

inline uint32_t PD5Messages::GetYVelRef() {
  return yVelRef;
}

inline void PD5Messages::SetYVelRef(uint32_t velRef) {
  yVelRef = velRef;
}

inline uint32_t PD5Messages::GetZVelRef() {
  return zVelRef;
}

inline void PD5Messages::SetZVelRef(uint32_t velRef) {
  zVelRef = velRef;
}

inline uint32_t PD5Messages::GetEVelRef() {
  return eVelRef;
}

inline void PD5Messages::SetEVelRef(uint32_t velRef) {
  eVelRef = velRef;
}

inline uint32_t PD5Messages::GetRefLayerStart() {
  return refLayerStart;
}

inline void PD5Messages::SetRefLayerStart(uint32_t refLayerStart) {
  refLayerStart = refLayerStart;
}

inline uint32_t PD5Messages::GetRefLayerEnd() {
  return refLayerEnd;
}

inline void PD5Messages::SetRefLayerEnd(uint32_t refLayerEnd) {
  refLayerEnd = refLayerEnd;
}

inline uint8_t PD5Messages::GetRefLayerStatus() {
  return refLayerStatus;
}

inline void PD5Messages::SetRefLayerStatus(uint8_t refLayerStatus) {
  refLayerStatus = refLayerStatus;
}

inline uint8_t PD5Messages::GetHour() {
  return hour;
}

inline void PD5Messages::SetHour(uint8_t hour) {
  hour = hour;
}

inline uint8_t PD5Messages::GetMinute() {
  return minute;
}

inline void PD5Messages::SetMinute(uint8_t minute) {
  minute = minute;
}

inline uint8_t PD5Messages::GetSecond() {
  return second;
}

inline void PD5Messages::SetSecond(uint8_t second) {
  second = second;
}

inline uint8_t PD5Messages::GetHundredth() {
  return hundredth;
}

inline void PD5Messages::SetHundredth(uint8_t hundredth) {
  hundredth = hundredth;
}

inline uint32_t PD5Messages::GetBitResult() {
  return bitResult;
}

inline void PD5Messages::SetBitResult(uint32_t bitResult) {
  bitResult = bitResult;
}

inline uint32_t PD5Messages::GetSpeedOfSound() {
  return speedOfSound;
}

inline void PD5Messages::SetSpeedOfSound(uint32_t speedOfSound) {
  speedOfSound = speedOfSound;
}

inline uint32_t PD5Messages::GetTemperature() {
  return temperature;
}

inline void PD5Messages::SetTemperature(uint32_t temperature) {
  temperature = temperature;
}

inline uint8_t PD5Messages::GetSalinity() {
  return salinity;
}

inline void PD5Messages::SetSalinity(uint8_t salinity) {
  salinity = salinity;
}

inline uint32_t PD5Messages::GetDepth() {
  return depth;
}

inline void PD5Messages::SetDepth(uint32_t depth) {
  depth = depth;
}

inline uint32_t PD5Messages::GetPitch() {
  return pitch;
}

inline void PD5Messages::SetPitch(uint32_t pitch) {
  pitch = pitch;
}

inline uint32_t PD5Messages::GetRoll() {
  return roll;
}

inline void PD5Messages::SetRoll(uint32_t roll) {
  roll = roll;
}

inline uint32_t PD5Messages::GetHeading() {
  return heading;
}

inline void PD5Messages::SetHeading(uint32_t heading) {
  heading = heading;
}

inline uint32_t PD5Messages::GetDmgBottomEast() {
  return dmgBottomEast;
}

inline void PD5Messages::SetDmgBottomEast(uint32_t dmgBottomEast) {
  dmgBottomEast = dmgBottomEast;
}

inline uint32_t PD5Messages::GetDmgBottomNorth() {
  return dmgBottomNorth;
}

inline void PD5Messages::SetDmgBottomNorth(uint32_t dmgBottomNorth) {
  dmgBottomNorth = dmgBottomNorth;
}

inline uint32_t PD5Messages::GetDmgBottomUp() {
  return dmgBottomUp;
}

inline void PD5Messages::SetDmgBottomUp(uint32_t dmgBottomUp) {
  dmgBottomUp = dmgBottomUp;
}

inline uint32_t PD5Messages::GetDmgBottomError() {
  return dmgBottomError;
}

inline void PD5Messages::SetDmgBottomError(uint32_t dmgBottomError) {
  dmgBottomError = dmgBottomError;
}

inline uint32_t PD5Messages::GetRefBottomEast() {
  return refBottomEast;
}

inline void PD5Messages::SetRefBottomEast(uint32_t refBottomEast) {
  refBottomEast = refBottomEast;
}

inline uint32_t PD5Messages::GetRefBottomNorth() {
  return refBottomNorth;
}

inline void PD5Messages::SetRefBottomNorth(uint32_t refBottomNorth) {
  refBottomNorth = refBottomNorth;
}

inline uint32_t PD5Messages::GetRefBottomUp() {
  return refBottomUp;
}

inline void PD5Messages::SetRefBottomUp(uint32_t refBottomUp) {
  refBottomUp = refBottomUp;
}

inline uint32_t PD5Messages::GetRefBottomError() {
  return refBottomError;
}

inline void PD5Messages::SetRefBottomError(uint32_t refBottomError) {
  refBottomError = refBottomError;
}

#endif //PROVIDER_DVL_PD5MESSAGES_H
