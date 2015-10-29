//
// Created by jeremie on 10/29/15.
//

#ifndef PROVIDER_DVL_PD5PACKET_H
#define PROVIDER_DVL_PD5PACKET_H

#include <stdint.h>

class PD5Packet {
 public:
  static const uint32_t MESSAGE_SIZE = 86;
  PD5Packet();
  ~PD5Packet();
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
inline uint8_t PD5Packet::GetSystemConfig() {
  return systemConfig;
}

inline void PD5Packet::SetSystemConfig(uint8_t systemConfig) {
  systemConfig = systemConfig;
}

inline uint32_t PD5Packet::GetXVelBtm() {
  return xVelBtm;
}

inline void PD5Packet::SetXVelBtm(uint32_t velBtm) {
  xVelBtm = velBtm;
}

inline uint32_t PD5Packet::GetYVelBtm() {
  return yVelBtm;
}

inline void PD5Packet::SetYVelBtm(uint32_t velBtm) {
  yVelBtm = velBtm;
}

inline uint32_t PD5Packet::GetZVelBtm() {
  return zVelBtm;
}

inline void PD5Packet::SetZVelBtm(uint32_t velBtm) {
  zVelBtm = velBtm;
}

inline uint32_t PD5Packet::GetEVelBtm() {
  return eVelBtm;
}

inline void PD5Packet::SetEVelBtm(uint32_t velBtm) {
  eVelBtm = velBtm;
}

inline uint32_t PD5Packet::GetBm1RngBtm() {
  return bm1RngBtm;
}

inline void PD5Packet::SetBm1RngBtm(uint32_t bm1RngBtm) {
  bm1RngBtm = bm1RngBtm;
}

inline uint32_t PD5Packet::GetBm2RngBtm() {
  return bm2RngBtm;
}

inline void PD5Packet::SetBm2RngBtm(uint32_t bm2RngBtm) {
  bm2RngBtm = bm2RngBtm;
}

inline uint32_t PD5Packet::GetBm3RngBtm() {
  return bm3RngBtm;
}

inline void PD5Packet::SetBm3RngBtm(uint32_t bm3RngBtm) {
  bm3RngBtm = bm3RngBtm;
}

inline uint32_t PD5Packet::GetBm4RngBtm() {
  return bm4RngBtm;
}

inline void PD5Packet::SetBm4RngBtm(uint32_t bm4RngBtm) {
  bm4RngBtm = bm4RngBtm;
}

inline uint32_t PD5Packet::GetBottomStatus() {
  return bottomStatus;
}

inline void PD5Packet::SetBottomStatus(uint32_t bottomStatus) {
  bottomStatus = bottomStatus;
}

inline uint32_t PD5Packet::GetBeam1LowCorrelation() {
  return beam1LowCorrelation;
}

inline void PD5Packet::SetBeam1LowCorrelation(uint32_t beam1LowCorrelation) {
  beam1LowCorrelation = beam1LowCorrelation;
}

inline uint32_t PD5Packet::GetBeam1LowEchoAmplitude() {
  return beam1LowEchoAmplitude;
}

inline void PD5Packet::SetBeam1LowEchoAmplitude(uint32_t beam1LowEchoAmplitude) {
  beam1LowEchoAmplitude = beam1LowEchoAmplitude;
}

inline uint32_t PD5Packet::GetBeam2LowCorrelation() {
  return beam2LowCorrelation;
}

inline void PD5Packet::SetBeam2LowCorrelation(uint32_t beam2LowCorrelation) {
  beam2LowCorrelation = beam2LowCorrelation;
}

inline uint32_t PD5Packet::GetBeam2LowEchoAmplitude() {
  return beam2LowEchoAmplitude;
}

inline void PD5Packet::SetBeam2LowEchoAmplitude(uint32_t beam2LowEchoAmplitude) {
  beam2LowEchoAmplitude = beam2LowEchoAmplitude;
}

inline uint32_t PD5Packet::GetBeam3LowCorrelation() {
  return beam3LowCorrelation;
}

inline void PD5Packet::SetBeam3LowCorrelation(uint32_t beam3LowCorrelation) {
  beam3LowCorrelation = beam3LowCorrelation;
}

inline uint32_t PD5Packet::GetBeam3LowEchoAmplitude() {
  return beam3LowEchoAmplitude;
}

inline void PD5Packet::SetBeam3LowEchoAmplitude(uint32_t beam3LowEchoAmplitude) {
  beam3LowEchoAmplitude = beam3LowEchoAmplitude;
}

inline uint32_t PD5Packet::GetBeam4LowCorrelation() {
  return beam4LowCorrelation;
}

inline void PD5Packet::SetBeam4LowCorrelation(uint32_t beam4LowCorrelation) {
  beam4LowCorrelation = beam4LowCorrelation;
}

inline uint32_t PD5Packet::GetBeam4LowEchoAmplitude() {
  return beam4LowEchoAmplitude;
}

inline void PD5Packet::SetBeam4LowEchoAmplitude(uint32_t beam4LowEchoAmplitude) {
  beam4LowEchoAmplitude = beam4LowEchoAmplitude;
}

inline uint32_t PD5Packet::GetXVelRef() {
  return xVelRef;
}

inline void PD5Packet::SetXVelRef(uint32_t velRef) {
  xVelRef = velRef;
}

inline uint32_t PD5Packet::GetYVelRef() {
  return yVelRef;
}

inline void PD5Packet::SetYVelRef(uint32_t velRef) {
  yVelRef = velRef;
}

inline uint32_t PD5Packet::GetZVelRef() {
  return zVelRef;
}

inline void PD5Packet::SetZVelRef(uint32_t velRef) {
  zVelRef = velRef;
}

inline uint32_t PD5Packet::GetEVelRef() {
  return eVelRef;
}

inline void PD5Packet::SetEVelRef(uint32_t velRef) {
  eVelRef = velRef;
}

inline uint32_t PD5Packet::GetRefLayerStart() {
  return refLayerStart;
}

inline void PD5Packet::SetRefLayerStart(uint32_t refLayerStart) {
  refLayerStart = refLayerStart;
}

inline uint32_t PD5Packet::GetRefLayerEnd() {
  return refLayerEnd;
}

inline void PD5Packet::SetRefLayerEnd(uint32_t refLayerEnd) {
  refLayerEnd = refLayerEnd;
}

inline uint8_t PD5Packet::GetRefLayerStatus() {
  return refLayerStatus;
}

inline void PD5Packet::SetRefLayerStatus(uint8_t refLayerStatus) {
  refLayerStatus = refLayerStatus;
}

inline uint8_t PD5Packet::GetHour() {
  return hour;
}

inline void PD5Packet::SetHour(uint8_t hour) {
  hour = hour;
}

inline uint8_t PD5Packet::GetMinute() {
  return minute;
}

inline void PD5Packet::SetMinute(uint8_t minute) {
  minute = minute;
}

inline uint8_t PD5Packet::GetSecond() {
  return second;
}

inline void PD5Packet::SetSecond(uint8_t second) {
  second = second;
}

inline uint8_t PD5Packet::GetHundredth() {
  return hundredth;
}

inline void PD5Packet::SetHundredth(uint8_t hundredth) {
  hundredth = hundredth;
}

inline uint32_t PD5Packet::GetBitResult() {
  return bitResult;
}

inline void PD5Packet::SetBitResult(uint32_t bitResult) {
  bitResult = bitResult;
}

inline uint32_t PD5Packet::GetSpeedOfSound() {
  return speedOfSound;
}

inline void PD5Packet::SetSpeedOfSound(uint32_t speedOfSound) {
  speedOfSound = speedOfSound;
}

inline uint32_t PD5Packet::GetTemperature() {
  return temperature;
}

inline void PD5Packet::SetTemperature(uint32_t temperature) {
  temperature = temperature;
}

inline uint8_t PD5Packet::GetSalinity() {
  return salinity;
}

inline void PD5Packet::SetSalinity(uint8_t salinity) {
  salinity = salinity;
}

inline uint32_t PD5Packet::GetDepth() {
  return depth;
}

inline void PD5Packet::SetDepth(uint32_t depth) {
  depth = depth;
}

inline uint32_t PD5Packet::GetPitch() {
  return pitch;
}

inline void PD5Packet::SetPitch(uint32_t pitch) {
  pitch = pitch;
}

inline uint32_t PD5Packet::GetRoll() {
  return roll;
}

inline void PD5Packet::SetRoll(uint32_t roll) {
  roll = roll;
}

inline uint32_t PD5Packet::GetHeading() {
  return heading;
}

inline void PD5Packet::SetHeading(uint32_t heading) {
  heading = heading;
}

inline uint32_t PD5Packet::GetDmgBottomEast() {
  return dmgBottomEast;
}

inline void PD5Packet::SetDmgBottomEast(uint32_t dmgBottomEast) {
  dmgBottomEast = dmgBottomEast;
}

inline uint32_t PD5Packet::GetDmgBottomNorth() {
  return dmgBottomNorth;
}

inline void PD5Packet::SetDmgBottomNorth(uint32_t dmgBottomNorth) {
  dmgBottomNorth = dmgBottomNorth;
}

inline uint32_t PD5Packet::GetDmgBottomUp() {
  return dmgBottomUp;
}

inline void PD5Packet::SetDmgBottomUp(uint32_t dmgBottomUp) {
  dmgBottomUp = dmgBottomUp;
}

inline uint32_t PD5Packet::GetDmgBottomError() {
  return dmgBottomError;
}

inline void PD5Packet::SetDmgBottomError(uint32_t dmgBottomError) {
  dmgBottomError = dmgBottomError;
}

inline uint32_t PD5Packet::GetRefBottomEast() {
  return refBottomEast;
}

inline void PD5Packet::SetRefBottomEast(uint32_t refBottomEast) {
  refBottomEast = refBottomEast;
}

inline uint32_t PD5Packet::GetRefBottomNorth() {
  return refBottomNorth;
}

inline void PD5Packet::SetRefBottomNorth(uint32_t refBottomNorth) {
  refBottomNorth = refBottomNorth;
}

inline uint32_t PD5Packet::GetRefBottomUp() {
  return refBottomUp;
}

inline void PD5Packet::SetRefBottomUp(uint32_t refBottomUp) {
  refBottomUp = refBottomUp;
}

inline uint32_t PD5Packet::GetRefBottomError() {
  return refBottomError;
}

inline void PD5Packet::SetRefBottomError(uint32_t refBottomError) {
  refBottomError = refBottomError;
}

#endif //PROVIDER_DVL_PD5MESSAGES_H
