//
// Created by jeremie on 10/29/15.
//

#ifndef PROVIDER_DVL_PD5PACKET_H
#define PROVIDER_DVL_PD5PACKET_H

#include <stdint.h>

namespace dvl_teledyne {
class PD5Packet {
 public:
  static const uint32_t MESSAGE_SIZE = 86;
  static const uint8_t PD4 = 0;
  static const uint8_t PD5 = 1;

  PD5Packet();
  ~PD5Packet();
  int8_t GetSystemConfig();
  void SetSystemConfig(int8_t systemConfig);

  int16_t GetXVelBtm();
  void SetXVelBtm(int16_t velBtm);

  int16_t GetYVelBtm();
  void SetYVelBtm(int16_t velBtm);

  int16_t GetZVelBtm();
  void SetZVelBtm(int16_t velBtm);

  int16_t GetEVelBtm();
  void SetEVelBtm(int16_t velBtm);

  uint16_t GetBm1RngBtm();
  void SetBm1RngBtm(uint16_t bm1RngBtm);

  uint16_t GetBm2RngBtm();
  void SetBm2RngBtm(uint16_t bm2RngBtm);

  uint16_t GetBm3RngBtm();
  void SetBm3RngBtm(uint16_t bm3RngBtm);

  uint16_t GetBm4RngBtm();
  void SetBm4RngBtm(uint16_t bm4RngBtm);

  int8_t GetBottomStatus();
  void SetBottomStatus(int8_t bottomStatus);

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

  int16_t GetXVelRef();
  void SetXVelRef(int16_t velRef);

  int16_t GetYVelRef();
  void SetYVelRef(int16_t velRef);

  int16_t GetZVelRef();
  void SetZVelRef(int16_t velRef);

  int16_t GetEVelRef();
  void SetEVelRef(int16_t velRef);

  uint16_t GetRefLayerStart();
  void SetRefLayerStart(uint16_t refLayerStart);

  uint16_t GetRefLayerEnd();
  void SetRefLayerEnd(uint16_t refLayerEnd);

  int8_t GetRefLayerStatus();
  void SetRefLayerStatus(int8_t refLayerStatus);

  int8_t GetHour();
  void SetHour(int8_t hour);

  int8_t GetMinute();
  void SetMinute(int8_t minute);

  int8_t  GetSecond();
  void SetSecond(int8_t  second);

  int8_t GetHundredth();
  void SetHundredth(int8_t  hundredth);

  uint32_t GetBitResult();
  void SetBitResult(uint32_t bitResult);

  uint32_t GetSpeedOfSound();
  void SetSpeedOfSound(uint32_t speedOfSound);

  int16_t GetTemperature();
  void SetTemperature(int16_t temperature);

  int8_t GetSalinity();
  void SetSalinity(int8_t salinity);

  uint16_t GetDepth();
  void SetDepth(uint16_t depth);

  int16_t GetPitch();
  void SetPitch(int16_t pitch);

  int16_t GetRoll();
  void SetRoll(int16_t roll);

  uint16_t GetHeading();
  void SetHeading(uint16_t heading);

  int32_t GetDmgBottomEast();
  void SetDmgBottomEast(int32_t dmgBottomEast);

  int32_t GetDmgBottomNorth();
  void SetDmgBottomNorth(int32_t dmgBottomNorth);

  int32_t GetDmgBottomUp();
  void SetDmgBottomUp(int32_t dmgBottomUp);

  int32_t GetDmgBottomError();
  void SetDmgBottomError(int32_t dmgBottomError);

  int32_t GetRefBottomEast();
  void SetRefBottomEast(int32_t refBottomEast);

  int32_t GetRefBottomNorth();
  void SetRefBottomNorth(int32_t refBottomNorth);

  int32_t GetRefBottomUp();
  void SetRefBottomUp(int32_t refBottomUp);

  int32_t GetRefBottomError();
  void SetRefBottomError(int32_t refBottomError);

 private:
  // System config
  int8_t systemConfig;

  // Bottom Velocities
  int16_t xVelBtm;
  int16_t yVelBtm;
  int16_t zVelBtm;
  int16_t eVelBtm;

  // Beam Range
  uint16_t bm1RngBtm;
  uint16_t bm2RngBtm;
  uint16_t bm3RngBtm;
  uint16_t bm4RngBtm;

  // Bottom status
  int8_t bottomStatus;

  uint32_t beam1LowCorrelation;
  uint32_t beam1LowEchoAmplitude;
  uint32_t beam2LowCorrelation;
  uint32_t beam2LowEchoAmplitude;
  uint32_t beam3LowCorrelation;
  uint32_t beam3LowEchoAmplitude;
  uint32_t beam4LowCorrelation;
  uint32_t beam4LowEchoAmplitude;

  // WaterMass Velocities
  int16_t xVelRef;
  int16_t yVelRef;
  int16_t zVelRef;
  int16_t eVelRef;

  // WaterMass Start End Status
  uint16_t refLayerStart;
  uint16_t refLayerEnd;
  int8_t refLayerStatus;

  // Time of first ping (TOFP)
  int8_t hour;
  int8_t minute;
  int8_t second;
  int8_t hundredth;

  // Built-in test result
  uint32_t bitResult;

  // Environemental data
  uint32_t speedOfSound;
  int16_t temperature;
  int8_t salinity;

  // Instrument data
  uint16_t depth;
  int16_t pitch;
  int16_t roll;
  uint16_t heading;

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
//==============================================================================
// INLINE CODE SECTION
//------------------------------------------------------------------------------
//
inline int8_t  PD5Packet::GetSystemConfig() {
  return systemConfig;
}

inline void PD5Packet::SetSystemConfig(int8_t systemConfig_) {
  systemConfig = systemConfig_;
}

inline int16_t PD5Packet::GetXVelBtm() {
  return xVelBtm;
}

inline void PD5Packet::SetXVelBtm(int16_t velBtm) {
  xVelBtm = velBtm;
}

inline int16_t PD5Packet::GetYVelBtm() {
  return yVelBtm;
}

inline void PD5Packet::SetYVelBtm(int16_t velBtm) {
  yVelBtm = velBtm;
}

inline int16_t PD5Packet::GetZVelBtm() {
  return zVelBtm;
}

inline void PD5Packet::SetZVelBtm(int16_t velBtm) {
  zVelBtm = velBtm;
}

inline int16_t PD5Packet::GetEVelBtm() {
  return eVelBtm;
}

inline void PD5Packet::SetEVelBtm(int16_t velBtm) {
  eVelBtm = velBtm;
}

inline uint16_t PD5Packet::GetBm1RngBtm() {
  return bm1RngBtm;
}

inline void PD5Packet::SetBm1RngBtm(uint16_t bm1RngBtm_) {
  bm1RngBtm = bm1RngBtm_;
}

inline uint16_t PD5Packet::GetBm2RngBtm() {
  return bm2RngBtm;
}

inline void PD5Packet::SetBm2RngBtm(uint16_t bm2RngBtm_) {
  bm2RngBtm = bm2RngBtm_;
}

inline uint16_t PD5Packet::GetBm3RngBtm() {
  return bm3RngBtm;
}

inline void PD5Packet::SetBm3RngBtm(uint16_t bm3RngBtm_) {
  bm3RngBtm = bm3RngBtm_;
}

inline uint16_t PD5Packet::GetBm4RngBtm() {
  return bm4RngBtm;
}

inline void PD5Packet::SetBm4RngBtm(uint16_t bm4RngBtm_) {
  bm4RngBtm = bm4RngBtm_;
}

inline int8_t PD5Packet::GetBottomStatus() {
  return bottomStatus;
}

inline void PD5Packet::SetBottomStatus(int8_t bottomStatus_) {
  bottomStatus = bottomStatus_;
}

inline uint32_t PD5Packet::GetBeam1LowCorrelation() {
  return beam1LowCorrelation;
}

inline void PD5Packet::SetBeam1LowCorrelation(uint32_t beam1LowCorrelation_) {
  beam1LowCorrelation = beam1LowCorrelation_;
}

inline uint32_t PD5Packet::GetBeam1LowEchoAmplitude() {
  return beam1LowEchoAmplitude;
}

inline void PD5Packet::SetBeam1LowEchoAmplitude(uint32_t beam1LowEchoAmplitude_) {
  beam1LowEchoAmplitude = beam1LowEchoAmplitude_;
}

inline uint32_t PD5Packet::GetBeam2LowCorrelation() {
  return beam2LowCorrelation;
}

inline void PD5Packet::SetBeam2LowCorrelation(uint32_t beam2LowCorrelation_) {
  beam2LowCorrelation = beam2LowCorrelation_;
}

inline uint32_t PD5Packet::GetBeam2LowEchoAmplitude() {
  return beam2LowEchoAmplitude;
}

inline void PD5Packet::SetBeam2LowEchoAmplitude(uint32_t beam2LowEchoAmplitude_) {
  beam2LowEchoAmplitude = beam2LowEchoAmplitude_;
}

inline uint32_t PD5Packet::GetBeam3LowCorrelation() {
  return beam3LowCorrelation;
}

inline void PD5Packet::SetBeam3LowCorrelation(uint32_t beam3LowCorrelation_) {
  beam3LowCorrelation = beam3LowCorrelation_;
}

inline uint32_t PD5Packet::GetBeam3LowEchoAmplitude() {
  return beam3LowEchoAmplitude;
}

inline void PD5Packet::SetBeam3LowEchoAmplitude(uint32_t beam3LowEchoAmplitude_) {
  beam3LowEchoAmplitude = beam3LowEchoAmplitude_;
}

inline uint32_t PD5Packet::GetBeam4LowCorrelation() {
  return beam4LowCorrelation;
}

inline void PD5Packet::SetBeam4LowCorrelation(uint32_t beam4LowCorrelation_) {
  beam4LowCorrelation = beam4LowCorrelation_;
}

inline uint32_t PD5Packet::GetBeam4LowEchoAmplitude() {
  return beam4LowEchoAmplitude;
}

inline void PD5Packet::SetBeam4LowEchoAmplitude(uint32_t beam4LowEchoAmplitude_) {
  beam4LowEchoAmplitude = beam4LowEchoAmplitude_;
}

inline int16_t PD5Packet::GetXVelRef() {
  return xVelRef;
}

inline void PD5Packet::SetXVelRef(int16_t velRef) {
  xVelRef = velRef;
}

inline int16_t PD5Packet::GetYVelRef() {
  return yVelRef;
}

inline void PD5Packet::SetYVelRef(int16_t velRef) {
  yVelRef = velRef;
}

inline int16_t PD5Packet::GetZVelRef() {
  return zVelRef;
}

inline void PD5Packet::SetZVelRef(int16_t velRef) {
  zVelRef = velRef;
}

inline int16_t PD5Packet::GetEVelRef() {
  return eVelRef;
}

inline void PD5Packet::SetEVelRef(int16_t velRef) {
  eVelRef = velRef;
}

inline uint16_t PD5Packet::GetRefLayerStart() {
  return refLayerStart;
}

inline void PD5Packet::SetRefLayerStart(uint16_t refLayerStart_) {
  refLayerStart = refLayerStart_;
}

inline uint16_t PD5Packet::GetRefLayerEnd() {
  return refLayerEnd;
}

inline void PD5Packet::SetRefLayerEnd(uint16_t refLayerEnd_) {
  refLayerEnd = refLayerEnd_;
}

inline int8_t PD5Packet::GetRefLayerStatus() {
  return refLayerStatus;
}

inline void PD5Packet::SetRefLayerStatus(int8_t refLayerStatus_) {
  refLayerStatus = refLayerStatus_;
}

inline int8_t PD5Packet::GetHour() {
  return hour;
}

inline void PD5Packet::SetHour(int8_t hour_) {
  hour = hour_;
}

inline int8_t PD5Packet::GetMinute() {
  return minute;
}

inline void PD5Packet::SetMinute(int8_t minute_) {
  minute = minute_;
}

inline int8_t PD5Packet::GetSecond() {
  return second;
}

inline void PD5Packet::SetSecond(int8_t  second_) {
  second = second_;
}

inline int8_t PD5Packet::GetHundredth() {
  return hundredth;
}

inline void PD5Packet::SetHundredth(int8_t hundredth_) {
  hundredth = hundredth_;
}

inline uint32_t PD5Packet::GetBitResult() {
  return bitResult;
}

inline void PD5Packet::SetBitResult(uint32_t bitResult_) {
  bitResult = bitResult_;
}

inline uint32_t PD5Packet::GetSpeedOfSound() {
  return speedOfSound;
}

inline void PD5Packet::SetSpeedOfSound(uint32_t speedOfSound_) {
  speedOfSound = speedOfSound_;
}

inline int16_t PD5Packet::GetTemperature() {
  return temperature;
}

inline void PD5Packet::SetTemperature(int16_t temperature_) {
  temperature = temperature_;
}

inline int8_t PD5Packet::GetSalinity() {
  return salinity;
}

inline void PD5Packet::SetSalinity(int8_t salinity_) {
  salinity = salinity_;
}

inline uint16_t PD5Packet::GetDepth() {
  return depth;
}

inline void PD5Packet::SetDepth(uint16_t depth_) {
  depth = depth_;
}

inline int16_t PD5Packet::GetPitch() {
  return pitch;
}

inline void PD5Packet::SetPitch(int16_t pitch_) {
  pitch = pitch_;
}

inline int16_t PD5Packet::GetRoll() {
  return roll;
}

inline void PD5Packet::SetRoll(int16_t roll_) {
  roll = roll_;
}

inline uint16_t PD5Packet::GetHeading() {
  return heading;
}

inline void PD5Packet::SetHeading(uint16_t heading_) {
  heading = heading_;
}

inline int32_t PD5Packet::GetDmgBottomEast() {
  return dmgBottomEast;
}

inline void PD5Packet::SetDmgBottomEast(int32_t dmgBottomEast_) {
  dmgBottomEast = dmgBottomEast_;
}

inline int32_t PD5Packet::GetDmgBottomNorth() {
  return dmgBottomNorth;
}

inline void PD5Packet::SetDmgBottomNorth(int32_t dmgBottomNorth_) {
  dmgBottomNorth = dmgBottomNorth_;
}

inline int32_t PD5Packet::GetDmgBottomUp() {
  return dmgBottomUp;
}

inline void PD5Packet::SetDmgBottomUp(int32_t dmgBottomUp_) {
  dmgBottomUp = dmgBottomUp_;
}

inline int32_t PD5Packet::GetDmgBottomError() {
  return dmgBottomError;
}

inline void PD5Packet::SetDmgBottomError(int32_t dmgBottomError_) {
  dmgBottomError = dmgBottomError_;
}

inline int32_t PD5Packet::GetRefBottomEast() {
  return refBottomEast;
}

inline void PD5Packet::SetRefBottomEast(int32_t refBottomEast_) {
  refBottomEast = refBottomEast_;
}

inline int32_t PD5Packet::GetRefBottomNorth() {
  return refBottomNorth;
}

inline void PD5Packet::SetRefBottomNorth(int32_t refBottomNorth_) {
  refBottomNorth = refBottomNorth_;
}

inline int32_t PD5Packet::GetRefBottomUp() {
  return refBottomUp;
}

inline void PD5Packet::SetRefBottomUp(int32_t refBottomUp_) {
  refBottomUp = refBottomUp_;
}

inline int32_t PD5Packet::GetRefBottomError() {
  return refBottomError;
}

inline void PD5Packet::SetRefBottomError(int32_t refBottomError_) {
  refBottomError = refBottomError_;
}

}// end workspace teledyne_dvl
#endif //PROVIDER_DVL_PD5MESSAGES_H
