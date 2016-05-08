/**
 * \file	pd0_message.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DVL_TELEDYNE_PD0PARSER_HPP
#define DVL_TELEDYNE_PD0PARSER_HPP

#include <stdint.h>
#include <boost/static_assert.hpp>
#include <eigen3/Eigen/Eigen>
#include <provider_dvl/base/BaseTime.hpp>
#include <vector>

#include <provider_dvl/driver/PD0Messages.hpp>

namespace dvl_teledyne {
using namespace PD0Message;
class PD0Parser {
 protected:
  int extractPacket(uint8_t const* buffer, size_t size,
                    size_t max_size = 0) const;
  int getSizeOfMessage(uint16_t msg_id) const;
  void invalidateCellReadings();
  void parseMessage(uint8_t const* buffer, size_t size);
  void parseFixedLeader(uint8_t const* buffer, size_t size);
  void parseVariableLeader(uint8_t const* buffer, size_t size);
  void parseQualityReadings(uint8_t const* buffer, size_t size);
  void parseCorrelationReadings(uint8_t const* buffer, size_t size);
  void parseIntensityReadings(uint8_t const* buffer, size_t size);
  void parseVelocityReadings(uint8_t const* buffer, size_t size);
  void parseBottomTrackingReadings(uint8_t const* buffer, size_t size);

 public:
  DeviceInfo deviceInfo;
  AcquisitionConfiguration acqConf;
  OutputConfiguration outputConf;
  Status status;
  CellReadings cellReadings;
  BottomTrackingConfiguration bottomTrackingConf;
  BottomTracking bottomTracking;

  void parseEnsemble(uint8_t const* data, size_t size);
};
}

#endif
