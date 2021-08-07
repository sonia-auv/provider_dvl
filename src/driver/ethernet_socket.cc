/**
 * \file	ethernet_socket.cc
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	09/05/2017
 *
 * \copyright Copyright (c) 2017 S.O.N.I.A. All rights reserved.
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

#include <ros/ros.h>
#include "driver/ethernet_socket.h"

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
EthernetSocket::EthernetSocket() {}

//------------------------------------------------------------------------------
//
EthernetSocket::~EthernetSocket() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void EthernetSocket::ConnectUDP(int port) {

  bzero(&server_udp_, sizeof(server_udp_));
  server_udp_.sin_addr.s_addr = htonl(INADDR_ANY);
  server_udp_.sin_family = AF_INET;
  server_udp_.sin_port = htons(port);

  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  ROS_ASSERT(socket_ != -1);
  
  ROS_ASSERT(bind(socket_, (struct sockaddr*)&server_udp_, sizeof(server_udp_)) != -1);

  ROS_DEBUG("Connected\n");
}

void EthernetSocket::ConnectTCP(int port) {

  bzero(&server_tcp_, sizeof(server_tcp_));
  server_tcp_.sin_addr.s_addr = inet_addr(hostname_.c_str());
  server_tcp_.sin_family = AF_INET;
  server_tcp_.sin_port = htons(port);

  ROS_ASSERT(connect(socket_, (struct sockaddr *) &server_tcp_, sizeof(server_tcp_)) != -1);

  ROS_DEBUG("Connected\n");
}

//------------------------------------------------------------------------------
//
void EthernetSocket::Receive() {
  socklen_t len = sizeof(dvl_);
  if (recvfrom(socket_, data_, sizeof(data_), 0, (struct sockaddr*) &dvl_, &len) < 0) {
    ROS_INFO("Receive failed");
  }
  ROS_DEBUG("Reply received");
}

//------------------------------------------------------------------------------
//
char* EthernetSocket::GetRawData() {
    return data_;
}

