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

  bzero(&server_, sizeof(server_));
  server_.sin_addr.s_addr = htonl(INADDR_ANY);
  server_.sin_family = AF_INET;
  server_.sin_port = htons(port);

  socketUDP_ = socket(AF_INET, SOCK_DGRAM, 0);
  ROS_ASSERT(socketUDP_ != -1);
  
  ROS_ASSERT(bind(socketUDP_, (struct sockaddr*)&server_, sizeof(server_)) != -1);

  ROS_INFO_STREAM("Connected UDP");
}

//------------------------------------------------------------------------------
//
void EthernetSocket::ConnectTCP(int port) {

  socklen_t clilen;

  bzero(&server_, sizeof(server_));
  server_.sin_addr.s_addr = htonl(INADDR_ANY);
  server_.sin_family = AF_INET;
  server_.sin_port = htons(port);

  socketTCP_ = socket(AF_INET, SOCK_STREAM, 0);
  ROS_ASSERT(socketTCP_ != -1);

  //ROS_ASSERT(bind(socketTCP_, (struct sockaddr*)&server_, sizeof(server_)) != -1);

  //listen(socketTCP_, 5);
  
  //clilen = sizeof(cli_addr_);

  //cli_socket_ = accept(socketTCP_, (struct sockaddr*)&cli_addr_, &clilen);

  //ROS_INFO_STREAM("Connection accepted");
  //ROS_ASSERT(cli_socket_ != -1);

  ROS_INFO_STREAM("Try to connect to the server");

  ROS_ASSERT(connect(socketTCP_, (struct sockaddr *) &server_, sizeof(server_)) != -1);

  ROS_INFO_STREAM("Connected TCP\n");
}

//------------------------------------------------------------------------------
//
void EthernetSocket::Receive() {
  socklen_t len = sizeof(dvl_);
  if (recvfrom(socketUDP_, data_, sizeof(data_), 0, (struct sockaddr*) &dvl_, &len) < 0) {
    ROS_INFO("Receive failed");
  }
  ROS_DEBUG("Receive successed");
}

//------------------------------------------------------------------------------
//
void EthernetSocket::Send(char *data) {
  if(send(cli_socket_, data, strlen(data), 0) < 0) {
    ROS_INFO("Send failed");
  }
  ROS_DEBUG("Send successed");
}
//------------------------------------------------------------------------------
//
char* EthernetSocket::GetRawData() {
    return data_;
}

