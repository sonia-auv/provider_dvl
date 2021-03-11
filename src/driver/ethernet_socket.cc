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
  struct sockaddr_in server;

  bzero(&server, sizeof(server));
  server.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr(address.c_str());
  server.sin_family = AF_INET;
  server.sin_port = htons(port);

  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_ == -1) {
    ROS_DEBUG("Could not create socket");
  }
  //bind(socket_, (struct sockaddr*)&address, sizeof(address)); 

  if (connect(socket_, (struct sockaddr *) &server, sizeof(server)) < 0) {
    ROS_DEBUG("Connect error");
    return;
  }
  ROS_DEBUG("Connected\n");
}

/*void EthernetSocket::Connect(std::string address, int port) {
  struct sockaddr_in server;

  socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_ == -1) {
    ROS_DEBUG("Could not create socket");
  }
  //bind(socket_, (struct sockaddr*)&address, sizeof(address)); 
  server.sin_addr.s_addr = INADDR_ANY;//inet_addr(address.c_str());
  server.sin_family = AF_INET;
  server.sin_port = htons(port);

  if (connect(socket_, (struct sockaddr *) &server, sizeof(server)) < 0) {
    ROS_DEBUG("Connect error");
    return;
  }

  ROS_DEBUG("Connected\n");
}*/

//------------------------------------------------------------------------------
//
void EthernetSocket::Receive() {
  char *message = "0";

  sendto(socket_, message, 4096, 0 (struct sockaddr*)NULL, sizeof(servaddr));
  
  if (recvfrom(socket_, data_, sizeof(data_), 0, (struct sockaddr*) &dvl_adress, &addr_len) < 0) {
    ROS_DEBUG("Receive failed");
  }
  ROS_DEBUG("Reply received");
}

//------------------------------------------------------------------------------
//
char* EthernetSocket::GetRawData() {
    return data_;
}

