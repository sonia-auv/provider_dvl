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
void EthernetSocket::Connect(std::string address, int port) {
  int socket_desc;
  struct sockaddr_in server;
  char *message, server_reply[2000];

  //Create socket
  socket_desc = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_desc == -1) {
    ROS_DEBUG("Could not create socket");
  }

  server.sin_addr.s_addr = inet_addr("192.168.0.240");
  server.sin_family = AF_INET;
  server.sin_port = htons(9002);

  //Connect to remote server
  if (connect(socket_desc, (struct sockaddr *) &server, sizeof(server)) < 0) {
    ROS_DEBUG("connect error");
    return;
  }

  ROS_DEBUG("Connected\n");

  //Receive a reply from the server
  if (recv(socket_desc, server_reply, 2000, 0) < 0) {
    ROS_DEBUG("recv failed");
  }
  ROS_DEBUG("Reply received\n");
  ROS_DEBUG(server_reply);
}
