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
EthernetSocket::EthernetSocket()
: mData{nullptr}, mSize{}
{
}

EthernetSocket::EthernetSocket(size_t dataSize)
: mData{new char[dataSize]}, mSize{dataSize}
{
  if (!mData) 
  {
    ROS_INFO_STREAM("Data container is empty, closing connection");
    this->~EthernetSocket();
  }
}

//------------------------------------------------------------------------------
//
EthernetSocket::~EthernetSocket() 
{
  close(socketTCP_);
  close(socketUDP_);
  delete(mData);
}

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
void EthernetSocket::ConnectTCP(std::string addr, int port) {

  bzero(&server_, sizeof(server_));

  server_.sin_addr.s_addr = inet_addr(addr.c_str());
  server_.sin_family = AF_INET;
  server_.sin_port = htons(port);

  socketTCP_ = socket(AF_INET, SOCK_STREAM, 0);
  ROS_ASSERT(socketTCP_ != -1);

  ROS_ASSERT(connect(socketTCP_, (struct sockaddr *) &server_, sizeof(server_)) != -1);

  ROS_INFO_STREAM("Connected TCP");
}

//------------------------------------------------------------------------------
//
bool EthernetSocket::Receive(int socket) {
  socklen_t len = sizeof(dvl_);
  if ( recvfrom(socketTCP_, mData, mSize, 0, (struct sockaddr*) &dvl_, &len) < 0) {
    ROS_INFO_STREAM("Receive failed");
    return false;
  }
  ROS_DEBUG("Receive successed");
  return true;
}

//------------------------------------------------------------------------------
//
bool EthernetSocket::ReceiveUDP() {
  return Receive(socketUDP_);
}
//------------------------------------------------------------------------------
//
bool EthernetSocket::ReceiveTCP() {
  return Receive(socketTCP_);
}

//------------------------------------------------------------------------------
//
void EthernetSocket::Send(char *data) {
  if(send(socketTCP_, data, strlen(data), 0) < 0) {
    ROS_INFO_STREAM("Send failed");
  }
  ROS_DEBUG("Send successed");
}
//------------------------------------------------------------------------------
//
char* EthernetSocket::GetRawData() {
    return mData;
}

