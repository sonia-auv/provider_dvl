/**
 * \file	ethernet_socket.h
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

#ifndef ETHERNET_SOCKET_H
#define ETHERNET_SOCKET_H

#include <string>

#include <sys/socket.h>
#include <arpa/inet.h>

class EthernetSocket {
 public:
  //==========================================================================
  // P U B L I C   C / D T O R S

  EthernetSocket();
  ~EthernetSocket();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void ConnectUDP(int port);
  void ConnectTCP(int port);
  void Receive();
  char* GetRawData();

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::string hostname_ = "192.168.0.32";

  int socket_;
  sockaddr_in server_udp_, server_tcp_, dvl_;
  char data_[88];  //fix with real data

};

#endif //ETHERNET_SOCKET_H
