/**
 * \file	main.cc
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	07/05/2017
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
#include <string>
#include "provider_dvl/provider_dvl_node.h"
#include "dvl_data.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "provider_dvl");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  std::string dvl_type;

  if (nh->hasParam("/Provider_dvl/dvl_type")) {
    nh->getParam("/Provider_dvl/dvl_type", dvl_type);
  }

  if(dvl_type == "Pathfinder")
  {
      PathfinderDvl provider_dvl_Pathfinder(nh,"192.168.0.32", 1035, 1033);
      provider_dvl_Pathfinder.Spin();
  }
  else if(dvl_type == "Nortek")
  {
      NortekDvl provider_dvl_Nortek(nh,"192.168.0.240", 9002);
      provider_dvl_Nortek.Spin();
  }
  
  

  return 0;
}
