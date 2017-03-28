# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
#
# \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
#
# \section LICENSE
#
# This file is part of S.O.N.I.A. software.
#
# S.O.N.I.A. software is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# S.O.N.I.A. software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)
project(provider_dvl)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s")

set(provider_dvl_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    message_generation
    lib_atlas

    )

add_message_files(
        FILES
        AcquisitionConfiguration.msg
        BottomTracking.msg
        BottomTracking.msg
        CellReading.msg
        CellReadings.msg
        DeviceInfo.msg
        OutputConfiguration.msg
        PD0Packet.msg
        Sensors.msg
        Status.msg
)

generate_messages(
        DEPENDENCIES

        std_msgs
)

# Declare this project as a catkin package and export the necessary build flags
catkin_package(
    INCLUDE_DIRS ${provider_dvl_SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime lib_atlas
)





#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE provider_dvl_FILES
    "${provider_dvl_SRC_DIR}/*.cc"
    "${provider_dvl_SRC_DIR}/*.cpp"
    "${provider_dvl_SRC_DIR}/*.h"
    "${provider_dvl_SRC_DIR}/*.hpp")

list(REMOVE_ITEM provider_dvl_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/${provider_dvl_SRC_DIR}/${PROJECT_NAME}/Main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/${provider_dvl_SRC_DIR}/${PROJECT_NAME}/MainConfigure.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/${provider_dvl_SRC_DIR}/${PROJECT_NAME}/MainInfo.cpp)

# Include all the directories where source can be found.
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${provider_dvl_SRC_DIR}
    ${lib_atlas_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

set(TARGET_NAME provider_dvl_node)

# Creating the ROS Node
add_executable(${PROJECT_NAME}_node ${provider_dvl_SRC_DIR}/${PROJECT_NAME}/Main.cpp ${provider_dvl_FILES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${lib_atlas_LIBRARIES})

set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE CXX)




#==========================================================================
# U N I T   T E S T S

if (CATKIN_ENABLE_TESTING AND ${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif ()
