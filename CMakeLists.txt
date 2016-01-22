cmake_minimum_required(VERSION 2.8.3)
project(provider_dvl)

if ( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Debug CACHE STRING
          "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
          FORCE )
endif ( NOT CMAKE_BUILD_TYPE )

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11" )
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3" )

set(provider_dvl_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sonia_msgs
)
#===============================================================================
# G E N E R A T E   M E S S A G E S   &   S E R V I C E S

# Declare this project as a catkin package and export the necessary build flags
catkin_package(
    INCLUDE_DIRS ${provider_dvl_SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS roscpp sonia_msgs
)

#===============================================================================
# I N C L U D E   F I L E S

# Get all the files of the projects.
file(GLOB_RECURSE provider_dvl_FILES
        "${provider_dvl_SRC_DIR}/provider_dvl/base/*.cpp"
        "${provider_dvl_SRC_DIR}/provider_dvl/base/*.hpp"
        "${provider_dvl_SRC_DIR}/provider_dvl/driver/*.cpp"
        "${provider_dvl_SRC_DIR}/provider_dvl/driver/*.hpp"
        "${provider_dvl_SRC_DIR}/provider_dvl/message_builder.hpp"
        "${provider_dvl_SRC_DIR}/provider_dvl/message_builder.cpp")

# Include all the directories where source can be found.
include_directories(
        ${provider_dvl_SRC_DIR}
        ${catkin_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

set( TARGET_NAME provider_dvl_node )

# Creating the ROS Node
add_executable( ${TARGET_NAME} src/provider_dvl/Main.cpp ${provider_dvl_FILES})
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES} )
set_target_properties( ${TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Creating the Info exec
set( TARGET_NAME info )
add_executable( ${TARGET_NAME} src/provider_dvl/MainInfo.cpp ${provider_dvl_FILES})
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES} )
set_target_properties( ${TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX )

# Creating the Config exec
set( TARGET_NAME conf )
add_executable( ${TARGET_NAME} src/provider_dvl/MainConfigure.cpp ${provider_dvl_FILES})
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})
set_target_properties( ${TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX )

#==========================================================================
# C R E A T I N G   U N I T   T E S T S
