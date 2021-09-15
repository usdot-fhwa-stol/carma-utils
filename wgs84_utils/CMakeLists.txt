
# Copyright (C) 2019-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 2.8.3)
project(wgs84_utils)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wgs84_utils_library
  CATKIN_DEPENDS roscpp tf2 tf2_ros tf2_geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers */*.hpp */*.h)

add_library(wgs84_utils_library src/wgs84_utils/wgs84_utils.cpp src/wgs84_utils/proj_tools.cpp)
target_link_libraries(wgs84_utils_library)
add_dependencies(wgs84_utils_library ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS wgs84_utils_library
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

##########
## Test ##
##########

catkin_add_gmock(wgs84_utils_test test/test_main.cpp test/proj_tools_test.cpp)

target_link_libraries( wgs84_utils_test
  wgs84_utils_library
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)
