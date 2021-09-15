
# Copyright (C) 2020-2021 LEIDOS.
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
project(trajectory_utils)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cav_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_utils
  DEPENDS EIGEN3 # Eigen is a special case where is needs to be in caps here but not in find_package
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(trajectory_utils 
  src/trajectory_utils/quintic_coefficient_calculator.cpp
  src/trajectory_utils/conversions/conversions.cpp
  src/trajectory_utils/trajectory_utils.cpp
)

add_dependencies(trajectory_utils ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_utils ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS trajectory_utils
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#############
## Testing ##
#############

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gmock(${PROJECT_NAME}-test 
  test/${PROJECT_NAME}/main.cpp 
  test/${PROJECT_NAME}/quintic_coefficient_calculator_test.cpp
  test/${PROJECT_NAME}/test_conversions.cpp
  test/${PROJECT_NAME}/test_trajectory_utils.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
