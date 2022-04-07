
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

cmake_minimum_required(VERSION 3.5)
project(wgs84_utils)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

find_package(ament_lint_auto REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers */*.hpp */*.h)

ament_auto_add_library(wgs84_utils_library SHARED 
                        src/wgs84_utils/wgs84_utils.cpp 
                        src/wgs84_utils/proj_tools.cpp)

target_link_libraries(wgs84_utils_library)

# Install
ament_auto_package(
        INSTALL_TO_SHARE
)

##########
## Test ##
##########


if(BUILD_TESTING)  

  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_wgs84_utils test/test_main.cpp)

  ament_target_dependencies(test_wgs84_utils ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_wgs84_utils wgs84_utils_library)

endif()

