# Copyright 2024 Leidos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(ros2_lifecycle_manager)

# CARMA builds packages in an environment with both ROS 1 and ROS 2 installed.
# This check gracefully skips the current package if the sourced ROS environment
# is not the specified version. This call must come before any other ROS
# dependencies becasue ROS 1 does not have some of the required packages.
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)

include(dependencies.cmake)

carma_package()

ament_auto_add_library(ros2_lifecycle_manager SHARED
  src/ros2_lifecycle_manager.cpp
)

if(BUILD_TESTING)
  # These CMake commands were added to ament_cmake_auto in ROS 2 Humble. Until
  # CARMA supports ROS 2 Humble, we will use package-local copies.
  include(cmake/ament_auto_find_test_dependencies.cmake)
  include(cmake/ament_auto_add_gtest.cmake)

  ament_auto_add_executable(test_lifecycle_node
    test/test_node.cpp
  )

  ament_add_gtest_executable(test_lifecycle_gtest
    test/test_lifecycle_manager.cpp
  )

  # ament_add_gtest_executable() does not automaticallly link libraries
  target_link_libraries(test_lifecycle_gtest
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ros2_lifecycle_manager
  )

  # TODO(CAR-6015): Replace this with launch_testing package
  ament_add_test(test_lifecycle
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${PROJECT_SOURCE_DIR}/test/launch_lifecycle_test.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    TIMEOUT 20
    ENV
      TEST_EXECUTABLE=$<TARGET_FILE:test_lifecycle_gtest>
  )
endif()

ament_auto_package()
