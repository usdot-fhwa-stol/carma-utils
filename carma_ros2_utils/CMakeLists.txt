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

cmake_minimum_required(VERSION 3.5)
project(carma_ros2_utils)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()


# Includes
include_directories(
  include
)

# TODO(CAR-6017): Remove when we drop support for Foxy
# rclcpp_lifecycle has breaking API changes starting with Humble
if(("$ENV{ROS_DISTRO}" STREQUAL "foxy") OR ("$ENV{ROS_DISTRO}" STREQUAL "galactic"))
  set(carma_ros2_utils_LIFECYCLE_PUBLISHER_INTERFACE_TYPE
    rclcpp_lifecycle::LifecyclePublisherInterface
  )
else()
  set(carma_ros2_utils_LIFECYCLE_PUBLISHER_INTERFACE_TYPE
    rclcpp_lifecycle::ManagedEntityInterface
  )
endif()

configure_file(
  src/carma_lifecycle_node.hpp.in
  carma_ros2_utils/carma_lifecycle_node.hpp
)

# Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/carma_lifecycle_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
)

include(GNUInstallDirs)

INSTALL(FILES ${PROJECT_BINARY_DIR}/carma_ros2_utils/carma_lifecycle_node.hpp
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/carma_ros2_utils
)

ament_auto_add_library(carma_ros2_utils_timers SHARED
  src/timers/testing/TestTimer.cpp
  src/timers/testing/TestTimerFactory.cpp
  src/timers/testing/TestClock.cpp
  src/timers/ROSTimer.cpp
  src/timers/ROSTimerFactory.cpp
)

ament_auto_add_library(lifecycle_component_wrapper SHARED
  src/lifecycle_component_wrapper.cpp
)

ament_auto_add_library(carma_component_manager SHARED
  src/component_manager.cpp
)

ament_auto_add_executable(lifecycle_component_wrapper_st
  src/lifecycle_component_wrapper_st.cpp
)

ament_auto_add_executable(lifecycle_component_wrapper_mt
  src/lifecycle_component_wrapper_mt.cpp
)

ament_auto_add_executable(carma_component_container
  src/component_container.cpp
)

ament_auto_add_executable(carma_component_container_mt
  src/component_container_mt.cpp
)

# Link local targets

target_link_libraries(carma_ros2_utils_timers
  ${PROJECT_NAME}
)

target_link_libraries(lifecycle_component_wrapper
  ${PROJECT_NAME}
)

target_link_libraries(lifecycle_component_wrapper_st
  lifecycle_component_wrapper
  ${PROJECT_NAME}
)

target_link_libraries(lifecycle_component_wrapper_mt
  lifecycle_component_wrapper
  ${PROJECT_NAME}
)

target_link_libraries(carma_component_container
  carma_component_manager
)

target_link_libraries(carma_component_container_mt
  carma_component_manager
)

if(BUILD_TESTING)

  add_subdirectory(test)

  rclcpp_components_register_nodes(test_minimal_node "carma_ros2_utils_testing::MinimalNode")

endif()

ament_python_install_package(${PROJECT_NAME})

ament_auto_package()
