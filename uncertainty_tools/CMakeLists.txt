
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
project(uncertainty_tools)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake_auto REQUIRED)

include_directories(
  include
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}-test test/${PROJECT_NAME}/uncertainty_tools_test.cpp)
  ament_target_dependencies(${PROJECT_NAME}-test ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})

endif()

# Install
ament_auto_package()
