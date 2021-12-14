
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
carma_package()

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)
 
  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS)

  catkin_package()

  ###########
  ## Build ##
  ###########

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    include_directories(
      include
      ${catkin_INCLUDE_DIRS}
    )

  add_library(${PROJECT_NAME} src/${PROJECT_NAME}/${PROJECT_NAME}.cpp)
  target_link_libraries(${PROJECT_NAME})
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


  #############
  ## Install ##
  #############

  ## Mark executables and/or libraries for installation
  install(TARGETS ${PROJECT_NAME}
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

  # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  catkin_add_gmock(${PROJECT_NAME}-test test/${PROJECT_NAME}/uncertainty_tools_test.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()

else() #ROS2

  find_package(ament_cmake_auto REQUIRED)

  include_directories(
    include
  )

  ament_auto_add_library(${PROJECT_NAME}
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

  ament_auto_package(
    INSTALL_TO_SHARE
  )

endif()

