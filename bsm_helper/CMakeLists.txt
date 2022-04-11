# Copyright (C) 2021 LEIDOS.
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
project(bsm_helper)

find_package(carma_cmake_common REQUIRED)
carma_package()

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1) # ROS 1

  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    
    roscpp
  )
  ## System dependencies are found with CMake's conventions
  find_package(Eigen3 REQUIRED)

  ## The catkin_package macro generates cmake config files for your package
  catkin_package(
  INCLUDE_DIRS include
    LIBRARIES bsm_helper
    CATKIN_DEPENDS  roscpp
    DEPENDS EIGEN3 # Eigen is a special case where is needs to be in caps here but not in find_package
  )

  ## Specify additional locations of header files
  include_directories(
  include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )

  ## Declare a C++ library
  add_library(bsm_helper
        src/bsm_helper.cpp
  )

  ## Add cmake target dependencies of the library
  add_dependencies(bsm_helper ${catkin_EXPORTED_TARGETS})


  ## Specify libraries to link a library or executable target against
  target_link_libraries(bsm_helper ${catkin_LIBRARIES})


  ## Mark executables for installation
  install(TARGETS bsm_helper
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


  ## Mark cpp header files for installation
  install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  ## Testing
  catkin_add_gmock(${PROJECT_NAME}-test 
    test/main.cpp 
    test/test_bsm_helper.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
  )
  target_link_libraries(${PROJECT_NAME}-test bsm_helper ${catkin_LIBRARIES})

else() # ROS 2

  find_package(ament_cmake_auto REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  ament_auto_find_build_dependencies()


  # Includes
  include_directories(
    include
  )

  # Build
  ament_auto_add_library(bsm_helper
    src/bsm_helper.cpp
  )

  if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(${PROJECT_NAME}-test 
      test/main.cpp 
      test/test_bsm_helper.cpp
    )

    ament_target_dependencies(${PROJECT_NAME}-test ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

    target_link_libraries(${PROJECT_NAME}-test bsm_helper)

  endif()

  ament_auto_package()

endif()

