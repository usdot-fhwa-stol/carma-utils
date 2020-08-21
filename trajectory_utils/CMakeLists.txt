cmake_minimum_required(VERSION 2.8.3)
project(trajectory_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
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
  CATKIN_DEPENDS roscpp
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

# ## Add gtest based cpp test target and link libraries
# catkin_add_gmock(${PROJECT_NAME}-test 
#   test/${PROJECT_NAME}/main.cpp 
#   test/${PROJECT_NAME}/quintic_coefficient_calculator_test.cpp
#   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
# )
# target_link_libraries(${PROJECT_NAME}-test trajectory_utils ${catkin_LIBRARIES})



# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gmock(${PROJECT_NAME}-test test/${PROJECT_NAME}/main.cpp test/${PROJECT_NAME}/quintic_coefficient_calculator_test.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
