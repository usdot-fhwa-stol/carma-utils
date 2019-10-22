cmake_minimum_required(VERSION 2.8.3)
project(sigpack)

# CMake File Structure for Non-Catkin package follows REP-136
# https://www.ros.org/reps/rep-0136.html

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

# Find required packages
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/findFFTW)
message(STATUS ${PROJECT_SOURCE_DIR})
find_package(FFTW REQUIRED)
find_package(Armadillo REQUIRED)

###########
## Build ##
###########

##
## Sigpack is a header only library so there is no compilation step
##
include_directories(
 sigpack
)

#############
## Install ##
#############

## Install header files
install(DIRECTORY ${INCLUDE_DIRECTORIES}
  DESTINATION include/${PROJECT_NAME}
)

## Install package.xml file in accordance with REP-136
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############

# There is currently no plan to integrate this third-party libraries unit tests
