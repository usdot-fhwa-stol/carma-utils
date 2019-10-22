cmake_minimum_required(VERSION 2.8.3)
project(sigpack)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

# Find required packages
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/findFFTW)

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

# Mark as catkin package so other packages can easily find this
catkin_package(
  INCLUDE_DIRS ${INCLUDE_DIRECTORIES}
  DEPENDS FFTW Armadillo
)

#############
## Install ##
#############

## Install header files
install(DIRECTORY ${INCLUDE_DIRECTORIES}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

# There is currently no plan to integrate this third-party libraries unit tests
