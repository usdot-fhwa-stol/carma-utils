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
# The Armadillo name is case sensative and should be camel case here and all caps everywhere else


###########
## Build ##
###########

##
## Sigpack is a header only library so there is no compilation step
##
include_directories(
 sigpack
 ${catkin_INCLUDE_DIRS}
 ${FFTW_INCLUDE_DIRS}
 ${ARMADILLO_INCLUDE_DIRS}
)

message(STATUS "INCLUDE DIRS" ${INCLUDE_DIRECTORIES})
message(STATUS "ARMADILLO" ${ARMADILLO_INCLUDE_DIRS})
message(STATUS "FFTW" ${FFTW_INCLUDE_DIRS})

message(STATUS "LIBS" ${INCLUDE_DIRECTORIES})
message(STATUS "ARMADILLO" ${ARMADILLO_LIBRARIES})
message(STATUS "FFTW" ${FFTW_LIBRARIES})

# Mark as catkin package so other packages can easily find this
catkin_package(
  INCLUDE_DIRS sigpack ${ARMADILLO_INCLUDE_DIRS} ${FFTW_INCLUDE_DIRS}
  LIBRARIES ${ARMADILLO_LIBRARIES} ${FFTW_LIBRARIES}
  DEPENDS FFTW ARMADILLO
)

#############
## Install ##
#############

## Install header files
install(DIRECTORY sigpack
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

# There is currently no plan to integrate this third-party libraries unit tests
