cmake_minimum_required(VERSION 2.8.3)
project(sigpack)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

# Find required packages
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/findFFTW)
find_package(catkin REQUIRED)
find_package(FFTW REQUIRED)
find_package(Armadillo REQUIRED) 
# The Armadillo name is case sensative and should be camel case here and all caps everywhere else


###########
## Build ##
###########

##
## Sigpack is a header only library so there is no compilation step
##
catkin_package(
  INCLUDE_DIRS include ${ARMADILLO_INCLUDE_DIRS} ${FFTW_INCLUDE_DIRS}
  LIBRARIES ${ARMADILLO_LIBRARIES} ${FFTW_LIBRARIES}
  DEPENDS FFTW ARMADILLO
)

include_directories(
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


#############
## Install ##
#############

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

# There is currently no plan to integrate this third-party libraries unit tests
