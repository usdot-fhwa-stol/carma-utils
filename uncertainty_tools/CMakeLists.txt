cmake_minimum_required(VERSION 2.8.3)
project(uncertainty_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS roscpp
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers */*.hpp */*.h)

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
