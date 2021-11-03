cmake_minimum_required(VERSION 3.5)
project(carma_ros2_utils)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(carma_msgs REQUIRED)

set(dependencies
    carma_msgs
    lifecycle_msgs
    rclcpp
    rclcpp_lifecycle
    rcutils
    rclcpp_components
)

carma_define_ament_library(${PROJECT_NAME} "${dependencies}" src/carma_lifecycle_node.cpp)

if(BUILD_TESTING)

  add_subdirectory(test)

endif()

ament_package()
