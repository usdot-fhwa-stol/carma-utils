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
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(rcpputils REQUIRED)
find_package(Boost REQUIRED)

set(dependencies
    carma_msgs
    lifecycle_msgs
    rclcpp
    rclcpp_lifecycle
    rcutils
    rclcpp_components
    ament_index_cpp
    class_loader
    composition_interfaces
    rcpputils
    Boost
)

carma_define_ament_library(${PROJECT_NAME} "${dependencies}" src/carma_lifecycle_node.cpp)

set(lifecycle_wrapper_dependencies
  ${dependencies}
  ${PROJECT_NAME}
)

carma_define_ament_library(lifecycle_component_wrapper "${lifecycle_wrapper_dependencies}" src/lifecycle_component_wrapper.cpp)

set(wrapper_node_dependencies
  ${lifecycle_wrapper_dependencies}
  lifecycle_component_wrapper
)

carma_define_ament_executable(lifecycle_component_wrapper_st "${wrapper_node_dependencies}" src/lifecycle_component_wrapper_st.cpp)

carma_define_ament_executable(lifecycle_component_wrapper_mt "${wrapper_node_dependencies}" src/lifecycle_component_wrapper_mt.cpp)

if(BUILD_TESTING)

  add_subdirectory(test)

endif()

ament_package()
