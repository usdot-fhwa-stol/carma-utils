cmake_minimum_required(VERSION 3.5)
project(carma_ros2_utils)

find_package(ament_cmake REQUIRED)
find_package(carma_common REQUIRED)
find_package(ros2_lifecycle_manager_msgs REQUIRED)
find_package(ros2_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(action_msgs REQUIRED)
find_package(cav_msgs REQUIRED)

set(dependencies
    action_msgs
    cav_msgs
    lifecycle_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    rcutils
    rclcpp_components
    ros2_lifecycle_manager_msgs
    ros2_utils
)

carma_package()

include_directories(include)

set(library_name ${PROJECT_NAME}_core)
add_subdirectory(src)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
