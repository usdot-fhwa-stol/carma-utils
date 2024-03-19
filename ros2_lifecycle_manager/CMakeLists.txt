cmake_minimum_required(VERSION 3.16)
project(ros2_lifecycle_manager)

include(dependencies.cmake)

ament_auto_add_library(ros2_lifecycle_manager SHARED
  src/ros2_lifecycle_manager.cpp
)

if(BUILD_TESTING)
  # These CMake commands were added to ament_cmake_auto in ROS 2 Humble. Until
  # CARMA supports ROS 2 Humble, we will use package-local copies.
  include(cmake/ament_auto_find_test_dependencies.cmake)
  include(cmake/ament_auto_add_gtest.cmake)

  ament_auto_add_executable(test_lifecycle_node
    test/test_node.cpp
  )

  ament_add_gtest_executable(test_lifecycle_gtest
    test/test_lifecycle_manager.cpp
  )

  # ament_add_gtest_executable() does not automaticallly link libraries
  target_link_libraries(test_lifecycle_gtest
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ros2_lifecycle_manager
  )

  # TODO(CAR-6015): Replace this with launch_testing package
  ament_add_test(test_lifecycle
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${PROJECT_SOURCE_DIR}/test/launch_lifecycle_test.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    TIMEOUT 20
    ENV
      TEST_EXECUTABLE=$<TARGET_FILE:test_lifecycle_gtest>
  )
endif()

ament_auto_package()
