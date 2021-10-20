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

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ros2_lifecycle_manager REQUIRED)

  set(dependencies ${dependencies} ros2_lifecycle_manager)


  add_executable(test_carma_lifecycle_node
    test/test_node.cpp
  )

  target_link_libraries(test_carma_lifecycle_node ${PROJECT_NAME})

  ament_target_dependencies(test_carma_lifecycle_node ${dependencies} )

  target_include_directories(test_carma_lifecycle_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )


  ament_add_gtest_executable(test_lifecycle_gtest
    test/test_lifecycle_manager.cpp
  )

  target_include_directories(test_lifecycle_gtest
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_lifecycle_gtest
    ${dependencies}
  )

  ament_add_test(test_lifecycle
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/launch_lifecycle_test.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    TIMEOUT 30
    ENV
      TEST_EXECUTABLE=$<TARGET_FILE:test_lifecycle_gtest>
  )

endif()

ament_package()
