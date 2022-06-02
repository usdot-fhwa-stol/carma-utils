find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ros2_lifecycle_manager REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED) 

set(dependencies ${dependencies} ros2_lifecycle_manager std_srvs)


add_executable(test_carma_lifecycle_node
  test_node.cpp
)

target_link_libraries(test_carma_lifecycle_node ${PROJECT_NAME})

ament_target_dependencies(test_carma_lifecycle_node ${dependencies} )

target_include_directories(test_carma_lifecycle_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(test_minimal_node SHARED
  test_minimal_node.cpp
)

ament_target_dependencies(test_minimal_node
  rclcpp
  rclcpp_components
)

add_launch_test(
  launch_test_lifecycle_wrapper.py
)


ament_add_gtest_executable(test_lifecycle_gtest
  test_lifecycle_manager.cpp
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
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/launch_lifecycle_test.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 30
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:test_lifecycle_gtest>
)

ament_add_gtest(test_carma_lifecycle_gtest
  test_carma_lifecycle_exceptions.cpp
  test_carma_lifecycle_helpers.cpp
)

target_include_directories(test_carma_lifecycle_gtest
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(test_carma_lifecycle_gtest ${PROJECT_NAME})

ament_target_dependencies(test_carma_lifecycle_gtest
  ${dependencies}
)

ament_add_gtest(test_timers
  timers/TestingTimers.cpp
)

target_include_directories(test_timers
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(test_timers carma_ros2_utils_timers ${PROJECT_NAME})

ament_target_dependencies(test_timers
  ${dependencies}
)

ament_add_gtest(test_containers
  containers/containers_test.cpp
)

target_link_libraries(test_containers ${PROJECT_NAME})

install(
  TARGETS test_lifecycle_gtest test_carma_lifecycle_node test_carma_lifecycle_gtest test_minimal_node test_timers
  # EXPORT should not be needed for unit tests
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME} # In Ament, Executables are installed to lib/${PROJECT_NAME} not bin 
  INCLUDES DESTINATION include
)

# This is a bit of a hack to allow for rclcpp_component registration in this subdirectory 
# Based off sloretz's answer from ROS Answers here https://answers.ros.org/question/361289/ros2-components-registration-from-subdirectory-cmakelists-file/
#set(AMENT_EXTENSIONS_ament_package ${AMENT_EXTENSIONS_ament_package} PARENT_SCOPE)