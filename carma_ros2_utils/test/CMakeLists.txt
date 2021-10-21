find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ros2_lifecycle_manager REQUIRED)
find_package(std_srvs REQUIRED)

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

install(
  TARGETS test_lifecycle_gtest test_carma_lifecycle_node test_carma_lifecycle_gtest
  # EXPORT should not be needed for unit tests
  LIBRARY DESTINATION lib/${PROJECT_NAME} # In Ament, Executables are installed to lib/${PROJECT_NAME} not bin 
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)