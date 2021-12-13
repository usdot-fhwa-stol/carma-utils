cmake_minimum_required(VERSION 3.5)
project(carma_ros2_utils)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Includes
include_directories(
  include
)

# Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/carma_lifecycle_node.cpp
)

ament_auto_add_library(lifecycle_component_wrapper SHARED
  src/lifecycle_component_wrapper.cpp
)

ament_auto_add_library(carma_component_manager SHARED
  src/component_manager.cpp
)

ament_auto_add_executable(lifecycle_component_wrapper_st
  src/lifecycle_component_wrapper_st.cpp
)

ament_auto_add_executable(lifecycle_component_wrapper_mt
  src/lifecycle_component_wrapper_mt.cpp
)

ament_auto_add_executable(carma_component_container
  src/component_container.cpp
)

ament_auto_add_executable(carma_component_container_mt
  src/component_container_mt.cpp
)

# Link local targets
target_link_libraries(lifecycle_component_wrapper
  ${PROJECT_NAME}
)

target_link_libraries(lifecycle_component_wrapper_st
  lifecycle_component_wrapper
  ${PROJECT_NAME}
)

target_link_libraries(lifecycle_component_wrapper_mt
  lifecycle_component_wrapper
  ${PROJECT_NAME}
)

target_link_libraries(carma_component_container
  carma_component_manager
)

target_link_libraries(carma_component_container_mt
  carma_component_manager
)

if(BUILD_TESTING)

  add_subdirectory(test)

  rclcpp_components_register_nodes(test_minimal_node "carma_ros2_utils_testing::MinimalNode")

endif()

ament_auto_package()
