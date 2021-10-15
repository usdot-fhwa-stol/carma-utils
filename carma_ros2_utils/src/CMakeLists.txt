add_library(${library_name} SHARED
  carma_lifecycle_node.cpp
)

set(node_plugins "")
target_compile_definitions(${library_name}
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(${library_name} "carma_ros2_utils::CarmaNode")
set(node_plugins "${node_plugins}carma_ros2_utils::CarmaNode;$<TARGET_FILE:carma_ros2_utils_core>\n")

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
DESTINATION lib/${PROJECT_NAME}
)
