# ros2_lifecycle_manager

This package provides a library for managing multiple ROS2 Lifecycle Nodes.
The user registers the fully qualified graph names of running ROS nodes. Then the C++ API can be used to trigger the transitions for all of those nodes.

## Example

```c++
#include <ros2_lifecycle_manager/ros2_lifecycle_manager.hpp>
// node is some ros node or ros lifecycle node
Ros2LifecycleManager manager(node.get_node_base_interface(), 
                             node.get_node_graph_interface(), 
                             node.get_node_logging_interface(), 
                             node.get_node_services_interface()));

std::vector<std::string> managed_nodes = {"/my_node_1", "/my_node_2"};

manager.set_managed_nodes(managed_nodes);

// Activate node by walking it through lifecycle
manager.configure();
bool success = manager.activate(); // Success status returned by all transition calls
if (success)
    manager.shutdown(); // If one of the nodes failed to activate maybe assume fault

// Shutdown the node cleanly
manager.deactivate();
manager.cleanup();
manager.shutdown();

// NOTE: In order for the responses for these calls to actually get processed your node needs to be spinning

```
