# carma_ros2_utils

This package contains utility libraries for using ROS2 with carma.

## CarmaLifecycleNode

The CarmaLifecycleNode provides default exception handling and SystemAlert handling for components in carma-platform. All new nodes in carma-platform are expected to be built off CarmaLifecycleNode and implemented as components.

### Example

```c++
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

// Node extends the CarmaLifecycleNode
class CarmaLifecycleNodeTest : public carma_ros2_utils::CarmaLifecycleNode
{
public:
  CarmaLifecycleNodeTest(const rclcpp::NodeOptions &options)
      : CarmaLifecycleNode(options) {}

  ~CarmaLifecycleNodeTest() {};

  // Override the required transition handler to load configurations
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Configured!");

    // Create subscriptions
    system_alert_sub_ = create_subscription<carma_msgs::msg::SystemAlert>(
          "/some_topic", 100,
          [](auto) {}, this, std::placeholders::_1));

    // You would also create parameters/services/timers etc. here

    return CallbackReturn::SUCCESS;
  }

  private:
    rclcpp::Subscription<carma_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
  // For this example there is no reason to override the other transition handlers
};
```

## Lifecycle Component Wrapper

The lifecycle component wrapper provides a mechanism for launch time wrapping of component rclcpp::Node to make them adhere to the ROS2 Lifecycle node state machine in a minimal way. A ROS2 component node can be thought of as having 2 possible overarching states: Loaded and Unloaded This means that they can be conceptually mapped onto the Lifecycle node state machine. Specifically, nodes can be loaded on activation and unloaded on deactivation/error/shutdown. While loading/unloading a node may incur a fair bit of overhead, it has the benefit of ensuring to the user that the non-lifecycle node will not actively interfere with other nodes in the system unless in the ACTIVE state. Under such a system the only new requirement would be a node container capable of enforcing this architecture. This would effectively be an alternative to the component_managers currently available in rclcpp. Then the user could wrap their external nodes with the existing launch commands used for component loading. The LifecycleComponentWrapper class in this package is this alternative component manager. Examples of how to use it are shown below.

### Example

```python
def generate_launch_description():
  
    lifecycle_container = ComposableNodeContainer( # Standard setup for using ROS2 components
        package='rclcpp_components',
        name='lifecycle_component_wrapper', 
        executable='lifecycle_component_wrapper_mt', # Select a multi-threaded (_mt) or single threaded (_st) wrapper lifecycle executable
        namespace="/",
        composable_node_descriptions=[
            ComposableNode( # This non-lifecycle node is now a exposed as a lifecycle node thanks to the wrapper
                package='cool_pkg',
                name='regular_node',
                plugin='cool_pkg_namespace::MyRegularNode',
            ),
        ]
    )

    return LaunchDescription([
        lifecycle_container,
    ])
```
