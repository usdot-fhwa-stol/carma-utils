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
