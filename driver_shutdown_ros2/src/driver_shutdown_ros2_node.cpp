/*
 * Copyright (C) 2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
#include "driver_shutdown_ros2/driver_shutdown_ros2_node.hpp"

namespace driver_shutdown_ros2
{
  namespace std_ph = std::placeholders;

  Node::Node(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {

  }

  carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
  {

    // Setup subscribers
    alert_sub_ = create_subscription<carma_msgs::msg::SystemAlert>("/system_alert", 100,
                                                              std::bind(&Node::alert_callback, this, std_ph::_1));

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  void Node::alert_callback(carma_msgs::msg::SystemAlert::UniquePtr msg)
  {
    RCLCPP_INFO_STREAM(  get_logger(), "system alert callback called with value: " << (int) msg->type);

    if (msg->type == carma_msgs::msg::SystemAlert::SHUTDOWN) {
      // If we received a shutdown then we will shutdown
      shutdown();
    }
  }

} // driver_shutdown_ros2

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(driver_shutdown_ros2::Node)
