// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "carma_ros2_utils/carma_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace carma_ros2_utils
{

CarmaNode::CarmaNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("carma_node", "", options)
{
  system_alert_pub_ = create_publisher<cav_msgs::msg::SystemAlert>(
    system_alert_topic_, 10);

  system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(
    system_alert_topic_, 1,
    std::bind(&CarmaNode::on_system_alert, this, std::placeholders::_1));
}

std::shared_ptr<carma_ros2_utils::CarmaNode>
CarmaNode::shared_from_this()
{
  return std::static_pointer_cast<carma_ros2_utils::CarmaNode>(
    rclcpp::Node::shared_from_this());
}

void
CarmaNode::publish_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  system_alert_pub_->publish(*msg);
}

void
CarmaNode::on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received SystemAlert message of type: %u", msg->type);
}

}  // namespace carma_ros2_utils

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(carma_ros2_utils::CarmaNode)
