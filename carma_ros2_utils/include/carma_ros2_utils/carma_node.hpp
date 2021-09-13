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

#ifndef CARMA_ROS2_UTILS__CARMA_NODE_HPP_
#define CARMA_ROS2_UTILS__CARMA_NODE_HPP_

#include <memory>
#include <string>

#include "carma_ros2_utils/visibility_control.hpp"
#include "cav_msgs/msg/system_alert.hpp"
#include "rclcpp/rclcpp.hpp"

namespace carma_ros2_utils
{

class CarmaNode : public rclcpp::Node
{
public:
  CARMA_ROS2_UTILS_PUBLIC
  explicit CarmaNode(const rclcpp::NodeOptions & options);

  std::shared_ptr<carma_ros2_utils::CarmaNode> shared_from_this();

  void publish_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);
  virtual void on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);

protected:
  const std::string system_alert_topic_{"/system_alert"};
  rclcpp::Subscription<cav_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
  std::shared_ptr<rclcpp::Publisher<cav_msgs::msg::SystemAlert>> system_alert_pub_;
};

}  // namespace carma_ros2_utils

#endif  // CARMA_ROS2_UTILS__CARMA_NODE_HPP_
