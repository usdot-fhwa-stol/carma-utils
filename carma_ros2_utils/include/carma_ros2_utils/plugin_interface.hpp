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

#ifndef CARMA_ROS2_UTILS__PLUGIN_INTERFACE_HPP_
#define CARMA_ROS2_UTILS__PLUGIN_INTERFACE_HPP_

#include "carma_ros2_utils/carma_lifecycle_node.hpp"
#include "ros2_utils/lifecycle_interface.hpp"

namespace carma_ros2_utils
{

class PluginInterface : public ros2_utils::LifecycleInterface
{
public:
  virtual void initialize(const carma_ros2_utils::CarmaLifecycleNode::SharedPtr node) = 0;
};

}  // namespace carma_ros2_utils

#endif  // CARMA_ROS2_UTILS__PLUGIN_INTERFACE_HPP_
