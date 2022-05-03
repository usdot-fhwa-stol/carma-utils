/*
 * Copyright (C) 2020-2021 LEIDOS.
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
#include <carma_ros2_utils/timers/ROSTimerFactory.h>

// <SONAR_IGNORE_START> // Disable sonar cloud analysis for ROS dependant logic
namespace carma_ros2_utils
{
namespace timers
{
ROSTimerFactory::~ROSTimerFactory(){};
std::unique_ptr<Timer> ROSTimerFactory::buildTimer(uint32_t id, rclcpp::Duration duration,
                                                   std::function<void()> callback, bool oneshot,
                                                   bool autostart)
{
  std::unique_ptr<Timer> timer_ptr(new ROSTimer(weak_node_pointer_));
  timer_ptr->initializeTimer(duration, callback, oneshot, autostart);
  timer_ptr->setId(id);
  return timer_ptr;
}

void setCarmaLifecycleNode(std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer)
{
  ROSTimerFactory::weak_node_pointer_ = weak_node_pointer;
}

}  // namespace timers
}  // namespace carma_ros2_utils