#pragma once
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

#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include "../Timer.hpp"

namespace carma_ros2_utils
{
namespace timers
{
namespace testing
{
/**
 * @brief Implementation of the Clock interface that is targeted for use in Unit Testing.
 *        Internally rclcpp::Time objects are used for getting the clock time meaning this class does support simulated
 * time and rclcpp::Time::setNow() semantics. This class should NOT be used in production code as it does not provide the
 * same threading behavior as rclcpp::Timer.
 */
class TestClock : public rclcpp::Clock
{

public:
  //// Overrides
  explicit TestClock(rcl_clock_type_t clock_type = RCL_ROS_TIME);

  rclcpp::Time now();

  void setNow(const rclcpp::Time& time);

private:
  rclcpp::Time current_time_{0, 0};

};
}  // namespace testing
}  // namespace timers
}  // namespace carma_ros2_utils