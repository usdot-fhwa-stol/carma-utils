#pragma once
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
#include <rclcpp/time.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include "../TimerFactory.h"
#include "../Timer.h"
#include "TestTimer.h"

namespace carma_ros2_utils
{
namespace timers
{
namespace testing
{
/**
 * @brief Implementation of the TimerFactory interface that is targeted for use in Unit Testing.
 *        Returns instances of TestTimer.
 *        This class should NOT be used in production code.
 */
class TestTimerFactory : public TimerFactory
{
public:
  ~TestTimerFactory();

  /**
   * @brief Set the rclcpp clock interface which will be used by this test timer
   * 
   * @param clock_interface The interface to set
   */ 
  void setClockInterface(rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface);

  //// Overrides
  std::unique_ptr<Timer> buildTimer(uint32_t id, rclcpp::Duration duration,
                                    std::function<void()> callback, bool oneshot = false,
                                    bool autostart = true) override;

private:
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface_; //! Interface used for accessing current time from rclcpp
};
}  // namespace testing
}  // namespace timers
}  // namespace carma_ros2_utils