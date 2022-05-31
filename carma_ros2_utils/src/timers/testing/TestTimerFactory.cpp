/*
 * Copyright (C) 2020-2022 LEIDOS.
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
#include <carma_ros2_utils/timers/testing/TestTimerFactory.hpp>
namespace carma_ros2_utils
{
namespace timers
{
namespace testing
{
TestTimerFactory::~TestTimerFactory(){}

void TestTimerFactory::setNow(const rclcpp::Time& current_time)
{
  for (std::shared_ptr<TestClock> time: clock_log_)
  {
    time->setNow(current_time);
  }
}

rclcpp::Time TestTimerFactory::now()
{
  if (!clock_log_.empty())
    return clock_log_.back()->now();
  else
    return rclcpp::Time(0,0);
}

std::unique_ptr<Timer> TestTimerFactory::buildTimer(uint32_t id, rclcpp::Duration duration,
                                                    std::function<void()> callback, bool oneshot,
                                                    bool autostart)
{
  std::shared_ptr<TestClock> clock = std::make_shared<TestClock>();
  if (!clock_log_.empty())
    clock->setNow(clock_log_.back()->now());
  
  clock_log_.push_back(clock);
  std::unique_ptr<Timer> timer(new TestTimer(clock));
  timer->setId(id);
  timer->initializeTimer(duration, callback, oneshot, autostart);
  return timer;
}
}  // namespace testing
}  // namespace timers
}  // namespace carma_ros2_utils