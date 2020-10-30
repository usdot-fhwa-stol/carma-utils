/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include <vector>
#include <trajectory_utils.h>
#include <exception>
#include <stdexcept>
#include <math.h>
#include <algorithm>

namespace trajectory_utils
{
std::vector<double> apply_accel_limits_by_distance(std::vector<double> downtracks, std::vector<double> speeds,
                                                   double accel_limit, double decel_limit)
{
  if (downtracks.size() != speeds.size())
  {
    throw std::invalid_argument("Downtracks and speeds do not have the same size");
  }

  if (accel_limit <= 0 || decel_limit <= 0)
  {
    throw std::invalid_argument("Accel and Decel limits should be positive");
  }

  std::vector<double> output;
  output.reserve(downtracks.size());

  if (downtracks.size() == 0)
  {
    return output;
  }

  output.push_back(speeds[0]);  // First point will be unchanged

  for (int i = 1; i < downtracks.size(); i++)
  {
    double delta_d = downtracks[i] - downtracks[i - 1];
    double prev_speed = speeds[i - 1];
    double cur_speed = speeds[i];
    double new_speed = cur_speed;
    if (cur_speed > prev_speed)
    {  // Acceleration case
      new_speed = std::min(cur_speed, sqrt(prev_speed * prev_speed + 2 * accel_limit * delta_d));
    }
    else if (cur_speed < prev_speed)
    {  // Deceleration case
      new_speed = std::max(cur_speed, sqrt(prev_speed * prev_speed - 2 * decel_limit * delta_d));
    }
    else
    {  // No change case
      new_speed = std::max(0.0, new_speed);
    }

    output.push_back(new_speed);
  }

  return output;
}

std::vector<double> apply_accel_limits_by_time(std::vector<double> times, std::vector<double> speeds,
                                               double accel_limit, double decel_limit)
{
  if (times.size() != speeds.size())
  {
    throw std::invalid_argument("Times and speeds do not have the same size");
  }

  if (accel_limit <= 0 || decel_limit <= 0)
  {
    throw std::invalid_argument("Accel and Decel limits should be positive");
  }

  std::vector<double> output;
  output.reserve(times.size());

  if (times.size() == 0)
  {
    return output;
  }

  output.push_back(speeds[0]);  // First point will be unchanged

  for (int i = 1; i < times.size(); i++)
  {
    double delta_t = times[i] - times[i - 1];
    double prev_speed = speeds[i - 1];
    double cur_speed = speeds[i];
    double new_speed = cur_speed;
    if (cur_speed > prev_speed)
    {  // Acceleration case
      new_speed = std::min(cur_speed, prev_speed + accel_limit * delta_t);
    }
    else if (cur_speed < prev_speed)
    {  // Deceleration case
      new_speed = std::max(cur_speed, prev_speed - decel_limit * delta_t);
    }
    else
    {  // No change case
      new_speed = std::max(0.0, new_speed);
    }

    output.push_back(new_speed);
  }

  return output;
}
};  // namespace trajectory_utils