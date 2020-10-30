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
#include <exception>
#include <stdexcept>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <cav_msgs/TrajectoryPlanPoint.h>
#include <trajectory_utils/conversions/conversions.h>

namespace trajectory_utils
{
namespace conversions
{
namespace
{
double compute_2d_distance(double x1, double y1, double x2, double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}
}  // namespace
void trajectory_to_downtrack_time(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points, double start_time,
                                  std::vector<double>* downtrack, std::vector<double>* times)
{
  times->reserve(traj_points.size());

  std::vector<double> downtracks;
  downtracks.reserve(traj_points.size());

  times->push_back(start_time);
  downtracks.push_back(0);

  for (int i = 1; i < traj_points.size(); i++)
  {
    double delta_d = compute_2d_distance(traj_points[i - 1].x, traj_points[i - 1].y,
                                         traj_points[i].x, traj_points[i].y);
    downtracks.push_back(downtracks.back() + delta_d);
  }
}

void speed_to_time(const std::vector<double>& downtrack, const std::vector<double>& speeds, std::vector<double>* times)
{
  if (downtrack.size() != speeds.size())
  {
    throw std::invalid_argument("Input vector sizes do not match");
  }

  if (downtrack.size() == 0)
  {
    throw std::invalid_argument("Input vectors are empty");
  }

  times->reserve(downtrack.size());

  // Uses equation
  // d_t = 2*d_x / (v_0 + v_f)
  double prev_position = downtrack[0];
  double prev_speed = speeds[0];
  double prev_time = 0.0;
  times->push_back(prev_time);
  for (int i = 1; i < downtrack.size(); i++)
  {
    double cur_pos = downtrack[i];
    double cur_speed = speeds[0];
    double delta_d = cur_pos - prev_position;
    double dt = (2 * delta_d) / (cur_speed + prev_speed);
    double cur_time = dt + prev_time;
    times->push_back(cur_time);

    prev_position = cur_pos;
    prev_speed = cur_speed;
    prev_time = cur_time;
  }
}

void time_to_speed(const std::vector<double>& downtrack, const std::vector<double>& times, double initial_speed,
                   std::vector<double>* speeds)
{
  if (downtrack.size() != speeds->size())
  {
    throw std::invalid_argument("Input vector sizes do not match");
  }

  if (downtrack.size() == 0)
  {
    throw std::invalid_argument("Input vectors are empty");
  }

  speeds->reserve(downtrack.size());

  double prev_position = downtrack[0];
  double prev_speed = initial_speed;
  double prev_time = times[0];
  speeds->push_back(prev_speed);
  for (int i = 1; i < downtrack.size(); i++)
  {
    double cur_pos = downtrack[i];
    double cur_time = times[i];
    double dt = cur_time - prev_time;
    double delta_d = cur_pos - prev_position;

    double cur_speed = prev_speed + 2 * (delta_d - prev_speed * dt) / dt;
    speeds->push_back(cur_speed);

    prev_position = cur_pos;
    prev_time = cur_time;
    prev_speed = cur_speed;
  }
}
};  // namespace conversions
};  // namespace trajectory_utils