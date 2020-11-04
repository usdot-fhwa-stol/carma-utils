/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <trajectory_utils/conversions/conversions.h>

namespace trajectory_utils
{
namespace conversions
{
TEST(trajectory_utils_conversions_test, trajectory_to_downtrack_time)
{
  // Nominal case
  std::vector<cav_msgs::TrajectoryPlanPoint> traj_points;
  double start_time = 0;

  std::vector<double> downtracks, times;

  cav_msgs::TrajectoryPlanPoint p1, p2, p3;

  double step_size = 1;
  p1.x = 0;
  p1.y = 0;
  p1.target_time = ros::Time(start_time);

  p2.x = step_size;
  p2.y = 0;
  p2.target_time = ros::Time(1.0);

  p3.x = step_size;
  p3.y = step_size;
  p3.target_time = ros::Time(1.5);

  traj_points = { p1, p2, p3 };

  trajectory_to_downtrack_time(traj_points, start_time, &downtracks, &times);

  ASSERT_EQ(downtracks.size(), times.size());
  ASSERT_EQ(downtracks.size(), traj_points.size());

  for (size_t i = 0; i < downtracks.size(); i++)
  {
    ASSERT_EQ(downtracks[i], i * step_size);
    ASSERT_NEAR(times[i], traj_points[i].target_time.toSec(), 0.0000000001);
  }

  // Non-Zero start time
  start_time = 0.5;

  downtracks = {};
  times = {};

  traj_points[0].target_time = ros::Time(start_time);

  trajectory_to_downtrack_time(traj_points, start_time, &downtracks, &times);

  ASSERT_EQ(downtracks.size(), times.size());
  ASSERT_EQ(downtracks.size(), traj_points.size());

  for (size_t i = 0; i < downtracks.size(); i++)
  {
    ASSERT_EQ(downtracks[i], i * step_size);
    ASSERT_NEAR(times[i], traj_points[i].target_time.toSec(), 0.0000000001);
  }

  // Empty input
  traj_points = {};
  downtracks = {};
  times = {};

  trajectory_to_downtrack_time(traj_points, start_time, &downtracks, &times);

  ASSERT_EQ(downtracks.size(), times.size());
  ASSERT_EQ(downtracks.size(), traj_points.size());
}
}  // namespace conversions
}  // namespace trajectory_utils
