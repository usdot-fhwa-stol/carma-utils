#pragma once
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

namespace trajectory_utils
{
namespace conversions
{
void speed_to_time(const std::vector<double>& downtrack, const std::vector<double>& speeds, std::vector<double>* times);

void time_to_speed(const std::vector<double>& downtrack, const std::vector<double>& times, double initial_speed,
                   std::vector<double>* speeds);

void trajectory_to_downtrack_time(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points, double start_time,
                                  std::vector<double>* downtrack, std::vector<double>* times);
};  // namespace conversions
};  // namespace trajectory_utils