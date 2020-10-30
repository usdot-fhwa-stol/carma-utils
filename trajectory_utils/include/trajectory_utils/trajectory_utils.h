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
std::vector<double> apply_accel_limits_by_distance(std::vector<double> downtracks, std::vector<double> speeds,
                                                   double accel_limit, double decel_limit);

std::vector<double> apply_accel_limits_by_time(std::vector<double> times, std::vector<double> speeds,
                                               double accel_limit, double decel_limit);

template<class T>
std::vector<T> shift_by_lookahead(std::vector<T> values, unsigned int lookahead_count)
{
  std::vector<T> output;
  output.reserve(values.size());

  for (int i = 0; i < values.size(); i++)
  {
    T lookahead_value = 0;
    if (i + lookahead_count < values.size() - 1)
    {
      lookahead_value = values[i + lookahead_count];
    }
    else
    {
      lookahead_value = values[values.size() - 1];
    }

    output.push_back(lookahead_value);
  }

  return output;
}

};  // namespace trajectory_utils