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

/**
 * Convienance file for including carma_utils headers. Does not include testing headers.
 */

#include <vector>
namespace carma_utils
{
namespace containers
{
template <class T>
std::vector<T> downsample_vector(const std::vector<T>& input, unsigned int n)
{
  std::vector<T> output;

  output.reserve((input.size() / n) + 1);

  for (int i = 0; i < input.size(); i += n)
  {
    output.push_back(input[i]);
  }

  return output;
}
};  // namespace containers
};  // namespace carma_utils
