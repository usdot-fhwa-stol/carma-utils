#pragma once
/*
 * Copyright (C) 2018-2019 LEIDOS.
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
#include <tuple>

namespace uncertainty_tools {
  /**
   * \brief Function to compute magnitude with an uncertainty value from component vectors containing their own uncertainties
   * 
   * \param values List of vector component values for a 3 dimensional space with should be a 3 element list. 1 value for each component
   * \param uncertainties List of vector uncertainties (95% confidence interval). Each value should match an element in the values list
   * \throws std::invalid_argument if the input values list and uncertainties list have unequal sizes or a size of 0
   * 
   * \return An std::tuple where the first value is the resultant magnitude and the second value is that magnitudes uncertainty.
   */ 
  std::tuple<double,double> computeVectorMagnitudeAndUncertainty(const std::vector<double>& values, const std::vector<double>& uncertainties);
};