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

#include <string>
#include <vector>
#include <tuple>
#include <math.h>
#include <exception>
#include <uncertainty_tools/uncertainty_tools.h>


namespace uncertainty_tools {

  std::tuple<double,double> computeVectorMagnitudeAndUncertainty(
    const std::vector<double>& values, const std::vector<double>& uncertainties
  ) {
    // An example of the math used to compute uncertainties of resultant vectors can be found here
    // http://spiff.rit.edu/classes/phys216/workshops/w2x/hypotenuse.html

    if (values.size() == 0 || values.size() != uncertainties.size()) {
      throw std::invalid_argument("Invalid: Input values list and uncertainties list have unequal sizes or a size of 0");
    }

    double sum_of_unc_sqr = 0;
    double sum_of_sqr_val = 0;
    for (int i=0; i<values.size(); i++) {
      const double val = values[i];
      const double unc = uncertainties[i];
      const double val_sqr = val * val;

      const double frac_unc_of_sqr = 2*(unc/val);
      const double unc_of_sqr = frac_unc_of_sqr * val_sqr;
      sum_of_unc_sqr += unc_of_sqr;
      sum_of_sqr_val += val_sqr;
    }

    const double resultant_mag = sqrt(sum_of_sqr_val);
    const double frac_var_of_resultant_sqr = sum_of_unc_sqr / sum_of_sqr_val;
    const double frac_var_of_resultant = 0.5 * frac_var_of_resultant_sqr;
    const double var_of_resultant = resultant_mag * frac_var_of_resultant;

    return std::make_tuple(resultant_mag, var_of_resultant);
  }
};