/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include <uncertainty_tools/uncertainty_tools.h>


TEST(uncertainty_tools_test, TestComputeVectorMagnitudeAndUncertainty)
{   
  // Test calculation
  std::vector<double> components, uncertainties;
  components.push_back(5.3);
  uncertainties.push_back(0.2);

  components.push_back(15.4);
  uncertainties.push_back(0.3);

  std::tuple<double,double> result = uncertainty_tools::computeVectorMagnitudeAndUncertainty(components, uncertainties);
  ASSERT_NEAR(16.3, std::get<0>(result), 0.1);
  ASSERT_NEAR(0.3, std::get<1>(result), 0.1);

  // Test throw behavior
  // First list empty
  try {
    std::vector<double> componentsE;
    std::tuple<double,double> result = uncertainty_tools::computeVectorMagnitudeAndUncertainty(componentsE, uncertainties);
    FAIL() << "Exception not thrown when provided with empty lists";
  } catch (const std::invalid_argument& e) {
    // PASS
  } catch (const std::exception& e) {
    FAIL() << "Exception thrown not of expected type std::invalid_argument. Instead: " << e.what();
  }

  // Second list empty
  try {
    std::vector<double> uncertaintiesE;
    std::tuple<double,double> result = uncertainty_tools::computeVectorMagnitudeAndUncertainty(components, uncertaintiesE);
    FAIL() << "Exception not thrown when provided with empty lists";
  } catch (const std::invalid_argument& e) {
    // PASS
  } catch (const std::exception& e) {
    FAIL() << "Exception thrown not of expected type std::invalid_argument. Instead: " << e.what();
  }

  // Both list empty
  try {
    std::vector<double> componentsE, uncertaintiesE;
    std::tuple<double,double> result = uncertainty_tools::computeVectorMagnitudeAndUncertainty(componentsE, uncertaintiesE);
    FAIL() << "Exception not thrown when provided with empty lists";
  } catch (const std::invalid_argument& e) {
    // PASS
  } catch (const std::exception& e) {
    FAIL() << "Exception thrown not of expected type std::invalid_argument. Instead: " << e.what();
  }

  // Unequal Sizes
  try {
    components.push_back(0.1);
    std::tuple<double,double> result = uncertainty_tools::computeVectorMagnitudeAndUncertainty(components, uncertainties);
    FAIL() << "Exception not thrown when provided with empty lists";
  } catch (const std::invalid_argument& e) {
    // PASS
  } catch (const std::exception& e) {
    FAIL() << "Exception thrown not of expected type std::invalid_argument. Instead: " << e.what();
  }
}

// Run all the tests
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

