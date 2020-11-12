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
#include <trajectory_utils/trajectory_utils.h>

namespace trajectory_utils
{
TEST(trajectory_utils_test, shift_by_lookahead)
{
  std::vector<int> values = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  std::vector<int> result = shift_by_lookahead(values, 3);

  ASSERT_EQ(result.size(), values.size());
  ASSERT_EQ(result[0], 3);
  ASSERT_EQ(result[1], 4);
  ASSERT_EQ(result[2], 5);
  ASSERT_EQ(result[3], 6);
  ASSERT_EQ(result[4], 7);
  ASSERT_EQ(result[5], 8);
  ASSERT_EQ(result[6], 9);
  ASSERT_EQ(result[7], 9);
  ASSERT_EQ(result[8], 9);
  ASSERT_EQ(result[9], 9);

}

TEST(trajectory_utils_test, adaptive_lookahead2)
{

  std::vector<double> x = {-32.2275,-32.9451,-33.7889,-34.6362,-37.6845,-38.5269,-39.3471,-40.1668,-40.9865,-41.7977,-42.5936,-43.3935,
    -44.2032,-45.0063,-45.7982,-46.5973,-47.3826,-48.1593,-48.8992,-49.602};
  std::vector<double> y = {43.3619,44.3048,45.4042,46.4835,50.3516,51.3965,52.4607,53.5275,54.5867,55.6295,56.684,57.7326,58.7774,59.8203,
    60.8619,61.8803,62.894,63.8828,64.8561,65.8323};
  std::vector<double> v = {36.2213,36.1335,36.0107,35.8571,35.628,35.2642,35.1134,34.9891,34.8161,34.6563,34.5101,34.3622,34.2016,34.041,
    33.8296,33.6189,33.1606,32.6554,32.0566,31.4245};

  int idx = 1;

  double adaptive_lookahead = get_lookahead(v[idx]);
  double adaptive_speed = 0.0;
  
  for (int i = idx+1; i<v.size(); i++){
    double dist = sqrt(pow(x[i]-x[idx], 2) + pow(y[i]-y[idx], 2));
    if (abs(dist - adaptive_lookahead)<1.0){
      adaptive_speed = v[i];
    }
  }

  EXPECT_NEAR(adaptive_lookahead, 18, 0.5);
  EXPECT_NEAR(adaptive_speed, 34, 0.5);
  

  std::vector<double> shifted_v = shift_by_lookahead(v, 8);
  std::vector<double> shifted_x = shift_by_lookahead(x, 8);
  std::vector<double> shifted_y = shift_by_lookahead(y, 8);
  double shifted_lookahead = sqrt(pow(shifted_x[0]-x[idx],2)+pow(shifted_y[0]-y[idx],2));
  EXPECT_NEAR(shifted_lookahead, 13, 0.1);
  EXPECT_NEAR(shifted_v[0], 34.8, 0.1);


}

TEST(trajectory_utils_test, time_boundary_index)
{
  // Nominal case
  std::vector<double> downtracks = {0, 1, 2, 3, 4, 5, 6, 7 };
  std::vector<double> speeds = { 1, 1, 1, 1, 1, 1, 1, 1 };
  // 0, 1, 2, 3, 4, 5, 6

  ASSERT_EQ(6, time_boundary_index(downtracks, speeds, 5.0));

  // Test edge cases

  speeds = {1,1,1};
  ASSERT_THROW(time_boundary_index(downtracks, speeds, 5.0), std::invalid_argument);

  downtracks = {};
  speeds = {};
  ASSERT_EQ(0, time_boundary_index(downtracks, speeds, 5.0));

  downtracks = {0, 1, 2, 3, 4, 5, 6, 7 };
  speeds = { 1, 1, 1, 1, 1, 1, 1, 1 };

  ASSERT_EQ(0, time_boundary_index(downtracks, speeds, 0.0));
}

TEST(trajectory_utils_test, constrain_speed_for_curvature)
{
    double speed1 = constrain_speed_for_curvature(1.0, 1.0);
    ASSERT_NEAR(1.0, speed1, 0.005);

    double speed2 = constrain_speed_for_curvature(10.0, 3.33);
    ASSERT_NEAR(std::sqrt(3.33 / 10.0), speed2, 0.005);

    double speed3 = constrain_speed_for_curvature(5.0, 10.0);
    ASSERT_NEAR(std::sqrt(10.0 / 5.0), speed3, 0.005);

    double speed4 = constrain_speed_for_curvature(100.0, 1.0);
    ASSERT_NEAR(std::sqrt(1.0 / 100.0), speed4, 0.005);

    double speed5 = constrain_speed_for_curvature(0.0, 0.0);
    ASSERT_TRUE(std::isinf(speed5));
}

TEST(trajectory_utils_test, constrained_speeds_for_curvatures)
{
  std::vector<double> curvatures_1 = {1.0, 10.0, 5.0, 100.0, 0.0};
  std::vector<double> out_1 = constrained_speeds_for_curvatures(curvatures_1, 1.0);

  ASSERT_EQ(curvatures_1.size(), out_1.size());
  ASSERT_NEAR(1.0, out_1[0], 0.005);
  ASSERT_NEAR(std::sqrt(1.0 / 10.0), out_1[1], 0.005);
  ASSERT_NEAR(std::sqrt(1.0 / 5.0), out_1[2], 0.005);
  ASSERT_NEAR(std::sqrt(1.0 / 100.0), out_1[3], 0.005);
  ASSERT_TRUE(std::isinf(out_1[4]));

  std::vector<double> curvatures_2 = {1.0, 2.0, 3.0, 4.0, 5.0,
                                      6.0, 7.0, 8.0, 9.0, 10.0};
  std::vector<double> out_2 = constrained_speeds_for_curvatures(curvatures_2, 2.0);

  ASSERT_EQ(curvatures_2.size(), out_2.size());
  ASSERT_NEAR(std::sqrt(2.0 / 1.0), out_2[0], 0.005);
  ASSERT_NEAR(std::sqrt(2.0 / 2.0), out_2[1], 0.005);
  ASSERT_NEAR(std::sqrt(2.0 / 3.0), out_2[2], 0.005);
  ASSERT_NEAR(std::sqrt(2.0 / 4.0), out_2[3], 0.005);
  ASSERT_NEAR(std::sqrt(2.0 / 5.0), out_2[4], 0.005);
  ASSERT_NEAR(std::sqrt(2.0 / 6.0), out_2[5], 0.005);
  ASSERT_NEAR(std::sqrt(2.0 / 7.0), out_2[6], 0.005);
  ASSERT_NEAR(std::sqrt(2.0 / 8.0), out_2[7], 0.005);
  ASSERT_NEAR(std::sqrt(2.0 / 9.0), out_2[8], 0.005);
  ASSERT_NEAR(std::sqrt(2.0 / 10.0), out_2[9], 0.005);
}

TEST(trajectory_utils_test, apply_accel_limits_by_distance)
{
  // Accel case
  std::vector<double> speeds_a = {0.0, 0.0, 0.0, 0.0, 0.0,
                                  5.0, 5.0, 5.0, 5.0, 5.0};

  std::vector<double> downtracks = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  std::vector<double> limited_a;
  limited_a = apply_accel_limits_by_distance(downtracks, speeds_a, 3.0, 3.0);

  ASSERT_EQ(downtracks.size(), limited_a.size());
  ASSERT_NEAR(0.0, limited_a[0], 0.01);
  ASSERT_NEAR(0.0, limited_a[1], 0.01);
  ASSERT_NEAR(0.0, limited_a[2], 0.01);
  ASSERT_NEAR(0.0, limited_a[3], 0.01);
  ASSERT_NEAR(0.0, limited_a[4], 0.01);
  ASSERT_NEAR(2.449489742783178, limited_a[5], 0.01);
  ASSERT_NEAR(3.4641016151377544, limited_a[6], 0.01);
  ASSERT_NEAR(4.242640687119285, limited_a[7], 0.01);
  ASSERT_NEAR(4.898979485566356, limited_a[8], 0.01);
  ASSERT_NEAR(5.0, limited_a[9], 0.01);

  // Decel case
  speeds_a = {5.0, 5.0, 5.0, 5.0, 5.0,
              0.0, 0.0, 0.0, 0.0, 0.0};

  downtracks = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  limited_a = apply_accel_limits_by_distance(downtracks, speeds_a, 3.0, 3.0);

  ASSERT_EQ(downtracks.size(), limited_a.size());
 
  ASSERT_NEAR(5.0, limited_a[0], 0.01);
  ASSERT_NEAR(5.0, limited_a[1], 0.01);
  ASSERT_NEAR(5.0, limited_a[2], 0.01);
  ASSERT_NEAR(5.0, limited_a[3], 0.01);
  ASSERT_NEAR(5.0, limited_a[4], 0.01);
  ASSERT_NEAR(4.358898943540674, limited_a[5], 0.01);
  ASSERT_NEAR(3.6055512754639905, limited_a[6], 0.01);
  ASSERT_NEAR(2.6457513110645934, limited_a[7], 0.01);
  ASSERT_NEAR(1.0000000000000107, limited_a[8], 0.01);
  ASSERT_NEAR(0.0, limited_a[9], 0.01);

  // Edge cases
  ASSERT_THROW(apply_accel_limits_by_distance(downtracks, speeds_a, -3.0, 3.0), std::invalid_argument);
  ASSERT_THROW(apply_accel_limits_by_distance(downtracks, speeds_a, 3.0, -3.0), std::invalid_argument);

  speeds_a = {1,1,1};
  ASSERT_THROW(apply_accel_limits_by_distance(downtracks, speeds_a, 3.0, 3.0), std::invalid_argument);

  speeds_a = {};
  downtracks = {};
  limited_a = apply_accel_limits_by_distance(downtracks, speeds_a, 3.0, 3.0);
  ASSERT_EQ(limited_a.size(), 0);
}

TEST(trajectory_utils_test, apply_accel_limits_by_time) {
  // Accel case
  std::vector<double> speeds_a = {0.0, 0.0, 0.0, 0.0, 0.0,
                                  5.0, 5.0, 5.0, 5.0, 5.0};

  std::vector<double> times = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  std::vector<double> limits_a = apply_accel_limits_by_time(times, speeds_a, 3.0, 3.0);

  ASSERT_EQ(times.size(), limits_a.size());
  ASSERT_NEAR(0.0, limits_a[0], 0.01);
  ASSERT_NEAR(0.0, limits_a[1], 0.01);
  ASSERT_NEAR(0.0, limits_a[2], 0.01);
  ASSERT_NEAR(0.0, limits_a[3], 0.01);
  ASSERT_NEAR(0.0, limits_a[4], 0.01);
  ASSERT_NEAR(3.0, limits_a[5], 0.01);
  ASSERT_NEAR(5.0, limits_a[6], 0.01);
  ASSERT_NEAR(5.0, limits_a[7], 0.01);
  ASSERT_NEAR(5.0, limits_a[8], 0.01);
  ASSERT_NEAR(5.0, limits_a[9], 0.01);

  // Decel case
  speeds_a = {5.0, 5.0, 5.0, 5.0, 5.0,
              0.0, 0.0, 0.0, 0.0, 0.0};

  times = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  limits_a = apply_accel_limits_by_time(times, speeds_a, 3.0, 3.0);

  ASSERT_EQ(times.size(), limits_a.size());
  ASSERT_NEAR(5.0, limits_a[0], 0.01);
  ASSERT_NEAR(5.0, limits_a[1], 0.01);
  ASSERT_NEAR(5.0, limits_a[2], 0.01);
  ASSERT_NEAR(5.0, limits_a[3], 0.01);
  ASSERT_NEAR(5.0, limits_a[4], 0.01);
  ASSERT_NEAR(2.0, limits_a[5], 0.01);
  ASSERT_NEAR(0.0, limits_a[6], 0.01);
  ASSERT_NEAR(0.0, limits_a[7], 0.01);
  ASSERT_NEAR(0.0, limits_a[8], 0.01);
  ASSERT_NEAR(0.0, limits_a[9], 0.01);

  // Edge cases
  ASSERT_THROW(apply_accel_limits_by_distance(times, speeds_a, -3.0, 3.0), std::invalid_argument);
  ASSERT_THROW(apply_accel_limits_by_distance(times, speeds_a, 3.0, -3.0), std::invalid_argument);

  speeds_a = {1,1,1};
  ASSERT_THROW(apply_accel_limits_by_distance(times, speeds_a, 3.0, 3.0), std::invalid_argument);

  speeds_a = {};
  times = {};
  limits_a = apply_accel_limits_by_distance(times, speeds_a, 3.0, 3.0);
  ASSERT_EQ(limits_a.size(), 0);
}

}  // namespace trajectory_utils
