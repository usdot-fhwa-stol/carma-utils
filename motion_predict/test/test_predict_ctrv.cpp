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

#include "predict_ctrv.h"
#include "../src/predict_ctrv.cpp"
#include <gtest/gtest.h>

namespace predict
{
namespace ctrv
{
/*
struct CTRV_State
{
double x = 0;
double y = 0;
double yaw = 0;
double v = 0;  // magnitude of the speed
double yaw_rate = 0;
};

cav_msgs::PredictedState predictStep(const cav_msgs::ExternalObject& obj, const double delta_t,
                                 const float process_noise_max, const double confidence_drop_rate);

cav_msgs::PredictedState predictStep(const cav_msgs::PredictedState& obj, const double delta_t,
                                 const double confidence_drop_rate);

std::vector<cav_msgs::PredictedState> predictPeriod(const cav_msgs::ExternalObject& obj, const double delta_t,
                                                const double period, const float process_noise_max, const double
confidence_drop_rate);
*/
TEST(predict_ctrv, buildCTRVState)
{
  geometry_msgs::Pose pose;
  pose.position.x = 1.3;
  pose.position.y = 1.4;
  pose.position.z = 2.5;

  // 90 Deg rotation about z and 0.1 deg rotation around x,y for noisy input
  pose.orientation.x = 0.0012341;
  pose.orientation.y = 0;
  pose.orientation.z = 0.7071068;
  pose.orientation.w = 0.7071057;

  geometry_msgs::Twist twist;
  twist.linear.x = 4.5;
  twist.linear.y = 2;
  twist.linear.z = 5;

  // 5 Deg per sec rotation about z and 0.1 deg/s rotation around x,y for noisy input
  twist.angular.x = 0.00174533;
  twist.angular.y = 0.00174533;
  twist.angular.z = 0.0872665;

  CTRV_State result = buildCTRVState(pose, twist);
  ASSERT_NEAR(result.x, 1.3, 0.000001);
  ASSERT_NEAR(result.y, 1.4, 0.000001);
  ASSERT_NEAR(result.yaw, 1.98902433, 0.00001);
  ASSERT_NEAR(result.v, 4.9244289009, 0.000001);
  ASSERT_NEAR(result.yaw_rate, 0.0872665, 0.0000001);
}

TEST(predict_ctrv, buildPredictionFromCTRVState)
{
  CTRV_State state;
  state.x = 1.3;
  state.y = 1.4;
  state.yaw = 1.98902433;
  state.v = 4.9244289009;
  state.yaw_rate = 0.0872665;

  geometry_msgs::Pose pose;
  pose.position.x = 1.3;
  pose.position.y = 1.4;
  pose.position.z = 2.5;

  // 90 Deg rotation about z and 0.1 deg rotation around x,y for noisy input
  pose.orientation.x = 0.0012341;
  pose.orientation.y = 0;
  pose.orientation.z = 0.7071068;
  pose.orientation.w = 0.7071057;

  geometry_msgs::Twist twist;
  twist.linear.x = 4.5;
  twist.linear.y = 2;
  twist.linear.z = 5;

  // 5 Deg per sec rotation about z and 0.1 deg/s rotation around x,y for noisy input
  twist.angular.x = 0.00174533;
  twist.angular.y = 0.00174533;
  twist.angular.z = 0.0872665;

  cav_msgs::PredictedState result = buildPredictionFromCTRVState(state, pose, twist);
  ASSERT_NEAR(result.predicted_position.position.x, pose.position.x, 0.00001);
  ASSERT_NEAR(result.predicted_position.position.y, pose.position.y, 0.00001);
  ASSERT_NEAR(result.predicted_position.position.z, pose.position.z, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.x, pose.orientation.x, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.y, pose.orientation.y, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.z, pose.orientation.z, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.w, pose.orientation.w, 0.00001);

  ASSERT_NEAR(result.predicted_velocity.linear.x, twist.linear.x, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.linear.y, twist.linear.y, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.linear.z, twist.linear.z, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.angular.x, twist.angular.x, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.angular.y, twist.angular.y, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.angular.z, twist.angular.z, 0.00001);
}

TEST(predict_ctrv, CTRVPredict)
{
  // Regular prediction
  CTRV_State state;
  state.x = 1.3;
  state.y = 1.4;
  state.yaw = 1.5708;
  state.v = 4.9244289009;
  state.yaw_rate = 0.0872665;

  CTRV_State result = CTRVPredict(state, 0.1);

  ASSERT_NEAR(result.x, 1.29785, 0.00001);
  ASSERT_NEAR(result.y, 58.3224, 0.0001);
  ASSERT_NEAR(result.yaw, 1.57953, 0.00001);
  ASSERT_NEAR(result.v, state.v, 0.00001);
  ASSERT_NEAR(result.yaw_rate, state.yaw_rate, 0.00001);

  // Divide by 0 case
  state.x = 1.3;
  state.y = 1.4;
  state.yaw = 1.5708;
  state.v = 4.9244289009;
  state.yaw_rate = 0.0;

  result = CTRVPredict(state, 0.1);

  ASSERT_NEAR(result.x, 1.2999819, 0.0001);
  ASSERT_NEAR(result.y, 1.89244, 0.0001);
  ASSERT_NEAR(result.yaw, 1.5708, 0.00001);
  ASSERT_NEAR(result.v, state.v, 0.00001);
  ASSERT_NEAR(result.yaw_rate, state.yaw_rate, 0.00001);
}

}  // namespace ctrv
}  // namespace predict