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

#include "../src/predict_ctrv.cpp"
#include <gtest/gtest.h>

namespace motion_predict
{
namespace ctrv
{
TEST(predict_ctrv, buildCTRVState)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.3;
  pose.position.y = 1.4;
  pose.position.z = 2.5;

  // 90 Deg rotation about z and 0.1 deg rotation around x,y for noisy input
  pose.orientation.x = 0.0012341;
  pose.orientation.y = 0;
  pose.orientation.z = 0.7071068;
  pose.orientation.w = 0.7071057;

  geometry_msgs::msg::Twist twist;
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
  // twist.linear is expected in the object's body frame, so the propagation yaw is the
  // object's own heading (~90 deg from pose.orientation) composed with the local velocity
  // vector's angle (atan2(2, 4.5) =~ 0.41822), not the velocity angle alone.
  ASSERT_NEAR(result.yaw, 1.98902, 0.00001);
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

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.3;
  pose.position.y = 1.4;
  pose.position.z = 2.5;

  // 90 Deg rotation about z and 0.1 deg rotation around x,y for noisy input
  pose.orientation.x = 0.0012341;
  pose.orientation.y = 0;
  pose.orientation.z = 0.7071068;
  pose.orientation.w = 0.7071057;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 4.5;
  twist.linear.y = 2;
  twist.linear.z = 5;

  // 5 Deg per sec rotation about z and 0.1 deg/s rotation around x,y for noisy input
  twist.angular.x = 0.00174533;
  twist.angular.y = 0.00174533;
  twist.angular.z = 0.0872665;

  carma_perception_msgs::msg::PredictedState result = buildPredictionFromCTRVState(state, pose, twist);
  ASSERT_NEAR(result.predicted_position.position.x, pose.position.x, 0.00001);
  ASSERT_NEAR(result.predicted_position.position.y, pose.position.y, 0.00001);
  ASSERT_NEAR(result.predicted_position.position.z, pose.position.z, 0.00001);
  // The predicted orientation is the object's own propagated heading: state.yaw with the
  // local velocity vector's slip angle (atan2(2, 4.5)) subtracted back out. Here that
  // recovers ~90 deg, i.e. the original pose's yaw, expressed as a pure-yaw quaternion
  // (any roll/pitch noise in the original orientation is not carried through).
  ASSERT_NEAR(result.predicted_position.orientation.x, 0.0, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.y, 0.0, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.z, 0.70711, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.w, 0.70710, 0.00001);

  ASSERT_NEAR(result.predicted_velocity.linear.x, twist.linear.x, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.linear.y, twist.linear.y, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.linear.z, twist.linear.z, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.angular.x, twist.angular.x, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.angular.y, twist.angular.y, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.angular.z, twist.angular.z, 0.00001);
}

TEST(predict_ctrv, CTRVPredict) {
  // Regular prediction
  CTRV_State state;
  state.x = 1.3;
  state.y = 1.4;
  state.yaw = 1.5708;
  state.v = 4.9244289009;
  state.yaw_rate = 0.0872665;

  CTRV_State result = CTRVPredict(state, 0.1);

  ASSERT_NEAR(result.x, 1.29785, 0.00001);
  ASSERT_NEAR(result.y, 1.89244, 0.0001);
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

  ASSERT_NEAR(result.x, 1.3, 0.0001);
  ASSERT_NEAR(result.y, 1.89244, 0.0001);
  ASSERT_NEAR(result.yaw, 1.5708, 0.00001);
  ASSERT_NEAR(result.v, state.v, 0.00001);
  ASSERT_NEAR(result.yaw_rate, state.yaw_rate, 0.00001);
}

TEST(predict_ctrv, predictStepExternal)
{
  carma_perception_msgs::msg::ExternalObject obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.header.frame_id = "my_frame";
  obj.pose.pose.position.x = 5.0;
  obj.pose.pose.position.y = 1.4; // Added y position
  // Set the orientation quaternion for 0.79 radians yaw
  double yaw_angle = 0.79;
  obj.pose.pose.orientation.w = cos(yaw_angle / 2);
  obj.pose.pose.orientation.x = 0.0;
  obj.pose.pose.orientation.y = 0.0;
  obj.pose.pose.orientation.z = sin(yaw_angle / 2);
  obj.pose.covariance[0] = 1;
  obj.pose.covariance[7] = 1;
  obj.pose.covariance[35] = 1;
  // velocity.twist.linear is expected in the object's own body frame (x = forward, y = left),
  // matching pose.orientation. Driving straight forward with no lateral slip is therefore
  // expressed as pure x speed, regardless of the object's heading in the map.
  obj.velocity.twist.linear.x = 4.9244289009;
  obj.velocity.twist.linear.y = 0.0;
  obj.velocity.covariance[0] = 999;
  obj.velocity.covariance[7] = 999;
  obj.velocity.covariance[35] = 999;

  carma_perception_msgs::msg::PredictedState result = motion_predict::ctrv::predictStep(obj, 0.1, 1000, 0.99);

  EXPECT_NEAR(5.3466, result.predicted_position.position.x, 0.00001);  // Verify x position update
  EXPECT_NEAR(1.7498, result.predicted_position.position.y, 0.00001);  // Verify y position update
  EXPECT_NEAR(4.9244289009, result.predicted_velocity.linear.x, 0.00001); // Verify velocity speed
  EXPECT_NEAR(0.0, result.predicted_velocity.linear.y, 0.00001); // Verify velocity speed
  // No lateral slip and zero yaw rate, so the predicted heading should match the original.
  EXPECT_NEAR(obj.pose.pose.orientation.z, result.predicted_position.orientation.z, 0.00001);
  EXPECT_NEAR(obj.pose.pose.orientation.w, result.predicted_position.orientation.w, 0.00001);
  EXPECT_NEAR(0.99, result.predicted_position_confidence, 0.01);
  EXPECT_NEAR(0.001, result.predicted_velocity_confidence, 0.001);

  rclcpp::Time new_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int32_t(0.1 * 1e9))); // Increase by 0.1 sec
  int32_t new_time_sec = int32_t(new_time.nanoseconds() / 1e9);
  uint32_t new_time_nanosec = new_time.nanoseconds() - (new_time_sec * 1e9);
  EXPECT_EQ(result.header.stamp.sec, new_time_sec);
  EXPECT_EQ(result.header.stamp.nanosec, new_time_nanosec);
  EXPECT_EQ(result.header.frame_id, obj.header.frame_id);
}

TEST(predict_ctrv, predictStep)
{
  carma_perception_msgs::msg::PredictedState obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.header.frame_id = "my_frame";
  obj.predicted_position.position.x = 5.0;
  obj.predicted_position.orientation.w = 1.0;
  obj.predicted_position_confidence = 1;
  obj.predicted_velocity_confidence = 0.5;

  carma_perception_msgs::msg::PredictedState result = motion_predict::ctrv::predictStep(obj, 0.1, 0.99);

  ASSERT_NEAR(5.0, result.predicted_position.position.x, 0.00001);  // Verify update functions were called
  ASSERT_NEAR(0.99, result.predicted_position_confidence, 0.01);
  ASSERT_NEAR(0.495, result.predicted_velocity_confidence, 0.0001);

  rclcpp::Time new_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int32_t(0.1 * 1e9))); // Increase by 0.1 sec
  int32_t new_time_sec = int32_t(new_time.nanoseconds() / 1e9);
  uint32_t new_time_nanosec = new_time.nanoseconds() - (new_time_sec*1e9);
  ASSERT_EQ(result.header.stamp.sec, new_time_sec);
  ASSERT_EQ(result.header.stamp.nanosec, new_time_nanosec);
  ASSERT_EQ(result.header.frame_id, obj.header.frame_id);
}

TEST(predict_ctrv, predictPeriod)
{
  carma_perception_msgs::msg::ExternalObject obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.header.frame_id = "my_frame";
  obj.pose.pose.position.x = 5.0;
  obj.pose.pose.orientation.w = 1.0;
  obj.velocity.twist.linear.x = 1.0;
  obj.pose.covariance[0] = 1;
  obj.pose.covariance[7] = 1;
  obj.pose.covariance[35] = 1;
  obj.velocity.covariance[0] = 999;
  obj.velocity.covariance[7] = 999;
  obj.velocity.covariance[35] = 999;

  std::vector<carma_perception_msgs::msg::PredictedState> results = motion_predict::ctrv::predictPeriod(obj, 0.1, 0.21, 1000, 0.99);

  ASSERT_NEAR(5.1, results[0].predicted_position.position.x, 0.00001);  // Verify update functions were called
  ASSERT_NEAR(0.99, results[0].predicted_position_confidence, 0.01);
  ASSERT_NEAR(0.001, results[0].predicted_velocity_confidence, 0.001);

  rclcpp::Time new_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int32_t(0.1 * 1e9))); // Increase by 0.1 sec
  int32_t new_time_sec = int32_t(new_time.nanoseconds() / 1e9);
  uint32_t new_time_nanosec = new_time.nanoseconds() - (new_time_sec*1e9);
  ASSERT_EQ(results[0].header.stamp.sec, new_time_sec);
  ASSERT_EQ(results[0].header.stamp.nanosec, new_time_nanosec);
  ASSERT_EQ(results[0].header.frame_id, obj.header.frame_id);

  ASSERT_NEAR(5.2, results[1].predicted_position.position.x, 0.00001);  // Verify update functions were called
  ASSERT_NEAR(0.9801, results[1].predicted_position_confidence, 0.01);
  ASSERT_NEAR(0.00099, results[1].predicted_velocity_confidence, 0.00001);

  new_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int32_t(0.2 * 1e9))); // Increase by 0.2 sec
  new_time_sec = int32_t(new_time.nanoseconds() / 1e9);
  new_time_nanosec = new_time.nanoseconds() - (new_time_sec*1e9);
  ASSERT_EQ(results[1].header.stamp.sec, new_time_sec);
  ASSERT_EQ(results[1].header.stamp.nanosec, new_time_nanosec);
  ASSERT_EQ(results[1].header.frame_id, obj.header.frame_id);
}

TEST(predict_ctrv, yawQuaternionConversions)
{
  // Round trip yaw -> quaternion -> yaw, including negative angles and angles near +/- pi
  for (double yaw : { 0.0, 0.79, M_PI / 2, -M_PI / 2, 3.0, -3.0 })
  {
    geometry_msgs::msg::Quaternion q = getQuaternionFromYaw(yaw);
    EXPECT_NEAR(0.0, q.x, 0.000001);
    EXPECT_NEAR(0.0, q.y, 0.000001);
    EXPECT_NEAR(1.0, q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 0.000001);  // unit quaternion
    EXPECT_NEAR(yaw, getYawFromQuaternion(q), 0.000001);
  }

  // Yaw is still extracted correctly from a quaternion with a little roll/pitch noise
  geometry_msgs::msg::Quaternion noisy;
  noisy.x = 0.0012341;
  noisy.y = 0;
  noisy.z = 0.7071068;
  noisy.w = 0.7071057;
  EXPECT_NEAR(M_PI / 2, getYawFromQuaternion(noisy), 0.00001);
}

TEST(predict_ctrv, buildCTRVStateBodyFrameVelocity)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 2.0;
  pose.position.y = -1.0;
  pose.orientation = getQuaternionFromYaw(-M_PI / 2);  // facing -y in the map

  geometry_msgs::msg::Twist twist;

  // Pure forward body velocity: travel direction equals the object's heading
  twist.linear.x = 3.0;
  twist.linear.y = 0.0;
  CTRV_State result = buildCTRVState(pose, twist);
  EXPECT_NEAR(-M_PI / 2, result.yaw, 0.00001);
  EXPECT_NEAR(3.0, result.v, 0.00001);

  // Pure leftward body velocity (sliding sideways): travel direction is heading + 90 deg
  twist.linear.x = 0.0;
  twist.linear.y = 1.5;
  result = buildCTRVState(pose, twist);
  EXPECT_NEAR(0.0, result.yaw, 0.00001);
  EXPECT_NEAR(1.5, result.v, 0.00001);

  // Reversing: travel direction is opposite the heading
  twist.linear.x = -2.0;
  twist.linear.y = 0.0;
  result = buildCTRVState(pose, twist);
  EXPECT_NEAR(M_PI / 2, result.yaw, 0.00001);
  EXPECT_NEAR(2.0, result.v, 0.00001);

  // Stationary: no velocity direction, so the yaw falls back to the object's heading
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  result = buildCTRVState(pose, twist);
  EXPECT_NEAR(-M_PI / 2, result.yaw, 0.00001);
  EXPECT_NEAR(0.0, result.v, 0.00001);
}

TEST(predict_ctrv, buildPredictionFromCTRVStateRemovesSlip)
{
  geometry_msgs::msg::Pose pose;
  pose.position.z = 1.0;
  pose.orientation = getQuaternionFromYaw(0.0);

  // Object is sliding directly left (slip angle of 90 deg) and its propagated
  // direction of travel is 1.0 rad, so its own heading should be 1.0 - pi/2.
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 2.0;

  CTRV_State state;
  state.x = 3.0;
  state.y = 4.0;
  state.yaw = 1.0;
  state.v = 2.0;

  carma_perception_msgs::msg::PredictedState result = buildPredictionFromCTRVState(state, pose, twist);
  EXPECT_NEAR(3.0, result.predicted_position.position.x, 0.00001);
  EXPECT_NEAR(4.0, result.predicted_position.position.y, 0.00001);
  EXPECT_NEAR(1.0, result.predicted_position.position.z, 0.00001);
  EXPECT_NEAR(1.0 - M_PI / 2, getYawFromQuaternion(result.predicted_position.orientation), 0.00001);
}

TEST(predict_ctrv, predictStepExternalForwardUsesHeading)
{
  // Regression test for carma-platform#2407: a body-frame forward velocity must be
  // propagated along the object's heading, not along the map x axis.
  carma_perception_msgs::msg::ExternalObject obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.pose.pose.position.x = 5.0;
  obj.pose.pose.position.y = 1.4;
  obj.pose.pose.orientation = getQuaternionFromYaw(M_PI / 2);  // facing +y in the map
  obj.velocity.twist.linear.x = 2.0;                            // 2 m/s forward
  obj.velocity.twist.linear.y = 0.0;

  carma_perception_msgs::msg::PredictedState result = predictStep(obj, 0.1, 1000, 0.99);

  EXPECT_NEAR(5.0, result.predicted_position.position.x, 0.00001);
  EXPECT_NEAR(1.6, result.predicted_position.position.y, 0.00001);
  EXPECT_NEAR(M_PI / 2, getYawFromQuaternion(result.predicted_position.orientation), 0.00001);
}

TEST(predict_ctrv, predictStepExternalSidewaysDrift)
{
  // Object faces +x but slides left (+y). It should move along +y while keeping its heading.
  carma_perception_msgs::msg::ExternalObject obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.pose.pose.position.x = 1.0;
  obj.pose.pose.position.y = 2.0;
  obj.pose.pose.orientation = getQuaternionFromYaw(0.0);
  obj.velocity.twist.linear.x = 0.0;
  obj.velocity.twist.linear.y = 1.5;

  carma_perception_msgs::msg::PredictedState result = predictStep(obj, 0.2, 1000, 0.99);

  EXPECT_NEAR(1.0, result.predicted_position.position.x, 0.00001);
  EXPECT_NEAR(2.3, result.predicted_position.position.y, 0.00001);
  EXPECT_NEAR(0.0, getYawFromQuaternion(result.predicted_position.orientation), 0.00001);
  EXPECT_NEAR(0.0, result.predicted_velocity.linear.x, 0.00001);  // body-frame velocity is unchanged
  EXPECT_NEAR(1.5, result.predicted_velocity.linear.y, 0.00001);
}

TEST(predict_ctrv, predictStepExternalReversing)
{
  // Object faces +x but drives backwards. It should move along -x while keeping its heading.
  carma_perception_msgs::msg::ExternalObject obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.pose.pose.orientation = getQuaternionFromYaw(0.0);
  obj.velocity.twist.linear.x = -2.0;

  carma_perception_msgs::msg::PredictedState result = predictStep(obj, 0.5, 1000, 0.99);

  EXPECT_NEAR(-1.0, result.predicted_position.position.x, 0.00001);
  EXPECT_NEAR(0.0, result.predicted_position.position.y, 0.00001);
  EXPECT_NEAR(0.0, getYawFromQuaternion(result.predicted_position.orientation), 0.00001);
}

TEST(predict_ctrv, predictStepExternalYawRateRotatesOrientation)
{
  // Previously the predicted orientation was copied from the input. With a non-zero yaw
  // rate it should now rotate by yaw_rate * delta_t.
  carma_perception_msgs::msg::ExternalObject obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.pose.pose.orientation = getQuaternionFromYaw(0.0);
  obj.velocity.twist.linear.x = 3.0;
  obj.velocity.twist.angular.z = 0.5;

  carma_perception_msgs::msg::PredictedState result = predictStep(obj, 0.4, 1000, 0.99);

  EXPECT_NEAR(1.19202, result.predicted_position.position.x, 0.00001);
  EXPECT_NEAR(0.11960, result.predicted_position.position.y, 0.00001);
  EXPECT_NEAR(0.2, getYawFromQuaternion(result.predicted_position.orientation), 0.00001);
  EXPECT_NEAR(0.0998334, result.predicted_position.orientation.z, 0.00001);
  EXPECT_NEAR(0.9950042, result.predicted_position.orientation.w, 0.00001);
}

TEST(predict_ctrv, predictPeriodWithSlipAndYawRate)
{
  // Chained predictions must keep the slip angle out of the reported heading at every step,
  // so the heading grows by exactly yaw_rate * delta_t per step.
  carma_perception_msgs::msg::ExternalObject obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.pose.pose.orientation = getQuaternionFromYaw(0.3);
  obj.velocity.twist.linear.x = 2.0;
  obj.velocity.twist.linear.y = 1.0;
  obj.velocity.twist.angular.z = 0.2;

  std::vector<carma_perception_msgs::msg::PredictedState> results = predictPeriod(obj, 0.1, 0.25, 1000, 0.99);

  ASSERT_EQ(3u, results.size());

  EXPECT_NEAR(0.15996, results[0].predicted_position.position.x, 0.00001);
  EXPECT_NEAR(0.15624, results[0].predicted_position.position.y, 0.00001);
  EXPECT_NEAR(0.32, getYawFromQuaternion(results[0].predicted_position.orientation), 0.00001);

  EXPECT_NEAR(0.31676, results[1].predicted_position.position.x, 0.00001);
  EXPECT_NEAR(0.31565, results[1].predicted_position.position.y, 0.00001);
  EXPECT_NEAR(0.34, getYawFromQuaternion(results[1].predicted_position.orientation), 0.00001);

  EXPECT_NEAR(0.47034, results[2].predicted_position.position.x, 0.00001);
  EXPECT_NEAR(0.47817, results[2].predicted_position.position.y, 0.00001);
  EXPECT_NEAR(0.36, getYawFromQuaternion(results[2].predicted_position.orientation), 0.00001);
}

}  // namespace ctrv
}  // namespace motion_predict
