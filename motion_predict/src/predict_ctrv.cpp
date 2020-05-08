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

#include "motion_predict.h"

namespace predict
{
namespace ctrv
{
struct CTRV_State
{
  double x = 0;
  double y = 0;
  double yaw = 0;
  double v = 0;  // magnitude of the speed
  double yaw_rate = 0;
};

CTRV_State buildCTRVState(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist)
{
  geometry_msgs::Quaternion quat = pose.orientation;
  Eigen::Quaternionf e_quat(quat.w, quat.x, quat.y, quat.z);
  Eigen::Vector3f rpy = e_quat.toRotationMatrix().eulerAngles(0, 1, 2);
  Eigen::Vector2f x_y_vel(twist.linear.x, twist.linear.y);

  CTRV_State state;
  state.x = pose.position.x;
  state.y = pose.position.y;
  state.yaw = rpy[2];
  state.v = x_y_vel.norm();
  state.yaw_rate = twist.angular.z;

  return state;
}

cav_msgs::PredictedState buildPredictionFromCTRVState(const CTRV_State& state, const geometry_msgs::Pose& original_pose,
                                                      const geometry_msgs::Twist& original_twist)
{
  cav_msgs::PredictedState pobj;

  // Map position
  pobj.predicted_position.position.x = state.x;
  pobj.predicted_position.position.y = state.y;
  pobj.predicted_position.position.z = original_pose.position.z;

  // Map orientation
  Eigen::Quaternionf original_quat(original_pose.orientation.w, original_pose.orientation.x,
                                   original_pose.orientation.y, original_pose.orientation.z);
  Eigen::Vector3f original_rpy = original_quat.toRotationMatrix().eulerAngles(0, 1, 2);

  Eigen::Quaternionf final_quat;
  final_quat = Eigen::AngleAxisf(original_rpy[0], Eigen::Vector3f::UnitX()) *
               Eigen::AngleAxisf(original_rpy[1], Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(state.yaw, Eigen::Vector3f::UnitZ());

  pobj.predicted_position.orientation.x = final_quat.x();
  pobj.predicted_position.orientation.y = final_quat.y();
  pobj.predicted_position.orientation.z = final_quat.z();
  pobj.predicted_position.orientation.w = final_quat.w();

  // Map twist
  // Constant velocity model means twist remains unchanged
  pobj.predicted_velocity = original_twist;

  return pobj;
}

CTRV_State CTRVPredict(const CTRV_State& state, const double delta_t)
{
  CTRV_State next_state;
	// TODO implement divide by 0 logic here https://winfriedauner.de/projects/unscented/ctrv/
  double v_w = state.v / state.yaw_rate;
  double sin_yaw = sin(state.yaw);
  double wT = state.yaw_rate * delta_t;

  next_state.x = v_w * sin(wT + state.yaw) - v_w * sin_yaw + state.x;
  next_state.y = -v_w * cos(wT + state.yaw) + v_w * sin_yaw + state.y;
  next_state.yaw = wT + state.yaw;
  next_state.v = state.v;
  next_state.yaw_rate = state.yaw_rate;

  return next_state;
}

// Forward predict an external object
cav_msgs::PredictedState predictStep(const cav_msgs::ExternalObject& obj, const double delta_t,
                                     const float process_noise_max, const double confidence_drop_rate)
{
  // Get initial state
  CTRV_State state = buildCTRVState(obj.pose.pose, obj.velocity.twist);

  // Predict Motion
  CTRV_State next_state = CTRVPredict(state, delta_t);

  // Convert CTRV to predicted state object
  cav_msgs::PredictedState pobj = buildPredictionFromCTRVState(next_state, obj.pose.pose, obj.velocity.twist);

  // Compute confidence values
  Eigen::MatrixXd P(4, 4);

  P(0, 0) = obj.pose.covariance[0];
  P(1, 1) = obj.pose.covariance[7];
  P(2, 2) = obj.velocity.covariance[0];
  P(3, 3) = obj.velocity.covariance[7];

  // Average diagonal of process noise
  double position_process_noise_avg = (P(0, 0) + P(1, 1)) / 2;
  double velocity_process_noise_avg = (P(2, 2) + P(3, 3)) / 2;

  // Map process noise average to confidence
  Motion::MotionPredict mp;
  pobj.predicted_position_confidence = mp.Mapping(position_process_noise_avg, process_noise_max) * confidence_drop_rate;
  pobj.predicted_velocity_confidence = mp.Mapping(velocity_process_noise_avg, process_noise_max) * confidence_drop_rate;

  return pobj;
}

// Forward predict a prediction
cav_msgs::PredictedState predictStep(const cav_msgs::PredictedState& obj, const double delta_t,
                                     const double confidence_drop_rate)
{
  // Get initial state
  CTRV_State state = buildCTRVState(obj.predicted_position, obj.predicted_velocity);

  // Predict Motion
  CTRV_State next_state = CTRVPredict(state, delta_t);

  // Convert CTRV to predicted state object
  cav_msgs::PredictedState pobj =
      buildPredictionFromCTRVState(next_state, obj.predicted_position, obj.predicted_velocity);

  // Map process noise average to confidence
  pobj.predicted_position_confidence = obj.predicted_position_confidence * confidence_drop_rate;
  pobj.predicted_velocity_confidence = obj.predicted_velocity_confidence * confidence_drop_rate;
}

std::vector<cav_msgs::PredictedState> predictPeriod(const cav_msgs::ExternalObject& obj, const double delta_t,
                                                    const double period, const float process_noise_max, const double confidence_drop_rate)
{
	std::vector<cav_msgs::PredictedState> predicted_states = { predictStep(obj, delta_t, process_noise_max, confidence_drop_rate) };

	double t = delta_t;
	while (t < period) {
		predicted_states.emplace_back( predictStep( predicted_states.back(), delta_t, confidence_drop_rate) );
		t += delta_t;
	}
}

}  // namespace ctrv

}  // namespace predict
