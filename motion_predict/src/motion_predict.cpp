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

namespace Motion{

cav_msgs::PredictedState MotionPredict::CVPredict(const cav_msgs::ExternalObject &obj,const double &delta_t,const double &ax,const double &ay,const float &process_noise_max)
{
	//double delta_t=1.5; 

	Eigen::VectorXd x(4);

	x(0)=obj.pose.pose.position.x; // Position X
	x(1)=obj.pose.pose.position.y; // Position Y
	// x(2)=obj.pose.pose.position.z; // Position Z

	x(2)=obj.velocity.twist.linear.x; // Linear Velocity X
	x(3)=obj.velocity.twist.linear.y; // Linear Velocity Y
	// x(5)=obj.velocity.twist.linear.z; // Linear Velocity Z

	Eigen::MatrixXd F=Eigen::MatrixXd::Identity(x.size(), x.size()); // Generate identity matrix

	F(0,3)=delta_t;
	F(1,4)=delta_t;
	
	x = F * x; // State transitation matrix

	Eigen::MatrixXd P(4,4);

	P(0,0)=obj.pose.covariance[0];
	P(1,1)=obj.pose.covariance[7];
	P(2,2)=obj.velocity.covariance[0];
	P(3,3)=obj.velocity.covariance[7];

	Eigen::MatrixXd Q(4,4);

	double delta_t2 = delta_t * delta_t;
    double delta_t3 = delta_t2 * delta_t;
    double delta_t4 = delta_t3 * delta_t;

    Q << (delta_t4 / 4 * ax), 0,( delta_t3 / 2 * ax), 0, 0, (delta_t4 / 4 * ay), 0,( delta_t3 / 2 * ay), (delta_t3 / 2 * ax), 0,(delta_t2*ax), 0 , 0 , (delta_t3 / 2 * ay), 0 , (delta_t2*ay); // Process Noise Matrix
	
    Eigen::MatrixXd Ft = F.transpose();
    P = F * P * Ft + Q; //State Covariance Matrix

    cav_msgs::PredictedState pobj;

    pobj.header=obj.header; // Header

    pobj.predicted_position.position.x=x(0); // Predicted Position X
    pobj.predicted_position.position.y=x(1); // Predicted Position Y
    pobj.predicted_position.position.z=obj.pose.pose.position.z; // Predicted Position Z

    float position_process_noise_avg=(P(0,0)+P(1,1))/2;

    pobj.predicted_position_confidence=Mapping(position_process_noise_avg,process_noise_max);

    pobj.predicted_velocity.linear.x=x(2); // Predicted Linear Velocity X
    pobj.predicted_velocity.linear.y=x(3); // Predicted Linear Velocity Y
    pobj.predicted_velocity.linear.z=obj.velocity.twist.linear.z; // Predicted Linear Velocity Z

    float velocity_process_noise_avg=(P(2,2)+P(3,3))/2;

    pobj.predicted_velocity_confidence=Mapping(velocity_process_noise_avg,process_noise_max);

    return pobj;
}

float MotionPredict::Mapping(const float &input,const float &process_noise_max)
{
	// Note as the value of the covariance increases confidence value decreases.

	float input_start = 1; // The lowest number of the range input.
    float input_end = process_noise_max; // The lowest number of the range input.
    float output_start = 0; // The lowest number of the range output.
    float output_end = 1; // The largest number of the range ouput.
	float output = (input - input_start) / (input_end - input_start) * (output_end - output_start) + output_start;
	
	return output;
}


}