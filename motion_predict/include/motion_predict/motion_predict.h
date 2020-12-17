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

#ifndef MOTION_PREDICT_H
#define MOTION_PREDICT_H

#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/PredictedState.h>
#include <cav_msgs/ExternalObjectList.h>
#include <Eigen/Dense>
#include <vector>

namespace motion_predict{

namespace cv{

    /*! 
    \brief  Mapping is used to map input range to an output range of different bandwidth.
    \param  input is the current value of the process noise.
    \param  process_noise_max is the maxium process noise of the system
    */

    double Mapping(const double input,const double process_noise_max);

    /*! 
    \brief  predictState is used to predict future state.
    \param  pose is position and orientation (m).
    \param  twist is velocity (m/s).
    \param  delta_t time predicted into the future (sec).
    */

    cav_msgs::PredictedState predictState(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist,const double delta_t);

    /*! 
    \brief  externalPredict populates motion prediction with future pose and velocity.
    \param  obj external object.
    \param  delta_t prediciton time into the future (sec)
    \param  ax acceleration noise along x-axis (m^2/s^4)
    \param  ay acceleration noise along y-axis (m^2/s^4)
    \param  process_noise_max is the maximum process noise of the system
    */

    cav_msgs::PredictedState externalPredict(const cav_msgs::ExternalObject &obj,const double delta_t,const double ax,const double ay,const double process_noise_max);

    /*! 
    \brief  externalPeriod populates sequence of predicted motion of the object.
    \param  obj external object.
    \param  delta_t prediciton time into the future (sec)
    \param  period sequence/time steps (sec)
    \param  ax acceleration noise along x-axis (m^2/s^4)
    \param  ay acceleration noise along y-axis (m^2/s^4)
    \param  process_noise_max is the maximum process noise of the system
    \param  confidence_drop_rate rate of drop in confidence with time
    */

    std::vector<cav_msgs::PredictedState> predictPeriod(const cav_msgs::ExternalObject& obj, const double delta_t, const double period,const double ax,const double ay ,const double process_noise_max,const double confidence_drop_rate);

    /*! 
    \brief  Mapping is used to map input range to an output range of different bandwidth.
    \param  obj predicted object
    \param  delta_t time predicted into the future (sec)
    \param  confidence_drop_rate rate of drop in confidence with time
    */

    cav_msgs::PredictedState predictStep(const cav_msgs::PredictedState& obj, const double delta_t, const double confidence_drop_rate);
   

}//cv

}//motion_predict

#endif /* MOTION_PREDICT_H */