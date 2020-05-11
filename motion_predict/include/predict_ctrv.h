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

#ifndef PREDICT_CTRV_H
#define PREDICT_CTRV_H

#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/PredictedState.h>

namespace predict
{
namespace ctrv
{
std::vector<cav_msgs::PredictedState> predictPeriod(const cav_msgs::ExternalObject& obj, const double delta_t,
                                                    const double period, const float process_noise_max, const double confidence_drop_rate);
/*!
 * \fn CTRVPredict(const cav_msgs::ExternalObject &obj)
 * \brief CTRVPredict populates motion prediction with future pose and velocity.
 *     The predicted motion is
 *
 * \param  obj external object.
 * \param  delta_t prediction time into the future
 * \param  process_noise_max is the maximum process noise of the system
 *
 * \return The predicted state of the external object at time t + delta_t
 */

// Forward predict an external object
cav_msgs::PredictedState predictStep(const cav_msgs::ExternalObject& obj, const double delta_t,
                                     const float process_noise_max);

// Forward predict a prediction // TODO comment
cav_msgs::PredictedState predictStep(const cav_msgs::PredictedState& obj, const double delta_t,
                                     const double confidence_drop_rate);

}  // namespace ctrv

}  // namespace predict

#endif /* PREDICT_CTRV_H */