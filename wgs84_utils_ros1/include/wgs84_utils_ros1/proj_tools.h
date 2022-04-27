#pragma once
/*
 * Copyright (C) 2021 LEIDOS.
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
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include "wgs84_utils.h"

namespace wgs84_utils
{
namespace proj_tools
{
/**
 * \brief Extracts the +axis field value from a proj string if it exists.
 *        If the field does not exist then this returns "enu" which is the default assumed by the proj library.
 *
 * \param proj_string A proj string following standard proj library notation
 * \throw std::invalid_argument If the axis tag is provided but is empty such as "+axis=" or "+axis= "
 *
 * \return The extracted axis 3 letter field value. For example a string like "+proj=tmerc +lat_0=0.0 +lon_0=0.0
 * +axis=ned" would return the value "ned"
 */
std::string getAxisFromProjString(std::string proj_string);

/**
 * \brief Returns a 3d rotation to align a provided righthanded proj library compliant +axis specification with an NED
 * (North East Down) frame at the same location. At the time of this methods creation proj only supported axis
 * specifications where x and y align with lat/lon in some form. This method only supports right handed axis
 * orientations at this time such as "enu" and "ned" but NOT "wnu" which would be left handed.
 *
 * \param axis The value of an +axis tag extracted from a proj library projection to get rotation to NED from. For
 * example "enu"
 * 
 * \throw std::invalid_arugmnet if provided axis is left handed instead of right handed
 *
 * \return The rotation which would need to be applied to the provided axis for it to become and NED axis.
 *
 */
tf2::Quaternion getRotationOfNEDFromProjAxis(const std::string& axis);

}  // namespace proj_tools
}  // namespace wgs84_utils