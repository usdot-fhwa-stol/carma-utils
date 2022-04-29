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
#include <wgs84_utils_ros1/proj_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <algorithm>

namespace wgs84_utils
{
namespace proj_tools
{
std::string getAxisFromProjString(std::string proj_string)
{
  boost::erase_all(proj_string, " ");  // Remove white space from string

  std::vector<std::string> strs;
  boost::split(strs, proj_string, boost::is_any_of("+"));  // Split on + sign used to denote proj parameters

  std::string axis = "enu";  // Default axis alignment is ENU in proj

  for (const auto& param : strs)  // Iterate over proj string parameters until axis parameter is found
  {
    size_t axis_pos = param.find("axis=");
    if (axis_pos != 0)
    {
      continue;
    }

    size_t equal_pos = param.find("=");
    if (equal_pos >= param.size() - 1)
    {
      throw std::invalid_argument("Cannot extract +axis tag as georeference has an +axis tag which "
                                  "is empty: " +
                                  param);
    }

    axis = param.substr(param.find("=") + 1);

    break;  // If axis is found no reason to keep iterating.
  }

  return axis;
}

tf2::Quaternion getRotationOfNEDFromProjAxis(const std::string& axis)
{
  /*

    Proj Axis notation
    “e” - Easting
    “w” - Westing
    “n” - Northing
    “s” - Southing
    “u” - Up
    “d” - Down

    */

  tf2::Quaternion axis_in_ned;

  // Convert axis into transform with NED frame
  // Only support right handed coordinate systems at this time
  if (axis.compare("enu") == 0)
  {  // East North Up

    axis_in_ned.setRPY(180.0 * wgs84_utils::DEG2RAD, 0,
                       90.0 * wgs84_utils::DEG2RAD);  // Convert from NED frame using fixed-axis roll pitch yaw
                                                      // rotations
  }
  else if (axis.compare("nwu") == 0)
  {  // North West Up

    axis_in_ned.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, 0);
  }
  else if (axis.compare("wsu") == 0)
  {  // West South Up

    axis_in_ned.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, -90.0 * wgs84_utils::DEG2RAD);
  }
  else if (axis.compare("seu") == 0)
  {  // South East Up

    axis_in_ned.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, -180.0 * wgs84_utils::DEG2RAD);
  }
  else if (axis.compare("ned") == 0)
  {  // North East Down

    axis_in_ned = tf2::Quaternion::getIdentity();
  }
  else if (axis.compare("wnd") == 0)
  {  // West North Down

    axis_in_ned.setRPY(0, 0, -90.0 * wgs84_utils::DEG2RAD);
  }
  else if (axis.compare("swd") == 0)
  {  // South West Down

    axis_in_ned.setRPY(0, 0, 180.0 * wgs84_utils::DEG2RAD);
  }
  else if (axis.compare("esd") == 0)
  {  // East South Down

    axis_in_ned.setRPY(0, 0, 90.0 * wgs84_utils::DEG2RAD);
  }
  else
  {
    throw std::invalid_argument("Proj axis to enu conversion only supports projections using right handed coordinate frames");
  }

  return axis_in_ned.inverse();  // Inverse rotation to get rotations required to come from existing axis to ned
}
}  // namespace proj_tools
}  // namespace wgs84_utils