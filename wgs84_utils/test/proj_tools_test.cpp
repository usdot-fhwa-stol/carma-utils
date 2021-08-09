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
#include <gtest/gtest.h>
#include <wgs84_utils/proj_tools.h>


TEST(proj_tools, getAxisFromProjString)
{
  std::string base_proj = "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";

  std::string axis = wgs84_utils::proj_tools::getAxisFromProjString(base_proj);
  ASSERT_TRUE(axis.compare("enu") == 0); // Default case

  axis = wgs84_utils::proj_tools::getAxisFromProjString(base_proj + " +axis=enu ");
  ASSERT_TRUE(axis.compare("enu") == 0);

  axis = wgs84_utils::proj_tools::getAxisFromProjString(base_proj + " +axis = ned ");
  ASSERT_TRUE(axis.compare("ned") == 0);

  axis = wgs84_utils::proj_tools::getAxisFromProjString(base_proj + " +axis = nwu +h_0=10.0");
  ASSERT_TRUE(axis.compare("nwu") == 0);

  ASSERT_THROW(wgs84_utils::proj_tools::getAxisFromProjString(base_proj + " +axis = "), std::invalid_argument); // Empty case
}

void assertNearQuat(const tf2::Quaternion& q1, const tf2::Quaternion& q2, double bounds) {
  ASSERT_NEAR(q1.x(), q2.x(), bounds);
  ASSERT_NEAR(q1.y(), q2.y(), bounds);
  ASSERT_NEAR(q1.z(), q2.z(), bounds);
  ASSERT_NEAR(q1.w(), q2.w(), bounds);
}

TEST(proj_tools, getRotationOfNEDFromProjAxis) {
  tf2::Quaternion rot = wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis("enu");

  tf2::Quaternion solution;
  solution.setRPY(180.0 * wgs84_utils::DEG2RAD, 0,
                       90.0 * wgs84_utils::DEG2RAD);
  solution = solution.inverse();

  assertNearQuat(rot, solution, 0.00001);

  rot = wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis("nwu");

  solution.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, 0);
  solution = solution.inverse();

  assertNearQuat(rot, solution, 0.00001);

  rot = wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis("wsu");

  solution.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, -90.0 * wgs84_utils::DEG2RAD);
  solution = solution.inverse();

  assertNearQuat(rot, solution, 0.00001);

  rot = wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis("seu");

  solution.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, -180.0 * wgs84_utils::DEG2RAD);
  solution = solution.inverse();

  assertNearQuat(rot, solution, 0.00001);

  rot = wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis("ned");

  solution = tf2::Quaternion::getIdentity();
  solution = solution.inverse();

  assertNearQuat(rot, solution, 0.00001);

  rot = wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis("wnd");

  solution.setRPY(0, 0, -90.0 * wgs84_utils::DEG2RAD);
  solution = solution.inverse();

  assertNearQuat(rot, solution, 0.00001);

  rot = wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis("swd");

  solution.setRPY(0, 0, 180.0 * wgs84_utils::DEG2RAD);
  solution = solution.inverse();

  assertNearQuat(rot, solution, 0.00001);

  rot = wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis("esd");

  solution.setRPY(0, 0, 90.0 * wgs84_utils::DEG2RAD);
  solution = solution.inverse();

  assertNearQuat(rot, solution, 0.00001);

  ASSERT_THROW(wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis("exception"), std::invalid_argument);
  
}
