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
#include <gtest/gtest.h>

namespace Motion
{

    TEST(MotionPredictTest, testMappingMid)
    {
        MotionPredict mp;

        float input=500;
        float process_noise_max=1000;
                
        EXPECT_NEAR(0.499499, mp.Mapping(input,process_noise_max),0.00001);
     
    }

    TEST(MotionPredictTest, testMappingLow)
    {
        MotionPredict mp;
        
        float input=25;
        float process_noise_max=1000;
       
        
        EXPECT_NEAR(0.024024, mp.Mapping(input,process_noise_max),0.00001);
    }

    TEST(MotionPredictTest, testMappingHigh)
    {
        MotionPredict mp;
        
        float input=750;
        float process_noise_max=1000;
          
        EXPECT_NEAR(0.74975, mp.Mapping(input,process_noise_max),0.00001);
    }


 /*  TEST(MotionPredictTest, testCVPredict)
    {
        MotionPredict mp;

        cav_msgs::ExternalObject obj;

        obj.pose.pose.position.x=
        obj.pose.pose.position.y=
        obj.pose.pose.position.z=
        obj.velocity.twist.linear.x=
        obj.velocity.twist.linear.y=
        obj.velocity.twist.linear.z=

        obj.pose.covariance[0]=1;
        obj.pose.covariance[7]=1;
        obj.velocity.covariance[0]=1000;
        obj.velocity.covariance[7]=1000;

        cav_msgs::PredictedState pobj;

        pobj=mp.CVPredict(obj,1,9,9,1000);

        EXPECT_EQ(1, pobj.predicted_position.position.x);
        EXPECT_EQ(2, pobj.predicted_position.position.y);
        EXPECT_EQ(3, pobj.predicted_position.position.z);
        EXPECT_EQ(4, pobj.predicted_position_confidence);

        EXPECT_EQ(5, pobj.predicted_velocity.linear.x);
        EXPECT_EQ(6, pobj.predicted_velocity.linear.y);
        EXPECT_EQ(7, pobj.predicted_velocity.linear.z);
        EXPECT_EQ(8, pobj.predicted_velocity_confidence);
    }*/

}