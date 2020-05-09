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

    TEST(MotionPredictTest, testpredictState)
    {
        geometry_msgs::Pose pose;
        pose.position.x = 1.3;
        pose.position.y = 1.4;
        pose.position.z = 2.5;

        geometry_msgs::Twist twist;
        twist.linear.x = 4.5;
        twist.linear.y = 2;
        twist.linear.z = 5;

        MotionPredict mp;
        double delta_t=0.1;

        cav_msgs::PredictedState pobj = mp.predictState(pose,twist,delta_t);
                 
        EXPECT_NEAR(2, pobj.predicted_position.position.x ,0.00001);
        EXPECT_NEAR(2.05,  pobj.predicted_position.position.y,0.00001);
        EXPECT_NEAR(2.5,  pobj.predicted_position.position.z,0.00001);

        EXPECT_NEAR(3,pobj.predicted_velocity.linear.x ,0.00001);
        EXPECT_NEAR(5.5, pobj.predicted_velocity.linear.y,0.00001);
        EXPECT_NEAR(5,  pobj.predicted_velocity.linear.z,0.00001);
    }


   

    TEST(MotionPredictTest, testexternalPredict)
    {
        MotionPredict mp;
       
        double delta_t=0.1;
        double ax=10;
        double ay=10;
        float process_noise_max=1000;

        cav_msgs::ExternalObject obj;

        mp.externalPredict(obj,delta_t,ax,ay,process_noise_max);        
                 
        EXPECT_NEAR(2, pobj.predicted_position_confidence ,0.00001);
        EXPECT_NEAR(2.05, pobj.predicted_velocity_confidence,0.00001);

    }


   /* TEST(MotionPredictTest, testexternalPredict)
    {
        MotionPredict mp;
       
        double delta_t=0.1;
        double ax=10;
        double ay=10;
        float process_noise_max=1000;

        cav_msgs::ExternalObject obj;

        mp.externalPredict(obj,delta_t,ax,ay,process_noise_max);        
                 
        EXPECT_NEAR(2, pobj.predicted_position_confidence ,0.00001);
        EXPECT_NEAR(2.05, pobj.predicted_velocity_confidence,0.00001);

    }


    TEST(MotionPredictTest, testexternalPredict)
    {
        MotionPredict mp;
       
        double delta_t=0.1;
        double ax=10;
        double ay=10;
        float process_noise_max=1000;

        cav_msgs::ExternalObject obj;

        mp.externalPredict(obj,delta_t,ax,ay,process_noise_max);        
                 
        EXPECT_NEAR(2, pobj.predicted_position_confidence ,0.00001);
        EXPECT_NEAR(2.05, pobj.predicted_velocity_confidence,0.00001);

    } */

}