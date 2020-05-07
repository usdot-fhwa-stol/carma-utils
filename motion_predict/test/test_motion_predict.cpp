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

namespace motion_predict{

namespace cv{

    TEST(MotionPredictTest, testMappingMid)
    {
        double input=500;
        double process_noise_max=1000;
                
        EXPECT_NEAR(0.499499,Mapping(input,process_noise_max),0.00001);
     
    }

    TEST(MotionPredictTest, testMappingLow)
    {
        double input=25;
        double process_noise_max=1000;
               
        EXPECT_NEAR(0.024024,Mapping(input,process_noise_max),0.00001);
    }

    TEST(MotionPredictTest, testMappingHigh)
    {
          
        double input=750;
        double process_noise_max=1000;
          
        EXPECT_NEAR(0.74975,Mapping(input,process_noise_max),0.00001);
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
  
        double delta_t=0.1;

        cav_msgs::PredictedState pobj = predictState(pose,twist,delta_t);
                 
        EXPECT_NEAR(1.75, pobj.predicted_position.position.x ,0.00001);
        EXPECT_NEAR(1.6,  pobj.predicted_position.position.y,0.00001);
        EXPECT_NEAR(2.5,  pobj.predicted_position.position.z,0.00001);

        EXPECT_NEAR(4.5,pobj.predicted_velocity.linear.x ,0.00001);
        EXPECT_NEAR(2, pobj.predicted_velocity.linear.y,0.00001);
        EXPECT_NEAR(5,  pobj.predicted_velocity.linear.z,0.00001);
    }
   
    TEST(MotionPredictTest, testexternalPredictLowCovariancePositionHighCovarianceVelocity)
    {
          
        double delta_t=0.1;
        double ax=9;
        double ay=9;
        double process_noise_max=1000;

        cav_msgs::ExternalObject obj;

        obj.pose.covariance[0]=1; // X
  		obj.pose.covariance[7]=1; // Y
        obj.velocity.covariance[0]=999; // Vx
        obj.velocity.covariance[7]=999; // Vy

       cav_msgs::PredictedState pobj=externalPredict(obj,delta_t,ax,ay,process_noise_max);        
                 
        EXPECT_NEAR(0.0100002, pobj.predicted_position_confidence ,0.00001);
        EXPECT_NEAR(0.999089, pobj.predicted_velocity_confidence,0.00001);

    }


    TEST(MotionPredictTest, testexternalPredictHighCovariancePositionLowCovarianceVelocity)
    {
        double delta_t=0.1;
        double ax=9;
        double ay=9;
        double process_noise_max=1000;

        cav_msgs::ExternalObject obj;

        obj.pose.covariance[0]=999; // X
  		obj.pose.covariance[7]=999; // Y
        obj.velocity.covariance[0]=1; // Vx
        obj.velocity.covariance[7]=1; // Vy

        cav_msgs::PredictedState pobj=externalPredict(obj,delta_t,ax,ay,process_noise_max);        
                 
        EXPECT_NEAR(0.999009, pobj.predicted_position_confidence ,0.00001);
        EXPECT_NEAR(9.00901e-005, pobj.predicted_velocity_confidence,0.00001);

    }

    TEST(MotionPredictTest, testexternalPredictDeltaT1)
    {
        double delta_t=1;
        double ax=9;
        double ay=9;
        double process_noise_max=1000;

        cav_msgs::ExternalObject obj;

        obj.pose.covariance[0]=1; // X
  		obj.pose.covariance[7]=1; // Y
        obj.velocity.covariance[0]=999; // Vx
        obj.velocity.covariance[7]=999; // Vy

        cav_msgs::PredictedState pobj=externalPredict(obj,delta_t,ax,ay,process_noise_max);        
                 
        EXPECT_NEAR(1.00225, pobj.predicted_position_confidence ,0.00001);
        EXPECT_NEAR(1.00801, pobj.predicted_velocity_confidence,0.00001);

    }


    TEST(MotionPredictTest, testpredictStep)
    {
        
        double delta_t=0.1;
        double confidence_drop_rate=0.1;

        cav_msgs::PredictedState pobj;

        pobj.predicted_position.position.x=1.3; // Predicted Position X
  		pobj.predicted_position.position.y=1.4; // Predicted Position Y
        pobj.predicted_position.position.z=2.5; // Predicted Position Z

        pobj.predicted_velocity.linear.x=4.5; // Predicted Linear Velocity X
        pobj.predicted_velocity.linear.y=2; // Predicted Linear Velocity Y
        pobj.predicted_velocity.linear.z=5; // Predicted Linear Velocity Z

        pobj.predicted_position_confidence=0.0100002; // Position process noise confidence
        pobj.predicted_velocity_confidence=0.999089; // Velocity process noise confidence

        pobj=predictStep(pobj,delta_t,confidence_drop_rate);   

        EXPECT_NEAR(1.75, pobj.predicted_position.position.x ,0.00001);
        EXPECT_NEAR(1.6,  pobj.predicted_position.position.y,0.0001);
        EXPECT_NEAR(2.5,  pobj.predicted_position.position.z,0.0001);

        EXPECT_NEAR(4.5,pobj.predicted_velocity.linear.x ,0.0001);
        EXPECT_NEAR(2, pobj.predicted_velocity.linear.y,0.0001);
        EXPECT_NEAR(5,  pobj.predicted_velocity.linear.z,0.0001);     
            
        EXPECT_NEAR(0.00100002, pobj.predicted_position_confidence ,0.0001);
        EXPECT_NEAR(0.0999089, pobj.predicted_velocity_confidence,0.0001);
    }

    TEST(MotionPredictTest, testpredictPeriod)
    {
        double delta_t=0.1;
        double ax=9;
        double ay=9;
        double process_noise_max=1000;
        double confidence_drop_rate=0.1;
        double period=1.0;

        cav_msgs::ExternalObject obj;

        obj.pose.pose.position.x=1.3;
        obj.pose.pose.position.y=1.4;
        obj.pose.pose.position.z=2.5;
        obj.velocity.twist.linear.x=4.5;
        obj.velocity.twist.linear.y=2;
        obj.velocity.twist.linear.z=5;

        obj.pose.covariance[0]=1; // X
        obj.pose.covariance[7]=1; // Y
        obj.velocity.covariance[0]=999; // Vx
        obj.velocity.covariance[7]=999; // Vy
 
        std::vector<cav_msgs::PredictedState> plist=predictPeriod(obj,delta_t,period,ax,ay ,process_noise_max,confidence_drop_rate);        
                 
        EXPECT_NEAR(1.75, plist[0].predicted_position.position.x ,0.00001);
        EXPECT_NEAR(1.6,  plist[0].predicted_position.position.y,0.0001);
        EXPECT_NEAR(2.5,  plist[0].predicted_position.position.z,0.0001);

        EXPECT_NEAR(4.5,plist[0].predicted_velocity.linear.x ,0.0001);
        EXPECT_NEAR(2, plist[0].predicted_velocity.linear.y,0.0001);
        EXPECT_NEAR(5,  plist[0].predicted_velocity.linear.z,0.0001);     
            
        EXPECT_NEAR(0.0100002, plist[0].predicted_position_confidence ,0.0001);
        EXPECT_NEAR(0.999089, plist[0].predicted_velocity_confidence,0.0001);

        EXPECT_NEAR(2.2, plist[1].predicted_position.position.x ,0.00001);
        EXPECT_NEAR(1.8,  plist[1].predicted_position.position.y,0.0001);
        EXPECT_NEAR(2.5,  plist[1].predicted_position.position.z,0.0001);

        EXPECT_NEAR(4.5,plist[1].predicted_velocity.linear.x ,0.0001);
        EXPECT_NEAR(2, plist[1].predicted_velocity.linear.y,0.0001);
        EXPECT_NEAR(5,  plist[1].predicted_velocity.linear.z,0.0001);     
            
        EXPECT_NEAR(0.00100002, plist[1].predicted_position_confidence ,0.0001);
        EXPECT_NEAR(0.0999089, plist[1].predicted_velocity_confidence,0.0001);
    }

}//cv

}//motion_predict
