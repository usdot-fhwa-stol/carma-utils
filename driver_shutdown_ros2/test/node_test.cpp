/*
 * Copyright (C) 2022 LEIDOS.
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
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "driver_shutdown_ros2/driver_shutdown_ros2_node.hpp"


// Test shutdown behavior
TEST(Test_driver_shutdown_ros2, alert_test){

    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<driver_shutdown_ros2::Node>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    std::unique_ptr<carma_msgs::msg::SystemAlert> msg = std::make_unique<carma_msgs::msg::SystemAlert>();
    msg->type = carma_msgs::msg::SystemAlert::CAUTION;

    worker_node->alert_callback(move(msg)); // Check non-shutdown

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, node->get_current_state().id());

    msg = std::make_unique<carma_msgs::msg::SystemAlert>();
    msg->type = carma_msgs::msg::SystemAlert::SHUTDOWN;

    worker_node->alert_callback(move(msg)); // Check shutdown

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, node->get_current_state().id());

}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 