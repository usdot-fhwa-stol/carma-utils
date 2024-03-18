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

#include <chrono>
#include <future>
#include <memory>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "ros2_lifecycle_manager/ros2_lifecycle_manager.hpp"

using std::chrono_literals::operator""ms;

/**
 * This test is meant to exercise the standard flow of the lifecycle manager.
 * To allow for callbacks to be processed the test makes use of a rather roundabout thread paradigm
 * Care should be taken to understand this code before trying to duplicate it in other test scenarios.
 * There may be better approaches such as the launch_testing framework.
 */
TEST(LifecycleManagerTest, BasicTest)
{
  // Test constructor
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_node"); // Node to connect to ROS network

  std::ignore = rcutils_logging_set_logger_level(
      node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  ros2_lifecycle_manager::Ros2LifecycleManager lifecycle_mgr_( // Construct lifecycle manager
      node->get_node_base_interface(),
      node->get_node_graph_interface(),
      node->get_node_logging_interface(),
      node->get_node_services_interface());

  // Test set_managed_nodes
  lifecycle_mgr_.set_managed_nodes({"test_lifecycle_node_1", "test_lifecycle_node_2"});

  std::this_thread::sleep_for(2000ms);

  // Test get_managed_nodes
  auto managed_nodes = lifecycle_mgr_.get_managed_nodes();

  RCLCPP_INFO(node->get_logger(), "Spinning");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto spin_future{std::async(std::launch::async, [&executor]
                              { executor.spin(); })};

  ASSERT_EQ((uint8_t)2, managed_nodes.size());
  ASSERT_EQ((uint8_t)0, managed_nodes[0].compare("test_lifecycle_node_1"));
  ASSERT_EQ((uint8_t)0, managed_nodes[1].compare("test_lifecycle_node_2"));

  // Test get managed node states
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, lifecycle_mgr_.get_managed_node_state("unknown_node"));

  // Test Configure
  ASSERT_TRUE(lifecycle_mgr_.configure(2000ms, 2000ms).empty());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

  // Test Activate
  ASSERT_TRUE(lifecycle_mgr_.activate(2000ms, 2000ms).empty());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

  // Test Deactivate
  ASSERT_TRUE(lifecycle_mgr_.deactivate(2000ms, 2000ms).empty());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

  // Test cleanup
  ASSERT_TRUE(lifecycle_mgr_.cleanup(2000ms, 2000ms).empty());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

  RCLCPP_INFO(node->get_logger(), "Using requested state methods");

  // Bring to active via requested state
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_mgr_.transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "test_lifecycle_node_1", 2000ms, 2000ms));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));

  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_mgr_.transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "test_lifecycle_node_2", 2000ms, 2000ms));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

  // Bring to unconfigured via requested state
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_mgr_.transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "test_lifecycle_node_1", 2000ms, 2000ms));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));

  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_mgr_.transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "test_lifecycle_node_2", 2000ms, 2000ms));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

  RCLCPP_INFO(node->get_logger(), "Done with requested state methods");

  // Bring back to active then shutdown via transitions
  ASSERT_TRUE(lifecycle_mgr_.configure(2000ms, 2000ms).empty());
  ASSERT_TRUE(lifecycle_mgr_.activate(2000ms, 2000ms).empty());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

  ASSERT_TRUE(lifecycle_mgr_.shutdown(2000ms, 2000ms).empty());
  ASSERT_TRUE(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED == lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1") || lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN == lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
  ASSERT_TRUE(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED == lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2") || lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN == lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

  executor.cancel();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool success = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return success;
}
