/*
 * Copyright (C) 2024 LEIDOS.
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

#ifndef ROS2_LIFECYCLE_MANAGER__ROS2_LIFECYCLE_UTILS_HPP_
#define ROS2_LIFECYCLE_MANAGER__ROS2_LIFECYCLE_UTILS_HPP_

#include <unordered_map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace ros2_lifecycle_manager
{
// Define a map for transition IDs to human-readable names
std::unordered_map<uint8_t, std::string> transition_map = {
    {lifecycle_msgs::msg::Transition::TRANSITION_CREATE, "TRANSITION_CREATE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "TRANSITION_CONFIGURE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, "TRANSITION_CLEANUP"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "TRANSITION_ACTIVATE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "TRANSITION_DEACTIVATE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, "TRANSITION_UNCONFIGURED_SHUTDOWN"},
    {lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN, "TRANSITION_INACTIVE_SHUTDOWN"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN, "TRANSITION_ACTIVE_SHUTDOWN"},
    {lifecycle_msgs::msg::Transition::TRANSITION_DESTROY, "TRANSITION_DESTROY"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_SUCCESS, "TRANSITION_ON_CONFIGURE_SUCCESS"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_FAILURE, "TRANSITION_ON_CONFIGURE_FAILURE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_ERROR, "TRANSITION_ON_CONFIGURE_ERROR"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_SUCCESS, "TRANSITION_ON_CLEANUP_SUCCESS"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_FAILURE, "TRANSITION_ON_CLEANUP_FAILURE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_ERROR, "TRANSITION_ON_CLEANUP_ERROR"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_SUCCESS, "TRANSITION_ON_ACTIVATE_SUCCESS"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_FAILURE, "TRANSITION_ON_ACTIVATE_FAILURE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_ERROR, "TRANSITION_ON_ACTIVATE_ERROR"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_SUCCESS, "TRANSITION_ON_DEACTIVATE_SUCCESS"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_FAILURE, "TRANSITION_ON_DEACTIVATE_FAILURE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_ERROR, "TRANSITION_ON_DEACTIVATE_ERROR"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_SUCCESS, "TRANSITION_ON_SHUTDOWN_SUCCESS"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_FAILURE, "TRANSITION_ON_SHUTDOWN_FAILURE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_ERROR, "TRANSITION_ON_SHUTDOWN_ERROR"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_SUCCESS, "TRANSITION_ON_ERROR_SUCCESS"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_FAILURE, "TRANSITION_ON_ERROR_FAILURE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_ERROR, "TRANSITION_ON_ERROR_ERROR"},
    {lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS, "TRANSITION_CALLBACK_SUCCESS"},
    {lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE, "TRANSITION_CALLBACK_FAILURE"},
    {lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR, "TRANSITION_CALLBACK_ERROR"}
};

// Function to get human-readable name from transition ID
std::string get_transition_name(uint8_t id)
{
    auto it = transition_map.find(id);
    if (it != transition_map.end())
    {
        return it->second;
    }
    return "UNKNOWN_TRANSITION";
}

} // namespace ros2_lifecycle_manager

#endif // ROS2_LIFECYCLE_MANAGER__ROS2_LIFECYCLE_UTILS_HPP_
