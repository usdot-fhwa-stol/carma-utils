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

#include "ros2_lifecycle_manager/ros2_lifecycle_manager.hpp"

namespace ros2_lifecycle_manager
{

  Ros2LifecycleManager::Ros2LifecycleManager(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
      rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services)
      : node_base_(node_base), node_graph_(node_graph), node_logging_(node_logging), node_services_(node_services)
  {
    service_callback_group_ = node_base_->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
  }

  void Ros2LifecycleManager::set_managed_nodes(const std::vector<std::string> &nodes)
  {

    // Recreate and store ManagedNodes instances
    managed_node_names_.clear();
    managed_node_names_.reserve(nodes.size());
    managed_nodes_.clear();
    managed_nodes_.reserve(nodes.size());
    node_map_.clear();
    node_map_.reserve(nodes.size());

    for (const auto &node : nodes)
    {
      add_managed_node(node);
    }
  }

  void Ros2LifecycleManager::add_managed_node(const std::string& node)
  {
    // If this node was already added then return
    if (node_map_.find(node) != node_map_.end())
    {
      return;
    }

    node_map_.emplace(node, managed_node_names_.size())
    managed_node_names_.push_back(node); // Store node names

    // Create a new ManagedNode instance
    RCLCPP_INFO_STREAM(node_logging_->get_logger(), "Creating Managed Node with topics: " << (node + change_state_topic_) << " and " << (node + get_state_topic_));
    ManagedNode managed_node(node,
                              create_client<lifecycle_msgs::srv::ChangeState>(node + change_state_topic_),
                              create_client<lifecycle_msgs::srv::GetState>(node + get_state_topic_));

    managed_nodes_.push_back(managed_node);
  }

  std::vector<std::string> Ros2LifecycleManager::get_managed_nodes()
  {
    return managed_node_names_;
  }

  uint8_t Ros2LifecycleManager::get_managed_node_state(const std::string &node_name)
  {
    auto it = node_map_.find(node_name);
    if (it == node_map_.end()) // Check if the requested node is being managed
    {

      RCLCPP_ERROR_STREAM(
          node_logging_->get_logger(), "State for node: " << node_name << " could not be provided as that node was not being managed. ");

      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto node = managed_nodes_.at((*it).second);

    // Check for service. If not read return unknown
    if (!node.get_state_client->service_is_ready())
    {
      RCLCPP_ERROR_STREAM(
          node_logging_->get_logger(), "State for node: " << node_name << " could not be provided as that node's service is not ready ");
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    // Send request
    auto future_result = node.get_state_client->async_send_request(request);

    auto future_status = future_result.wait_for(std_nanosec(10000000L)); // 10 millisecond delay

    if (future_status != std::future_status::ready)
    {
      RCLCPP_ERROR_STREAM(
          node_logging_->get_logger(), "Server time out while getting current state for node with name: " << node_name);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    return future_result.get()->current_state.id;
  }

  std::vector<std::string> Ros2LifecycleManager::configure(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered, std::vector<std::string> nodes)
  {
    return transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, ordered, connection_timeout, call_timeout, nodes);
  }

  std::vector<std::string> Ros2LifecycleManager::cleanup(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered, std::vector<std::string> nodes)
  {
    return transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, ordered, connection_timeout, call_timeout, nodes, std::vector<std::string> nodes);
  }

  std::vector<std::string> Ros2LifecycleManager::activate(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered, std::vector<std::string> nodes)
  {
    return transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, ordered, connection_timeout, call_timeout, nodes);
  }

  std::vector<std::string> Ros2LifecycleManager::deactivate(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered, std::vector<std::string> nodes)
  {
    return transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, ordered, connection_timeout, call_timeout, nodes);
  }

  std::vector<std::string> Ros2LifecycleManager::shutdown(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered, std::vector<std::string> nodes)
  {
    // To avoid having to track all the node sates and pick the appropriate shutdown we will simply shut down in order of most dangerous
    // Active is shutdown first to avoid more data being published, this is followed by inactive and finally unconfigured    
    auto failed_nodes = transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN, ordered, connection_timeout, call_timeout, nodes);
    transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN, ordered, connection_timeout, call_timeout, nodes);
    transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, ordered, connection_timeout, call_timeout, nodes);
    
    return failed_nodes; 
  }

  std::vector<std::string> Ros2LifecycleManager::transition_multiplex(uint8_t transition, bool ordered, const std_nanosec &connection_timeout, const std_nanosec &call_timeout, std::vector<std::string> nodes)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

    lifecycle_msgs::msg::Transition t_msg;
    t_msg.id = transition;

    request->transition = t_msg;

    std::vector<ManagedNode> nodes_to_transition;
    if (nodes.empty())
    {
      // If empty then transition all nodes
      nodes_to_transition = managed_nodes_;
    } 
    else
    {
      nodes_to_transition.reserve(nodes.size());

      // Extract the Managed nodes which need to be triggered
      for (const auto& node_name : nodes)
      {
        nodes_to_transition.emplace_back( managed_nodes_[node_map_[node_name]] );
      }

    }

    std::vector<std::string> failed_nodes; // Store failed nodes

    // If ordered argument was true then we will transition the nodes in sequence
    if (ordered)
    {
      // Iterate over each node and transition them
      for (auto node : nodes_to_transition)
      {

        // Wait for service
        if (!waitForService<lifecycle_msgs::srv::ChangeState>(node.change_state_client, connection_timeout))
        {
          failed_nodes.push_back(node.node_name);
          continue;
        }

        RCLCPP_INFO_STREAM(
            node_logging_->get_logger(), "Calling node: " << node.node_name);

        // Call service
        ChangeStateSharedFutureWithRequest future_result = node.change_state_client->async_send_request(request, [](ChangeStateSharedFutureWithRequest) {});

        // Wait for response
        if (!wait_on_change_state_future(future_result, call_timeout))
        {
          failed_nodes.push_back(node.node_name);
        }
      }
    }
    else // If ordered was not true then we shall call all the nodes first and then wait for their responses
    {
      std::vector<ChangeStateSharedFutureWithRequest> futures;
      futures.reserve(nodes_to_transition.size());
      std::unordered_map<size_t, std::string> future_node_map; // Map of future index to node name for error tracking

      for (auto node : nodes_to_transition)
      {
        // Wait for service
        if (!waitForService<lifecycle_msgs::srv::ChangeState>(node.change_state_client, connection_timeout))
        {
          failed_nodes.push_back(node.node_name);
          continue;
        }

        RCLCPP_INFO_STREAM(
            node_logging_->get_logger(), "Calling node a-sync: " << node.node_name);

        // Call service and record future
        futures.emplace_back(node.change_state_client->async_send_request(request, [](ChangeStateSharedFutureWithRequest) {}));
        future_node_map.emplace(futures.size() - 1, node.node_name);
      }
      size_t i = 0;
      for (auto future : futures)
      {
        RCLCPP_INFO_STREAM(node_logging_->get_logger(), "Waiting on future for node: " << future_node_map.at(i));
        
        if (!wait_on_change_state_future(future, call_timeout))
        {
          failed_nodes.push_back(future_node_map.at(i));
        }
        i++;
      }
    }

    return failed_nodes;
  }

  bool Ros2LifecycleManager::wait_on_change_state_future(const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFutureWithRequest &future,
                                                         const std_nanosec &timeout)
  {
    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    RCLCPP_INFO(
        node_logging_->get_logger(), "Waiting for future");
      
    auto future_status = future.wait_for(timeout);

    if (future_status != std::future_status::ready)
    {
      RCLCPP_ERROR(
          node_logging_->get_logger(), "Server time out while getting current state for node");
      return false;
    }

    // We have an answer, let's print our success.
    if (future.get().second->success)
    {
      RCLCPP_INFO(
          node_logging_->get_logger(), "Transition %d successfully triggered.", static_cast<int>(future.get().first->transition.id));
      return true;
    }
    else
    {
      RCLCPP_WARN(
          node_logging_->get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(future.get().first->transition.id));
      return false;
    }

    return true;
  }

} // namespace ros2_lifecycle_manager
