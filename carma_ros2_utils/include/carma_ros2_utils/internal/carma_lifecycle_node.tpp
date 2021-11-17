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

#ifdef CARMA_ROS2_UTILS__CARMA_LIFECYCLE_NODE_HPP_

namespace carma_ros2_utils
{

  template <
      typename MessageT,
      typename CallbackT,
      typename AllocatorT = std::allocator<void>,
      typename CallbackMessageT =
          typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
      typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
      typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
          CallbackMessageT,
          AllocatorT>>
  std::shared_ptr<SubscriptionT>
  CarmaLifecycleNode::create_subscription(
      const std::string &topic_name,
      const rclcpp::QoS &qos,
      CallbackT &&callback, // temporary variable (can be lambda or std::bind)
      const rclcpp_lifecycle::SubscriptionOptionsWithAllocator<AllocatorT> &options,
      typename MessageMemoryStrategyT::SharedPtr msg_mem_strat)
  {

    return rclcpp_lifecycle::LifecycleNode::create_subscription<MessageT>(
        topic_name, qos,
        // The move capture "callback = std::move(callback)" is specifically needed
        // here because of the unique_ptr<> in the callback arguments 
        // when the callback is provided with std::bind
        // the returned functor degrades to be move-constructable not copy-constructable
        // https://www.cplusplus.com/reference/functional/bind/
        // using &callback instead can result in a non-deterministic seg fault
        [callback = std::move(callback), this](std::unique_ptr<MessageT> m)
        {
          try
          {
            callback(std::move(m));
          }
          catch (const std::exception &e)
          {
            handle_primary_state_exception(e);
          }
        },
        options, msg_mem_strat);
  }

  template <typename MessageT, typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
  CarmaLifecycleNode::create_publisher(
      const std::string &topic_name,
      const rclcpp::QoS &qos,
      const rclcpp_lifecycle::PublisherOptionsWithAllocator<AllocatorT> &options)
  {
    auto pub = rclcpp_lifecycle::LifecycleNode::create_publisher<MessageT>(topic_name, qos, options);
    lifecycle_publishers_.push_back(pub);
    return pub;
  }

  template <typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
  std::shared_ptr<rclcpp::TimerBase>
  CarmaLifecycleNode::create_wall_timer(
      std::chrono::duration<DurationRepT, DurationT> period,
      CallbackT callback,
      rclcpp::CallbackGroup::SharedPtr group)
  {

    // Local copy of the callback has to be made here because the method is not written to take an rvalue reference
    auto callack_func = [callback = std::move(callback), this]() -> void
    {
      try
      {
        callback();
      }
      catch (const std::exception &e)
      {
        handle_primary_state_exception(e);
      }
    };

    auto timer = rclcpp_lifecycle::LifecycleNode::create_wall_timer(period, callack_func, group);
    timers_.push_back(timer);
    return timer;
  }

  template <typename CallbackT>
  typename rclcpp::TimerBase::SharedPtr
  CarmaLifecycleNode::create_timer(
      rclcpp::Clock::SharedPtr clock,
      rclcpp::Duration period,
      CallbackT &&callback,
      rclcpp::CallbackGroup::SharedPtr group)
  {
    auto timer = rclcpp::create_timer(
        this,
        clock,
        period,
        [callback = std::move(callback), this]() -> void
        {
          try
          {
            callback();
          }
          catch (const std::exception &e)
          {
            handle_primary_state_exception(e);
          }
        },
        group);

    timers_.push_back(timer);
    return timer;
  }

  template <typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr
  CarmaLifecycleNode::create_service(
      const std::string &service_name,
      CallbackT &&callback,
      const rmw_qos_profile_t &qos_profile,
      rclcpp::CallbackGroup::SharedPtr group)
  {
    return rclcpp_lifecycle::LifecycleNode::create_service<ServiceT>(
        service_name, [callback = std::move(callback), this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<typename ServiceT::Request> req, std::shared_ptr<typename ServiceT::Response> resp)
        {
          try
          {
            callback(header, req, resp);
          }
          catch (const std::exception &e)
          {
            handle_primary_state_exception(e);
          }
        },
        qos_profile, group);
  }

  template <class ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr
  CarmaLifecycleNode::create_client (
    const std::string service_name, 
    const rmw_qos_profile_t & qos_profile,
    rclcpp::CallbackGroup::SharedPtr group
  )
  {
    // nullptr is the default argument for group
    // when nullptr is provided use the class level service group
    // this is needed instead of a default argument because you cannot change default arguments in overrides
    if (group == nullptr) { 
      group = this->service_callback_group_;
    }

    return rclcpp_lifecycle::LifecycleNode::create_client<ServiceT>(
        service_name,
        qos_profile,
        group); // Our override specifies a different default callback group
  }

  template<typename T>
  boost::optional<std::string> CarmaLifecycleNode::update_params(const std::unordered_map<std::string, std::reference_wrapper<T>>& update_targets,
                   const std::vector< rclcpp::Parameter > & new_params) {

    for (auto param : new_params) {

      if (update_targets.find(param.get_name()) == update_targets.end()) {
        // We are not interested in this parameter
        continue;
      }

      auto& target = param.at(param.get_name());

      switch (param.get_type())
      {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          if (std::is_same_v<T, bool>) {
            target = param.get_value();
          }
          continue;
        
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          if (std::is_same_v<T, int>) {
            target = param.get_value();
          }
          continue;

        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          if (std::is_same_v<T, double>) {
            target = param.get_value();
          }
          continue;

        case rclcpp::ParameterType::PARAMETER_STRING:
          if (std::is_same_v<T, std::string>) {
            target = param.get_value();
          }
          continue;

        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
          if (std::is_same_v<T, std::vector<uint8_t>>) {
            target = param.get_value();
          }
          continue;

        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
          if (std::is_same_v<T, std::vector<bool>>) {
            target = param.get_value();
          }
          continue;

        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
          if (std::is_same_v<T, std::vector<int>>) {
            target = param.get_value();
          }
          continue;

        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
          if (std::is_same_v<T, std::vector<std::string>>) {
            target = param.get_value();
          }
          continue;
      
      default:
        break;
      }
      
      std::string error = "Cannot update parameter " + param.get_name() + " it has mismatched type " + typeid(T).name() + " and " + param.get_type_name()
      RCLCPP_ERROR_STREAM(get_logger(), error);
      
      return string;
    }

    return boost::none;

  }

} // namespace carma_ros2_utils

#endif // CARMA_ROS2_UTILS__CARMA_LIFECYCLE_NODE_HPP_"
