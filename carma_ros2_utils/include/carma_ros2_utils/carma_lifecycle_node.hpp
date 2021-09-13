// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CARMA_ROS2_UTILS__CARMA_LIFECYCLE_NODE_HPP_
#define CARMA_ROS2_UTILS__CARMA_LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "carma_ros2_utils/visibility_control.hpp"
#include "cav_msgs/msg/system_alert.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/node_thread.hpp"

namespace carma_ros2_utils
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CarmaLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  CARMA_ROS2_UTILS_PUBLIC
  explicit CarmaLifecycleNode(const rclcpp::NodeOptions & options);
  virtual ~CarmaLifecycleNode();

  carma_ros2_utils::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override;
  carma_ros2_utils::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override;
  carma_ros2_utils::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override;
  carma_ros2_utils::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override;
  carma_ros2_utils::CallbackReturn on_error(const rclcpp_lifecycle::State & /*state*/) override;

  std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> shared_from_this();

  void publish_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);
  virtual void on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);




  // Create and return a Subscription.
  /**
   * \param[in] topic_name The topic to subscribe on.
   * \param[in] callback The user-defined callback function.
   * \param[in] qos The quality of service for this subscription. NOTE: There is a conversion constructor for size_t allowing you to specify the history depth only. This is the analog to queue size in ros1
   * \param[in] options The subscription options for this subscription.
   * \param[in] msg_mem_strat The message memory strategy to use for allocating messages.
   * \return Shared pointer to the created subscription.
   */
  template<
    typename MessageT,
    typename CallbackT,
    typename AllocatorT = std::allocator<void>,
    typename CallbackMessageT =
    typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
    typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
    typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
      CallbackMessageT,
      AllocatorT
    >>
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback, // temporary variable (can be lambda or std::bind)
    const SubscriptionOptionsWithAllocator<AllocatorT> & options =
    create_default_subscription_options<AllocatorT>(),
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (
      MessageMemoryStrategyT::create_default()
    )
  ) {

    return rclcpp_lifecycle::LifecycleNode::create_subscription<MessageT>(topic_name, qos, 
    [&callback, this] (std::unique_ptr<MessageT> m) { 
      try {
        callback(std::move(m)); 
      } catch (const std::exception& e) {
        // TODO add exception handling here
      }
      
    },
    options, msg_mem_strat);

  }

  /**
   * TODO still need to override
   * /// Add a callback for when parameters are being set.
  /**
   * \sa rclcpp::Node::add_on_set_parameters_callback
  //  */
  // RCLCPP_LIFECYCLE_PUBLIC
  // RCUTILS_WARN_UNUSED
  // rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr
  // add_on_set_parameters_callback(
  //   rclcpp_lifecycle::LifecycleNode::OnParametersSetCallbackType callback);
  // TODO also override the deprecated method set_on_parameters_set_callback
   //* 
  // */ 

  /// Create a timer.
  /**
   * \param[in] period Time interval between triggers of the callback.
   * \param[in] callback User-defined callback function.
   * \param[in] group Callback group to execute this timer's callback in.
   */
  template<typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
  std::shared_ptr<rclcpp::TimerBase> // NOTE: TimerBase must be used here to account for the fact that the exception handling lambda will have a different type from the input callback type due to being a possible differnet location in code (member vs non-member method etc.).
      // Therefore the old return statement of typename rclcpp::WallTimer<CallbackT>::SharedPtr is replaced with TimerBase
  create_wall_timer(
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr) {

      // Local copy of the callback has to be made here because the method is not written to take an rvalue reference
      auto callack_func = [&callback, this] () -> void {
        try {
          callback();
        } catch(const std::exception& e) {
          // TODO handle exception
        }
      };

      return rclcpp_lifecycle::LifecycleNode::create_wall_timer(period, callack_func, group);
    }

    template<typename CallbackT> // NOTE: In foxy the LifecycleNode api is slightly out of sync with the node api so there is not a create_timer method there. We use rclcpp directly here
typename rclcpp::TimerBase::SharedPtr
create_timer(
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Duration period,
  CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return rclcpp::create_timer(
  this,
  clock,
  period,
  [&callback, this] () -> void {
        try {
          callback();
        } catch(const std::exception& e) {
          // TODO handle exception
        }
      },
  group);
}

// TODO document in this file that transition ids can be found here: https://github.com/ros2/rcl_interfaces/blob/foxy/lifecycle_msgs/msg/Transition.msg


  /// Create and return a Service.
  /**
   * \sa rclcpp::Node::create_service
   */
  template<typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::CallbackGroup::SharedPtr group = nullptr) {
      return rclcpp_lifecycle::LifecycleNode::create_service<ServiceT>(service_name, [&callback] (
        std::shared_ptr<rmw_request_id_t> header,
        std::shared_ptr<typename ServiceT::Request> req, 
        std::shared_ptr<typename ServiceT::Response> resp) { 
          try {
            callback(header, req, resp); 
          } catch(const std::exception& e) {
            // TODO handle exception
          }
        }, qos_profile, group);
    }

protected:
  rclcpp::Node::SharedPtr rclcpp_node_;
  std::unique_ptr<ros2_utils::NodeThread> rclcpp_thread_;
  void create_rclcpp_node(const rclcpp::NodeOptions & options);

  const std::string system_alert_topic_{"/system_alert"};
  rclcpp::Subscription<cav_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<cav_msgs::msg::SystemAlert>>
  system_alert_pub_;
};

}  // namespace carma_ros2_utils

#endif  // CARMA_ROS2_UTILS__CARMA_LIFECYCLE_NODE_HPP_"
