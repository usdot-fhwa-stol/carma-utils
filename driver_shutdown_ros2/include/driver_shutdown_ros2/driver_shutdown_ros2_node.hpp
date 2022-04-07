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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <carma_msgs/msg/system_alert.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace driver_shutdown_ros2
{

  /**
   * \brief Simple node which shuts itself down on receipt of a SystemAlert::SHUTDOWN message.
   *        It is used to exit launch files from disperate containers.  
   * 
   */
  class Node : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<carma_msgs::msg::SystemAlert> alert_sub_;

  public:
    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);

    /**
     * \brief System alert callback to exit node if shutdown is received. 
     */
    void alert_callback(carma_msgs::msg::SystemAlert::UniquePtr msg);


    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

  };

} // driver_shutdown_ros2
