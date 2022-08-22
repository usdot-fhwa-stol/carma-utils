#ifndef ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_INTERFACE_HPP_
#define ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_INTERFACE_HPP_

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

#include <memory>
#include <string>
#include <chrono>

namespace ros2_lifecycle_manager
{

  using std_nanosec = std::chrono::nanoseconds;

  /**
   * \brief This interface defines a C++ API to be used for lifecycle management which will manage rclcpp::LifecycleNode(s)
   *  
   * The user should use set_managed_nodes to provide a list of nodes to manage.
   * The user can then walk the nodes through their lifecycle using the provided 
  *  configure, cleanup, activate, deactivate, and shutdown methods. 
   * 
   */
  class LifecycleManagerInterface
  {
  public:
    /**
     * @brief Virtual destructor to ensure delete safety for pointers to implementing classes
     *
     */
    virtual ~LifecycleManagerInterface(){};

    /**
     * \brief Set the nodes which will be managed by the implementing object. 
     * 
     * \param nodes The list of Fully Qualified Names for nodes to manage.
     *              The order of these nodes will be obeyed in the transition methods if the ordered argument is used.
     */
    virtual void set_managed_nodes(const std::vector<std::string> &nodes) = 0;

    /**
     * \brief Add a single node to the set of managed nodes
     * 
     * \param node The node to add to the set of managed nodes. 
     *             If node is already present, then nothing happens 
     */ 
    virtual void add_managed_node(const std::string& node) = 0;

    /**
     * \brief Returns the list of managed node's Fully Qualified Names
     */
    virtual std::vector<std::string> get_managed_nodes() = 0;

    /**
     * \brief Returns the Lifecycle state of the provided node
     *      
     * \param call_timeout The length of time in nanoseconds to wait for successful transition execution for EACH node. Default 50ms
     * 
     * NOTE: This call expects the service to already be available
     */
    virtual uint8_t get_managed_node_state(const std::string &node, const std_nanosec &call_timeout=std_nanosec(50000000L)) = 0;


    /**
     * \brief Attempts to move the specified managed node to the specified state
     * 
     * \param state The lifecycle state to move to. As defined in the lifecycle_msgs::State message primary states enum
     * \param node The managed node to transition.  
     * \param connection_timeout The length of time in nanoseconds to wait for connection to be established with EACH node. 
     * \param call_timeout The length of time in nanoseconds to wait for successful transition execution for EACH node. 
     * 
     * \return The resulting state after the attempt
     */ 
    virtual uint8_t transition_node_to_state(const uint8_t state, const std::string& node, const std_nanosec &connection_timeout, const std_nanosec &call_timeout) = 0;

    /**
     * \brief Send the Configure transition to all managed nodes. Returns true if all nodes transitioned successfully.
     * 
     * \param connection_timeout The length of time in nanoseconds to wait for connection to be established with EACH node. 
     * \param call_timeout The length of time in nanoseconds to wait for successful transition execution for EACH node. 
     * \param ordered If true then the nodes will be transitioned in order of the list provided by set_managed_nodes. 
     *                If false the nodes will all be triggered at once.
     * 
     * \param nodes The specific nodes to transition. If empty then all managed nodes will be transitioned
     * 
     * \return A list of any nodes that failed to execute the requested transition
     */
    virtual std::vector<std::string> configure(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true, std::vector<std::string> nodes = {}) = 0;

    //! \brief Same as configure() except for Cleanup transition
    virtual std::vector<std::string> cleanup(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true, std::vector<std::string> nodes = {}) = 0;

    //! \brief Same as configure() except for Activate transition
    virtual std::vector<std::string> activate(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true, std::vector<std::string> nodes = {}) = 0;

    //! \brief Same as configure() except for Deactivate transition
    virtual std::vector<std::string> deactivate(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true, std::vector<std::string> nodes = {}) = 0;

    //! \brief Same as configure() except for Shutdown transition
    // NOTE: The set of returned nodes may not be as meaningful as the other methods due to the nature of the shutdown transition.
    virtual std::vector<std::string> shutdown(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true, std::vector<std::string> nodes = {}) = 0;
  };

} // namespace ros2_lifecycle_manager

#endif // ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_INTERFACE_HPP_
