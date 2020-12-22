
/*------------------------------------------------------------------------------
* Copyright (C) 2018-2021 LEIDOS.
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

------------------------------------------------------------------------------*/

#ifndef SOCKETCAN_RECOVER_CTRL_H
#define SOCKETCAN_RECOVER_CTRL_H

#include <can_msgs/CanState.h>
#include <socketcan_interface/socketcan.h>
#include <ros/ros.h>

namespace socketcan_bridge
{

class SocketCANRecoverCtrl
{
public:

	/**
     * @brief Constructor for the recovery control class
     *
     * Initializes the publisher, and sets up a timer to periodically check the bus state
     */
    SocketCANRecoverCtrl(ros::NodeHandle* nh, ros::NodeHandle* nh_param, boost::shared_ptr<can::DriverInterface> driver);

    ~SocketCANRecoverCtrl()
    {
        timer_.stop();
    }

private:

	/**
 	* @brief Publishes the status of the bus
 	*/
    void publishStatus(const can::State & state);

    /**
     * @brief Recover the bus from an error state
     *
     * Calls the driver's recover() function
     */
    void recover();

    /**
     * @brief Checks the state of the bus, if !statie.isReady() then the
     * recover timer is started to fire in 5secs, otherwise we stop the timer
     */
    void stateCallback(const can::State & s);


    ros::Publisher state_pub_;
    boost::shared_ptr<can::DriverInterface> driver_;
    ros::WallTimer timer_;

    can::StateInterface::StateListener::Ptr state_listener_;

};

};  // namespace socketcan_bridge


#endif
