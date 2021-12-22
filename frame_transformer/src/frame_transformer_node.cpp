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
#include "frame_transformer/frame_transformer_node.hpp"
#include <boost/algorithm/string/trim.hpp>

namespace frame_transformer
{
  namespace std_ph = std::placeholders;

  Node::Node(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
  }

  std::unique_ptr<TransformerBase> Node::build_transformer() {
    
    // NOTE: IMU messages could be added once imu_pipeline package is ported to noetic and added as a dependency

    auto type = config_.message_type;
    boost::algorithm::trim(type); // Trim white space from the type string

    if (type == "geometry_msgs/Vector3Stamped")
      return std::make_unique<Transformer<geometry_msgs::msg::Vector3Stamped>>(config_.target_frame, buffer_, shared_from_this(), config_.queue_size);
    
    else if (type == "geometry_msgs/PointStamped")
      return std::make_unique<Transformer<geometry_msgs::msg::PointStamped>>(config_.target_frame, buffer_, shared_from_this(), config_.queue_size);
    
    else if (type == "geometry_msgs/PoseStamped")
      return std::make_unique<Transformer<geometry_msgs::msg::PoseStamped>>(config_.target_frame, buffer_, shared_from_this(), config_.queue_size);
    
    else if (type == "geometry_msgs/PoseWithCovarianceStamped")
      return std::make_unique<Transformer<geometry_msgs::msg::PoseWithCovarianceStamped>>(config_.target_frame, buffer_, shared_from_this(), config_.queue_size);
    
    else if (type == "geometry_msgs/QuaternionStamped")
      return std::make_unique<Transformer<geometry_msgs::msg::QuaternionStamped>>(config_.target_frame, buffer_, shared_from_this(), config_.queue_size);
    
    else if (type == "geometry_msgs/TransformStamped")
      return std::make_unique<Transformer<geometry_msgs::msg::TransformStamped>>(config_.target_frame, buffer_, shared_from_this(), config_.queue_size);

    else if (type == "sensor_msgs/PointCloud2")
      return std::make_unique<Transformer<sensor_msgs::msg::PointCloud2>>(config_.target_frame, buffer_, shared_from_this(), config_.queue_size);

    else
      return nullptr

  }

  carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    // Reset config
    config_ = Config();

    // Load parameters
    config_.message_type = declare_parameter<std::string>("message_type", config_.message_type);
    config_.target_frame = declare_parameter<std::string>("target_frame", config_.target_frame);
    config_.queue_size = declare_parameter<int>("queue_size", config_.queue_size);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded params: " << config_);

    // NOTE: Due to the nature of this node generating sub/pubs based of parameters. 
    //       Dynamically changing the parameters is not recommended. Therefore a parameter callback is not implemented. 


    // Set the transformer based on the specified message type
    transformer_ = build_transformer();

    if (!transformer_) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to build transformer for specified message_type: " << config_.message_type);
      return CallbackReturn::FAILURE;
    }

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn Node::handle_on_cleanup(const rclcpp_lifecycle::State &) {

    transformer_.reset(nullptr); // On cleanup clear the old transformer

  }


} // frame_transformer

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(frame_transformer::Node)
