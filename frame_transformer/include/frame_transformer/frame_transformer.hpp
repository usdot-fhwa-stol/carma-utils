#pragma once

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

#include "frame_transformer_base.hpp"
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace frame_transformer
{

  /**
   * \brief Class template for data transformers which use the tf2_ros::Buffer.transform() method to perform transforms on ros messages.
   *        The class sets up publishers and subscribers using the provided node for the specified message type.
   *        The specified message type must have a tf2_ros::Buffer.doTransform specialization for its type in order for it to compile.
   * 
   * \tparam The message type to transform. Must be supported by tf2_ros::Buffer.doTransform
   */
  template <class T>
  class Transformer : public TransformerBase // TransformerBase allows for type polymorphism in the containing Node and constructor enforcement
  {

  protected:
    // Subscribers
    carma_ros2_utils::SubPtr<T> input_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<T> output_pub_;


  public:

    /**
     * \brief Constructor which sets up the required publishers and subscribers
     * 
     * See comments in TransformerBase for parameter descriptions
     */ 
    Transformer(const std::string &target_frame, std::shared_ptr<tf2_ros::Buffer> buffer, std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> node, size_t queue_size)
    : TransformerBase(target_frame, buffer, node, queue_size) {

      input_sub_ = node_->create_subscription<T>("input", queue_size_,
                                                  std::bind(&Transformer::input_callback, this, std_ph::_1));

      // Setup publishers
      output_pub_ = node_->create_publisher<T>("output", queue_size_);
    }


    /**
     * \brief Helper method which takes in an input message and transforms it to the provided frame.
     *        Returns false if the provided timeout is exceeded for getting the transform or the transform could not be computed
     * 
     * \param in The input message to transform
     * \param[out] out The preallocated location for the output message
     * \param target_frame The frame the out message data will be in
     * \param timeout A timeout which if exceeded will result in a false return and invalid out value. This call may block for this period.
     *                If set to zero, then lookup will be attempted only once. 
     * 
     * \return True if transform succeeded, false if timeout exceeded or transform could not be performed. 
     */ 
    bool transform(const T &in, T &out, const std::string &target_frame, const rclcpp::Duration timeout)
    {

      T out_msg;

      try
      {
        buffer_.transform(in_msg.get(), out_msg, target_frame, timeout);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE_STREAM(this->get_logger(), this->get_clock(), rclcpp::Duration(1.0), "Failed to get transform with exception: " << ex.what());
        return false;
      }

      return true;
    }

    /**
     * \brief Callback for input data. Transforms the data then republishes it
     * 
     * NOTE: This method can be specialized for unique preallocation approaches for large messages such as point clouds or images
     * 
     * \param in_msg The input message to transform
     */ 
    void input_callback(std::unique_ptr<T> in_msg)
    {
      T out_msg;

      if (!transform(in_msg, out_msg, target_frame_, timeout_))
      {
        return;
      }

      output_pub_.publish(out_msg);
    }
  };

  // Specialization of input_callback for PointCloud2 messages to allow for preallocation of points vector
  // This is done due to the large size of that data set
  template <>
  inline void Node<sensor_msgs::msg::PointCloud2>::input_callback(std::unique_ptr<sensor_msgs::msg::PointCloud2> in_msg) {

    sensor_msgs::msg::PointCloud2 out_msg;
    out.data.reserve(in->data.size()); // Preallocate points vector

    if (!transform(in_msg, out_msg, target_frame_, timeout_))
    {
      return;
    }

    output_pub_.publish(out_msg);
  }

}