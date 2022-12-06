// Copyright 2016 Open Source Robotics Foundation, Inc.
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

/**
 * @file publisher_member_function.h
 * @author Sharmitha Ganesan (sganesa3@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"
#include "string_srv/srv/change.hpp"
#include <chrono> // NOLINT
#include <memory> // NOLINT
#include <string> // NOLINT

/**
 * @brief class MinimalPublisher
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   */
  MinimalPublisher();
  void set_frame(geometry_msgs::msg::TransformStamped);
  geometry_msgs::msg::TransformStamped  get_frame();

 private:
  /** 
   * @brief timer callback to publish msg and logging levels
   *
   */
  void timer_callback();
  /**
   * @brief SERVICE CALLBACK
   *
   * @param request
   * @param response
   */
  void get_count_callback(
      const std::shared_ptr<string_srv::srv::Change::Request>,
      std::shared_ptr<string_srv::srv::Change::Response>);
 
  /**
   * @brief TF BROADCAST
   * 
   */
  void make_transforms();
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std_msgs::msg::String message;
  int f_num;
  std::string string_var;
  geometry_msgs::msg::TransformStamped child;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<string_srv::srv::Change>::SharedPtr string_service_;
  size_t count_;
};
