/*
*****************************************************************************
 * MIT License
Copyright (c) 2022 Sharmitha Ganesan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
* *******************************************************************************
*/
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
#pragma once
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
