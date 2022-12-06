/******************************************************************************
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
 * @file publisher_member_function.cpp
 * @author Sharmitha Ganesan (sganesa3@umd.edu)
 * @brief basic publisher function with logging levels
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "cpp_pubsub/publisher_member_function.hpp"
using namespace std::placeholders;    // NOLINT
using namespace std::chrono_literals; // NOLINT

MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0) {
  if (rcutils_logging_set_logger_level(
          this->get_logger().get_name(),
          RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) == RCUTILS_RET_OK) {
    RCLCPP_INFO_STREAM(this->get_logger(), "DEBUG SET");
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "DEBUG FAILED");
  }

  this->declare_parameter("freq", 100);
  f_num = this->get_parameter("freq").get_parameter_value().get<int>();
  this->set_parameter(rclcpp::Parameter("freq", f_num));
  publisher_ = this->create_publisher<std_msgs::msg::String>("talker_bot", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(f_num),
      std::bind(&MinimalPublisher::timer_callback, this));
  // broadcast talk
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_timer_ = this->create_wall_timer(
      300ms, std::bind(&MinimalPublisher::make_transforms, this));
  // Create a service for modifying str
  std::string string_service_name = "service_topic";
  string_service_ = this->create_service<string_srv::srv::Change>(
      string_service_name,
      std::bind(&MinimalPublisher::get_count_callback, this, _1, _2));
}

/**
 * @brief timer callback to publish msg and logging levels
 *
 */
void MinimalPublisher::timer_callback() {
  if (count_ == 0) {
    string_var = " BEFORE SERVICING : I am talker  ";
    message.data = string_var + std::to_string(count_++);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Publishing : " << message.data.c_str());
  } else {
    if (count_ % 5 == 0) {
      if (count_ % 10 == 0) {
        message.data = string_var + std::to_string(count_++);
        RCLCPP_FATAL_STREAM(this->get_logger(),
                            "Publishing: " << message.data.c_str());
      } else if (count_ % 10 != 0) {
        message.data = string_var + std::to_string(count_++);
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Publishing: " << message.data.c_str());
      }
    } else {
      if (count_ % 2 != 0) {
        message.data = string_var + std::to_string(count_++);
        RCLCPP_DEBUG_STREAM(this->get_logger(),
                            "Publishing " << message.data.c_str());
      } else {
        message.data = string_var + std::to_string(count_++);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Publishing: " << message.data.c_str());
      }
    }
  }
  publisher_->publish(message);
}
/**
 * @brief SERVICE CALLBACK
 *
 * @param request
 * @param response
 */
void MinimalPublisher::get_count_callback(
    const std::shared_ptr<string_srv::srv::Change::Request> request,
    std::shared_ptr<string_srv::srv::Change::Response> response) {
  request->a = " AFTER SERVICING: I am NEW talker ";
  string_var = request->a;
  RCLCPP_WARN_STREAM(this->get_logger(),
                     "SERVICE CALLED AND BASE STRING CHANGED");
  response->b = "SERVICE DONE";
}

void MinimalPublisher::make_transforms() {
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "talk";
  t.transform.translation.x = 5;
  t.transform.translation.y = 5;
  t.transform.translation.z = 5;
  tf2::Quaternion q;
  q.setRPY(0.7, 0.5, 1);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  MinimalPublisher::set_frame(t);

  tf_broadcaster_->sendTransform(t);
}
void MinimalPublisher::set_frame(geometry_msgs::msg::TransformStamped t) {
  child = t;
}
geometry_msgs::msg::TransformStamped MinimalPublisher::get_frame() {
  return child;
}
