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
 * @file publisher_member_function.cpp
 * @author Sharmitha Ganesan (sganesa3@umd.edu)
 * @brief basic publisher function with logging levels
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "cpp_pubsub/publisher_member_function.h"
using namespace std::placeholders;    // NOLINT
using namespace std::chrono_literals; // NOLINT


MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0) {
  if (rcutils_logging_set_logger_level(
          this->get_logger().get_name(),
          RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) ==
      RCUTILS_RET_OK) {
    RCLCPP_INFO_STREAM(this->get_logger(), "DEBUG SET");
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "DEBUG FAILED");
  }

  this->declare_parameter("freq", 100);
  f_num = this->get_parameter("freq").get_parameter_value().get<int>();
  this->set_parameter(rclcpp::Parameter("freq", f_num));
  publisher_ =
      this->create_publisher<std_msgs::msg::String>("talker_bot", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(f_num),
      std::bind(&MinimalPublisher::timer_callback, this));
  // broadcast talk
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_timer_ = this->create_wall_timer(300ms, std::bind(&MinimalPublisher::make_transforms, this));
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
geometry_msgs::msg::TransformStamped  MinimalPublisher::get_frame() {
  return child;
}
int main(int argc, char *argv[]) {
  // if (argc != 8) {
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("rclcpp"), "Invalid number of parameters\nusage: "
  //     "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
  //     "child_frame_name x y z roll pitch yaw");
  //   return 1;
  // }
  // // As the parent frame of the transform is `world`, it is
  // // necessary to check that the frame name passed is different
  // if (strcmp(argv[1], "world") == 0) {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Your static turtle name cannot be 'world'");
  //   return 2;
  // }
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "\n MULTIPLES OF 5 set as ERROR\n MULTIPLES OF 5 AND 10 "
                     "set as FATAL \n EVEN NUMBERS set as INFO \n ODD NUMBERS "
                     "set as DEBUG \n SERVICE CALLS enables WARN");
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
