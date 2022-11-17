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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "string_srv/srv/change.hpp"
#include <chrono> // NOLINT
#include <memory> // NOLINT
#include <string> // NOLINT

using namespace std::placeholders;    // NOLINT
using namespace std::chrono_literals; // NOLINT

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
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
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
    // Create a service for modifying str
    std::string string_service_name = "service_topic";
    string_service_ = this->create_service<string_srv::srv::Change>(
        string_service_name,
        std::bind(&MinimalPublisher::get_count_callback, this, _1, _2));
  }

 private:
  /**
   * @brief timer callback to publish msg and logging levels
   *
   */
  void timer_callback() {
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
  void get_count_callback(
      const std::shared_ptr<string_srv::srv::Change::Request> request,
      std::shared_ptr<string_srv::srv::Change::Response> response) {
    request->a = " AFTER SERVICING: I am NEW talker ";
    string_var = request->a;
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "SERVICE CALLED AND BASE STRING CHANGED");
    response->b = "SERVICE DONE";
  }

  std_msgs::msg::String message;
  int f_num;
  std::string string_var;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<string_srv::srv::Change>::SharedPtr string_service_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "\n MULTIPLES OF 5 set as ERROR\n MULTIPLES OF 5 AND 10 "
                     "set as FATAL \n EVEN NUMBERS set as INFO \n ODD NUMBERS "
                     "set as DEBUG \n SERVICE CALLS enables WARN");
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
