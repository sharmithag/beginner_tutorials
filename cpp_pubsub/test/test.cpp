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

#include <gtest/gtest.h>
#include <stdlib.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <cstdint>
#include "cpp_pubsub/publisher_member_function.hpp"

using namespace std::chrono_literals; // NOLINT
/**
 * @brief Class testpub for testing tf frame translation and rotation
 * 
 */
class TestPub : public ::testing::Test {
 public:
  TestPub() {}

  void SetUp() override {
    rclcpp::init(0, nullptr);
    minimal_pub_ = std::make_shared<MinimalPublisher>();
    clock_ = std::make_unique<rclcpp::Clock>(RCL_ROS_TIME);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<MinimalPublisher> minimal_pub_;
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<std::thread> pub_thread_;
};

TEST_F(TestPub, TestTF) {

  auto start = clock_->now();
  double duration_sec = 0;
  while (duration_sec < 3) {
    rclcpp::spin_some(minimal_pub_);
    duration_sec = (clock_->now() - start).seconds();
  }

  geometry_msgs::msg::TransformStamped t;

  // lookupTransform
  try {
    t = tf_buffer_->lookupTransform("world", "talk", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "Could not find transform from world to talk");
    FAIL();
  }

  EXPECT_FLOAT_EQ(t.transform.translation.x,
                  minimal_pub_->get_frame().transform.translation.x);
  EXPECT_FLOAT_EQ(t.transform.translation.y,
                  minimal_pub_->get_frame().transform.translation.x);
  EXPECT_FLOAT_EQ(t.transform.translation.z,
                  minimal_pub_->get_frame().transform.translation.x);
  EXPECT_FLOAT_EQ(t.transform.rotation.x,
                  minimal_pub_->get_frame().transform.rotation.x);
  EXPECT_FLOAT_EQ(t.transform.rotation.y,
                  minimal_pub_->get_frame().transform.rotation.y);
  EXPECT_FLOAT_EQ(t.transform.rotation.z,
                  minimal_pub_->get_frame().transform.rotation.z);
  EXPECT_FLOAT_EQ(t.transform.rotation.w,
                  minimal_pub_->get_frame().transform.rotation.w);
