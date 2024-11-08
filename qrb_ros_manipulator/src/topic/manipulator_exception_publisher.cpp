/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topic/manipulator_exception_publisher.hpp"

constexpr char const* exception_topic_name = "manipulator_exception";

using namespace std::placeholders;

namespace qrb_ros
{
namespace manipulator_controller
{

ManipulatorExceptionTopic::ManipulatorExceptionTopic(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<qrb::manipulator_management::ManipulatorManagement> manipulator)
  : node_(std::move(node)), manipulator_(std::move(manipulator))
{
  exception_topic_ = node_->create_publisher<ManipulatorExceptionMsg>(exception_topic_name, 10);
  std::function<void(int, int err_code)> exception_publish_function_ =
      std::bind(&ManipulatorExceptionTopic::publish_manipulator_exception, this, _1, _2);

  manipulator_->register_exception_notify_callback(exception_publish_function_);
}

void ManipulatorExceptionTopic::publish_manipulator_exception(int manipulator_id, int error_code)
{
  auto time = node_->get_clock()->now();
  auto exception_msg = std::make_unique<ManipulatorExceptionMsg>();

  std_msgs::msg::Header header;

  header.frame_id = exception_topic_name;
  header.stamp.sec = time.seconds();
  header.stamp.nanosec = time.nanoseconds();

  exception_msg->header = header;
  exception_msg->manipulator_id = manipulator_id;
  exception_msg->error_code = error_code;

  exception_topic_->publish(std::move(exception_msg));
  RCLCPP_INFO(node_->get_logger(), "ManipulatorExceptionTopic publish done ");
}

}  // namespace manipulator_controller
}  // namespace qrb_ros
