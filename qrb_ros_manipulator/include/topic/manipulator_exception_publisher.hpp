/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_MANIPULATOR_EXCEPTION_PUBLISHER_HPP_
#define QRB_ROS_MANIPULATOR_EXCEPTION_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "qrb_manipulator_controller/manipulator_management.hpp"
#include "qrb_manipulator_controller/manipulator_interface.hpp"

#include "qrb_ros_manipulator_msgs/msg/manipulator_exception.hpp"

using ManipulatorExceptionMsg = qrb_ros_manipulator_msgs::msg::ManipulatorException;

namespace qrb_ros
{
namespace manipulator_controller
{

class ManipulatorExceptionTopic
{
public:
    explicit ManipulatorExceptionTopic(rclcpp::Node::SharedPtr node,
                                    std::shared_ptr<qrb::manipulator_management::ManipulatorManagement> manipulator);

private:
    void publish_manipulator_exception(int exception, int error_code);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<qrb::manipulator_management::ManipulatorManagement> manipulator_;
    rclcpp::Publisher<ManipulatorExceptionMsg>::SharedPtr exception_topic_;
};

}  // namespace manipulator_controller
}  // namespace qrb_ros
#endif  // QRB_ROS_MANIPULATOR_EXCEPTION_PUBLISHER_HPP_
