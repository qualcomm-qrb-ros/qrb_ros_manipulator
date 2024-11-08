/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_MANIPULATOR_CONTROLLER_HPP_
#define QRB_ROS_MANIPULATOR_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "topic/manipulator_exception_publisher.hpp"
#include "service/manipulator_control_service_server.hpp"


namespace qrb_ros
{
namespace manipulator_controller
{
class ManipulatorController : public rclcpp::Node
{
public:
  explicit ManipulatorController(const rclcpp::NodeOptions& options);
  //~ManipulatorController();
  bool manipulator_run();
  void manipulator_shutdown();

private:
  bool manipulator_get_parameters();
  bool manipulator_get_parameters(std::vector<qrb::manipulator_management::
                                                          ManipulatorConfig> &manipulators);

  std::shared_ptr<manipulator_controller::ManipulatorExceptionTopic> exception_;
  std::shared_ptr<manipulator_controller::ManipulatorControllerService> controller_;
  std::shared_ptr<qrb::manipulator_management::ManipulatorManagement> manipulator_;
};

}  // namespace manipulator_controller
}  // namespace qrb_ros

#endif  // QRB_ROS_MANIPULATOR_CONTROLLER_HPP_
