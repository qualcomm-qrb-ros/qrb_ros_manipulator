/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <string>
#include <iostream>
#include <cstdlib>

#include "qrb_manipulator_controller/manipulator_management.hpp"
#include "qrb_ros_manipulator/manipulator_controller.hpp"

namespace qrb_ros
{
namespace manipulator_controller
{

ManipulatorController::ManipulatorController(const rclcpp::NodeOptions& options)
  : rclcpp::Node("manipulator_controller", options)
{
}

bool ManipulatorController::manipulator_get_parameters(
    std::vector<qrb::manipulator_management::ManipulatorConfig>& manipulators)
{
  int manipulator_num = 0;
  qrb::manipulator_management::ManipulatorConfig config;

  manipulator_num = this->declare_parameter<int>("manipulator_num", 0);
  if (0 == manipulator_num) {
    return false;
  }

  manipulators.clear();

  for (int i = 0; i < manipulator_num; i++) {
    config.arm_type = qrb::manipulator_management::ManipulatorType(
        this->declare_parameter<int>("arm_type_" + std::to_string(i + 1), 0));

    config.dev_id = this->declare_parameter<int>("dev_id_" + std::to_string(i + 1), 0);
    config.ip = this->declare_parameter<std::string>("ip_" + std::to_string(i + 1), "0.0.0.0");

    RCLCPP_INFO(this->get_logger(), "ManipulatorController get parameters type=%d, id=%d,ip=%s",
                (int)config.arm_type, config.dev_id, config.ip.c_str());
    std::cout << config.dev_id << std::endl;
    manipulators.push_back(config);
  }
  return true;
}

bool ManipulatorController::manipulator_run()
{
  auto node_ptr = shared_from_this();

  manipulator_ = std::make_shared<qrb::manipulator_management::ManipulatorManagement>();
  RCLCPP_INFO(this->get_logger(), "ManipulatorController start run");
  std::vector<qrb::manipulator_management::ManipulatorConfig> manipulators;
  if (!this->manipulator_get_parameters(manipulators)) {
    RCLCPP_INFO(this->get_logger(), "ManipulatorController get parameters failed");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "ManipulatorController get parameters done ");
  for (long unsigned int i = 0; i < manipulators.size(); i++) {
    manipulator_->add_manipulator(manipulators[i].arm_type, manipulators[i].dev_id,
                                  manipulators[i].ip);
  }
  RCLCPP_INFO(this->get_logger(), "ManipulatorController start create service done ");
  exception_ =
      std::make_shared<manipulator_controller::ManipulatorExceptionTopic>(node_ptr, manipulator_);
  controller_ = std::make_shared<manipulator_controller::ManipulatorControllerService>(
      node_ptr, manipulator_);

  return true;
}

void ManipulatorController::manipulator_shutdown()
{
  manipulator_->manipulator_release();
}

}  // namespace manipulator_controller
}  // namespace qrb_ros
