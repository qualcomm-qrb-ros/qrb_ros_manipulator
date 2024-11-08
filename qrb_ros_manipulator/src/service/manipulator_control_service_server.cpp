/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <array>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "service/manipulator_control_service_server.hpp"

constexpr char const* get_control_mode = "manipulator_get_control_mode";
constexpr char const* set_control_mode = "manipulator_set_control_mode";
constexpr char const* move_joint_pose = "manipulator_move_joint_pose";
constexpr char const* get_joint_pose = "manipulator_get_joint_pose";
constexpr char const* move_tcp_pose = "manipulator_move_tcp_pose";
constexpr char const* get_tcp_pose = "manipulator_get_tcp_pose";
constexpr char const* target_reachable = "manipulator_target_reachable";
constexpr char const* claw_control = "manipulator_claw_control";
constexpr char const* claw_get_status = "manipulator_claw_get_status";

using namespace std::placeholders;

namespace qrb_ros
{
namespace manipulator_controller
{
ManipulatorControllerService::ManipulatorControllerService(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<qrb::manipulator_management::ManipulatorManagement> manipulator)
  : node_(std::move(node)), manipulator_(std::move(manipulator))
{
  // init manipulator service

  callback_group_misc_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_control_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  get_mode_server_ = node_->create_service<ManipulatorGetControlMode>(
      get_control_mode,
      std::bind(&ManipulatorControllerService::manipulator_get_control_mode_cb, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_misc_);

  set_mode_server_ = node_->create_service<ManipulatorSetControlMode>(
      set_control_mode,
      std::bind(&ManipulatorControllerService::manipulator_set_control_mode_cb, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_misc_);

  move_joint_pose_server_ = node_->create_service<ManipulatorMoveJointPose>(
      move_joint_pose,
      std::bind(&ManipulatorControllerService::manipulator_move_joint_pose_cb, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_control_);

  get_joint_pose_server_ = node_->create_service<ManipulatorGetJointPose>(
      get_joint_pose,
      std::bind(&ManipulatorControllerService::manipulator_get_joint_pose_cb, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_control_);

  move_tcp_pose_server_ = node_->create_service<ManipulatorMoveTcpPose>(
      move_tcp_pose,
      std::bind(&ManipulatorControllerService::manipulator_move_tcp_pose_cb, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_control_);

  get_tcp_pose_server_ = node_->create_service<ManipulatorGetTcpPose>(
      get_tcp_pose,
      std::bind(&ManipulatorControllerService::manipulator_get_tcp_pose_cb, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_control_);

  target_reachable_server_ = node_->create_service<ManipulatorTargetReachable>(
      target_reachable,
      std::bind(&ManipulatorControllerService::manipulator_target_reachable_cb, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_misc_);

  claw_control_server_ = node_->create_service<ManipulatorClawControl>(
      claw_control,
      std::bind(&ManipulatorControllerService::manipulator_claw_control_cb, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_control_);

  get_claw_status_server = node_->create_service<ManipulatorClawGetStatus>(
      claw_get_status,
      std::bind(&ManipulatorControllerService::manipulator_claw_get_status_cb, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_control_);

  RCLCPP_INFO(node_->get_logger(), "ManipulatorControllerService: start running...");
}

void ManipulatorControllerService::manipulator_get_control_mode_cb(
    const std::shared_ptr<rmw_request_id_t> /**request_header**/,
    const std::shared_ptr<ManipulatorGetControlMode::Request> request,
    std::shared_ptr<ManipulatorGetControlMode::Response> response)
{
  int mode = -1;

  manipulator_->manipulator_get_control_mode(request->manipulator_id, mode);
  response->result = mode;
  RCLCPP_INFO(node_->get_logger(), "ManipulatorControllerService:(%d): state: %d",
              request->manipulator_id, response->result);
}

void ManipulatorControllerService::manipulator_set_control_mode_cb(
    const std::shared_ptr<rmw_request_id_t> /**request_header**/,
    const std::shared_ptr<ManipulatorSetControlMode::Request> request,
    std::shared_ptr<ManipulatorSetControlMode::Response> response)
{
  auto result = manipulator_->manipulator_set_control_mode(request->manipulator_id, request->mode);
  response->result = (bool)result;
  RCLCPP_INFO(node_->get_logger(),
              "ManipulatorControllerService:(%d): set control mode(%d) result:%d",
              request->manipulator_id, request->mode, (int)response->result);
}

void ManipulatorControllerService::manipulator_move_joint_pose_cb(
    const std::shared_ptr<rmw_request_id_t> /**request_header**/,
    const std::shared_ptr<ManipulatorMoveJointPose::Request> request,
    std::shared_ptr<ManipulatorMoveJointPose::Response> response)
{
  std::vector<double> joints_pose;

  for (int i = 0; i < request->joints_num; i++) {
    joints_pose.push_back(request->joints_pose[i]);
  }

  response->result =
      manipulator_->manipulator_move_joints(request->manipulator_id, joints_pose, request->speed,
                                            request->acc, request->time, request->radius);
  RCLCPP_INFO(node_->get_logger(), "ManipulatorControllerService:(%d): move joint pose result:%d",
              request->manipulator_id, (int)response->result);
}

void ManipulatorControllerService::manipulator_get_joint_pose_cb(
    const std::shared_ptr<rmw_request_id_t> /**request_header**/,
    const std::shared_ptr<ManipulatorGetJointPose::Request> request,
    std::shared_ptr<ManipulatorGetJointPose::Response> response)
{
  std::vector<double> joints_pose;

  response->result =
      manipulator_->manipulator_get_joints_position(request->manipulator_id, joints_pose);
  if (true == response->result) {
    response->joints_pose = joints_pose;
    response->joints_num = (int)joints_pose.size();
  }
  RCLCPP_INFO(node_->get_logger(), "ManipulatorControllerService:(%d): get joint num=:%d",
              request->manipulator_id,  response->joints_num);

  RCLCPP_INFO(node_->get_logger(), "ManipulatorControllerService:(%d): get joint pose result:%d",
              request->manipulator_id, (int)response->result);
}

void ManipulatorControllerService::manipulator_move_tcp_pose_cb(
    const std::shared_ptr<rmw_request_id_t> /**request_header**/,
    const std::shared_ptr<ManipulatorMoveTcpPose::Request> request,
    std::shared_ptr<ManipulatorMoveTcpPose::Response> response)
{
  response->result =
      manipulator_->manipulator_move_tcp(request->manipulator_id, request->pose, request->speed,
                                         request->acc, request->time, request->radius);

  RCLCPP_INFO(node_->get_logger(), "ManipulatorControllerService:(%d): get move tcp result:%d",
              request->manipulator_id, (int)response->result);
}

void ManipulatorControllerService::manipulator_get_tcp_pose_cb(
    const std::shared_ptr<rmw_request_id_t> /**request_header**/,
    const std::shared_ptr<ManipulatorGetTcpPose::Request> request,
    std::shared_ptr<ManipulatorGetTcpPose::Response> response)
{
  geometry_msgs::msg::Pose tcp_pose;

  response->result = manipulator_->manipulator_get_tcp_pose(request->manipulator_id, tcp_pose);
  if (true == response->result) {
    response->pose = tcp_pose;
  }

  RCLCPP_INFO(node_->get_logger(), "ManipulatorControllerService:(%d): get tcp pose result:%d",
              request->manipulator_id, (int)response->result);
}

void ManipulatorControllerService::manipulator_target_reachable_cb(
    const std::shared_ptr<rmw_request_id_t> /**request_header**/,
    const std::shared_ptr<ManipulatorTargetReachable::Request> request,
    std::shared_ptr<ManipulatorTargetReachable::Response> response)
{
  response->result =
      manipulator_->manipulator_target_reachable(request->manipulator_id, request->pose);

  RCLCPP_INFO(node_->get_logger(),
              "ManipulatorControllerService:(%d): tcp pose reachable result:%d",
              request->manipulator_id, (int)response->result);
}

void ManipulatorControllerService::manipulator_claw_control_cb(
    const std::shared_ptr<rmw_request_id_t> /**request_header**/,
    const std::shared_ptr<ManipulatorClawControl::Request> request,
    std::shared_ptr<ManipulatorClawControl::Response> response)
{
  std::vector<double> force = request->force;
  std::vector<double> amplitude = request->amplitude;

  if (force.size() != amplitude.size() && force.size() == 0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Error: ManipulatorControllerService:(%d):"
                 "claw control request data error",
                 request->manipulator_id);
  }

  response->result =
      manipulator_->manipulator_claw_control(request->manipulator_id, force, amplitude);

  RCLCPP_INFO(node_->get_logger(), "ManipulatorControllerService:(%d): claw control result:%d",
              request->manipulator_id, (int)response->result);
}

void ManipulatorControllerService::manipulator_claw_get_status_cb(
    const std::shared_ptr<rmw_request_id_t> /**request_header**/,
    const std::shared_ptr<ManipulatorClawGetStatus::Request> request,
    std::shared_ptr<ManipulatorClawGetStatus::Response> response)
{
  std::vector<double> force;
  std::vector<double> amplitude;

  response->result =
      manipulator_->manipulator_claw_get_status(request->manipulator_id, force, amplitude);

  if (true == response->result && force.size() > 0 && amplitude.size() > 0 &&
      amplitude.size() == force.size()) {
    response->force = force;
    response->amplitude = amplitude;
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "ManipulatorControllerService:(%d): claw get status failed:%d",
                 request->manipulator_id, (int)response->result);
  }

  RCLCPP_INFO(node_->get_logger(), "ManipulatorControllerService:(%d): claw get status result:%d",
              request->manipulator_id, (int)response->result);
}

}  // namespace manipulator_controller
}  // namespace qrb_ros
