/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_MANIPULATOR_CONTROL_SERVICE_SERVER_HPP_
#define QRB_ROS_MANIPULATOR_CONTROL_SERVICE_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "qrb_manipulator_controller/manipulator_management.hpp"
#include "qrb_manipulator_controller/manipulator_interface.hpp"

#include "qrb_ros_manipulator_msgs/srv/manipulator_get_control_mode.hpp"
#include "qrb_ros_manipulator_msgs/srv/manipulator_set_control_mode.hpp"
#include "qrb_ros_manipulator_msgs/srv/manipulator_move_joint_pose.hpp"
#include "qrb_ros_manipulator_msgs/srv/manipulator_get_joint_pose.hpp"
#include "qrb_ros_manipulator_msgs/srv/manipulator_move_tcp_pose.hpp"
#include "qrb_ros_manipulator_msgs/srv/manipulator_get_tcp_pose.hpp"
#include "qrb_ros_manipulator_msgs/srv/manipulator_target_reachable.hpp"
#include "qrb_ros_manipulator_msgs/srv/manipulator_claw_control.hpp"
#include "qrb_ros_manipulator_msgs/srv/manipulator_claw_get_status.hpp"

using ManipulatorGetControlMode = qrb_ros_manipulator_msgs::srv::ManipulatorGetControlMode;
using ManipulatorSetControlMode = qrb_ros_manipulator_msgs::srv::ManipulatorSetControlMode;
using ManipulatorMoveJointPose = qrb_ros_manipulator_msgs::srv::ManipulatorMoveJointPose;
using ManipulatorGetJointPose = qrb_ros_manipulator_msgs::srv::ManipulatorGetJointPose;
using ManipulatorMoveTcpPose = qrb_ros_manipulator_msgs::srv::ManipulatorMoveTcpPose;
using ManipulatorGetTcpPose = qrb_ros_manipulator_msgs::srv::ManipulatorGetTcpPose;
using ManipulatorTargetReachable = qrb_ros_manipulator_msgs::srv::ManipulatorTargetReachable;
using ManipulatorClawControl = qrb_ros_manipulator_msgs::srv::ManipulatorClawControl;
using ManipulatorClawGetStatus = qrb_ros_manipulator_msgs::srv::ManipulatorClawGetStatus;

namespace qrb_ros
{
namespace manipulator_controller
{

class ManipulatorControllerService
{
public:
    explicit ManipulatorControllerService(rclcpp::Node::SharedPtr node, std::shared_ptr<qrb::manipulator_management::ManipulatorManagement> manipulator);
    //~ManipulatorControllerService();

private:
    std::shared_ptr<qrb::manipulator_management::ManipulatorManagement> manipulator_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_control_;
    rclcpp::CallbackGroup::SharedPtr callback_group_misc_;

    rclcpp::Service<ManipulatorGetControlMode>::SharedPtr get_mode_server_;
    rclcpp::Service<ManipulatorSetControlMode>::SharedPtr set_mode_server_;

    rclcpp::Service<ManipulatorMoveJointPose>::SharedPtr move_joint_pose_server_;
    rclcpp::Service<ManipulatorGetJointPose>::SharedPtr  get_joint_pose_server_;

    rclcpp::Service<ManipulatorMoveTcpPose>::SharedPtr move_tcp_pose_server_;
    rclcpp::Service<ManipulatorGetTcpPose>::SharedPtr get_tcp_pose_server_;

    rclcpp::Service<ManipulatorTargetReachable>::SharedPtr target_reachable_server_;

    rclcpp::Service<ManipulatorClawControl>::SharedPtr claw_control_server_;
    rclcpp::Service<ManipulatorClawGetStatus>::SharedPtr get_claw_status_server;

    void manipulator_get_control_mode_cb(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManipulatorGetControlMode::Request> request,
    std::shared_ptr<ManipulatorGetControlMode::Response> response);

    void manipulator_set_control_mode_cb(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManipulatorSetControlMode::Request> request,
    std::shared_ptr<ManipulatorSetControlMode::Response> response);

    void manipulator_move_joint_pose_cb(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManipulatorMoveJointPose::Request> request,
    std::shared_ptr<ManipulatorMoveJointPose::Response> response);

    void manipulator_get_joint_pose_cb(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManipulatorGetJointPose::Request> request,
    std::shared_ptr<ManipulatorGetJointPose::Response> response);

    void manipulator_move_tcp_pose_cb(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManipulatorMoveTcpPose::Request> request,
    std::shared_ptr<ManipulatorMoveTcpPose::Response> response);

    void manipulator_get_tcp_pose_cb(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManipulatorGetTcpPose::Request> request,
    std::shared_ptr<ManipulatorGetTcpPose::Response> response);

    void manipulator_target_reachable_cb(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManipulatorTargetReachable::Request> request,
    std::shared_ptr<ManipulatorTargetReachable::Response> response);

    void manipulator_claw_control_cb(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManipulatorClawControl::Request> request,
    std::shared_ptr<ManipulatorClawControl::Response> response);

    void manipulator_claw_get_status_cb(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManipulatorClawGetStatus::Request> request,
    std::shared_ptr<ManipulatorClawGetStatus::Response> response);

};

}  // namespace manipulator_controller
}  // namespace qrb_ros
#endif  // QRB_ROS_MANIPULATOR_CONTROL_SERVICE_SERVER_HPP_
