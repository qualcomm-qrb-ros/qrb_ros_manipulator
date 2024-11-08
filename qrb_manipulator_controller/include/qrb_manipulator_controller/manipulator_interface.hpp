/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_MANIPULATOR_MANIPULATOR_INTERFACE_HPP_
#define QRB_MANIPULATOR_MANIPULATOR_INTERFACE_HPP_

#include <array>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>

namespace qrb
{
namespace manipulator_interface
{
enum class ManipulatorControlMode
{
  ARM_OFFLINE = 0,
  ARM_CONTROLLING,
  ARM_TEACHNING,
};

enum class ManipulatorException
{
  NORMAL = 0,
  NET_ERR,
  SYS_ERR,
  EMERGENCY,
  OTHER_ERR,
};

class ArmInterface
{
public:
  ArmInterface();

  virtual ~ArmInterface() {}

  virtual bool arm_init(const std::string ip) = 0;
  virtual bool arm_is_connected() = 0;
  virtual ManipulatorControlMode arm_get_control_mode() = 0;
  virtual bool arm_set_control_mode(const ManipulatorControlMode mode) = 0;
  virtual bool
  arm_move_tcp(const geometry_msgs::msg::Pose & pose, double v, double a, double t, double r) = 0;
  virtual bool arm_move_joint(const std::vector<double> & joint_pose,
      double v,
      double a,
      double t,
      double r) = 0;
  virtual std::vector<double> arm_get_joint_positions() = 0;
  virtual geometry_msgs::msg::Pose arm_get_tcp_pose() = 0;

  virtual bool arm_target_reachable(geometry_msgs::msg::Pose pose) = 0;
  virtual void arm_release() = 0;
  virtual bool arm_set_tcp(const std::array<double, 6> pose) = 0;
  virtual ManipulatorException arm_get_status() = 0;

  /* claw interface */
  virtual bool claw_init(const std::string claw) = 0;
  virtual bool claw_release() = 0;
  virtual bool claw_control(const std::vector<double> force,
      const std::vector<double> amplitude) = 0;
  virtual bool claw_get_status(std::vector<double> & force, std::vector<double> & amplitude) = 0;
};

}  // namespace manipulator_interface
}  // namespace qrb

#endif  // QRB_MANIPULATOR_MANIPULATOR_INTERFACE_HPP_
