/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_MANIPULATOR_LEBAI_IMPLEMENT_HPP_
#define QRB_MANIPULATOR_LEBAI_IMPLEMENT_HPP_
#include <eigen3/Eigen/Core>
#include <lebai/robot.hh>
#include <memory>
#include <mutex>

#include "qrb_manipulator_controller/manipulator_interface.hpp"

namespace qrb
{
namespace lebai_implement
{
class ARMLebai : public manipulator_interface::ArmInterface
{
public:
  ARMLebai();

  virtual ~ARMLebai() {}

  bool arm_init(const std::string ip) override;
  bool arm_is_connected() override;
  manipulator_interface::ManipulatorControlMode arm_get_control_mode() override;
  bool arm_set_control_mode(const manipulator_interface::ManipulatorControlMode mode) override;
  bool arm_move_tcp(const geometry_msgs::msg::Pose & pose,
      double v,
      double a,
      double t,
      double r) override;
  bool arm_move_joint(const std::vector<double> & joint_pose,
      double v,
      double a,
      double t,
      double r) override;
  std::vector<double> arm_get_joint_positions() override;
  geometry_msgs::msg::Pose arm_get_tcp_pose() override;
  bool arm_target_reachable(geometry_msgs::msg::Pose pose) override;
  void arm_release() override;
  bool arm_set_tcp(const std::array<double, 6> tcp_pose) override;
  manipulator_interface::ManipulatorException arm_get_status() override;

  /* claw interface */
  bool claw_init(const std::string claw) override;
  bool claw_release() override;
  bool claw_control(const std::vector<double> force, const std::vector<double> amplitude) override;
  bool claw_get_status(std::vector<double> & force, std::vector<double> & amplitude) override;

private:
  std::map<std::string, double> tcp_pose_array2map(std::array<double, 6> pose);
  std::array<double, 6> tcp_pose_map2array(std::map<std::string, double> tcp);
  Eigen::Vector3f quatern_to_euler(geometry_msgs::msg::Pose pose);
  geometry_msgs::msg::Pose euler_to_quatern(double roll, double pitch, double yaw);

  std::shared_ptr<lebai::l_master::Robot> lebai_sdk_;
  std::shared_ptr<std::mutex> mode_mutex_;
  manipulator_interface::ManipulatorControlMode lebai_mode_;
};

}  // namespace lebai_implement
}  // namespace qrb

#endif  // QRB_MANIPULATOR_LEBAI_IMPLEMENT_HPP_
