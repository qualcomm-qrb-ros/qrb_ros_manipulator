/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "qrb_manipulator_controller/lebai_implement.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <exception>
#include <iostream>
#include <lebai/discovery.hh>
#include <mutex>
#include <thread>
#include <tuple>

#include "lebai/robot.hh"

namespace qrb
{
namespace lebai_implement
{
enum class LebaiState
{
  SYSTEM_ERROR = -1,
  BUS_ERROR = 0,
  EMERGENCY_STOP = 1,
  INITLIZING = 2,
  INITLIZED = 4,
  IDLE = 5,
  SUSPEND = 6,
  RUNNING = 7,
  UPDATING = 8,
  BOOTING = 9,
  STOPING = 10,
  TEACHNING = 11,
  STOP = 12,
};

ARMLebai::ARMLebai()
{
  mode_mutex_ = std::make_shared<std::mutex>();
}

bool ARMLebai::arm_init(const std::string ip)
{
  lebai_sdk_ = std::make_shared<lebai::l_master::Robot>(ip, false);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  if (!lebai_sdk_->is_network_connected()) {
    std::cout << "DEBUG:ip:" << ip << "init failed, try to get new ip" << std::endl;
    /* try to get ip automatically */
    lebai::zeroconf::Discovery discovery;
    auto lebai_data = discovery.resolve();
    std::cout << "DEBUG: get lebai ip:" << lebai_data[0].ip_address << std::endl;
    lebai_sdk_ = std::make_shared<lebai::l_master::Robot>(lebai_data[0].ip_address, false);
  }

  if (lebai_sdk_->is_network_connected()) {
    lebai_sdk_->start_sys();
    lebai_mode_ = manipulator_interface::ManipulatorControlMode::ARM_CONTROLLING;
    std::cout << "DEBUG: lebai started" << std::endl;
    return true;
  }

  std::cout << "DEBUG: lebai connect failed" << std::endl;
  return false;
}

bool ARMLebai::arm_is_connected()
{
  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    return false;
  }

  return lebai_sdk_->is_network_connected();
}

std::map<std::string, double> ARMLebai::tcp_pose_array2map(std::array<double, 6> pose)
{
  std::map<std::string, double> arm_pose;

  arm_pose["x"] = pose[0];
  arm_pose["y"] = pose[1];
  arm_pose["z"] = pose[2];
  arm_pose["rx"] = pose[3];
  arm_pose["ry"] = pose[4];
  arm_pose["rz"] = pose[5];

  return arm_pose;
}

std::array<double, 6> ARMLebai::tcp_pose_map2array(std::map<std::string, double> tcp)
{
  std::array<double, 6> pose;
  pose[0] = tcp["x"];
  pose[1] = tcp["y"];
  pose[2] = tcp["z"];
  pose[3] = tcp["rx"];
  pose[4] = tcp["ry"];
  pose[5] = tcp["rz"];

  return pose;
}
Eigen::Vector3f ARMLebai::quatern_to_euler(geometry_msgs::msg::Pose pose)
{
  Eigen::Quaternion<float> q;

  q.x() = (float)pose.orientation.x;
  q.y() = (float)pose.orientation.y;
  q.z() = (float)pose.orientation.z;
  q.w() = (float)pose.orientation.w;

  /* roll,pitch,yaw */
  return q.toRotationMatrix().eulerAngles(0, 1, 2);
}

geometry_msgs::msg::Pose ARMLebai::euler_to_quatern(double roll, double pitch, double yaw)
{
  Eigen::Quaternion<float> q;
  geometry_msgs::msg::Pose pose;

  q = Eigen::AngleAxisf((float)roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf((float)pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf((float)yaw, Eigen::Vector3f::UnitZ());

  /* x,y,z,w */
  pose.orientation.x = (double)q.x();
  pose.orientation.y = (double)q.y();
  pose.orientation.z = (double)q.z();
  pose.orientation.w = (double)q.w();

  return pose;
}

manipulator_interface::ManipulatorControlMode ARMLebai::arm_get_control_mode()
{
  return lebai_mode_;
}

bool ARMLebai::arm_set_control_mode(const manipulator_interface::ManipulatorControlMode mode)
{
  bool res = false;

  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    return false;
  }

  if (!lebai_sdk_->is_network_connected()) {
    std::cerr << "Error: lebai network connect failed" << std::endl;
    return false;
  }

  mode_mutex_->lock();

  if (mode == lebai_mode_) {
    mode_mutex_->unlock();
    return true;
  }

  switch (mode) {
    case manipulator_interface::ManipulatorControlMode::ARM_OFFLINE: {
      lebai_sdk_->stop_sys();
      std::cout << "DEBUG: lebai set offline mode" << std::endl;
      res = true;
      break;
    }
    case manipulator_interface::ManipulatorControlMode::ARM_CONTROLLING: {
      if (lebai_mode_ == manipulator_interface::ManipulatorControlMode::ARM_TEACHNING) {
        lebai_sdk_->end_teach_mode();
        std::cout << "DEBUG: lebai teach mode disabled" << std::endl;
      } else
        lebai_sdk_->start_sys();

      std::cout << "DEBUG: lebai set controlling mode" << std::endl;
      res = true;
      break;
    }
    case manipulator_interface::ManipulatorControlMode::ARM_TEACHNING: {
      if (lebai_mode_ == manipulator_interface::ManipulatorControlMode::ARM_OFFLINE) {
        lebai_sdk_->start_sys();
      }
      lebai_sdk_->teach_mode();
      auto arm_mode = (LebaiState)lebai_sdk_->get_robot_mode();
      if (LebaiState::TEACHNING == arm_mode) {
        res = true;
        std::cout << "DEBUG: lebai teach mode enabled" << std::endl;
      } else
        std::cerr << "ERROR: lebai teach mode switch failed" << std::endl;
      break;
    }
    default:
      std::cerr << "Warning: lebai unrecognize control mode" << (int)mode << std::endl;
  }

  lebai_mode_ = res ? mode : lebai_mode_;

  mode_mutex_->unlock();
  return res;
}

bool ARMLebai::arm_move_tcp(const geometry_msgs::msg::Pose & pose,
    double v,
    double a,
    double t,
    double r)
{
  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    return false;
  }

  /* check lebai status */
  auto lebai_status = (LebaiState)lebai_sdk_->get_robot_mode();
  if (!(lebai_status == LebaiState::IDLE || lebai_status == LebaiState::RUNNING ||
      lebai_status == LebaiState::INITLIZED)) {
    std::cout << "ERROR:arm_move_tcp: lebai mode state=" << (int)lebai_status << std::endl;
    return false;
  }

  /* check target if reachable */
  if (false == this->arm_target_reachable(pose)) {
    std::cout << "ERROR:arm_move_tcp: target unreachable " << std::endl;
    return false;
  }

  /* calculate pose */
  Eigen::Vector3f angle = this->quatern_to_euler(pose);
  std::array<double, 6> tcp_arr = { pose.position.x, pose.position.y, pose.position.z,
    (double)angle.x(), (double)angle.y(), (double)angle.z() };

  std::cerr << "arm_move_tcp move angle = :" << angle.x() << "\t" << angle.y() << "\t" << angle.z()
            << std::endl;

  auto tcp = this->tcp_pose_array2map(tcp_arr);

  int result = lebai_sdk_->movej(tcp, v, a, t, r);

  if (result <= 0) {
    std::cerr << "arm_move_tcp move failed:" << result << std::endl;
    return false;
  }
  // lebai_sdk_->wait_move();
  std::cout << "DEBUG: lebai move tcp done" << std::endl;
  return true;
}

bool ARMLebai::arm_move_joint(const std::vector<double> & joint_pose,
    double v,
    double a,
    double t,
    double r)
{
  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    return false;
  }

  /* check arm mode */
  auto arm_mode = (LebaiState)lebai_sdk_->get_robot_mode();
  if (!(arm_mode == LebaiState::IDLE || arm_mode == LebaiState::RUNNING ||
      arm_mode == LebaiState::INITLIZED)) {
    std::cout << "ERROR:arm_move_joint: lebai mode state=" << (int)arm_mode << std::endl;
    return false;
  }

  int result = lebai_sdk_->movej(joint_pose, v, a, t, r);

  if (result <= 0) {
    std::cerr << "arm_move_joint failed:" << result << std::endl;
    return false;
  }
  // lebai_sdk_->wait_move();
  std::cout << "DEBUG: lebai move joint done" << std::endl;
  return true;
}

std::vector<double> ARMLebai::arm_get_joint_positions()
{
  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    throw "pointer is invalid";
  }
  return lebai_sdk_->get_actual_joint_positions();
}

/* get tcp pose */
geometry_msgs::msg::Pose ARMLebai::arm_get_tcp_pose()
{
  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    throw "pointer is invalid";
  }

  auto tcp = lebai_sdk_->get_target_tcp_pose();
  auto tcp_arr = this->tcp_pose_map2array(tcp);
  std::cerr << "arm_get_tcp_pose angle = :" << tcp_arr[3] << "\t" << tcp_arr[4] << "\t"
            << tcp_arr[5] << std::endl;

  geometry_msgs::msg::Pose pose = euler_to_quatern(tcp_arr[3], tcp_arr[4], tcp_arr[5]);
  pose.position.x = tcp_arr[0];
  pose.position.y = tcp_arr[1];
  pose.position.z = tcp_arr[2];

  // test
  Eigen::Vector3f test_angle = this->quatern_to_euler(pose);
  std::cerr << "arm_get_tcp_pose test angle = :" << test_angle.x() << "\t" << test_angle.y() << "\t"
            << test_angle.z() << std::endl;

  return pose;
}

bool ARMLebai::arm_target_reachable(geometry_msgs::msg::Pose pose)
{
  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    return false;
  }

  std::vector<double> joints_pose;
  lebai::l_master::KinematicsInverseResp kinematic_result;
  joints_pose = this->arm_get_joint_positions();

  /* calculate pose */
  Eigen::Vector3f angle = this->quatern_to_euler(pose);
  std::array<double, 6> tcp_arr = { pose.position.x, pose.position.y, pose.position.z,
    (double)angle.x(), (double)angle.y(), (double)angle.z() };

  auto tcp = this->tcp_pose_array2map(tcp_arr);

  kinematic_result = lebai_sdk_->kinematics_inverse(tcp, joints_pose);

  std::cout << "DEBUG: lebai target reachable=" << kinematic_result.ok << std::endl;
  return kinematic_result.ok;
}

void ARMLebai::arm_release()
{
  if (lebai_sdk_.get() == nullptr) {
    return;
  }

  lebai_sdk_->stop_sys();

  std::cout << "DEBUG: lebai release done " << std::endl;
}

bool ARMLebai::arm_set_tcp(const std::array<double, 6> tcp_pose)
{
  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    return false;
  }
  lebai_sdk_->set_tcp(tcp_pose);
  return true;
}

manipulator_interface::ManipulatorException ARMLebai::arm_get_status()
{
  manipulator_interface::ManipulatorException exception;

  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    throw "pointer is invalid";
  }

  auto arm_status = lebai_sdk_->get_robot_mode();

  switch ((LebaiState)arm_status) {
    case LebaiState::SYSTEM_ERROR: {
      exception = manipulator_interface::ManipulatorException::SYS_ERR;
      break;
    }
    case LebaiState::BUS_ERROR: {
      exception = manipulator_interface::ManipulatorException::NET_ERR;
      break;
    }
    case LebaiState::EMERGENCY_STOP: {
      exception = manipulator_interface::ManipulatorException::EMERGENCY;
      break;
    }
    case LebaiState::INITLIZING:
    case LebaiState::INITLIZED:
    case LebaiState::IDLE:
    case LebaiState::SUSPEND:
    case LebaiState::RUNNING:
    case LebaiState::UPDATING:
    case LebaiState::BOOTING:
    case LebaiState::STOPING:
    case LebaiState::TEACHNING:
    case LebaiState::STOP: {
      exception = manipulator_interface::ManipulatorException::NORMAL;
      break;
    }
    default: {
      std::cout << "Warning: ManipulatorException: " << (int)arm_status << std::endl;
      exception = manipulator_interface::ManipulatorException::NORMAL;
    }
  }
  return exception;
}

/* claw interface */
bool ARMLebai::claw_init(const std::string claw)
{
  std::cout << "INFO:init claw():" << claw << std::endl;
  /*lebai type_90 claw has been initialized by arm */
  return true;
}

bool ARMLebai::claw_release()
{
  return true;
}

bool ARMLebai::claw_control(const std::vector<double> force, const std::vector<double> amplitude)
{
  double claw_force = force[0];         /*0~100 */
  double claw_amplitude = amplitude[0]; /* 0~100 */

  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    return false;
  }

  claw_force = std::fabs(claw_force);
  if (claw_force > 100.0)
    claw_force = 100.0;

  claw_amplitude = std::fabs(claw_amplitude);
  if (claw_amplitude > 100.0)
    claw_amplitude = 100.0;

  lebai_sdk_->set_claw(claw_force, claw_amplitude);

  return true;
}

bool ARMLebai::claw_get_status(std::vector<double> & force, std::vector<double> & amplitude)
{
  if (lebai_sdk_.get() == nullptr) {
    std::cerr << "ERROR: lebai sdk uninitialized" << std::endl;
    return false;
  }

  std::tuple<double, double, bool> claw_status = lebai_sdk_->get_claw();
  force.clear();
  force.emplace_back(std::get<0>(claw_status));
  amplitude.clear();
  amplitude.emplace_back(std::get<1>(claw_status));

  return true;
}

}  // namespace lebai_implement
}  // namespace qrb
