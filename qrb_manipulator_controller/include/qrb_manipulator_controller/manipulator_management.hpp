/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_MANIPULATOR_MANIPULATOR_MANAGEMENT_HPP_
#define QRB_MANIPULATOR_MANIPULATOR_MANAGEMENT_HPP_

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "qrb_manipulator_controller/manipulator_interface.hpp"
#include "qrb_manipulator_controller/manipulator_sm.hpp"

namespace qrb
{
namespace manipulator_management
{
class ArmInterface;

#define MOVE_SPEED_DEFAULT (1.5)
#define MOVE_ACC_DEFAULT (1.0)
#define MOVE_TIME_DEFAULT (0.0)
#define MOVE_RADIUS_DEFAULT (0.0)

/* manipulator and claw type definition */

enum ManipulatorType
{
  LEBAI_LM3_CLAW_90 = 0,
  ARM_TYPE_MAX,
};

struct ManipulatorConfig
{
  enum ManipulatorType arm_type;
  int dev_id;
  std::array<double, 6> tcp_pose;
  std::string ip;
  int joints_num;
};

struct manipulator_dev
{
  ManipulatorConfig configuration;
  std::shared_ptr<qrb::manipulator_interface::ArmInterface> arm_interface;
  /* arm exception state */
  manipulator_interface::ManipulatorException arm_exception =
      manipulator_interface::ManipulatorException::NORMAL;
  std::shared_ptr<manipulator_sm::ManipulatorSm> state_machine;
  std::shared_ptr<std::mutex> control_mutex;
};

class ManipulatorManagement
{
public:
  ManipulatorManagement();

  virtual ~ManipulatorManagement();

  bool add_manipulator(int arm_type, int dev_id, std::string ip); /* create manipulator object */
  void manipulator_release(void);

  /* robot ARM control API */
  bool manipulator_get_control_mode(const int dev_id, int & mode);
  bool manipulator_set_control_mode(const int dev_id, const int mode);
  bool manipulator_move_tcp(const int dev_id,
      const geometry_msgs::msg::Pose pose,
      double v = MOVE_SPEED_DEFAULT,
      double a = MOVE_ACC_DEFAULT,
      double t = MOVE_TIME_DEFAULT,
      double r = MOVE_RADIUS_DEFAULT);
  bool manipulator_get_tcp_pose(const int dev_id, geometry_msgs::msg::Pose & tcp);

  bool manipulator_move_joints(int dev_id,
      const std::vector<double> & joint_pose,
      double v = MOVE_SPEED_DEFAULT,
      double a = MOVE_ACC_DEFAULT,
      double t = MOVE_TIME_DEFAULT,
      double r = MOVE_RADIUS_DEFAULT);
  bool manipulator_get_joints_position(const int dev_id, std::vector<double> & joint_pose);

  bool manipulator_target_reachable(const int dev_id, geometry_msgs::msg::Pose pose);

  bool manipulator_claw_control(const int dev_id,
      const std::vector<double> force,
      const std::vector<double> amplitude);
  bool manipulator_claw_get_status(const int dev_id,
      std::vector<double> & force,
      std::vector<double> & amplitude);

  /* check the manipulator exception status in the arm list. */
  manipulator_interface::ManipulatorException manipulator_get_exception_status(int arm_id);
  void register_exception_notify_callback(std::function<void(int, int err_code)> exception_cb);

private:
  /* get the specific arm interface */
  std::shared_ptr<manipulator_interface::ArmInterface> manipulator_get_interface(int dev_id);
  std::shared_ptr<manipulator_sm::ManipulatorSm> manipulator_get_sm(int dev_id);
  std::shared_ptr<std::mutex> manipulator_get_control_lock(int dev_id);
  /* thread for exception check */
  void create_exception_check_thread();
  void manipulator_destroy_exception();
  void manipulator_exception_check();

  std::map<int, manipulator_management::manipulator_dev> arm_list_;
  std::shared_ptr<std::mutex> arm_list_mutex_;
  /* exception callback*/
  std::function<void(int, int err_code)> exception_notify_function_{ nullptr };
  std::shared_ptr<std::thread> manipulator_exception_thread_;
  bool exception_running_ = true;
};

}  // namespace manipulator_management
}  // namespace qrb

#endif  // QRB_MANIPULATOR_MANIPULATOR_MANAGEMENT_HPP_
