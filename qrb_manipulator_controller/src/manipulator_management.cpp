/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <vector>
//#include <thread>
#include <pthread.h>

#include "qrb_manipulator_controller/lebai_implement.hpp"
#include "qrb_manipulator_controller/manipulator_management.hpp"

namespace qrb
{
namespace manipulator_management
{
#define EXCEPTION_CHECK_DELAY (5) /* Second */

static std::array<ManipulatorConfig, ARM_TYPE_MAX> g_manipulator_configuration = {
  { LEBAI_LM3_CLAW_90, 0, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, "10.20.17.1", 6 },
};

ManipulatorManagement::ManipulatorManagement()
{
  /* init arm list mutex */
  arm_list_mutex_ = std::make_shared<std::mutex>();
  arm_list_.clear();
  this->create_exception_check_thread();
}

ManipulatorManagement::~ManipulatorManagement()
{
  this->manipulator_destroy_exception();
}

bool ManipulatorManagement::add_manipulator(int type, int dev_id, std::string ip)
{
  struct manipulator_dev manipulator;

  // check the dev_id is valid.
  if (type > (int)ARM_TYPE_MAX) {
    std::cerr << "ERROR:  arm type is invalid:" << type << std::endl;
    return false;
  }

  manipulator.configuration = g_manipulator_configuration[type];
  manipulator.control_mutex = std::make_shared<std::mutex>();
  auto arm_type = (ManipulatorType)type;

  switch (arm_type) {
    case LEBAI_LM3_CLAW_90:
      manipulator.arm_interface = std::make_shared<lebai_implement::ARMLebai>();
      manipulator.state_machine = std::make_shared<manipulator_sm::ManipulatorSm>();
      if (false == manipulator.arm_interface->arm_init((const std::string)ip)) {
        std::cerr << "ERROR: robot arm init failed" << std::endl;
        return false;
      }

      /* set tcp */
      manipulator.control_mutex->lock();
      if (false == manipulator.arm_interface->arm_set_tcp(manipulator.configuration.tcp_pose)) {
        std::cerr << "ERROR: robot arm set tcp failed" << std::endl;
        manipulator.control_mutex->unlock();
        return false;
      }
      manipulator.control_mutex->unlock();

      /* set sm as controlling */
      manipulator.state_machine->manipulator_sm_event(
          manipulator_sm::ManipulatorSmEvent::EV_CMD_SWITCH_CONTROLLING);

      break;
    default:
      std::cerr << "ERROR: robot arm type is invalid " << arm_type << std::endl;
      return false;
  }

  manipulator.configuration.dev_id = dev_id;
  manipulator.configuration.ip = ip;
  std::cerr << "DEBUG: add_manipulator add dev= " << dev_id << std::endl;
  /* add manipulator in arm_list */
  arm_list_mutex_->lock();
  arm_list_.insert(std::pair<int, manipulator_management::manipulator_dev>(dev_id, manipulator));
  arm_list_mutex_->unlock();
  std::cerr << "DEBUG: add_manipulator list size= " << arm_list_.size() << std::endl;

  return true;
}

void ManipulatorManagement::manipulator_release()
{
  arm_list_mutex_->lock();
  for (std::map<int, manipulator_management::manipulator_dev>::iterator it = arm_list_.begin();
       it != arm_list_.end(); it++) {
    // get arm obj
    struct manipulator_dev * manipulator = &it->second;
    manipulator->arm_interface->arm_release();
  }
  arm_list_.clear();
  arm_list_mutex_->unlock();
}

std::shared_ptr<manipulator_interface::ArmInterface>
ManipulatorManagement::manipulator_get_interface(int dev_id)
{
  std::map<int, manipulator_management::manipulator_dev>::iterator it;

  it = arm_list_.find(dev_id);
  if (it == arm_list_.end()) {
    std::cerr << "ERROR: robot dev_id is invalid " << dev_id << std::endl;
    return nullptr;
  }

  return it->second.arm_interface;
}

std::shared_ptr<manipulator_sm::ManipulatorSm> ManipulatorManagement::manipulator_get_sm(int dev_id)
{
  std::map<int, manipulator_management::manipulator_dev>::iterator it;

  it = arm_list_.find(dev_id);
  if (it == arm_list_.end()) {
    std::cerr << "ERROR: robot dev_id is invalid " << dev_id << std::endl;
    return nullptr;
  }

  return it->second.state_machine;
}

/***************************************************** exception
 * *****************************************************/
void ManipulatorManagement::register_exception_notify_callback(
    std::function<void(int, int err_code)> exception_cb)
{
  exception_notify_function_ = exception_cb;
}

manipulator_interface::ManipulatorException ManipulatorManagement::manipulator_get_exception_status(
    int arm_id)
{
  std::map<int, manipulator_management::manipulator_dev>::iterator it;

  it = arm_list_.find(arm_id);
  if (it == arm_list_.end()) {
    std::cerr << "ERROR: robot arm_id is invalid " << arm_id << std::endl;
    return manipulator_interface::ManipulatorException::OTHER_ERR;
  }

  return it->second.arm_exception;
}
/*
void ManipulatorManagement::manipulator_exception_check(void *args)
{
    ManipulatorManagement * mm = (ManipulatorManagement*) args;

    while(exception_running_)
    {
        std::this_thread::sleep_for(std::chrono::seconds(EXCEPTION_CHECK_DELAY));

        arm_list_mutex_->lock();
        for (std::map<int arm_id, manipulator_management::manipulator_dev>
                ::iterator it = mm->arm_list_.begin(); it != mm->arm_list_.end(); it++)
        {
            //get arm obj
            struct manipulator_dev *manipulator = &it->second;
            manipulator->arm_exception = manipulator->arm->arm_get_status();

            if (manipulator->arm_exception != NORMAL)
            {
                //modify state machine
                if (mm->exception_notify_function_ != NULL)
                {
                    mm->exception_notify_function_(it->first, manipulator->arm_exception);
                }
            }
        }
        arm_list_mutex_->unlock();
    }
}
*/

void ManipulatorManagement::manipulator_exception_check()
{
  while (exception_running_) {
    std::this_thread::sleep_for(std::chrono::seconds(EXCEPTION_CHECK_DELAY));

    arm_list_mutex_->lock();
    for (std::map<int, manipulator_management::manipulator_dev>::iterator it = arm_list_.begin();
         it != arm_list_.end(); it++) {
      // get arm obj
      struct manipulator_dev * manipulator = &it->second;
      manipulator->arm_exception = manipulator->arm_interface->arm_get_status();

      if (manipulator->arm_exception != manipulator_interface::ManipulatorException::NORMAL) {
        // modify state machine
        if (exception_notify_function_) {
          std::cout << "INFO: exception_notify_function_ start" << std::endl;
          exception_notify_function_(it->first, (int)manipulator->arm_exception);
        }
        std::cout << "INFO: exception_notify_function_  end" << std::endl;
      }
    }
    arm_list_mutex_->unlock();
  }
}

/* create exception thread */
void ManipulatorManagement::create_exception_check_thread(void)
{
  exception_running_ = true;

  auto fun = [this]() -> void {
    this->manipulator_exception_check();
    pthread_setname_np(pthread_self(), "manipulator_exception");
  };

  manipulator_exception_thread_ = std::make_shared<std::thread>(fun);
}

void ManipulatorManagement::manipulator_destroy_exception()
{
  std::cout << "INFO: destroy exception thread" << std::endl;
  exception_running_ = false;
  if (manipulator_exception_thread_ && manipulator_exception_thread_->joinable()) {
    manipulator_exception_thread_->join();
    manipulator_exception_thread_ = nullptr;
  }
}

std::shared_ptr<std::mutex> ManipulatorManagement::manipulator_get_control_lock(int dev_id)
{
  std::map<int, manipulator_management::manipulator_dev>::iterator it;

  it = arm_list_.find(dev_id);
  if (it == arm_list_.end()) {
    std::cerr << "ERROR: robot dev_id is invalid " << dev_id << std::endl;
    return nullptr;
  }

  return it->second.control_mutex;
}

/******************************************** robot ARM control API
 * **************************************************/

bool ManipulatorManagement::manipulator_get_control_mode(const int dev_id, int & mode)
{
  auto arm_interface = this->manipulator_get_interface(dev_id);

  if (nullptr == arm_interface)
    return false;

  mode = (int)arm_interface->arm_get_control_mode();

  return true;
}

bool ManipulatorManagement::manipulator_set_control_mode(const int dev_id, const int mode)
{
  auto state_machine = this->manipulator_get_sm(dev_id);
  manipulator_sm::ManipulatorState current_state = state_machine->get_current_state();
  bool res = false;
  auto control_lock = this->manipulator_get_control_lock(dev_id);

  if (nullptr == state_machine)
    return false;

  if (current_state != manipulator_sm::ST_ERROR) {
    // set mode
    auto arm_interface = this->manipulator_get_interface(dev_id);
    if (nullptr == arm_interface)
      return false;

    control_lock->lock();
    auto update_mode = (manipulator_interface::ManipulatorControlMode)mode;
    res = arm_interface->arm_set_control_mode(update_mode);
    control_lock->unlock();

    // update state
    if (res == true) {
      switch (update_mode) {
        case manipulator_interface::ManipulatorControlMode::ARM_OFFLINE: {
          state_machine->manipulator_sm_event(
              manipulator_sm::ManipulatorSmEvent::EV_CMD_SWITCH_OFFLINING);
          break;
        }
        case manipulator_interface::ManipulatorControlMode::ARM_CONTROLLING: {
          state_machine->manipulator_sm_event(
              manipulator_sm::ManipulatorSmEvent::EV_CMD_SWITCH_CONTROLLING);
          break;
        }
        case manipulator_interface::ManipulatorControlMode::ARM_TEACHNING: {
          state_machine->manipulator_sm_event(
              manipulator_sm::ManipulatorSmEvent::EV_CMD_SWITCH_TEACHING);
          break;
        }
        default:
          std::cerr << "ERROR: manipulator mode  is invalid" << std::endl;
      }
    }
  }

  return res;
}

bool ManipulatorManagement::manipulator_move_tcp(const int dev_id,
    const geometry_msgs::msg::Pose pose,
    double v,
    double a,
    double t,
    double r)
{
  auto state_machine = this->manipulator_get_sm(dev_id);
  manipulator_sm::ManipulatorState current_state = state_machine->get_current_state();
  bool res = false;

  if (nullptr == state_machine)
    return false;

  if (current_state == manipulator_sm::ST_CONTROLLING) {
    auto arm_interface = this->manipulator_get_interface(dev_id);
    if (nullptr == arm_interface)
      return false;

    auto control_lock = this->manipulator_get_control_lock(dev_id);
    control_lock->lock();
    res = arm_interface->arm_move_tcp(pose, v, a, t, r);
    control_lock->unlock();
  } else {
    std::cerr << "ERROR: manipulator state isn't in CONTROLLING" << std::endl;
  }

  return res;
}

bool ManipulatorManagement::manipulator_move_joints(int dev_id,
    const std::vector<double> & joint_pose,
    double v,
    double a,
    double t,
    double r)
{
  auto state_machine = this->manipulator_get_sm(dev_id);
  manipulator_sm::ManipulatorState current_state = state_machine->get_current_state();
  bool res = false;

  if (nullptr == state_machine)
    return false;

  if (current_state == manipulator_sm::ST_CONTROLLING) {
    auto arm_interface = this->manipulator_get_interface(dev_id);
    if (nullptr == arm_interface)
      return false;

    auto control_lock = this->manipulator_get_control_lock(dev_id);
    control_lock->lock();
    res = arm_interface->arm_move_joint(joint_pose, v, a, t, r);
    control_lock->unlock();
  } else {
    std::cerr << "ERROR: manipulator state isn't in CONTROLLING" << std::endl;
  }

  return res;
}

bool ManipulatorManagement::manipulator_get_joints_position(const int dev_id,
    std::vector<double> & joint_pose)
{
  auto state_machine = this->manipulator_get_sm(dev_id);
  manipulator_sm::ManipulatorState current_state = state_machine->get_current_state();
  bool res = false;

  if (nullptr == state_machine)
    return false;

  if (current_state == manipulator_sm::ST_CONTROLLING ||
      current_state == manipulator_sm::ST_TEACHNING) {
    auto arm_interface = this->manipulator_get_interface(dev_id);
    if (nullptr == arm_interface)
      return false;

    auto control_lock = this->manipulator_get_control_lock(dev_id);
    control_lock->lock();
    joint_pose = arm_interface->arm_get_joint_positions();
    control_lock->unlock();
    res = true;
  } else {
    std::cerr << "ERROR: manipulator state isn't in CONTROLLING or TEACHNING" << std::endl;
  }

  return res;
}

bool ManipulatorManagement::manipulator_get_tcp_pose(const int dev_id,
    geometry_msgs::msg::Pose & tcp)
{
  auto state_machine = this->manipulator_get_sm(dev_id);
  manipulator_sm::ManipulatorState current_state = state_machine->get_current_state();
  bool res = false;

  if (nullptr == state_machine)
    return false;

  if (current_state == manipulator_sm::ST_CONTROLLING ||
      current_state == manipulator_sm::ST_TEACHNING) {
    auto arm_interface = this->manipulator_get_interface(dev_id);
    if (nullptr == arm_interface)
      return false;

    auto control_lock = this->manipulator_get_control_lock(dev_id);
    control_lock->lock();
    tcp = arm_interface->arm_get_tcp_pose();
    control_lock->unlock();
    res = true;
  } else {
    std::cerr << "ERROR: manipulator state isn't in CONTROLLING or TEACHNING" << std::endl;
  }

  return res;
}

bool ManipulatorManagement::manipulator_target_reachable(const int dev_id,
    geometry_msgs::msg::Pose pose)
{
  auto state_machine = this->manipulator_get_sm(dev_id);
  manipulator_sm::ManipulatorState current_state = state_machine->get_current_state();
  bool res = false;

  if (nullptr == state_machine)
    return false;

  if (current_state != manipulator_sm::ST_ERROR) {
    auto arm_interface = this->manipulator_get_interface(dev_id);
    if (nullptr == arm_interface)
      return false;

    res = arm_interface->arm_target_reachable(pose);
  } else {
    std::cerr << "ERROR: manipulator state in ERROR " << std::endl;
  }

  return res;
}

bool ManipulatorManagement::manipulator_claw_control(const int dev_id,
    const std::vector<double> force,
    const std::vector<double> amplitude)
{
  auto state_machine = this->manipulator_get_sm(dev_id);
  manipulator_sm::ManipulatorState current_state = state_machine->get_current_state();
  bool res = false;

  if (nullptr == state_machine)
    return false;

  if (current_state == manipulator_sm::ST_CONTROLLING) {
    auto arm_interface = this->manipulator_get_interface(dev_id);
    if (nullptr == arm_interface)
      return false;

    auto control_lock = this->manipulator_get_control_lock(dev_id);
    control_lock->lock();
    res = arm_interface->claw_control(force, amplitude);
    control_lock->unlock();
  } else {
    std::cerr << "ERROR: manipulator claw control failed, state isn't in CONTROLLING" << std::endl;
  }

  return res;
}

bool ManipulatorManagement::manipulator_claw_get_status(const int dev_id,
    std::vector<double> & force,
    std::vector<double> & amplitude)
{
  auto state_machine = this->manipulator_get_sm(dev_id);
  manipulator_sm::ManipulatorState current_state = state_machine->get_current_state();
  bool res = false;

  if (nullptr == state_machine)
    return false;

  if (current_state == manipulator_sm::ST_CONTROLLING ||
      current_state == manipulator_sm::ST_TEACHNING) {
    auto arm_interface = this->manipulator_get_interface(dev_id);
    if (nullptr == arm_interface)
      return false;
    auto control_lock = this->manipulator_get_control_lock(dev_id);
    control_lock->lock();
    res = arm_interface->claw_get_status(force, amplitude);
    control_lock->unlock();
  } else {
    std::cerr << "ERROR: manipulator get claw status failed, state isn't in CONTROLLING or "
                 "TEACHNING"
              << std::endl;
  }

  return res;
}

}  // namespace manipulator_management
}  // namespace qrb
