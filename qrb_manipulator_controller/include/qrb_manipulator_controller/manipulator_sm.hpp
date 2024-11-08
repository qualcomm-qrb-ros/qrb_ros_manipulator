/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_MANIPULATOR_MANIPULATOR_SM_HPP_
#define QRB_MANIPULATOR_MANIPULATOR_SM_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace qrb
{
namespace manipulator_sm
{
enum ManipulatorState
{
  ST_OFFLINING,
  ST_CONTROLLING,
  ST_TEACHNING,
  ST_ERROR,
};

enum ManipulatorSmEvent
{
  EV_CMD_SWITCH_OFFLINING = 0,
  EV_CMD_SWITCH_CONTROLLING,
  EV_CMD_SWITCH_TEACHING,
  EV_EXCEPTION,
  EV_MAX_NUMBER,
};

class ManipulatorSm
{
public:
  ManipulatorSm();

  virtual ~ManipulatorSm() {}
  ManipulatorState get_current_state();
  void manipulator_sm_event(ManipulatorSmEvent event);

private:
  ManipulatorState state_ = ST_OFFLINING;
  std::shared_ptr<std::mutex> state_mutex_;
};
}  // namespace manipulator_sm
}  // namespace qrb

#endif  // QRB_MANIPULATOR_MANIPULATOR_SM_HPP_
