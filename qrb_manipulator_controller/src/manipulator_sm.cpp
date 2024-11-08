/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "qrb_manipulator_controller/manipulator_sm.hpp"

#include <iostream>
#include <mutex>

namespace qrb
{
namespace manipulator_sm
{
struct manipulator_transform
{
  ManipulatorSmEvent event;
  ManipulatorState next_state;
};

/* state switch array */
static std::array<struct manipulator_transform, ManipulatorSmEvent::EV_MAX_NUMBER>
    manipulator_transitions = { {
        { ManipulatorSmEvent::EV_CMD_SWITCH_OFFLINING, ManipulatorState::ST_OFFLINING },
        { ManipulatorSmEvent::EV_CMD_SWITCH_CONTROLLING, ManipulatorState::ST_CONTROLLING },
        { ManipulatorSmEvent::EV_CMD_SWITCH_TEACHING, ManipulatorState::ST_TEACHNING },
        { ManipulatorSmEvent::EV_EXCEPTION, ManipulatorState::ST_ERROR },
    } };

ManipulatorSm::ManipulatorSm()
{
  state_mutex_ = std::make_shared<std::mutex>();
}

ManipulatorState ManipulatorSm::get_current_state()
{
  return state_;
}

void ManipulatorSm::manipulator_sm_event(ManipulatorSmEvent event)
{
  state_mutex_->lock();
  auto current_state = this->get_current_state();

  if (current_state != ST_ERROR && event < EV_MAX_NUMBER) {
    state_ = manipulator_transitions[event].next_state;
  }

  state_mutex_->unlock();
}

}  // namespace manipulator_sm
}  // namespace qrb
