/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <string>
#include <iostream>
#include <cstdlib>

#include "qrb_ros_manipulator/manipulator_controller.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

/**********************************main***************************************/
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto manipulator_ros =
      std::make_shared<qrb_ros::manipulator_controller::ManipulatorController>(options);
  exec.add_node(manipulator_ros);

  if (manipulator_ros->manipulator_run()) {
    exec.spin();
  }

  manipulator_ros->manipulator_shutdown();
  rclcpp::shutdown();
  return 0;
}
