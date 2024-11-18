/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include <rclcpp/rclcpp.hpp>

#include "trimble_driver_ros/gsof_client_ros.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trmb_ros::GsofClientRos>());
  rclcpp::shutdown();

  return 0;
}
