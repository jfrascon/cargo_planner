/**
 * @file main.cpp
 * @brief Entry point for the cargo_planner ROS 2 node.
 */

#include <rclcpp/rclcpp.hpp>

#include "cargo_planner/cargo_planner_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cargo_planner::CargoPlacerNode>());
  rclcpp::shutdown();
  return 0;
}
