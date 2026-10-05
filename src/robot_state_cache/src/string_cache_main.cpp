// Section: ROS process entry point for the reusable String sample-and-hold
// adapter.
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_state_cache/string_cache_node.hpp"

int main(int argc, char **argv) {
  // Special function: Initialize, spin, and cleanly release the node under
  // normal or Ctrl+C shutdown.
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_state_cache::StringCacheNode>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_state_cache"),
                 "Startup/runtime failure: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
