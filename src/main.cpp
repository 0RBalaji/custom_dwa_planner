#include "custom_dwa_planner/dwa_planner.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DWAPlanner>();
  RCLCPP_INFO(node->get_logger(), "Starting DWA Planner Node...");
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in DWA Planner: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}