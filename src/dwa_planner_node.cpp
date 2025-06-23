#include "dwa_planner.hpp"
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DWAPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
