#include "rclcpp/rclcpp.hpp"
#include "robotcar_navigation/waypoint_manager_core.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotcar_navigation::WaypointManagerNode>("wr_manager");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
