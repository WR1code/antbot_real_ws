#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robotcar_navigation/nav2_client_utils.hpp"
#include "robotcar_navigation/srv/get_charger_by_name.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("charger_get_position");

  auto client = node->create_client<robotcar_navigation::srv::GetChargerByName>(
    "waterplus/get_charger_name");
  auto request = std::make_shared<robotcar_navigation::srv::GetChargerByName::Request>();
  request->name = argc > 1 ? argv[1] : "c1";

  robotcar_navigation::srv::GetChargerByName::Response::SharedPtr response;
  if (robotcar_navigation::call_service<robotcar_navigation::srv::GetChargerByName>(
      node, client, request, response))
  {
    RCLCPP_INFO(
      node->get_logger(), "Get_charger_name: name = %s (%.2f, %.2f)",
      response->name.c_str(), response->pose.position.x, response->pose.position.y);
  } else {
    RCLCPP_ERROR(node->get_logger(), "No charger named [%s]", request->name.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
