#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotcar_navigation/nav2_client_utils.hpp"
#include "robotcar_navigation/srv/get_charger_by_name.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_name.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("wp_navi_server");

  bool new_command = false;
  bool charger_command = false;
  std::string waypoint_name;
  auto result_pub = node->create_publisher<std_msgs::msg::String>("waterplus/navi_result", 10);
  auto reached_pub = node->create_publisher<std_msgs::msg::String>("/waypoint_reached", 10);
  auto charge_result_pub = node->create_publisher<std_msgs::msg::String>(
    "/waterplus/charge_result", 10);
  auto dock_request_pub = node->create_publisher<std_msgs::msg::String>(
    "/antbot/dock_request", 10);
  auto waypoint_sub = node->create_subscription<std_msgs::msg::String>(
    "waterplus/navi_waypoint", 10,
    [&waypoint_name, &new_command, &charger_command](
      const std_msgs::msg::String::SharedPtr msg) {
      waypoint_name = msg->data;
      charger_command = false;
      new_command = true;
    });
  auto charger_sub = node->create_subscription<std_msgs::msg::String>(
    "waterplus/navi_charger", 10,
    [&waypoint_name, &new_command, &charger_command](
      const std_msgs::msg::String::SharedPtr msg) {
      waypoint_name = msg->data;
      charger_command = true;
      new_command = true;
    });
  auto waypoint_client = node->create_client<robotcar_navigation::srv::GetWaypointByName>(
    "waterplus/get_waypoint_name");
  auto charger_client = node->create_client<robotcar_navigation::srv::GetChargerByName>(
    "waterplus/get_charger_name");
  auto action_client =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose");

  rclcpp::Rate rate(30);
  while (rclcpp::ok()) {
    if (new_command) {
      bool success = false;
      std::string resolved_name;
      if (charger_command) {
        std_msgs::msg::String dock_request;
        dock_request.data = "{\"command\":\"navigate\",\"charger\":\"" + waypoint_name + "\"}";
        dock_request_pub->publish(dock_request);
        auto request = std::make_shared<robotcar_navigation::srv::GetChargerByName::Request>();
        request->name = waypoint_name;
        robotcar_navigation::srv::GetChargerByName::Response::SharedPtr response;
        if (robotcar_navigation::call_service<robotcar_navigation::srv::GetChargerByName>(
            node, charger_client, request, response) && !response->name.empty())
        {
          resolved_name = response->name;
          RCLCPP_INFO(
            node->get_logger(), "Navigate to charger: %s (%.2f, %.2f)",
            response->name.c_str(), response->pose.position.x, response->pose.position.y);
          success = robotcar_navigation::navigate_to_pose(node, action_client, response->pose);
        }
        std_msgs::msg::String charge_result;
        charge_result.data = success ?
          "{\"success\":true,\"charger\":\"" + resolved_name +
          "\",\"message\":\"已到达充电预停靠位，已请求开始停靠\"}" :
          "{\"success\":false,\"charger\":\"" + waypoint_name +
          "\",\"message\":\"无法到达充电点\"}";
        charge_result_pub->publish(charge_result);
        if (success) {
          dock_request.data = "{\"command\":\"start_docking\",\"charger\":\"" +
            resolved_name + "\"}";
          dock_request_pub->publish(dock_request);
        }
      } else {
        auto request = std::make_shared<robotcar_navigation::srv::GetWaypointByName::Request>();
        request->name = waypoint_name;
        robotcar_navigation::srv::GetWaypointByName::Response::SharedPtr response;
        if (robotcar_navigation::call_service<robotcar_navigation::srv::GetWaypointByName>(
            node, waypoint_client, request, response) && !response->name.empty())
        {
          resolved_name = response->name;
          RCLCPP_INFO(
            node->get_logger(), "Navigate to waypoint: %s (%.2f, %.2f)",
            response->name.c_str(), response->pose.position.x, response->pose.position.y);
          success = robotcar_navigation::navigate_to_pose(node, action_client, response->pose);
        }
        std_msgs::msg::String result;
        result.data = success ? "done" : "failure";
        result_pub->publish(result);
        if (success) {
          std_msgs::msg::String reached;
          reached.data = resolved_name;
          reached_pub->publish(reached);
        }
      }
      new_command = false;
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
