#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotcar_navigation/nav2_client_utils.hpp"
#include "robotcar_navigation/srv/get_num_of_waypoints.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_index.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_name.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("wp_nav_test");

  auto num_client = node->create_client<robotcar_navigation::srv::GetNumOfWaypoints>(
    "waterplus/get_num_waypoint");
  auto index_client = node->create_client<robotcar_navigation::srv::GetWaypointByIndex>(
    "waterplus/get_waypoint_index");
  auto action_client =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose");

  int waypoint_index = 0;
  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    auto num_request = std::make_shared<robotcar_navigation::srv::GetNumOfWaypoints::Request>();
    robotcar_navigation::srv::GetNumOfWaypoints::Response::SharedPtr num_response;
    if (!robotcar_navigation::call_service<robotcar_navigation::srv::GetNumOfWaypoints>(
        node, num_client, num_request, num_response))
    {
      break;
    }

    if (num_response->num <= 0) {
      RCLCPP_WARN(node->get_logger(), "No waypoints available");
      rate.sleep();
      continue;
    }
    if (waypoint_index >= num_response->num) {
      waypoint_index = 0;
    }

    auto index_request = std::make_shared<robotcar_navigation::srv::GetWaypointByIndex::Request>();
    index_request->index = waypoint_index;
    robotcar_navigation::srv::GetWaypointByIndex::Response::SharedPtr index_response;
    if (robotcar_navigation::call_service<robotcar_navigation::srv::GetWaypointByIndex>(
        node, index_client, index_request, index_response) && !index_response->name.empty())
    {
      RCLCPP_INFO(
        node->get_logger(), "Go to waypoint[%d]: %s (%.2f, %.2f)",
        waypoint_index, index_response->name.c_str(),
        index_response->pose.position.x, index_response->pose.position.y);
      if (robotcar_navigation::navigate_to_pose(node, action_client, index_response->pose)) {
        waypoint_index++;
      }
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
