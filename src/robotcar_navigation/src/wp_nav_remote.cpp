#include <sstream>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotcar_navigation/nav2_client_utils.hpp"
#include "robotcar_navigation/srv/get_num_of_waypoints.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_index.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_name.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

extern "C" {
#include "UDPServer.h"
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("wp_nav_remote");

  InitUDPServer(20181);

  auto num_client = node->create_client<robotcar_navigation::srv::GetNumOfWaypoints>(
    "waterplus/get_num_waypoint");
  auto index_client = node->create_client<robotcar_navigation::srv::GetWaypointByIndex>(
    "waterplus/get_waypoint_index");
  auto name_client = node->create_client<robotcar_navigation::srv::GetWaypointByName>(
    "waterplus/get_waypoint_name");
  auto behavior_pub = node->create_publisher<std_msgs::msg::String>("wpr1/behaviors", 30);
  auto grab_sub = node->create_subscription<std_msgs::msg::String>(
    "wpr1/grab_result", 30,
    [node](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_WARN(node->get_logger(), "[GrabResultCB] %s", msg->data.c_str());
    });
  auto pass_sub = node->create_subscription<std_msgs::msg::String>(
    "wpr1/pass_result", 30,
    [node](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_WARN(node->get_logger(), "[PassResultCB] %s", msg->data.c_str());
    });
  auto action_client =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose");

  auto num_request = std::make_shared<robotcar_navigation::srv::GetNumOfWaypoints::Request>();
  robotcar_navigation::srv::GetNumOfWaypoints::Response::SharedPtr num_response;
  if (robotcar_navigation::call_service<robotcar_navigation::srv::GetNumOfWaypoints>(
      node, num_client, num_request, num_response))
  {
    for (int i = 0; i < num_response->num; ++i) {
      auto index_request = std::make_shared<robotcar_navigation::srv::GetWaypointByIndex::Request>();
      index_request->index = i;
      robotcar_navigation::srv::GetWaypointByIndex::Response::SharedPtr index_response;
      if (robotcar_navigation::call_service<robotcar_navigation::srv::GetWaypointByIndex>(
          node, index_client, index_request, index_response))
      {
        RCLCPP_INFO(
          node->get_logger(), "Waypoint: %s (%.2f, %.2f)",
          index_response->name.c_str(),
          index_response->pose.position.x,
          index_response->pose.position.y);
      }
    }
  }

  ST_Ctrl control;
  rclcpp::Rate rate(1.0);
  while (rclcpp::ok()) {
    if (GetCtrlCmd(&control)) {
      if (control.ctrl == CTRL_MOVETO_NAME) {
        auto request = std::make_shared<robotcar_navigation::srv::GetWaypointByName::Request>();
        request->name = std::string(1, control.wp_name);
        robotcar_navigation::srv::GetWaypointByName::Response::SharedPtr response;
        if (robotcar_navigation::call_service<robotcar_navigation::srv::GetWaypointByName>(
            node, name_client, request, response) && !response->name.empty())
        {
          robotcar_navigation::navigate_to_pose(node, action_client, response->pose);
        }
      } else if (control.ctrl == CTRL_MOVETO_POS) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = control.x;
        pose.position.y = control.y;
        pose.position.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, control.angle * 3.14159 / 180.0);
        pose.orientation = tf2::toMsg(quat);
        robotcar_navigation::navigate_to_pose(node, action_client, pose);
      } else if (control.ctrl == CTRL_GRAB || control.ctrl == CTRL_PASS) {
        std_msgs::msg::String behavior;
        behavior.data = control.ctrl == CTRL_GRAB ? "grab start" : "pass start";
        behavior_pub->publish(behavior);
      }
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
