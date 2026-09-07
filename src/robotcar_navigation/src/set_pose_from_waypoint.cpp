#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotcar_navigation/nav2_client_utils.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_name.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("set_pose_from_waypoint_node");

  if (argc < 2) {
    RCLCPP_ERROR(node->get_logger(), "Usage: set_pose_from_waypoint <waypoint_name>");
    rclcpp::shutdown();
    return 1;
  }

  auto client = node->create_client<robotcar_navigation::srv::GetWaypointByName>(
    "/waterplus/get_waypoint_name");
  auto request = std::make_shared<robotcar_navigation::srv::GetWaypointByName::Request>();
  request->name = argv[1];

  robotcar_navigation::srv::GetWaypointByName::Response::SharedPtr response;
  if (!robotcar_navigation::call_service<robotcar_navigation::srv::GetWaypointByName>(
      node, client, request, response))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get waypoint [%s]", request->name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  auto publisher = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(1).transient_local());

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.stamp = node->now();
  initial_pose.header.frame_id = "map";
  initial_pose.pose.pose = response->pose;
  initial_pose.pose.covariance[0] = 0.25;
  initial_pose.pose.covariance[7] = 0.25;
  initial_pose.pose.covariance[35] = 0.0685;

  rclcpp::sleep_for(500ms);
  publisher->publish(initial_pose);
  RCLCPP_WARN(node->get_logger(), "Set initial pose from waypoint: %s", request->name.c_str());

  rclcpp::sleep_for(500ms);
  rclcpp::shutdown();
  return 0;
}
