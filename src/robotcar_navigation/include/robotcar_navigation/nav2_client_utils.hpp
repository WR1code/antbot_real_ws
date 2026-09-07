#ifndef ROBOTCAR_NAVIGATION__NAV2_CLIENT_UTILS_HPP_
#define ROBOTCAR_NAVIGATION__NAV2_CLIENT_UTILS_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robotcar_navigation
{

template<typename ServiceT>
bool call_service(
  const rclcpp::Node::SharedPtr & node,
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const typename ServiceT::Request::SharedPtr & request,
  typename ServiceT::Response::SharedPtr & response,
  const std::chrono::seconds timeout = std::chrono::seconds(5))
{
  if (!client->wait_for_service(timeout)) {
    RCLCPP_ERROR(node->get_logger(), "Service [%s] is not available", client->get_service_name());
    return false;
  }

  auto future = client->async_send_request(request);
  const auto status = rclcpp::spin_until_future_complete(node, future, timeout);
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Service call [%s] timed out", client->get_service_name());
    return false;
  }

  response = future.get();
  return true;
}

inline bool navigate_to_pose(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr & client,
  const geometry_msgs::msg::Pose & pose,
  const std::string & frame_id = "map",
  const std::chrono::seconds server_timeout = std::chrono::seconds(10))
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  if (!client->wait_for_action_server(server_timeout)) {
    RCLCPP_ERROR(node->get_logger(), "Nav2 NavigateToPose action server is not available");
    return false;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = frame_id;
  goal.pose.header.stamp = node->now();
  goal.pose.pose = pose;

  auto goal_handle_future = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send NavigateToPose goal");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_WARN(node->get_logger(), "NavigateToPose goal was rejected");
    return false;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed while waiting for NavigateToPose result");
    return false;
  }

  const auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(node->get_logger(), "NavigateToPose finished with non-success result");
    return false;
  }

  return result.result->error_code == NavigateToPose::Result::NONE;
}

}  // namespace robotcar_navigation

#endif  // ROBOTCAR_NAVIGATION__NAV2_CLIENT_UTILS_HPP_
