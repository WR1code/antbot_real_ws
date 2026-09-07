#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotcar_navigation/nav2_client_utils.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pose_navi_server");

  bool new_command = false;
  geometry_msgs::msg::Pose goal_pose;
  auto result_pub = node->create_publisher<std_msgs::msg::String>("waterplus/navi_result", 10);
  auto navi_pose_sub = node->create_subscription<geometry_msgs::msg::Pose>(
    "waterplus/navi_pose", 10,
    [&goal_pose, &new_command](const geometry_msgs::msg::Pose::SharedPtr msg) {
      goal_pose = *msg;
      new_command = true;
    });
  auto action_client =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose");

  rclcpp::Rate rate(30);
  while (rclcpp::ok()) {
    if (new_command) {
      const bool success = robotcar_navigation::navigate_to_pose(node, action_client, goal_pose);
      std_msgs::msg::String result;
      result.data = success ? "done" : "failure";
      result_pub->publish(result);
      new_command = false;
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
