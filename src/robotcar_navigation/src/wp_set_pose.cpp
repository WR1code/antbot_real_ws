#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_name.hpp"
#include "std_msgs/msg/string.hpp"

class WaypointSetPoseNode : public rclcpp::Node
{
public:
  WaypointSetPoseNode()
  : Node("wp_set_pose")
  {
    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1).transient_local());
    client_ = create_client<robotcar_navigation::srv::GetWaypointByName>(
      "/waterplus/get_waypoint_name");
    sub_ = create_subscription<std_msgs::msg::String>(
      "/waterplus/set_pose", 1,
      [this](const std_msgs::msg::String::SharedPtr msg) { set_pose_from_waypoint(msg->data); });

    RCLCPP_INFO(get_logger(), "Waiting for waypoint names on /waterplus/set_pose");
  }

private:
  void set_pose_from_waypoint(const std::string & waypoint_name)
  {
    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Service /waterplus/get_waypoint_name is not available");
      return;
    }

    auto request = std::make_shared<robotcar_navigation::srv::GetWaypointByName::Request>();
    request->name = waypoint_name;
    client_->async_send_request(
      request,
      [this, waypoint_name](rclcpp::Client<robotcar_navigation::srv::GetWaypointByName>::SharedFuture future) {
        auto response = future.get();
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = now();
        initial_pose.header.frame_id = "map";
        initial_pose.pose.pose = response->pose;
        initial_pose.pose.covariance[0] = 0.25;
        initial_pose.pose.covariance[7] = 0.25;
        initial_pose.pose.covariance[35] = 0.0685;
        initial_pose_pub_->publish(initial_pose);
        RCLCPP_WARN(get_logger(), "Set robot initial pose to waypoint: %s", waypoint_name.c_str());
      });
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Client<robotcar_navigation::srv::GetWaypointByName>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointSetPoseNode>());
  rclcpp::shutdown();
  return 0;
}
