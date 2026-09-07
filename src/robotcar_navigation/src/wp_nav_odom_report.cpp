#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

extern "C" {
#include "UDPClient.h"
}

static int Robot_ID = 1;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("wp_nav_odom_report");
  InitUDPClient((char *)"192.168.1.110", 20180);

  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener listener(buffer);
  rclcpp::Rate rate(10.0);

  while (rclcpp::ok()) {
    try {
      const auto transform = buffer.lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(1.0));
      const double yaw = tf2::getYaw(transform.transform.rotation);
      const double x = transform.transform.translation.x;
      const double y = transform.transform.translation.y;
      RCLCPP_WARN(node->get_logger(), "[Robot Pos]( %.2f , %.2f ) - %.2f", x, y, yaw);
      SendRobotState(Robot_ID, 1, x, y, yaw);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node->get_logger(), "[lookupTransform] %s", ex.what());
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
