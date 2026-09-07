#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("floor_texture_publisher");

  const auto frame_id = node->declare_parameter<std::string>("frame_id", "map");
  auto mesh_resource = node->declare_parameter<std::string>(
    "model", "package://robotcar_navigation/meshes/plane_textured.dae");
  const auto width = node->declare_parameter<double>("width", 4.19);
  const auto height = node->declare_parameter<double>("height", 4.18);
  const auto pos_x = node->declare_parameter<double>("pos_x", -0.004);
  const auto pos_y = node->declare_parameter<double>("pos_y", 0.001);
  const auto pos_z = node->declare_parameter<double>("pos_z", -0.02);
  const auto alpha = node->declare_parameter<double>("alpha", 1.0);

  if (mesh_resource.empty()) {
    RCLCPP_ERROR(node->get_logger(), "No floor model path provided.");
    rclcpp::shutdown();
    return 1;
  }

  if (mesh_resource.rfind("package://", 0) != 0 && mesh_resource.rfind("file://", 0) != 0) {
    mesh_resource = "file://" + mesh_resource;
  }

  const auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  auto marker_pub =
    node->create_publisher<visualization_msgs::msg::Marker>("~/floor_texture", marker_qos);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = node->get_clock()->now();
  marker.ns = "floor_texture";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = pos_x;
  marker.pose.position.y = pos_y;
  marker.pose.position.z = pos_z;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.scale.y = height;
  marker.scale.z = 1.0;
  marker.mesh_resource = mesh_resource;
  marker.mesh_use_embedded_materials = true;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = alpha;

  RCLCPP_INFO(
    node->get_logger(),
    "Publishing floor mesh in frame [%s]: size=%.3f x %.3f, position=(%.3f, %.3f, %.3f), model=%s",
    frame_id.c_str(),
    width,
    height,
    pos_x,
    pos_y,
    pos_z,
    mesh_resource.c_str());

  marker_pub->publish(marker);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
