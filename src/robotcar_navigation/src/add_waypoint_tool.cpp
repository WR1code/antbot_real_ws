#include "add_waypoint_tool.h"

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robotcar_navigation
{
namespace rviz_plugins
{

AddWaypointTool::AddWaypointTool()
{
  shortcut_key_ = 'a';
  name_property_ = new rviz_common::properties::StringProperty(
    "Waypoint Name",
    "",
    "Optional name for the next waypoint. Leave empty to use an automatically generated number.",
    getPropertyContainer());
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic",
    "/waterplus/add_waypoint",
    "Topic on which new waypoints are published.",
    getPropertyContainer(),
    SLOT(updateTopic()),
    this);
}

void AddWaypointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Add Waypoint");

  auto rviz_node = context_->getRosNodeAbstraction().lock();
  if (rviz_node) {
    node_ = rviz_node->get_raw_node();
  }
  updateTopic();
}

void AddWaypointTool::updateTopic()
{
  if (!node_) {
    return;
  }
  publisher_ = node_->create_publisher<robotcar_navigation::msg::Waypoint>(
    topic_property_->getStdString(), rclcpp::QoS(1));
}

void AddWaypointTool::onPoseSet(double x, double y, double theta)
{
  if (!publisher_) {
    return;
  }

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);

  robotcar_navigation::msg::Waypoint waypoint;
  waypoint.name = name_property_->getStdString();
  waypoint.frame_id = context_->getFixedFrame().toStdString();
  waypoint.pose.position.x = x;
  waypoint.pose.position.y = y;
  waypoint.pose.position.z = 0.0;
  waypoint.pose.orientation = tf2::toMsg(quat);

  RCLCPP_INFO(
    node_->get_logger(),
    "Request waypoint [%s] in frame [%s] at (%.3f, %.3f)",
    waypoint.name.empty() ? "auto" : waypoint.name.c_str(), waypoint.frame_id.c_str(), x, y);

  publisher_->publish(waypoint);
}

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

PLUGINLIB_EXPORT_CLASS(robotcar_navigation::rviz_plugins::AddWaypointTool, rviz_common::Tool)
