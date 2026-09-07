#include "add_charger_tool.h"

#include <sstream>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robotcar_navigation
{
namespace rviz_plugins
{

AddChargerTool::AddChargerTool()
{
  shortcut_key_ = 'c';
  name_property_ = new rviz_common::properties::StringProperty(
    "Charger Name", "", "Optional charger name; empty uses charger_N.",
    getPropertyContainer());
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic",
    "/waterplus/add_charger",
    "Topic on which new chargers are published.",
    getPropertyContainer(),
    SLOT(updateTopic()),
    this);
}

void AddChargerTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Add Charger");

  auto rviz_node = context_->getRosNodeAbstraction().lock();
  if (rviz_node) {
    node_ = rviz_node->get_raw_node();
  }
  updateTopic();
}

void AddChargerTool::updateTopic()
{
  if (!node_) {
    return;
  }
  publisher_ = node_->create_publisher<robotcar_navigation::msg::Waypoint>(
    topic_property_->getStdString(), rclcpp::QoS(1));
}

void AddChargerTool::onPoseSet(double x, double y, double theta)
{
  if (!publisher_) {
    return;
  }

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);

  robotcar_navigation::msg::Waypoint charger;
  charger.name = name_property_->getStdString();
  charger.frame_id = context_->getFixedFrame().toStdString();
  charger.pose.position.x = x;
  charger.pose.position.y = y;
  charger.pose.position.z = 0.0;
  charger.pose.orientation = tf2::toMsg(quat);

  RCLCPP_INFO(
    node_->get_logger(),
    "Add charger [%s] in frame [%s] at (%.3f, %.3f)",
    charger.name.empty() ? "auto" : charger.name.c_str(), charger.frame_id.c_str(), x, y);

  publisher_->publish(charger);
}

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

PLUGINLIB_EXPORT_CLASS(robotcar_navigation::rviz_plugins::AddChargerTool, rviz_common::Tool)
