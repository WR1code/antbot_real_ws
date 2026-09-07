#ifndef ADD_WAYPOINT_TOOL_H
#define ADD_WAYPOINT_TOOL_H

#ifndef Q_MOC_RUN
#include <QObject>

#include "rclcpp/rclcpp.hpp"
#include "robotcar_navigation/msg/waypoint.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#endif

namespace robotcar_navigation
{
namespace rviz_plugins
{

class AddWaypointTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  AddWaypointTool();
  ~AddWaypointTool() override = default;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<robotcar_navigation::msg::Waypoint>::SharedPtr publisher_;
  rviz_common::properties::StringProperty * name_property_;
  rviz_common::properties::StringProperty * topic_property_;
};

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

#endif  // ADD_WAYPOINT_TOOL_H
