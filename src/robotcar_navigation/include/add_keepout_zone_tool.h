#ifndef ROBOTCAR_NAVIGATION__ADD_KEEPOUT_ZONE_TOOL_H_
#define ROBOTCAR_NAVIGATION__ADD_KEEPOUT_ZONE_TOOL_H_

#ifndef Q_MOC_RUN
#include <memory>
#include <string>

#include <OgreVector.h>
#include <QObject>

#include "rclcpp/rclcpp.hpp"
#include "robotcar_navigation/msg/keepout_zone.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/tool.hpp"
#endif

namespace rviz_rendering
{
class BillboardLine;
class Shape;
class ViewportProjectionFinder;
}

namespace robotcar_navigation
{
namespace rviz_plugins
{

class AddKeepoutZoneTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  AddKeepoutZoneTool();
  ~AddKeepoutZoneTool() override = default;
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private Q_SLOTS:
  void updateTopic();

private:
  Ogre::Vector3 constrainedEnd(const Ogre::Vector3 & raw_end) const;
  std::string shapeName() const;
  void updatePreview(const Ogre::Vector3 & raw_end);
  void hidePreview();
  void publishZone(const Ogre::Vector3 & raw_end);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<msg::KeepoutZone>::SharedPtr publisher_;
  rviz_common::properties::StringProperty * name_property_;
  rviz_common::properties::EnumProperty * shape_property_;
  rviz_common::properties::FloatProperty * width_property_;
  rviz_common::properties::FloatProperty * height_property_;
  rviz_common::properties::StringProperty * topic_property_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
  std::shared_ptr<rviz_rendering::Shape> rectangle_preview_;
  std::shared_ptr<rviz_rendering::Shape> ellipse_preview_;
  std::shared_ptr<rviz_rendering::BillboardLine> outline_preview_;
  Ogre::Vector3 drag_start_ = Ogre::Vector3::ZERO;
  bool drawing_ = false;
};

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

#endif  // ROBOTCAR_NAVIGATION__ADD_KEEPOUT_ZONE_TOOL_H_
