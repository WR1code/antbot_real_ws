#include "add_speed_zone_tool.h"

#include <algorithm>
#include <cmath>

#include <OgreColourValue.h>
#include <OgreSceneNode.h>

#include <QCursor>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

namespace robotcar_navigation
{
namespace rviz_plugins
{

namespace
{
constexpr int kRectangle = 0;
constexpr int kSquare = 1;
constexpr int kEllipse = 2;
constexpr int kCircle = 3;
constexpr int kEllipseSegments = 64;
constexpr float kMinimumSize = 0.05F;
}

AddSpeedZoneTool::AddSpeedZoneTool()
{
  shortcut_key_ = 'v';
  setCursor(QCursor(Qt::CrossCursor));
  name_property_ = new rviz_common::properties::StringProperty(
    "Zone Name", "低速区", "保存限速区时使用的名称。", getPropertyContainer());
  shape_property_ = new rviz_common::properties::EnumProperty(
    "形状", "矩形", "选择常用绘图形状；正方形和圆形会自动保持等宽高。",
    getPropertyContainer());
  shape_property_->addOption("矩形", kRectangle);
  shape_property_->addOption("正方形", kSquare);
  shape_property_->addOption("椭圆", kEllipse);
  shape_property_->addOption("圆形", kCircle);
  width_property_ = new rviz_common::properties::FloatProperty(
    "当前宽度 X (m)", 2.0, "拖动时实时更新，也可用于查看最终尺寸。",
    getPropertyContainer());
  width_property_->setMin(kMinimumSize);
  width_property_->setMax(100.0);
  height_property_ = new rviz_common::properties::FloatProperty(
    "当前高度 Y (m)", 2.0, "拖动时实时更新，也可用于查看最终尺寸。",
    getPropertyContainer());
  height_property_->setMin(kMinimumSize);
  height_property_->setMax(100.0);
  speed_property_ = new rviz_common::properties::FloatProperty(
    "Max Speed (m/s)", 0.3, "区域内允许的最大平移速度。", getPropertyContainer());
  speed_property_->setMin(0.01);
  speed_property_->setMax(1.0);
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/waterplus/add_speed_zone", "Topic used to add speed zones.",
    getPropertyContainer(), SLOT(updateTopic()), this);
}

void AddSpeedZoneTool::onInitialize()
{
  setName("Add Speed Zone");
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
  rectangle_preview_ = std::make_shared<rviz_rendering::Shape>(
    rviz_rendering::Shape::Cube, context_->getSceneManager());
  ellipse_preview_ = std::make_shared<rviz_rendering::Shape>(
    rviz_rendering::Shape::Cylinder, context_->getSceneManager());
  outline_preview_ = std::make_shared<rviz_rendering::BillboardLine>(
    context_->getSceneManager());
  rectangle_preview_->setColor(0.12F, 0.55F, 1.0F, 0.32F);
  ellipse_preview_->setColor(0.12F, 0.55F, 1.0F, 0.32F);
  outline_preview_->setColor(0.10F, 0.70F, 1.0F, 0.98F);
  outline_preview_->setLineWidth(0.045F);
  outline_preview_->setMaxPointsPerLine(kEllipseSegments + 1);
  outline_preview_->setNumLines(1);
  hidePreview();

  auto abstraction = context_->getRosNodeAbstraction().lock();
  if (abstraction) {
    node_ = abstraction->get_raw_node();
  }
  updateTopic();
}

void AddSpeedZoneTool::activate()
{
  drawing_ = false;
  hidePreview();
  setStatus("选择形状和最大速度后，在地图上按住左键从一个角拖到对角；松开即添加。");
}

void AddSpeedZoneTool::deactivate()
{
  drawing_ = false;
  hidePreview();
}

Ogre::Vector3 AddSpeedZoneTool::constrainedEnd(const Ogre::Vector3 & raw_end) const
{
  const int shape = shape_property_->getOptionInt();
  if (shape != kSquare && shape != kCircle) {
    return raw_end;
  }
  const float dx = raw_end.x - drag_start_.x;
  const float dy = raw_end.y - drag_start_.y;
  const float side = std::max(std::abs(dx), std::abs(dy));
  return Ogre::Vector3(
    drag_start_.x + std::copysign(side, dx == 0.0F ? 1.0F : dx),
    drag_start_.y + std::copysign(side, dy == 0.0F ? 1.0F : dy), 0.0F);
}

std::string AddSpeedZoneTool::shapeName() const
{
  switch (shape_property_->getOptionInt()) {
    case kSquare: return "square";
    case kEllipse: return "ellipse";
    case kCircle: return "circle";
    default: return "rectangle";
  }
}

void AddSpeedZoneTool::updatePreview(const Ogre::Vector3 & raw_end)
{
  const Ogre::Vector3 end = constrainedEnd(raw_end);
  const float width = std::abs(end.x - drag_start_.x);
  const float height = std::abs(end.y - drag_start_.y);
  const Ogre::Vector3 center(
    (drag_start_.x + end.x) * 0.5F, (drag_start_.y + end.y) * 0.5F, 0.035F);
  const bool ellipse =
    shape_property_->getOptionInt() == kEllipse || shape_property_->getOptionInt() == kCircle;

  rectangle_preview_->getRootNode()->setVisible(!ellipse);
  ellipse_preview_->getRootNode()->setVisible(ellipse);
  auto & area = ellipse ? ellipse_preview_ : rectangle_preview_;
  area->setPosition(center);
  area->setScale(Ogre::Vector3(std::max(width, 0.001F), std::max(height, 0.001F), 0.06F));

  outline_preview_->getSceneNode()->setVisible(true);
  outline_preview_->clear();
  const Ogre::ColourValue color(0.10F, 0.70F, 1.0F, 0.98F);
  if (ellipse) {
    for (int index = 0; index <= kEllipseSegments; ++index) {
      const double angle = 2.0 * M_PI * index / kEllipseSegments;
      outline_preview_->addPoint(
        Ogre::Vector3(
          center.x + 0.5F * width * std::cos(angle),
          center.y + 0.5F * height * std::sin(angle), 0.075F), color);
    }
  } else {
    for (const Ogre::Vector3 & point : {
        Ogre::Vector3(drag_start_.x, drag_start_.y, 0.075F),
        Ogre::Vector3(end.x, drag_start_.y, 0.075F),
        Ogre::Vector3(end.x, end.y, 0.075F),
        Ogre::Vector3(drag_start_.x, end.y, 0.075F),
        Ogre::Vector3(drag_start_.x, drag_start_.y, 0.075F)})
    {
      outline_preview_->addPoint(point, color);
    }
  }

  if (width >= kMinimumSize) {
    width_property_->setFloat(width);
  }
  if (height >= kMinimumSize) {
    height_property_->setFloat(height);
  }
  setStatus(
    QString("正在绘制：%1 × %2 m，限速 %3 m/s；松开左键完成。")
    .arg(width, 0, 'f', 2).arg(height, 0, 'f', 2)
    .arg(speed_property_->getFloat(), 0, 'f', 2));
}

void AddSpeedZoneTool::hidePreview()
{
  if (rectangle_preview_) {
    rectangle_preview_->getRootNode()->setVisible(false);
  }
  if (ellipse_preview_) {
    ellipse_preview_->getRootNode()->setVisible(false);
  }
  if (outline_preview_) {
    outline_preview_->clear();
    outline_preview_->getSceneNode()->setVisible(false);
  }
}

void AddSpeedZoneTool::publishZone(const Ogre::Vector3 & raw_end)
{
  const Ogre::Vector3 end = constrainedEnd(raw_end);
  const float width = std::abs(end.x - drag_start_.x);
  const float height = std::abs(end.y - drag_start_.y);
  if (width < kMinimumSize || height < kMinimumSize || !publisher_) {
    setStatus("区域太小，未添加；请拖出至少 0.05 m × 0.05 m 的形状。");
    return;
  }
  msg::SpeedZone zone;
  zone.name = name_property_->getStdString();
  zone.frame_id = context_->getFixedFrame().toStdString();
  zone.pose.position.x = (drag_start_.x + end.x) * 0.5F;
  zone.pose.position.y = (drag_start_.y + end.y) * 0.5F;
  zone.pose.orientation.w = 1.0;
  zone.width = width;
  zone.height = height;
  zone.shape = shapeName();
  zone.max_speed = speed_property_->getFloat();
  zone.enabled = true;
  publisher_->publish(zone);
  setStatus(
    QString("已添加限速区：%1 × %2 m，≤ %3 m/s；可继续添加下一个。")
    .arg(width, 0, 'f', 2).arg(height, 0, 'f', 2).arg(zone.max_speed, 0, 'f', 2));
}

int AddSpeedZoneTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  const auto projection = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);
  if (event.leftDown() && projection.first) {
    drag_start_ = projection.second;
    drag_start_.z = 0.0F;
    drawing_ = true;
    updatePreview(projection.second);
    return Render;
  }
  if (drawing_ && projection.first) {
    updatePreview(projection.second);
    if (event.leftUp()) {
      publishZone(projection.second);
      drawing_ = false;
      hidePreview();
    }
    return Render;
  }
  if (drawing_ && event.leftUp()) {
    drawing_ = false;
    hidePreview();
    return Render;
  }
  return 0;
}

void AddSpeedZoneTool::updateTopic()
{
  if (node_) {
    publisher_ = node_->create_publisher<msg::SpeedZone>(
      topic_property_->getStdString(), rclcpp::QoS(10));
  }
}

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

PLUGINLIB_EXPORT_CLASS(
  robotcar_navigation::rviz_plugins::AddSpeedZoneTool,
  rviz_common::Tool)
