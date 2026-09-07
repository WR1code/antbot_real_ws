#ifndef ROBOTCAR_NAVIGATION__VEHICLE_STATUS_PANEL_H_
#define ROBOTCAR_NAVIGATION__VEHICLE_STATUS_PANEL_H_

#include <chrono>
#include <memory>

#include <QImage>
#include <QString>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"

class QLabel;
class QProgressBar;
class QTreeWidget;

namespace robotcar_navigation
{
namespace rviz_plugins
{

class VehicleStatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit VehicleStatusPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void updateDataStatus();

private:
  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handleImage(const sensor_msgs::msg::Image::SharedPtr message);
  void handleBattery(const sensor_msgs::msg::BatteryState::SharedPtr message);
  void handleIntent(const std_msgs::msg::String::SharedPtr message);
  void handleExtendedStatus(const std_msgs::msg::String::SharedPtr message);
  static QImage convertImage(const sensor_msgs::msg::Image & message);
  void showCameraImage(const QImage & image, const QString & encoding);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr intent_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr extended_status_sub_;

  QLabel * intent_value_;
  QLabel * intent_detail_;
  QLabel * speed_value_;
  QLabel * velocity_x_value_;
  QLabel * velocity_y_value_;
  QLabel * angular_z_value_;
  QLabel * odometry_status_;
  QProgressBar * battery_bar_;
  QLabel * battery_detail_;
  QTreeWidget * extended_status_;
  QLabel * camera_view_;
  QLabel * camera_status_;

  bool has_odometry_ = false;
  bool has_camera_ = false;
  bool has_battery_ = false;
  std::chrono::steady_clock::time_point last_odometry_time_;
  std::chrono::steady_clock::time_point last_camera_time_;
  std::chrono::steady_clock::time_point last_battery_time_;
};

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

#endif  // ROBOTCAR_NAVIGATION__VEHICLE_STATUS_PANEL_H_
