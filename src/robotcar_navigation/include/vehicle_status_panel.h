#ifndef ROBOTCAR_NAVIGATION__VEHICLE_STATUS_PANEL_H_
#define ROBOTCAR_NAVIGATION__VEHICLE_STATUS_PANEL_H_

#include <chrono>
#include <memory>

#include <QImage>
#include <QSet>
#include <QString>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class QLabel;
class QDoubleSpinBox;
class QEvent;
class QProgressBar;
class QPushButton;
class QRadioButton;
class QStackedWidget;
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

protected:
  bool eventFilter(QObject * watched, QEvent * event) override;

private Q_SLOTS:
  void updateDataStatus();
  void requestSafetyEnable();
  void requestSafetyStop();
  void publishKeyboardCommand();

private:
  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handleImage(const sensor_msgs::msg::Image::SharedPtr message);
  void handleBattery(const sensor_msgs::msg::BatteryState::SharedPtr message);
  void handleIntent(const std_msgs::msg::String::SharedPtr message);
  void handleExtendedStatus(const std_msgs::msg::String::SharedPtr message);
  void handleOperatorUiStatus(const std_msgs::msg::String::SharedPtr message);
  void selectTeleopMode(bool use_xbox);
  void setMappingEnabled(bool enabled);
  void saveMapping();
  void setButtonMotion(int forward, int left);
  void clearButtonMotion();
  static QImage convertImage(const sensor_msgs::msg::Image & message);
  void showCameraImage(const QImage & image, const QString & encoding);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr intent_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr extended_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operator_ui_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr keyboard_cmd_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr operator_enable_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr teleop_mode_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mapping_enable_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mapping_save_client_;

  QLabel * hardware_status_;
  QPushButton * safety_enable_button_;
  QPushButton * safety_stop_button_;
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
  QRadioButton * xbox_mode_button_;
  QRadioButton * keyboard_mode_button_;
  QLabel * teleop_status_;
  QStackedWidget * control_stack_;
  QLabel * keyboard_capture_;
  QDoubleSpinBox * keyboard_speed_;
  QPushButton * mapping_start_button_;
  QPushButton * mapping_stop_button_;
  QPushButton * mapping_save_button_;
  QLabel * mapping_status_;

  QSet<int> pressed_keys_;
  int button_forward_ = 0;
  int button_left_ = 0;
  bool button_motion_active_ = false;

  bool has_odometry_ = false;
  bool has_camera_ = false;
  bool has_battery_ = false;
  bool has_hardware_status_ = false;
  std::chrono::steady_clock::time_point last_odometry_time_;
  std::chrono::steady_clock::time_point last_camera_time_;
  std::chrono::steady_clock::time_point last_battery_time_;
  std::chrono::steady_clock::time_point last_hardware_status_time_;
};

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

#endif  // ROBOTCAR_NAVIGATION__VEHICLE_STATUS_PANEL_H_
