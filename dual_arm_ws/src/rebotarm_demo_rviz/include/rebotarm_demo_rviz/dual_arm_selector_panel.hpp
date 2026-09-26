#ifndef REBOTARM_DEMO_RVIZ__DUAL_ARM_SELECTOR_PANEL_HPP_
#define REBOTARM_DEMO_RVIZ__DUAL_ARM_SELECTOR_PANEL_HPP_

#include <cstdint>
#include <memory>

#include <QPointer>

#include "action_msgs/srv/cancel_goal.hpp"
#include "rebotarm_msgs/msg/arm_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class QComboBox;
class QDockWidget;
class QLabel;
class QPushButton;
class QStackedWidget;
class QTabWidget;
class QVBoxLayout;
class QWidget;

namespace rebotarm_demo_rviz
{

class DualArmSelectorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DualArmSelectorPanel(QWidget * parent = nullptr);
  ~DualArmSelectorPanel() override;
  void onInitialize() override;

private Q_SLOTS:
  void requestSelection();
  void enablePiperMotors();
  void disablePiperMotors();
  void togglePiperArmed();
  void togglePiperGravity();
  void stopPiperMotion();
  void reloadPiperZones();
  void updateSelected(const QString & robot);
  void organizeDockPages();

private:
  enum class ControlVisualState {Unknown, Pending, Enabled, Disabled, Error};

  void updateArmed(const QString & robot, bool armed);
  void updateStatus(const QString & text);
  void embedFeaturePages();
  void placeRobotControlPages(bool rebot_selected);
  void placePresetPanel(bool rebot_selected);
  void placeForbiddenZonePanel(bool rebot_selected);
  void placePiperSafetyControls(bool piper_selected);
  void restoreEmbeddedPanel(
    QPointer<QDockWidget> & dock, QPointer<QWidget> & content,
    QPointer<QWidget> & placeholder);
  void configurePiperControls(bool selected);
  void applyPiperControlStyles();
  void requestPiperMotorState(bool enabled);
  void updateRebotJointState(const sensor_msgs::msg::JointState::SharedPtr message);
  void updatePiperJointState(const sensor_msgs::msg::JointState::SharedPtr message);

  QComboBox * robot_combo_;
  QPushButton * select_button_;
  QLabel * mode_label_;
  QLabel * handoff_hint_label_;
  QLabel * current_label_;
  QLabel * rebot_state_label_;
  QLabel * piper_state_label_;
  QLabel * status_label_;
  QTabWidget * feature_tabs_;
  QStackedWidget * comprehensive_stack_;
  QWidget * rebot_loading_page_;
  QTabWidget * piper_control_tabs_;
  QWidget * piper_safety_page_;
  QVBoxLayout * piper_safety_layout_;
  QWidget * piper_actions_page_;
  QVBoxLayout * piper_actions_layout_;
  QLabel * piper_actions_placeholder_;
  QWidget * chassis_page_;
  QVBoxLayout * chassis_layout_;
  QTabWidget * chassis_tabs_;
  QLabel * piper_joint_label_;
  QLabel * piper_control_status_label_;
  QPushButton * piper_motor_enable_button_;
  QPushButton * piper_motor_disable_button_;
  QPushButton * piper_armed_button_;
  QPushButton * piper_gravity_button_;
  QPushButton * piper_stop_button_;
  QPushButton * piper_reload_zones_button_;
  QPointer<QWidget> piper_specific_safety_box_;
  QPointer<QDockWidget> rebot_control_dock_;
  QPointer<QWidget> rebot_control_content_;
  QPointer<QWidget> rebot_control_placeholder_;
  QPointer<QTabWidget> shared_control_tabs_;
  QPointer<QWidget> rebot_safety_page_;
  QPointer<QWidget> rebot_actions_page_;
  QPointer<QWidget> forbidden_zone_content_;
  QPointer<QVBoxLayout> rebot_safety_layout_;
  QPointer<QDockWidget> preset_dock_;
  QPointer<QWidget> preset_content_;
  QPointer<QWidget> preset_placeholder_;
  QPointer<QDockWidget> vehicle_control_dock_;
  QPointer<QWidget> vehicle_control_content_;
  QPointer<QWidget> vehicle_control_placeholder_;
  QPointer<QDockWidget> waypoint_dock_;
  QPointer<QWidget> waypoint_content_;
  QPointer<QWidget> waypoint_placeholder_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rebot_armed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr piper_armed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr rebot_joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr piper_joint_sub_;
  rclcpp::Subscription<rebotarm_msgs::msg::ArmStatus>::SharedPtr piper_arm_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr piper_teach_status_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr piper_enable_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr piper_armed_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr piper_gravity_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr piper_gravity_stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr piper_teach_cancel_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr piper_reload_zones_client_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr piper_move_cancel_client_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr piper_execute_cancel_client_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr piper_controller_cancel_client_;
  bool piper_armed_ = false;
  bool piper_armed_known_ = false;
  bool piper_motor_request_pending_ = false;
  bool piper_xbox_request_pending_ = false;
  bool piper_gravity_request_pending_ = false;
  bool piper_gravity_active_ = false;
  bool piper_gravity_transition_ = false;
  bool piper_motor_enabled_ = false;
  bool piper_teach_resting_ = false;
  bool piper_teach_recording_ = false;
  std::int64_t last_piper_arm_status_ms_ = 0;
  ControlVisualState piper_motor_visual_state_ = ControlVisualState::Unknown;
  ControlVisualState piper_xbox_visual_state_ = ControlVisualState::Unknown;
  bool piper_selected_ = false;
  std::int64_t last_piper_feedback_ms_ = 0;
  bool offline_preview_ = false;
  bool rebot_model_ready_ = false;
  bool piper_model_ready_ = false;
  QString selected_robot_;
};

}  // namespace rebotarm_demo_rviz

#endif  // REBOTARM_DEMO_RVIZ__DUAL_ARM_SELECTOR_PANEL_HPP_
