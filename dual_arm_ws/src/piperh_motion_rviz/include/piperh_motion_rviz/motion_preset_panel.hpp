#ifndef PIPERH_MOTION_RVIZ__MOTION_PRESET_PANEL_HPP_
#define PIPERH_MOTION_RVIZ__MOTION_PRESET_PANEL_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QMap>
#include <QStringList>

#include "moveit_msgs/action/move_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rebotarm_msgs/msg/arm_status.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QProgressBar;
class QPushButton;

namespace piperh_motion_rviz
{

struct MotionPoint
{
  QString name;
  std::array<double, 6> positions{};
  double speed_scale = 0.2;
};

struct PresetWorkspace
{
  std::vector<MotionPoint> points;
  QString active_group_file;
  QString group_name;
  QString last_directory;
  QStringList sequence;
  double sequence_speed_scale = 1.0;
  bool dirty = false;
};

class MotionPresetPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MotionPresetPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void newGroup();
  void saveGroup();
  void openGroup();
  void capturePoint();
  void applyPoint();
  void deletePoint();
  void selectPoint(int index);
  void executeSelected();
  void addSelectedToSequence();
  void useAllPoints();
  void removeSequenceItem();
  void moveSequenceItemUp();
  void moveSequenceItemDown();
  void startSequence();
  void stopExecution();

private:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using MoveGroupGoalHandle = rclcpp_action::ClientGoalHandle<MoveGroup>;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message);
  void selectedRobotCallback(const std_msgs::msg::String::SharedPtr message);
  void activateRobot(const QString & robot);
  void saveWorkspace();
  void restoreWorkspace();
  bool validPositions(const std::array<double, 6> & positions) const;
  QString robotDisplayName() const;
  QString settingsOrganization() const;
  void setStatus(const QString & text, bool error = false);
  void setDirty(bool dirty = true);
  void refreshPointList(const QString & selected = QString());
  int pointIndex(const QString & name) const;
  bool loadGroupFile(const QString & filename);
  bool writeGroupFile(const QString & filename);
  QString suggestedDirectory() const;
  QStringList sequenceNames() const;
  void startExecution(const QStringList & names, bool apply_sequence_speed);
  void sendCurrentTarget(int generation);
  void handleMotionResult(int generation, bool success, const QString & detail);
  bool currentTargetReached(const MotionPoint & point, QString & detail);
  void setEditingEnabled(bool enabled);
  void resetProgress();
  void showProgress(
    int completed, int total, int current_index, const QString & current_name,
    const QString & state, bool error = false);
  void publishProgress(
    const QString & state, int completed, int total, int current_index,
    const QString & current_name, const QString & message = QString());

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_robot_sub_;
  rclcpp::Subscription<rebotarm_msgs::msg::ArmStatus>::SharedPtr piper_arm_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr progress_pub_;
  rclcpp_action::Client<MoveGroup>::SharedPtr move_group_client_;
  std::mutex state_mutex_;
  std::array<double, 6> current_positions_{};
  std::chrono::steady_clock::time_point last_joint_state_{};
  std::chrono::steady_clock::time_point last_piper_arm_status_{};
  bool piper_motor_enabled_ = false;
  bool piper_control_loop_active_ = false;
  MoveGroupGoalHandle::SharedPtr current_goal_;

  QLabel * robot_label_;
  QLabel * active_group_label_;
  QComboBox * point_combo_;
  QLineEdit * point_name_edit_;
  std::array<QDoubleSpinBox *, 6> joint_spins_{};
  QDoubleSpinBox * speed_spin_;
  QDoubleSpinBox * sequence_speed_spin_;
  QListWidget * sequence_list_;
  QLabel * progress_label_;
  QProgressBar * progress_bar_;
  QLabel * status_label_;
  QPushButton * execute_button_;
  QPushButton * start_button_;
  QPushButton * stop_button_;
  std::vector<QWidget *> editing_widgets_;

  std::vector<MotionPoint> points_;
  QMap<QString, PresetWorkspace> workspaces_;
  QString active_group_file_;
  QString group_name_;
  QString last_directory_;
  QString active_robot_ = QStringLiteral("piperh");
  QString rebot_model_ = QStringLiteral("dm");
  QString single_joint_states_topic_ = QStringLiteral("/joint_states");
  QString single_move_action_ = QStringLiteral("/move_action");
  std::array<double, 6> lower_limits_{};
  std::array<double, 6> upper_limits_{};
  bool dual_arm_mode_ = false;
  int piper_driver_speed_percent_ = 25;
  bool active_robot_selected_ = false;
  bool dirty_ = false;
  std::atomic_bool execution_running_{false};
  QStringList active_sequence_;
  double active_sequence_speed_scale_ = 1.0;
  int active_index_ = -1;
  std::atomic<int> execution_generation_{0};
};

}  // namespace piperh_motion_rviz

#endif  // PIPERH_MOTION_RVIZ__MOTION_PRESET_PANEL_HPP_
