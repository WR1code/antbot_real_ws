#include "piperh_motion_rviz/motion_preset_panel.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

#include <QAbstractItemView>
#include <QAbstractItemModel>
#include <QBrush>
#include <QColor>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMetaObject>
#include <QProgressBar>
#include <QPushButton>
#include <QSaveFile>
#include <QSettings>
#include <QScrollArea>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>

#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace piperh_motion_rviz
{
namespace
{

constexpr double kRadiansToDegrees = 180.0 / M_PI;
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kCompletionTolerance = 0.01;
constexpr int kCompletionStableMilliseconds = 500;
constexpr std::array<double, 6> kPiperLower = {
  -2.618, 0.0, -2.96706, -2.356195, -1.56207, -3.14};
constexpr std::array<double, 6> kPiperUpper = {
  2.618, 3.14, 0.0, 2.356195, 1.56207, 3.14};
constexpr std::array<double, 6> kRebotDmLower = {-2.8, -3.14, -3.14, -1.87, -1.57, -3.14};
constexpr std::array<double, 6> kRebotDmUpper = {2.8, 0.005, 0.005, 1.57, 1.57, 3.14};
constexpr std::array<double, 6> kRebotRsLower = {-2.8, 0.0, 0.0, -1.57, -1.57, -3.14};
constexpr std::array<double, 6> kRebotRsUpper = {2.8, 3.14, 3.14, 1.57, 1.57, 3.14};
const std::array<QString, 6> kJointNames = {
  QStringLiteral("joint1"), QStringLiteral("joint2"), QStringLiteral("joint3"),
  QStringLiteral("joint4"), QStringLiteral("joint5"), QStringLiteral("joint6")};

QString jsonError(const QJsonParseError & error)
{
  return error.error == QJsonParseError::NoError ?
         QStringLiteral("根对象或字段格式不正确") : error.errorString();
}

QString ensureJsonSuffix(QString filename, const QString & suffix)
{
  if (filename.endsWith(suffix, Qt::CaseInsensitive)) {
    return filename;
  }
  if (filename.endsWith(QStringLiteral(".json"), Qt::CaseInsensitive)) {
    filename.chop(5);
  }
  return filename + suffix;
}

}  // namespace

MotionPresetPanel::MotionPresetPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto * content = new QWidget(scroll);
  auto * root = new QVBoxLayout(content);
  scroll->setWidget(content);
  outer->addWidget(scroll);

  robot_label_ = new QLabel(tr("当前机械臂：等待选择"), content);
  robot_label_->setStyleSheet(QStringLiteral("font-size: 15px; font-weight: bold; color: #16803a;"));
  root->addWidget(robot_label_);

  auto * group_box = new QGroupBox(tr("预设动作点组"), this);
  auto * group_layout = new QVBoxLayout(group_box);
  active_group_label_ = new QLabel(tr("当前组：未命名（尚未保存）"), group_box);
  active_group_label_->setWordWrap(true);
  auto * group_buttons = new QHBoxLayout();
  auto * new_group_button = new QPushButton(tr("新建"), group_box);
  auto * save_group_button = new QPushButton(tr("命名保存…"), group_box);
  auto * open_group_button = new QPushButton(tr("选择打开…"), group_box);
  group_buttons->addWidget(new_group_button);
  group_buttons->addWidget(save_group_button);
  group_buttons->addWidget(open_group_button);
  group_layout->addWidget(active_group_label_);
  group_layout->addLayout(group_buttons);
  root->addWidget(group_box);

  auto * point_box = new QGroupBox(tr("动作点（6 关节坐标）"), this);
  auto * point_layout = new QVBoxLayout(point_box);
  point_combo_ = new QComboBox(point_box);
  point_name_edit_ = new QLineEdit(point_box);
  point_name_edit_->setPlaceholderText(tr("动作点名称，例如：取料位"));
  point_layout->addWidget(point_combo_);
  point_layout->addWidget(point_name_edit_);

  auto * coordinates = new QGridLayout();
  for (std::size_t index = 0; index < joint_spins_.size(); ++index) {
    auto * label = new QLabel(tr("J%1 (°)").arg(index + 1), point_box);
    auto * spin = new QDoubleSpinBox(point_box);
    spin->setDecimals(3);
    spin->setSingleStep(1.0);
    spin->setRange(
      kPiperLower[index] * kRadiansToDegrees, kPiperUpper[index] * kRadiansToDegrees);
    spin->setSuffix(QStringLiteral("°"));
    joint_spins_[index] = spin;
    coordinates->addWidget(label, static_cast<int>(index / 3) * 2, static_cast<int>(index % 3));
    coordinates->addWidget(spin, static_cast<int>(index / 3) * 2 + 1, static_cast<int>(index % 3));
  }
  point_layout->addLayout(coordinates);

  auto * speed_row = new QHBoxLayout();
  speed_row->addWidget(new QLabel(tr("规划速度："), point_box));
  speed_spin_ = new QDoubleSpinBox(point_box);
  speed_spin_->setRange(1.0, 100.0);
  speed_spin_->setValue(20.0);
  speed_spin_->setDecimals(0);
  speed_spin_->setSuffix(QStringLiteral("%"));
  speed_row->addWidget(speed_spin_);
  speed_row->addStretch(1);
  point_layout->addLayout(speed_row);

  auto * capture_button = new QPushButton(tr("采集当前关节坐标并添加"), point_box);
  auto * point_buttons = new QHBoxLayout();
  auto * apply_button = new QPushButton(tr("应用修改/重命名"), point_box);
  auto * delete_button = new QPushButton(tr("删除"), point_box);
  execute_button_ = new QPushButton(tr("执行选中动作点"), point_box);
  point_buttons->addWidget(apply_button);
  point_buttons->addWidget(delete_button);
  point_buttons->addWidget(execute_button_);
  point_layout->addWidget(capture_button);
  point_layout->addLayout(point_buttons);
  root->addWidget(point_box);

  auto * sequence_box = new QGroupBox(tr("动作组执行顺序（随动作组一起保存）"), this);
  auto * sequence_layout = new QVBoxLayout(sequence_box);
  sequence_list_ = new QListWidget(sequence_box);
  sequence_list_->setObjectName(QStringLiteral("motion_sequence_list"));
  sequence_list_->setDragDropMode(QAbstractItemView::InternalMove);
  sequence_list_->setDefaultDropAction(Qt::MoveAction);
  sequence_list_->setSelectionMode(QAbstractItemView::SingleSelection);
  // This panel is embedded in another resizable scroll area. Without a cap,
  // QListWidget takes all remaining dock height and looks like a large blank
  // page when the sequence is empty, pushing the controls below off-screen.
  sequence_list_->setMinimumHeight(72);
  sequence_list_->setMaximumHeight(168);
  sequence_list_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  sequence_layout->addWidget(sequence_list_);

  auto * sequence_add_row = new QHBoxLayout();
  auto * add_button = new QPushButton(tr("添加选中"), sequence_box);
  auto * all_button = new QPushButton(tr("使用全部"), sequence_box);
  auto * remove_button = new QPushButton(tr("移除"), sequence_box);
  sequence_add_row->addWidget(add_button);
  sequence_add_row->addWidget(all_button);
  sequence_add_row->addWidget(remove_button);
  sequence_layout->addLayout(sequence_add_row);

  auto * sequence_move_row = new QHBoxLayout();
  auto * up_button = new QPushButton(tr("上移"), sequence_box);
  auto * down_button = new QPushButton(tr("下移"), sequence_box);
  sequence_move_row->addWidget(up_button);
  sequence_move_row->addWidget(down_button);
  sequence_layout->addLayout(sequence_move_row);

  auto * sequence_speed_row = new QHBoxLayout();
  sequence_speed_row->addWidget(new QLabel(tr("播放速度（相对动作点）："), sequence_box));
  sequence_speed_spin_ = new QDoubleSpinBox(sequence_box);
  sequence_speed_spin_->setObjectName(QStringLiteral("motion_sequence_speed"));
  sequence_speed_spin_->setRange(10.0, 500.0);
  sequence_speed_spin_->setSingleStep(10.0);
  sequence_speed_spin_->setDecimals(0);
  sequence_speed_spin_->setSuffix(QStringLiteral("%"));
  sequence_speed_spin_->setValue(100.0);
  sequence_speed_spin_->setToolTip(tr("每个动作点的实际规划速度 = 动作点速度 × 播放速度，最高 100%"));
  sequence_speed_row->addWidget(sequence_speed_spin_);
  sequence_speed_row->addStretch(1);
  sequence_layout->addLayout(sequence_speed_row);

  auto * progress_box = new QGroupBox(tr("动作进度"), sequence_box);
  auto * progress_layout = new QVBoxLayout(progress_box);
  progress_label_ = new QLabel(tr("未开始"), progress_box);
  progress_label_->setWordWrap(true);
  progress_bar_ = new QProgressBar(progress_box);
  progress_bar_->setRange(0, 1);
  progress_bar_->setValue(0);
  progress_bar_->setFormat(tr("0 / 0（0%）"));
  progress_layout->addWidget(progress_label_);
  progress_layout->addWidget(progress_bar_);
  sequence_layout->addWidget(progress_box);

  auto * run_row = new QHBoxLayout();
  start_button_ = new QPushButton(tr("开始动作组"), sequence_box);
  stop_button_ = new QPushButton(tr("停止"), sequence_box);
  stop_button_->setEnabled(false);
  run_row->addWidget(start_button_);
  run_row->addWidget(stop_button_);
  sequence_layout->addLayout(run_row);
  root->addWidget(sequence_box);

  status_label_ = new QLabel(tr("等待 ROS 初始化…"), this);
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);
  root->addStretch(1);

  editing_widgets_ = {
    new_group_button, save_group_button, open_group_button, point_combo_, point_name_edit_,
    capture_button, apply_button, delete_button, add_button, all_button, remove_button,
    up_button, down_button, sequence_list_};

  connect(new_group_button, &QPushButton::clicked, this, &MotionPresetPanel::newGroup);
  connect(save_group_button, &QPushButton::clicked, this, &MotionPresetPanel::saveGroup);
  connect(open_group_button, &QPushButton::clicked, this, &MotionPresetPanel::openGroup);
  connect(capture_button, &QPushButton::clicked, this, &MotionPresetPanel::capturePoint);
  connect(apply_button, &QPushButton::clicked, this, &MotionPresetPanel::applyPoint);
  connect(delete_button, &QPushButton::clicked, this, &MotionPresetPanel::deletePoint);
  connect(point_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, &MotionPresetPanel::selectPoint);
  connect(execute_button_, &QPushButton::clicked, this, &MotionPresetPanel::executeSelected);
  connect(add_button, &QPushButton::clicked, this, &MotionPresetPanel::addSelectedToSequence);
  connect(all_button, &QPushButton::clicked, this, &MotionPresetPanel::useAllPoints);
  connect(remove_button, &QPushButton::clicked, this, &MotionPresetPanel::removeSequenceItem);
  connect(up_button, &QPushButton::clicked, this, &MotionPresetPanel::moveSequenceItemUp);
  connect(down_button, &QPushButton::clicked, this, &MotionPresetPanel::moveSequenceItemDown);
  connect(sequence_list_->model(), &QAbstractItemModel::rowsMoved, this, [this]() {
    setDirty();
    resetProgress();
  });
  connect(start_button_, &QPushButton::clicked, this, &MotionPresetPanel::startSequence);
  connect(stop_button_, &QPushButton::clicked, this, &MotionPresetPanel::stopExecution);
  connect(sequence_speed_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, [this](double) {setDirty();});

  lower_limits_ = kPiperLower;
  upper_limits_ = kPiperUpper;
}

void MotionPresetPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus(tr("无法取得 RViz ROS 节点"), true);
    return;
  }
  node_ = abstraction->get_raw_node();
  const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  piper_arm_status_sub_ = node_->create_subscription<rebotarm_msgs::msg::ArmStatus>(
    "/piperh/arm_status", status_qos,
    [this](const rebotarm_msgs::msg::ArmStatus::SharedPtr message) {
      const bool enabled = message->enabled;
      const bool control_loop_active = message->control_loop_active;
      QMetaObject::invokeMethod(this, [this, enabled, control_loop_active]() {
        piper_motor_enabled_ = enabled;
        piper_control_loop_active_ = control_loop_active;
        last_piper_arm_status_ = std::chrono::steady_clock::now();
      }, Qt::QueuedConnection);
    });
  const auto string_parameter = [this](const std::string & name, const std::string & fallback) {
      return node_->has_parameter(name) ? node_->get_parameter(name).as_string() :
             node_->declare_parameter<std::string>(name, fallback);
    };
  dual_arm_mode_ = node_->has_parameter("motion_preset.dual_arm") ?
    node_->get_parameter("motion_preset.dual_arm").as_bool() :
    node_->declare_parameter<bool>("motion_preset.dual_arm", false);
  piper_driver_speed_percent_ = node_->has_parameter("motion_preset.piper_driver_speed_percent") ?
    static_cast<int>(node_->get_parameter("motion_preset.piper_driver_speed_percent").as_int()) :
    static_cast<int>(node_->declare_parameter<int64_t>(
      "motion_preset.piper_driver_speed_percent", 25));
  rebot_model_ = QString::fromStdString(
    string_parameter("motion_preset.rebot_model", "dm")).trimmed().toLower();
  single_joint_states_topic_ = QString::fromStdString(
    string_parameter("piperh_motion.joint_states_topic", "/joint_states")).trimmed();
  single_move_action_ = QString::fromStdString(
    string_parameter("piperh_motion.move_action", "/move_action")).trimmed();
  QString initial_robot = QString::fromStdString(
    string_parameter("motion_preset.initial_robot", "piperh")).trimmed().toLower();
  if (initial_robot != QStringLiteral("rebotarm") && initial_robot != QStringLiteral("piperh")) {
    initial_robot = QStringLiteral("piperh");
  }
  activateRobot(initial_robot);
  if (dual_arm_mode_) {
    auto selected_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    selected_robot_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/dual_arm/selected", selected_qos,
      [this](const std_msgs::msg::String::SharedPtr message) {selectedRobotCallback(message);});
  }
}

void MotionPresetPanel::selectedRobotCallback(const std_msgs::msg::String::SharedPtr message)
{
  const QString robot = QString::fromStdString(message->data).trimmed().toLower();
  QMetaObject::invokeMethod(this, [this, robot]() {
    if (robot == QStringLiteral("none")) {
      if (execution_running_) {stopExecution();}
      saveWorkspace();
      active_robot_selected_ = false;
      setEditingEnabled(false);
      start_button_->setEnabled(false);
      execute_button_->setEnabled(false);
      robot_label_->setText(tr("当前机械臂：安全切换中"));
      setStatus(tr("两台机械臂均已锁定；等待控制权切换完成"));
      return;
    }
    if (robot == QStringLiteral("rebotarm") || robot == QStringLiteral("piperh")) {
      activateRobot(robot);
    }
  }, Qt::QueuedConnection);
}

void MotionPresetPanel::activateRobot(const QString & robot)
{
  if (!node_ || (robot != QStringLiteral("rebotarm") && robot != QStringLiteral("piperh"))) {
    return;
  }
  if (execution_running_) {stopExecution();}
  if (active_robot_selected_ && !active_robot_.isEmpty()) {saveWorkspace();}
  active_robot_ = robot;
  active_robot_selected_ = true;
  // Match the available playback range to the configured driver speed.
  const double piper_speed_max = piper_driver_speed_percent_ >= 25 ? 500.0 :
    (piper_driver_speed_percent_ >= 10 ? 300.0 : 100.0);
  sequence_speed_spin_->setMaximum(
    active_robot_ == QStringLiteral("piperh") ? piper_speed_max : 200.0);

  if (active_robot_ == QStringLiteral("piperh")) {
    lower_limits_ = kPiperLower;
    upper_limits_ = kPiperUpper;
  } else if (rebot_model_ == QStringLiteral("rs")) {
    lower_limits_ = kRebotRsLower;
    upper_limits_ = kRebotRsUpper;
  } else {
    lower_limits_ = kRebotDmLower;
    upper_limits_ = kRebotDmUpper;
  }
  for (std::size_t index = 0; index < joint_spins_.size(); ++index) {
    joint_spins_[index]->setRange(
      lower_limits_[index] * kRadiansToDegrees,
      upper_limits_[index] * kRadiansToDegrees);
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_joint_state_ = {};
    current_goal_.reset();
  }
  const std::string prefix = "/" + active_robot_.toStdString();
  const std::string joint_states_topic = dual_arm_mode_ ?
    prefix + "/joint_states" : single_joint_states_topic_.toStdString();
  const std::string move_action = dual_arm_mode_ ?
    prefix + "/move_action" : single_move_action_.toStdString();
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      jointStateCallback(message);
    });
  progress_pub_ = node_->create_publisher<std_msgs::msg::String>(prefix + "/motion_progress", 10);
  move_group_client_ = rclcpp_action::create_client<MoveGroup>(node_, move_action);
  restoreWorkspace();
  setEditingEnabled(true);
  start_button_->setEnabled(true);
  execute_button_->setEnabled(true);
  stop_button_->setEnabled(false);
  robot_label_->setText(tr("当前机械臂：%1").arg(robotDisplayName()));
  setStatus(tr("%1 已连接 %2；等待 %3").arg(
      robotDisplayName(), QString::fromStdString(joint_states_topic),
      QString::fromStdString(move_action)));
}

void MotionPresetPanel::saveWorkspace()
{
  PresetWorkspace workspace;
  workspace.points = points_;
  workspace.active_group_file = active_group_file_;
  workspace.group_name = group_name_;
  workspace.last_directory = last_directory_;
  workspace.sequence = sequenceNames();
  workspace.sequence_speed_scale = sequence_speed_spin_->value() / 100.0;
  workspace.dirty = dirty_;
  workspaces_[active_robot_] = std::move(workspace);
}

void MotionPresetPanel::restoreWorkspace()
{
  if (workspaces_.contains(active_robot_)) {
    const auto & workspace = workspaces_[active_robot_];
    points_ = workspace.points;
    active_group_file_ = workspace.active_group_file;
    group_name_ = workspace.group_name;
    last_directory_ = workspace.last_directory;
    dirty_ = workspace.dirty;
    sequence_list_->clear();
    sequence_list_->addItems(workspace.sequence);
    sequence_speed_spin_->blockSignals(true);
    sequence_speed_spin_->setValue(workspace.sequence_speed_scale * 100.0);
    sequence_speed_spin_->blockSignals(false);
    if (workspace.sequence_speed_scale * 100.0 > sequence_speed_spin_->maximum()) {
      dirty_ = true;
    }
  } else {
    points_.clear();
    active_group_file_.clear();
    group_name_.clear();
    dirty_ = false;
    QSettings settings(settingsOrganization(), QStringLiteral("motion_preset_panel"));
    last_directory_ = settings.value(QStringLiteral("last_directory"), QDir::homePath()).toString();
    sequence_list_->clear();
    sequence_speed_spin_->blockSignals(true);
    sequence_speed_spin_->setValue(100.0);
    sequence_speed_spin_->blockSignals(false);
  }
  refreshPointList();
  setDirty(dirty_);
  resetProgress();
}

bool MotionPresetPanel::validPositions(const std::array<double, 6> & positions) const
{
  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (!std::isfinite(positions[index]) || positions[index] < lower_limits_[index] ||
      positions[index] > upper_limits_[index])
    {
      return false;
    }
  }
  return true;
}

QString MotionPresetPanel::robotDisplayName() const
{
  return active_robot_ == QStringLiteral("rebotarm") ? QStringLiteral("reBotArm") :
         QStringLiteral("Piper-H");
}

QString MotionPresetPanel::settingsOrganization() const
{
  return active_robot_ == QStringLiteral("rebotarm") ? QStringLiteral("rebotarm") :
         QStringLiteral("piperh");
}

void MotionPresetPanel::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (message->name.size() != message->position.size()) {
    return;
  }
  std::map<std::string, double> values;
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    values[message->name[index]] = message->position[index];
  }
  std::array<double, 6> ordered{};
  for (std::size_t index = 0; index < ordered.size(); ++index) {
    const auto iterator = values.find(kJointNames[index].toStdString());
    if (iterator == values.end() || !std::isfinite(iterator->second)) {
      return;
    }
    ordered[index] = iterator->second;
  }
  if (!validPositions(ordered)) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_positions_ = ordered;
  last_joint_state_ = std::chrono::steady_clock::now();
}

void MotionPresetPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "color: #e35d6a;" : "");
}

void MotionPresetPanel::setDirty(bool dirty)
{
  dirty_ = dirty;
  const QString file = active_group_file_.isEmpty() ? tr("未命名（尚未保存）") : active_group_file_;
  active_group_label_->setText(tr("当前组：%1%2").arg(file, dirty_ ? QStringLiteral(" *") : QString()));
}

void MotionPresetPanel::newGroup()
{
  points_.clear();
  active_group_file_.clear();
  group_name_.clear();
  sequence_list_->clear();
  sequence_speed_spin_->setValue(100.0);
  refreshPointList();
  setDirty(false);
  resetProgress();
  setStatus(tr("已新建空动作点组"));
}

QString MotionPresetPanel::suggestedDirectory() const
{
  if (!active_group_file_.isEmpty()) {
    return QFileInfo(active_group_file_).absolutePath();
  }
  return last_directory_.isEmpty() ? QDir::homePath() : last_directory_;
}

void MotionPresetPanel::saveGroup()
{
  QString filename = QFileDialog::getSaveFileName(
    this, tr("命名保存预设动作点组"), suggestedDirectory() + "/motion_group.motion.json",
    tr("%1 动作点组 (*.motion.json)").arg(robotDisplayName()));
  if (filename.isEmpty()) {
    return;
  }
  filename = ensureJsonSuffix(filename, QStringLiteral(".motion.json"));
  if (writeGroupFile(filename)) {
    active_group_file_ = QFileInfo(filename).absoluteFilePath();
    group_name_ = QFileInfo(filename).completeBaseName();
    last_directory_ = QFileInfo(filename).absolutePath();
    QSettings(settingsOrganization(), QStringLiteral("motion_preset_panel")).setValue(
      QStringLiteral("last_directory"), last_directory_);
    setDirty(false);
    setStatus(tr("动作点组已保存：%1").arg(active_group_file_));
  }
}

bool MotionPresetPanel::writeGroupFile(const QString & filename)
{
  QJsonArray joints;
  for (const auto & name : kJointNames) {
    joints.append(name);
  }
  QJsonArray points;
  for (const auto & point : points_) {
    QJsonArray positions;
    for (double value : point.positions) {
      positions.append(value);
    }
    QJsonObject object;
    object["name"] = point.name;
    object["positions"] = positions;
    object["speed_scale"] = point.speed_scale;
    points.append(object);
  }
  QJsonObject root;
  root["version"] = 2;
  root["robot"] = active_robot_;
  if (active_robot_ == QStringLiteral("rebotarm")) {
    root["robot_model"] = rebot_model_;
  }
  root["name"] = QFileInfo(filename).completeBaseName();
  root["joints"] = joints;
  root["points"] = points;
  QJsonArray sequence;
  for (const auto & name : sequenceNames()) {
    sequence.append(name);
  }
  root["sequence"] = sequence;
  root["sequence_speed_scale"] = sequence_speed_spin_->value() / 100.0;

  QSaveFile output(filename);
  if (!output.open(QIODevice::WriteOnly)) {
    setStatus(tr("无法写入动作点组：%1").arg(output.errorString()), true);
    return false;
  }
  output.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
  if (!output.commit()) {
    setStatus(tr("无法提交动作点组文件：%1").arg(output.errorString()), true);
    return false;
  }
  return true;
}

void MotionPresetPanel::openGroup()
{
  const QString filename = QFileDialog::getOpenFileName(
    this, tr("选择预设动作点组"), suggestedDirectory(),
    tr("%1 动作点组 (*.motion.json);;兼容旧版 JSON (*.json)").arg(robotDisplayName()));
  if (!filename.isEmpty()) {
    loadGroupFile(filename);
  }
}

bool MotionPresetPanel::loadGroupFile(const QString & filename)
{
  QFile input(filename);
  if (!input.open(QIODevice::ReadOnly)) {
    setStatus(tr("无法读取动作点组：%1").arg(input.errorString()), true);
    return false;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(input.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    setStatus(tr("动作点组 JSON 无效：%1").arg(jsonError(parse_error)), true);
    return false;
  }
  const QJsonObject root = document.object();
  if (root.value("motion_group").isString()) {
    const int legacy_version = root.value("version").toInt();
    const QString expected_group = root.value("motion_group").toString();
    if ((legacy_version != 1 && legacy_version != 2) ||
      !root.value("points").isArray() || expected_group.isEmpty())
    {
      setStatus(tr("旧版动作顺序组格式无效"), true);
      return false;
    }
    if ((legacy_version == 1 && active_robot_ != QStringLiteral("piperh")) ||
      (legacy_version == 2 && root.value("robot").toString() != active_robot_) ||
      (legacy_version == 2 && active_robot_ == QStringLiteral("rebotarm") &&
      !root.value("robot_model").toString().isEmpty() &&
      root.value("robot_model").toString() != rebot_model_))
    {
      setStatus(tr("旧版动作顺序组与当前机械臂或型号不匹配"), true);
      return false;
    }
    if (QFileInfo(expected_group).absoluteFilePath() == QFileInfo(filename).absoluteFilePath()) {
      setStatus(tr("该旧文件曾覆盖其动作点组，原关节坐标已不在文件中"), true);
      return false;
    }
    if (!loadGroupFile(expected_group)) {
      return false;
    }
    QStringList imported_sequence;
    for (const auto & value : root.value("points").toArray()) {
      const QString name = value.toString().trimmed();
      if (name.isEmpty() || pointIndex(name) < 0) {
        setStatus(tr("旧版顺序引用了不存在的动作点“%1”").arg(name), true);
        return false;
      }
      imported_sequence.append(name);
    }
    sequence_list_->clear();
    sequence_list_->addItems(imported_sequence);
    setDirty();
    resetProgress();
    setStatus(tr("已导入旧版顺序，共 %1 步；请保存为新的动作组文件").arg(
        imported_sequence.size()));
    return true;
  }
  if (root.value("format").toString() == QStringLiteral("rebot_teach_trajectory")) {
    setStatus(tr("选中的是拖动示教轨迹，不是预设动作点组"), true);
    return false;
  }
  const QJsonArray joints = root.value("joints").toArray();
  const QJsonArray json_points = root.value("points").toArray();
  const int version = root.value("version").toInt();
  if ((version != 1 && version != 2) || joints.size() != 6 ||
    !root.value("points").isArray())
  {
    setStatus(tr("动作点组版本或字段不受支持"), true);
    return false;
  }
  if (version == 1 && active_robot_ != QStringLiteral("piperh")) {
    setStatus(tr("旧版动作点组仅可用于 Piper-H；请勿跨机械臂执行"), true);
    return false;
  }
  if (version == 2 && root.value("robot").toString() != active_robot_) {
    setStatus(tr("该动作点组属于另一台机械臂，已拒绝加载"), true);
    return false;
  }
  if (version == 2 && active_robot_ == QStringLiteral("rebotarm") &&
    !root.value("robot_model").toString().isEmpty() &&
    root.value("robot_model").toString() != rebot_model_)
  {
    setStatus(tr("该动作点组属于另一种 reBotArm 型号，已拒绝加载"), true);
    return false;
  }
  for (int index = 0; index < joints.size(); ++index) {
    if (joints.at(index).toString() != kJointNames[static_cast<std::size_t>(index)]) {
      setStatus(tr("动作点组关节顺序必须是 joint1 到 joint6"), true);
      return false;
    }
  }

  std::vector<MotionPoint> loaded;
  std::set<QString> names;
  for (const auto & value : json_points) {
    if (!value.isObject()) {
      setStatus(tr("动作点组中存在非对象条目"), true);
      return false;
    }
    const QJsonObject object = value.toObject();
    MotionPoint point;
    point.name = object.value("name").toString().trimmed();
    const QJsonArray positions = object.value("positions").toArray();
    point.speed_scale = object.value("speed_scale").toDouble(0.2);
    if (point.name.isEmpty() || names.count(point.name) > 0 || positions.size() != 6 ||
      !std::isfinite(point.speed_scale) || point.speed_scale < 0.01 || point.speed_scale > 1.0)
    {
      setStatus(tr("动作点名称、坐标数量或速度比例无效"), true);
      return false;
    }
    for (int index = 0; index < positions.size(); ++index) {
      if (!positions.at(index).isDouble()) {
        setStatus(tr("动作点“%1”含有非数字坐标").arg(point.name), true);
        return false;
      }
      point.positions[static_cast<std::size_t>(index)] = positions.at(index).toDouble();
    }
    if (!validPositions(point.positions)) {
      setStatus(tr("动作点“%1”超出 %2 关节限位").arg(point.name, robotDisplayName()), true);
      return false;
    }
    names.insert(point.name);
    loaded.push_back(point);
  }

  QStringList loaded_sequence;
  const QJsonValue sequence_speed_value = root.value("sequence_speed_scale");
  if (!sequence_speed_value.isUndefined() && !sequence_speed_value.isDouble()) {
    setStatus(tr("动作组播放速度格式无效"), true);
    return false;
  }
  const double sequence_speed_scale = sequence_speed_value.isUndefined() ?
    1.0 : sequence_speed_value.toDouble();
  if (!std::isfinite(sequence_speed_scale) || sequence_speed_scale < 0.1 ||
    sequence_speed_scale > 5.0)
  {
    setStatus(tr("动作组播放速度必须在 10%–500% 之间"), true);
    return false;
  }
  if (root.contains("sequence")) {
    if (!root.value("sequence").isArray()) {
      setStatus(tr("动作组执行顺序字段格式无效"), true);
      return false;
    }
    for (const auto & value : root.value("sequence").toArray()) {
      const QString name = value.toString().trimmed();
      if (name.isEmpty() || names.count(name) == 0) {
        setStatus(tr("动作组执行顺序引用了不存在的动作点“%1”").arg(name), true);
        return false;
      }
      loaded_sequence.append(name);
    }
  } else {
    for (const auto & point : loaded) {
      loaded_sequence.append(point.name);
    }
  }

  points_ = std::move(loaded);
  active_group_file_ = QFileInfo(filename).absoluteFilePath();
  group_name_ = root.value("name").toString(QFileInfo(filename).completeBaseName());
  last_directory_ = QFileInfo(filename).absolutePath();
  QSettings(settingsOrganization(), QStringLiteral("motion_preset_panel")).setValue(
    QStringLiteral("last_directory"), last_directory_);
  sequence_list_->clear();
  sequence_list_->addItems(loaded_sequence);
  sequence_speed_spin_->setValue(sequence_speed_scale * 100.0);
  const bool speed_limited = sequence_speed_scale * 100.0 > sequence_speed_spin_->maximum();
  refreshPointList();
  setDirty(speed_limited);
  resetProgress();
  setStatus(speed_limited ?
    tr("已打开动作组；当前 %1 播放速度上限为 %2%，请重新保存动作组")
    .arg(robotDisplayName()).arg(sequence_speed_spin_->maximum(), 0, 'f', 0) :
    tr("已打开动作组：%1 个动作点，%2 个顺序步骤").arg(
      points_.size()).arg(loaded_sequence.size()));
  return true;
}

int MotionPresetPanel::pointIndex(const QString & name) const
{
  for (std::size_t index = 0; index < points_.size(); ++index) {
    if (points_[index].name == name) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

void MotionPresetPanel::refreshPointList(const QString & selected)
{
  const QString desired = selected.isEmpty() ? point_combo_->currentText() : selected;
  point_combo_->blockSignals(true);
  point_combo_->clear();
  for (const auto & point : points_) {
    point_combo_->addItem(point.name);
  }
  const int index = point_combo_->findText(desired);
  point_combo_->setCurrentIndex(index >= 0 ? index : (point_combo_->count() > 0 ? 0 : -1));
  point_combo_->blockSignals(false);
  selectPoint(point_combo_->currentIndex());
}

void MotionPresetPanel::capturePoint()
{
  std::array<double, 6> positions{};
  std::chrono::steady_clock::time_point received;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    positions = current_positions_;
    received = last_joint_state_;
  }
  if (received.time_since_epoch().count() == 0 ||
    std::chrono::steady_clock::now() - received > std::chrono::seconds(1))
  {
    setStatus(tr("没有 1 秒内的新鲜 /%1/joint_states，不能采集动作点").arg(
        active_robot_), true);
    return;
  }
  QString name = point_name_edit_->text().trimmed();
  if (name.isEmpty()) {
    int suffix = 1;
    do {
      name = tr("动作点%1").arg(suffix++, 2, 10, QLatin1Char('0'));
    } while (pointIndex(name) >= 0);
  }
  if (pointIndex(name) >= 0) {
    setStatus(tr("动作点名称“%1”已存在").arg(name), true);
    return;
  }
  MotionPoint point;
  point.name = name;
  point.positions = positions;
  point.speed_scale = speed_spin_->value() / 100.0;
  points_.push_back(point);
  setDirty();
  refreshPointList(name);
  setStatus(tr("已采集动作点“%1”").arg(name));
}

void MotionPresetPanel::selectPoint(int index)
{
  if (index < 0 || index >= static_cast<int>(points_.size())) {
    point_name_edit_->clear();
    return;
  }
  const auto & point = points_[static_cast<std::size_t>(index)];
  point_name_edit_->setText(point.name);
  for (std::size_t joint = 0; joint < point.positions.size(); ++joint) {
    joint_spins_[joint]->setValue(point.positions[joint] * kRadiansToDegrees);
  }
  speed_spin_->setValue(point.speed_scale * 100.0);
}

void MotionPresetPanel::applyPoint()
{
  const int index = point_combo_->currentIndex();
  if (index < 0 || index >= static_cast<int>(points_.size())) {
    setStatus(tr("请先选择动作点"), true);
    return;
  }
  const QString old_name = points_[static_cast<std::size_t>(index)].name;
  const QString new_name = point_name_edit_->text().trimmed();
  const int duplicate = pointIndex(new_name);
  if (new_name.isEmpty() || (duplicate >= 0 && duplicate != index)) {
    setStatus(tr("新名称不能为空且不能与已有动作点重名"), true);
    return;
  }
  MotionPoint updated;
  updated.name = new_name;
  updated.speed_scale = speed_spin_->value() / 100.0;
  for (std::size_t joint = 0; joint < updated.positions.size(); ++joint) {
    updated.positions[joint] = joint_spins_[joint]->value() * kDegreesToRadians;
  }
  if (!validPositions(updated.positions)) {
    setStatus(tr("坐标超出 %1 关节限位").arg(robotDisplayName()), true);
    return;
  }
  points_[static_cast<std::size_t>(index)] = updated;
  if (old_name != new_name) {
    for (int row = 0; row < sequence_list_->count(); ++row) {
      if (sequence_list_->item(row)->text() == old_name) {
        sequence_list_->item(row)->setText(new_name);
      }
    }
  }
  setDirty();
  refreshPointList(new_name);
  resetProgress();
  setStatus(tr("动作点“%1”已更新").arg(new_name));
}

void MotionPresetPanel::deletePoint()
{
  const int index = point_combo_->currentIndex();
  if (index < 0 || index >= static_cast<int>(points_.size())) {
    setStatus(tr("请先选择动作点"), true);
    return;
  }
  const QString name = points_[static_cast<std::size_t>(index)].name;
  points_.erase(points_.begin() + index);
  for (int row = sequence_list_->count() - 1; row >= 0; --row) {
    if (sequence_list_->item(row)->text() == name) {
      delete sequence_list_->takeItem(row);
    }
  }
  setDirty();
  refreshPointList();
  resetProgress();
  setStatus(tr("已删除动作点“%1”及顺序组中的对应引用").arg(name));
}

void MotionPresetPanel::executeSelected()
{
  const QString name = point_combo_->currentText();
  if (name.isEmpty()) {
    setStatus(tr("请先选择动作点"), true);
    return;
  }
  startExecution(QStringList{name}, false);
}

void MotionPresetPanel::addSelectedToSequence()
{
  const QString name = point_combo_->currentText();
  if (!name.isEmpty()) {
    sequence_list_->addItem(name);
    setDirty();
    resetProgress();
  }
}

void MotionPresetPanel::useAllPoints()
{
  sequence_list_->clear();
  for (const auto & point : points_) {
    sequence_list_->addItem(point.name);
  }
  setDirty();
  resetProgress();
}

void MotionPresetPanel::removeSequenceItem()
{
  const int row = sequence_list_->currentRow();
  if (row >= 0) {
    delete sequence_list_->takeItem(row);
    setDirty();
    resetProgress();
  }
}

void MotionPresetPanel::moveSequenceItemUp()
{
  const int row = sequence_list_->currentRow();
  if (row > 0) {
    auto * item = sequence_list_->takeItem(row);
    sequence_list_->insertItem(row - 1, item);
    sequence_list_->setCurrentRow(row - 1);
    setDirty();
    resetProgress();
  }
}

void MotionPresetPanel::moveSequenceItemDown()
{
  const int row = sequence_list_->currentRow();
  if (row >= 0 && row + 1 < sequence_list_->count()) {
    auto * item = sequence_list_->takeItem(row);
    sequence_list_->insertItem(row + 1, item);
    sequence_list_->setCurrentRow(row + 1);
    setDirty();
    resetProgress();
  }
}

QStringList MotionPresetPanel::sequenceNames() const
{
  QStringList names;
  for (int row = 0; row < sequence_list_->count(); ++row) {
    names.append(sequence_list_->item(row)->text());
  }
  return names;
}

void MotionPresetPanel::startSequence()
{
  startExecution(sequenceNames(), true);
}

void MotionPresetPanel::startExecution(const QStringList & names, bool apply_sequence_speed)
{
  if (execution_running_) {
    setStatus(tr("已有动作正在执行"), true);
    return;
  }
  if (names.isEmpty()) {
    setStatus(tr("动作顺序为空"), true);
    return;
  }
  for (const auto & name : names) {
    if (pointIndex(name) < 0) {
      setStatus(tr("动作顺序引用了不存在的动作点“%1”").arg(name), true);
      return;
    }
  }
  if (!move_group_client_ || !move_group_client_->action_server_is_ready()) {
    setStatus(tr("MoveIt /%1/move_action 尚未就绪，不能执行动作").arg(active_robot_), true);
    return;
  }
  if (active_robot_ == QStringLiteral("piperh")) {
    const bool status_fresh = last_piper_arm_status_.time_since_epoch().count() != 0 &&
      std::chrono::steady_clock::now() - last_piper_arm_status_ < std::chrono::seconds(1);
    if (!status_fresh) {
      setStatus(tr("Piper-H 控制状态未就绪，请等待状态更新后再播放"), true);
      return;
    }
    if (!piper_motor_enabled_) {
      setStatus(tr("Piper-H 电机未使能；请在“机械臂选择”页点击“使能 Piper-H 电机（现场确认）”"), true);
      return;
    }
    if (!piper_control_loop_active_) {
      setStatus(tr("Piper-H 控制回路未就绪，请检查关节反馈"), true);
      return;
    }
  }
  execution_running_ = true;
  active_sequence_ = names;
  active_sequence_speed_scale_ = apply_sequence_speed ?
    sequence_speed_spin_->value() / 100.0 : 1.0;
  active_index_ = 0;
  ++execution_generation_;
  setEditingEnabled(false);
  start_button_->setEnabled(false);
  execute_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  sendCurrentTarget(execution_generation_);
}

void MotionPresetPanel::sendCurrentTarget(int generation)
{
  if (!execution_running_ || generation != execution_generation_ || active_index_ < 0 ||
    active_index_ >= active_sequence_.size())
  {
    const int total = active_sequence_.size();
    execution_running_ = false;
    active_index_ = -1;
    setEditingEnabled(true);
    start_button_->setEnabled(true);
    execute_button_->setEnabled(true);
    stop_button_->setEnabled(false);
    showProgress(total, total, -1, QString(), tr("动作组完成"));
    publishProgress("completed", total, total, 0, QString());
    setStatus(tr("动作顺序已全部完成"));
    return;
  }

  const QString name = active_sequence_.at(active_index_);
  const int point_index = pointIndex(name);
  if (point_index < 0) {
    handleMotionResult(generation, false, tr("动作点在执行前被删除"));
    return;
  }
  const MotionPoint point = points_[static_cast<std::size_t>(point_index)];
  MoveGroup::Goal goal;
  goal.request.group_name = "arm";
  goal.request.num_planning_attempts = 5;
  goal.request.allowed_planning_time = 5.0;
  const double effective_speed = std::min(1.0, point.speed_scale * active_sequence_speed_scale_);
  goal.request.max_velocity_scaling_factor = effective_speed;
  goal.request.max_acceleration_scaling_factor = effective_speed;
  goal.request.start_state.is_diff = true;
  moveit_msgs::msg::Constraints constraints;
  constraints.name = point.name.toStdString();
  for (std::size_t joint = 0; joint < point.positions.size(); ++joint) {
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = kJointNames[joint].toStdString();
    constraint.position = point.positions[joint];
    constraint.tolerance_above = 0.001;
    constraint.tolerance_below = 0.001;
    constraint.weight = 1.0;
    constraints.joint_constraints.push_back(constraint);
  }
  goal.request.goal_constraints.push_back(constraints);
  goal.planning_options.plan_only = false;
  goal.planning_options.replan = true;
  goal.planning_options.replan_attempts = 2;
  goal.planning_options.replan_delay = 0.2;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  showProgress(active_index_, active_sequence_.size(), active_index_, name, tr("规划并执行中"));
  publishProgress(
    "executing", active_index_, active_sequence_.size(), active_index_ + 1, name);
  setStatus(tr("动作 %1/%2：正在规划并执行“%3”")
    .arg(active_index_ + 1).arg(active_sequence_.size()).arg(name));

  rclcpp_action::Client<MoveGroup>::SendGoalOptions options;
  const auto execution_client = move_group_client_;
  options.goal_response_callback =
    [this, generation, execution_client](const MoveGroupGoalHandle::SharedPtr & handle) {
      if (!handle) {
        if (generation != execution_generation_) {
          return;
        }
        QMetaObject::invokeMethod(this, [this, generation]() {
          handleMotionResult(generation, false, tr("MoveIt 拒绝了目标"));
        }, Qt::QueuedConnection);
        return;
      }
      bool stale = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stale = generation != execution_generation_;
        if (!stale) {
          current_goal_ = handle;
        }
      }
      // “停止”可能发生在服务器接受目标之前；目标一旦迟到也必须取消，
      // 不能让它脱离面板进度继续执行。
      if (stale && execution_client) {
        execution_client->async_cancel_goal(handle);
      }
    };
  options.feedback_callback = [this, generation](
    MoveGroupGoalHandle::SharedPtr,
    const std::shared_ptr<const MoveGroup::Feedback> feedback)
    {
      if (generation != execution_generation_) {
        return;
      }
      const QString state = QString::fromStdString(feedback->state);
      QMetaObject::invokeMethod(this, [this, generation, state]() {
        if (generation == execution_generation_ && execution_running_ && !state.isEmpty()) {
          setStatus(tr("MoveIt：%1").arg(state));
        }
      }, Qt::QueuedConnection);
    };
  options.result_callback = [this, generation](const MoveGroupGoalHandle::WrappedResult & result) {
      bool stale = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stale = generation != execution_generation_;
        if (!stale) {
          current_goal_.reset();
        }
      }
      if (stale) {
        return;
      }
      const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result &&
        result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      QString detail;
      if (!success) {
        const int error_code = result.result ? result.result->error_code.val : 0;
        detail = error_code == moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED ?
          tr("控制器中止轨迹（MoveIt -4）；请检查电机使能状态和关节跟踪日志") :
          tr("MoveIt 执行失败（结果 %1，错误码 %2）")
          .arg(static_cast<int>(result.code)).arg(error_code);
      }
      QMetaObject::invokeMethod(this, [this, generation, success, detail]() {
        handleMotionResult(generation, success, detail);
      }, Qt::QueuedConnection);
    };
  execution_client->async_send_goal(goal, options);
}

void MotionPresetPanel::handleMotionResult(int generation, bool success, const QString & detail)
{
  if (!execution_running_ || generation != execution_generation_) {
    return;
  }
  if (!success) {
    const int failed_index = active_index_;
    const QString failed_name = active_sequence_.value(failed_index);
    execution_running_ = false;
    active_index_ = -1;
    setEditingEnabled(true);
    start_button_->setEnabled(true);
    execute_button_->setEnabled(true);
    stop_button_->setEnabled(false);
    showProgress(
      std::max(0, failed_index), active_sequence_.size(), failed_index, failed_name,
      tr("动作失败"), true);
    publishProgress(
      "failed", std::max(0, failed_index), active_sequence_.size(), failed_index + 1,
      failed_name, detail);
    setStatus(detail.isEmpty() ? tr("动作执行失败并停止") : detail, true);
    return;
  }

  const int point_index = pointIndex(active_sequence_.value(active_index_));
  QString verification_detail;
  if (point_index < 0 ||
    !currentTargetReached(points_[static_cast<std::size_t>(point_index)], verification_detail))
  {
    handleMotionResult(
      generation, false,
      point_index < 0 ? tr("动作点在到位确认前被删除") : verification_detail);
    return;
  }

  const QString name = active_sequence_.value(active_index_);
  showProgress(
    active_index_, active_sequence_.size(), active_index_, name, tr("到位稳定确认中"));
  publishProgress(
    "settling", active_index_, active_sequence_.size(), active_index_ + 1, name,
    tr("真实关节已进入到位范围，正在确认稳定"));
  setStatus(tr("动作 %1/%2 “%3”已到位，稳定确认 0.5 秒")
    .arg(active_index_ + 1).arg(active_sequence_.size()).arg(name));

  QTimer::singleShot(kCompletionStableMilliseconds, this, [this, generation]() {
    if (!execution_running_ || generation != execution_generation_) {
      return;
    }
    const int current_point_index = pointIndex(active_sequence_.value(active_index_));
    QString stable_detail;
    if (current_point_index < 0 ||
      !currentTargetReached(
        points_[static_cast<std::size_t>(current_point_index)], stable_detail))
    {
      handleMotionResult(
        generation, false,
        current_point_index < 0 ? tr("动作点在稳定确认前被删除") : stable_detail);
      return;
    }
    ++active_index_;
    sendCurrentTarget(generation);
  });
}

bool MotionPresetPanel::currentTargetReached(const MotionPoint & point, QString & detail)
{
  std::array<double, 6> actual{};
  std::chrono::steady_clock::time_point received;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    actual = current_positions_;
    received = last_joint_state_;
  }
  if (received.time_since_epoch().count() == 0 ||
    std::chrono::steady_clock::now() - received > std::chrono::seconds(1))
  {
    detail = tr("MoveIt 已返回成功，但没有新鲜关节反馈，动作不计为完成");
    return false;
  }

  double maximum_error = 0.0;
  std::size_t worst_joint = 0;
  for (std::size_t index = 0; index < actual.size(); ++index) {
    const double error = std::abs(point.positions[index] - actual[index]);
    if (error > maximum_error) {
      maximum_error = error;
      worst_joint = index;
    }
  }
  if (maximum_error > kCompletionTolerance) {
    detail = tr("MoveIt 已返回成功，但 J%1 实际到位误差 %2 rad 超过 %3 rad，动作不计为完成")
      .arg(worst_joint + 1).arg(maximum_error, 0, 'f', 4).arg(kCompletionTolerance, 0, 'f', 3);
    return false;
  }
  return true;
}

void MotionPresetPanel::stopExecution()
{
  if (!execution_running_) {
    return;
  }
  const int stopped_index = active_index_;
  const QString stopped_name = active_sequence_.value(stopped_index);
  execution_running_ = false;
  active_index_ = -1;
  ++execution_generation_;
  MoveGroupGoalHandle::SharedPtr goal;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    goal = current_goal_;
    current_goal_.reset();
  }
  if (goal && move_group_client_) {
    move_group_client_->async_cancel_goal(goal);
  }
  setEditingEnabled(true);
  start_button_->setEnabled(true);
  execute_button_->setEnabled(true);
  stop_button_->setEnabled(false);
  showProgress(
    std::max(0, stopped_index), active_sequence_.size(), stopped_index, stopped_name,
    tr("已停止"), true);
  publishProgress(
    "stopped", std::max(0, stopped_index), active_sequence_.size(), stopped_index + 1,
    stopped_name, tr("用户停止"));
  setStatus(tr("已请求 MoveIt 取消当前动作"));
}

void MotionPresetPanel::setEditingEnabled(bool enabled)
{
  enabled = enabled && active_robot_selected_;
  for (auto * widget : editing_widgets_) {
    widget->setEnabled(enabled);
  }
  for (auto * spin : joint_spins_) {
    spin->setEnabled(enabled);
  }
  speed_spin_->setEnabled(enabled);
  sequence_speed_spin_->setEnabled(enabled);
}

void MotionPresetPanel::resetProgress()
{
  progress_label_->setText(tr("未开始"));
  progress_label_->setStyleSheet("");
  progress_bar_->setRange(0, 1);
  progress_bar_->setValue(0);
  progress_bar_->setFormat(tr("0 / 0（0%）"));
  progress_bar_->setStyleSheet("");
  for (int row = 0; row < sequence_list_->count(); ++row) {
    sequence_list_->item(row)->setBackground(QBrush());
    QFont font = sequence_list_->item(row)->font();
    font.setBold(false);
    sequence_list_->item(row)->setFont(font);
  }
}

void MotionPresetPanel::showProgress(
  int completed, int total, int current_index, const QString & current_name,
  const QString & state, bool error)
{
  completed = std::clamp(completed, 0, std::max(0, total));
  progress_bar_->setRange(0, std::max(1, total));
  progress_bar_->setValue(completed);
  progress_bar_->setFormat(total > 0 ? tr("%v / %m（%p%）") : tr("0 / 0（0%）"));
  progress_bar_->setStyleSheet(
    error ? "QProgressBar::chunk { background-color: #d9534f; }" : "");
  progress_label_->setStyleSheet(error ? "color: #e35d6a;" : "");
  if (current_index >= 0 && current_index < total) {
    progress_label_->setText(
      tr("%1：%2 / %3，当前“%4”，已完成 %5")
      .arg(state).arg(current_index + 1).arg(total).arg(current_name).arg(completed));
  } else {
    progress_label_->setText(tr("%1：已完成 %2 / %3").arg(state).arg(completed).arg(total));
  }

  if (sequence_list_->count() == total) {
    for (int row = 0; row < total; ++row) {
      auto * item = sequence_list_->item(row);
      item->setBackground(row < completed ? QColor(40, 167, 69, 90) : QBrush());
      QFont font = item->font();
      font.setBold(row == current_index);
      item->setFont(font);
    }
    if (current_index >= 0 && current_index < total) {
      sequence_list_->item(current_index)->setBackground(
        error ? QColor(220, 53, 69, 110) : QColor(255, 193, 7, 110));
      sequence_list_->scrollToItem(sequence_list_->item(current_index));
    }
  }
}

void MotionPresetPanel::publishProgress(
  const QString & state, int completed, int total, int current_index,
  const QString & current_name, const QString & message)
{
  if (!progress_pub_) {
    return;
  }
  QJsonObject progress;
  progress["state"] = state;
  progress["robot"] = active_robot_;
  if (active_robot_ == QStringLiteral("rebotarm")) {
    progress["robot_model"] = rebot_model_;
  }
  progress["completed"] = completed;
  progress["total"] = total;
  progress["current_index"] = current_index;
  progress["current_name"] = current_name;
  progress["message"] = message;
  std_msgs::msg::String output;
  output.data = QJsonDocument(progress).toJson(QJsonDocument::Compact).toStdString();
  progress_pub_->publish(output);
}

}  // namespace piperh_motion_rviz

PLUGINLIB_EXPORT_CLASS(piperh_motion_rviz::MotionPresetPanel, rviz_common::Panel)
