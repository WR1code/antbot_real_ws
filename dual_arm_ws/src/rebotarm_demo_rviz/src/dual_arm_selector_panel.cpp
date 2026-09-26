#include "rebotarm_demo_rviz/dual_arm_selector_panel.hpp"
#include "rebotarm_demo_rviz/demo_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>

#include <QApplication>
#include <QAction>
#include <QComboBox>
#include <QDockWidget>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMainWindow>
#include <QMetaObject>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QStringList>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_group.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace rebotarm_demo_rviz
{

namespace
{

constexpr auto kRebotModelName = "reBotArm 实机模型";
constexpr auto kPiperModelName = "Piper-H 实机模型";

std::int64_t steadyMilliseconds()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void setRobotModelVisibility(
  rviz_common::DisplayGroup * root, bool show_rebot, bool show_piper)
{
  if (!root) {return;}
  for (int index = 0; index < root->numDisplays(); ++index) {
    auto * display = root->getDisplayAt(index);
    if (!display || display->getClassId() != QStringLiteral(
        "rviz_default_plugins/RobotModel"))
    {
      continue;
    }
    if (display->getName() == QString::fromUtf8(kRebotModelName)) {
      display->setEnabled(show_rebot);
    } else if (display->getName() == QString::fromUtf8(kPiperModelName)) {
      display->setEnabled(show_piper);
    }
  }
}

bool hasCompleteArmState(const sensor_msgs::msg::JointState & message)
{
  if (message.name.size() != message.position.size()) {return false;}
  std::map<std::string, double> positions;
  for (std::size_t index = 0; index < message.name.size(); ++index) {
    positions[message.name[index]] = message.position[index];
  }
  for (int index = 1; index <= 6; ++index) {
    const auto iterator = positions.find("joint" + std::to_string(index));
    if (iterator == positions.end() || !std::isfinite(iterator->second)) {return false;}
  }
  return true;
}

}  // namespace

DualArmSelectorPanel::DualArmSelectorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  // Robot selection is a peer page, not a permanent header above every
  // feature. Keeping it in the same tab widget also prevents its status
  // controls from displacing the embedded panels when the dock is resized.
  feature_tabs_ = new QTabWidget(this);
  feature_tabs_->setObjectName(QStringLiteral("dual_arm_feature_tabs"));
  feature_tabs_->setTabPosition(QTabWidget::North);
  feature_tabs_->setUsesScrollButtons(true);
  feature_tabs_->setElideMode(Qt::ElideRight);
  layout->addWidget(feature_tabs_, 1);

  auto * selection_scroll = new QScrollArea(feature_tabs_);
  selection_scroll->setObjectName(QStringLiteral("dual_arm_selection_scroll"));
  selection_scroll->setWidgetResizable(true);
  selection_scroll->setFrameShape(QFrame::NoFrame);
  selection_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto * selection_page = new QWidget(selection_scroll);
  selection_page->setObjectName(QStringLiteral("dual_arm_selection_page"));
  auto * selection_layout = new QVBoxLayout(selection_page);
  selection_layout->setContentsMargins(4, 4, 4, 4);
  selection_scroll->setWidget(selection_page);

  auto * title = new QLabel(QStringLiteral("双机械臂统一工作台"), selection_page);
  title->setStyleSheet(QStringLiteral("font-size: 16px; font-weight: bold;"));
  selection_layout->addWidget(title);
  mode_label_ = new QLabel(
    QStringLiteral("两套驱动保持运行；切换不会重启程序。"), selection_page);
  mode_label_->setWordWrap(true);
  selection_layout->addWidget(mode_label_);

  robot_combo_ = new QComboBox(selection_page);
  robot_combo_->addItem(QStringLiteral("reBotArm"), QStringLiteral("rebotarm"));
  robot_combo_->addItem(QStringLiteral("Piper-H"), QStringLiteral("piperh"));
  selection_layout->addWidget(robot_combo_);
  select_button_ = new QPushButton(QStringLiteral("切换控制机械臂"), selection_page);
  connect(select_button_, &QPushButton::clicked, this, &DualArmSelectorPanel::requestSelection);
  selection_layout->addWidget(select_button_);

  current_label_ = new QLabel(QStringLiteral("当前控制：等待状态"), selection_page);
  current_label_->setStyleSheet(QStringLiteral("font-weight: bold;"));
  rebot_state_label_ = new QLabel(QStringLiteral("reBotArm：等待状态"), selection_page);
  piper_state_label_ = new QLabel(QStringLiteral("Piper-H：等待状态"), selection_page);
  status_label_ = new QLabel(QStringLiteral("等待控制权管理节点…"), selection_page);
  status_label_->setWordWrap(true);
  selection_layout->addWidget(current_label_);
  selection_layout->addWidget(rebot_state_label_);
  selection_layout->addWidget(piper_state_label_);
  selection_layout->addWidget(status_label_);
  handoff_hint_label_ = new QLabel(
    QStringLiteral("切换后仍是锁定状态：摇杆回中，再按 A 解锁。"), selection_page);
  handoff_hint_label_->setWordWrap(true);
  selection_layout->addWidget(handoff_hint_label_);
  selection_layout->addStretch(1);

  auto * comprehensive_scroll = new QScrollArea(feature_tabs_);
  comprehensive_scroll->setObjectName(QStringLiteral("dual_arm_comprehensive_scroll"));
  comprehensive_scroll->setWidgetResizable(true);
  comprehensive_scroll->setFrameShape(QFrame::NoFrame);
  comprehensive_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  comprehensive_stack_ = new QStackedWidget(comprehensive_scroll);
  comprehensive_scroll->setWidget(comprehensive_stack_);
  rebot_loading_page_ = new QWidget(comprehensive_stack_);
  auto * loading_layout = new QVBoxLayout(rebot_loading_page_);
  loading_layout->addWidget(new QLabel(QStringLiteral("正在载入完整机械臂控制…"), rebot_loading_page_));
  loading_layout->addStretch(1);
  comprehensive_stack_->addWidget(rebot_loading_page_);

  auto * piper_page = new QWidget(comprehensive_stack_);
  auto * piper_layout = new QVBoxLayout(piper_page);
  piper_layout->setContentsMargins(4, 4, 4, 4);
  auto * piper_title = new QLabel(QStringLiteral("Piper-H 机械臂控制"), piper_page);
  piper_title->setStyleSheet(QStringLiteral("font-size: 15px; font-weight: bold;"));
  piper_layout->addWidget(piper_title);
  piper_layout->addWidget(new QLabel(
      QStringLiteral("通用控制功能随当前机械臂切换；Piper-H 无夹爪，重力补偿由独立的软件控制器提供。"),
      piper_page));

  piper_control_tabs_ = new QTabWidget(piper_page);
  piper_control_tabs_->setObjectName(QStringLiteral("piper_control_tabs"));
  piper_control_tabs_->setTabPosition(QTabWidget::North);
  piper_control_tabs_->setUsesScrollButtons(true);
  piper_safety_page_ = new QWidget(piper_control_tabs_);
  piper_safety_page_->setObjectName(QStringLiteral("piper_safety_page"));
  piper_safety_layout_ = new QVBoxLayout(piper_safety_page_);
  auto * state_box = new QGroupBox(QStringLiteral("关节反馈"), piper_safety_page_);
  auto * state_layout = new QVBoxLayout(state_box);
  piper_joint_label_ = new QLabel(QStringLiteral("等待 /piperh/joint_states…"), state_box);
  piper_joint_label_->setWordWrap(true);
  state_layout->addWidget(piper_joint_label_);
  piper_safety_layout_->addWidget(state_box);
  auto * safety_box = new QGroupBox(QStringLiteral("安全与控制"), piper_safety_page_);
  safety_box->setObjectName(QStringLiteral("piper_specific_safety_box"));
  piper_specific_safety_box_ = safety_box;
  auto * safety_layout = new QVBoxLayout(safety_box);
  piper_motor_enable_button_ = new QPushButton(
    QStringLiteral("使能 Piper-H 电机（现场确认）"), safety_box);
  piper_motor_enable_button_->setObjectName(QStringLiteral("piper_motor_enable_button"));
  piper_motor_disable_button_ = new QPushButton(
    QStringLiteral("失能 Piper-H 电机"), safety_box);
  piper_motor_disable_button_->setObjectName(QStringLiteral("piper_motor_disable_button"));
  piper_armed_button_ = new QPushButton(QStringLiteral("等待 Xbox 控制器状态"), safety_box);
  piper_armed_button_->setObjectName(QStringLiteral("piper_xbox_arm_button"));
  piper_gravity_button_ = new QPushButton(QStringLiteral("开启 Piper-H 重力补偿"), safety_box);
  piper_gravity_button_->setObjectName(QStringLiteral("piper_gravity_compensation_button"));
  piper_gravity_button_->setAttribute(Qt::WA_AlwaysShowToolTips, true);
  piper_stop_button_ = new QPushButton(QStringLiteral("停止全部运动"), safety_box);
  piper_stop_button_->setStyleSheet(QStringLiteral("font-weight: bold; color: #b42318;"));
  piper_reload_zones_button_ = new QPushButton(QStringLiteral("重新加载禁区配置"), safety_box);
  piper_control_status_label_ = new QLabel(QStringLiteral("等待 Piper-H 控制接口…"), safety_box);
  piper_control_status_label_->setObjectName(QStringLiteral("piper_control_status_label"));
  piper_control_status_label_->setWordWrap(true);
  safety_layout->addWidget(piper_motor_enable_button_);
  safety_layout->addWidget(piper_motor_disable_button_);
  safety_layout->addWidget(piper_armed_button_);
  safety_layout->addWidget(piper_gravity_button_);
  safety_layout->addWidget(piper_stop_button_);
  safety_layout->addWidget(piper_reload_zones_button_);
  safety_layout->addWidget(piper_control_status_label_);
  piper_safety_layout_->addWidget(safety_box);
  piper_safety_layout_->addStretch(1);

  piper_actions_page_ = new QWidget(piper_control_tabs_);
  piper_actions_page_->setObjectName(QStringLiteral("piper_actions_page"));
  piper_actions_layout_ = new QVBoxLayout(piper_actions_page_);
  piper_actions_layout_->setContentsMargins(0, 0, 0, 0);
  piper_actions_placeholder_ = new QLabel(
    QStringLiteral("正在载入通用动作点、动作组与顺序回放…"), piper_actions_page_);
  piper_actions_placeholder_->setWordWrap(true);
  piper_actions_layout_->addWidget(piper_actions_placeholder_);
  piper_actions_layout_->addStretch(1);

  piper_control_tabs_->addTab(piper_safety_page_, QStringLiteral("真机与安全"));
  piper_control_tabs_->addTab(piper_actions_page_, QStringLiteral("演示与动作"));
  piper_layout->addWidget(piper_control_tabs_, 1);
  comprehensive_stack_->addWidget(piper_page);

  auto * chassis_scroll = new QScrollArea(feature_tabs_);
  chassis_scroll->setObjectName(QStringLiteral("dual_arm_chassis_scroll"));
  chassis_scroll->setWidgetResizable(true);
  chassis_scroll->setFrameShape(QFrame::NoFrame);
  chassis_scroll->viewport()->setAutoFillBackground(true);
  chassis_page_ = new QWidget(chassis_scroll);
  chassis_layout_ = new QVBoxLayout(chassis_page_);
  chassis_layout_->setContentsMargins(0, 0, 0, 0);
  chassis_tabs_ = new QTabWidget(chassis_page_);
  chassis_tabs_->setObjectName(QStringLiteral("chassis_control_tabs"));
  chassis_tabs_->setTabPosition(QTabWidget::North);
  chassis_tabs_->setUsesScrollButtons(true);
  auto * chassis_loading = new QLabel(
    QStringLiteral("正在载入底盘控制、状态、建图与航点工具…"), chassis_tabs_);
  chassis_loading->setObjectName(QStringLiteral("chassis_loading_page"));
  chassis_loading->setWordWrap(true);
  chassis_tabs_->addTab(chassis_loading, QStringLiteral("底盘功能"));
  chassis_layout_->addWidget(chassis_tabs_, 1);
  chassis_scroll->setWidget(chassis_page_);

  feature_tabs_->addTab(selection_scroll, QStringLiteral("机械臂选择"));
  feature_tabs_->addTab(comprehensive_scroll, QStringLiteral("机械臂控制"));
  feature_tabs_->addTab(chassis_scroll, QStringLiteral("底盘控制"));
  feature_tabs_->setTabEnabled(1, false);
  feature_tabs_->setTabEnabled(2, true);

  connect(piper_motor_enable_button_, &QPushButton::clicked,
    this, &DualArmSelectorPanel::enablePiperMotors);
  connect(piper_motor_disable_button_, &QPushButton::clicked,
    this, &DualArmSelectorPanel::disablePiperMotors);
  connect(piper_armed_button_, &QPushButton::clicked,
    this, &DualArmSelectorPanel::togglePiperArmed);
  connect(piper_gravity_button_, &QPushButton::clicked,
    this, &DualArmSelectorPanel::togglePiperGravity);
  connect(piper_stop_button_, &QPushButton::clicked,
    this, &DualArmSelectorPanel::stopPiperMotion);
  connect(piper_reload_zones_button_, &QPushButton::clicked,
    this, &DualArmSelectorPanel::reloadPiperZones);
  configurePiperControls(false);
}

DualArmSelectorPanel::~DualArmSelectorPanel()
{
  placePiperSafetyControls(false);
  placeForbiddenZonePanel(true);
  placeRobotControlPages(true);
  if (preset_content_) {
    if (rebot_control_content_) {
      auto * action_tabs = rebot_control_content_->findChild<QTabWidget *>(
        QStringLiteral("teach_action_mode_tabs"));
      if (action_tabs) {
        const int index = action_tabs->indexOf(preset_content_);
        if (index >= 0) {action_tabs->removeTab(index);}
      }
    }
    if (piper_actions_layout_) {piper_actions_layout_->removeWidget(preset_content_);}
  }
  restoreEmbeddedPanel(rebot_control_dock_, rebot_control_content_, rebot_control_placeholder_);
  restoreEmbeddedPanel(preset_dock_, preset_content_, preset_placeholder_);
  restoreEmbeddedPanel(
    vehicle_control_dock_, vehicle_control_content_, vehicle_control_placeholder_);
  restoreEmbeddedPanel(waypoint_dock_, waypoint_content_, waypoint_placeholder_);
}

void DualArmSelectorPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    updateStatus(QStringLiteral("无法取得 RViz ROS 节点"));
    select_button_->setEnabled(false);
    return;
  }
  node_ = abstraction->get_raw_node();
  offline_preview_ = node_->has_parameter("dual_arm.offline_preview") ?
    node_->get_parameter("dual_arm.offline_preview").as_bool() :
    node_->declare_parameter<bool>("dual_arm.offline_preview", false);
  if (offline_preview_) {
    mode_label_->setText(QStringLiteral(
      "离线双臂模型预览：不启动串口/CAN 真机驱动，切换只改变 RViz 中的模型和规划环境。"));
    mode_label_->setStyleSheet(QStringLiteral("color: #1677b8; font-weight: bold;"));
    rebot_state_label_->setText(QStringLiteral("reBotArm：离线假硬件"));
    piper_state_label_->setText(QStringLiteral("Piper-H：离线假硬件"));
    handoff_hint_label_->setText(QStringLiteral(
      "离线切换立即生效；不需要手柄，也不存在真机解锁步骤。"));
  }
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  request_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/dual_arm/select_request", rclcpp::QoS(10));
  selected_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/dual_arm/selected", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const QString value = QString::fromStdString(msg->data);
      QMetaObject::invokeMethod(this, [this, value]() {updateSelected(value);}, Qt::QueuedConnection);
    });
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/dual_arm/status", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const QString value = QString::fromStdString(msg->data);
      QMetaObject::invokeMethod(this, [this, value]() {updateStatus(value);}, Qt::QueuedConnection);
    });
  const auto armed_callback = [this](const QString & robot) {
      return [this, robot](const std_msgs::msg::Bool::SharedPtr msg) {
        const bool armed = msg->data;
        QMetaObject::invokeMethod(
          this, [this, robot, armed]() {updateArmed(robot, armed);}, Qt::QueuedConnection);
      };
    };
  rebot_armed_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/rebotarm/xbox/armed", qos, armed_callback(QStringLiteral("rebotarm")));
  piper_armed_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/piperh/xbox/armed", qos, armed_callback(QStringLiteral("piperh")));
  rebot_joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/rebotarm/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      QMetaObject::invokeMethod(
        this, [this, message]() {updateRebotJointState(message);}, Qt::QueuedConnection);
    });
  piper_joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/piperh/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      QMetaObject::invokeMethod(
        this, [this, message]() {updatePiperJointState(message);}, Qt::QueuedConnection);
    });
  piper_arm_status_sub_ = node_->create_subscription<rebotarm_msgs::msg::ArmStatus>(
    "/piperh/arm_status", qos,
    [this](const rebotarm_msgs::msg::ArmStatus::SharedPtr msg) {
      const QString state = QString::fromStdString(msg->state_machine);
      const bool enabled = msg->enabled;
      QMetaObject::invokeMethod(this, [this, state, enabled]() {
        last_piper_arm_status_ms_ = steadyMilliseconds();
        piper_motor_enabled_ = enabled;
        piper_gravity_active_ = state == QStringLiteral("GRAVITY_COMPENSATION_ACTIVE");
        piper_gravity_transition_ = state == QStringLiteral("GRAVITY_PRECHECK");
        configurePiperControls(piper_selected_);
      }, Qt::QueuedConnection);
    });
  piper_teach_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/piperh/teach/status", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
      const QString state = document.isObject() ?
        document.object().value(QStringLiteral("state")).toString() : QString();
      QMetaObject::invokeMethod(this, [this, state]() {
        piper_teach_resting_ = state == QStringLiteral("IDLE") ||
          state == QStringLiteral("READY");
        piper_teach_recording_ = state == QStringLiteral("RECORDING");
        configurePiperControls(piper_selected_);
      }, Qt::QueuedConnection);
    });
  piper_enable_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/piperh/motor/set_enabled");
  piper_armed_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/piperh/xbox/set_armed");
  piper_gravity_start_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/piperh/teach/gravity_mode/start");
  piper_gravity_stop_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/piperh/teach/gravity_mode/stop");
  piper_teach_cancel_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/piperh/teach/cancel");
  piper_reload_zones_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/piperh/forbidden_zone_manager/reload");
  piper_move_cancel_client_ = node_->create_client<action_msgs::srv::CancelGoal>(
    "/piperh/move_action/_action/cancel_goal");
  piper_execute_cancel_client_ = node_->create_client<action_msgs::srv::CancelGoal>(
    "/piperh/execute_trajectory/_action/cancel_goal");
  piper_controller_cancel_client_ = node_->create_client<action_msgs::srv::CancelGoal>(
    "/piperh/arm_controller/follow_joint_trajectory/_action/cancel_goal");
  // RViz creates some plugin docks after Panel::onInitialize(). Re-run the
  // grouping briefly so every control surface becomes a left-side tab page.
  QTimer::singleShot(0, this, &DualArmSelectorPanel::organizeDockPages);
  QTimer::singleShot(700, this, &DualArmSelectorPanel::organizeDockPages);
  QTimer::singleShot(1600, this, &DualArmSelectorPanel::organizeDockPages);
  QTimer::singleShot(3500, this, &DualArmSelectorPanel::organizeDockPages);
  QTimer::singleShot(8000, this, &DualArmSelectorPanel::organizeDockPages);
  QTimer::singleShot(15000, this, &DualArmSelectorPanel::organizeDockPages);
  QTimer::singleShot(30000, this, &DualArmSelectorPanel::organizeDockPages);
  auto * feedback_watchdog = new QTimer(this);
  feedback_watchdog->setInterval(500);
  connect(feedback_watchdog, &QTimer::timeout, this, [this]() {
      if (offline_preview_ || !piper_selected_ || last_piper_feedback_ms_ <= 0) {return;}
      if (steadyMilliseconds() - last_piper_feedback_ms_ <= 1500) {return;}
      piper_control_status_label_->setText(QStringLiteral(
        "Piper-H 反馈已超时；已禁止使能和实时控制，请检查 can0 与 USB-CAN 连接。"));
      configurePiperControls(true);
    });
  feedback_watchdog->start();
}

void DualArmSelectorPanel::organizeDockPages()
{
  auto * main_window = qobject_cast<QMainWindow *>(window());
  if (!main_window) {return;}
  QDockWidget * anchor = nullptr;
  for (QWidget * ancestor = parentWidget(); ancestor; ancestor = ancestor->parentWidget()) {
    anchor = qobject_cast<QDockWidget *>(ancestor);
    if (anchor) {break;}
  }
  if (!anchor) {return;}
  anchor->setProperty("rebot_preserve_dock_title", true);

  embedFeaturePages();
  const QStringList page_titles{
    QStringLiteral("Displays"), QStringLiteral("Views"),
    QStringLiteral("机械臂选择"), QStringLiteral("双臂统一工作台")};
  main_window->setTabPosition(Qt::LeftDockWidgetArea, QTabWidget::South);
  main_window->addDockWidget(Qt::LeftDockWidgetArea, anchor);
  for (auto * dock : main_window->findChildren<QDockWidget *>()) {
    const QString title = dock->windowTitle();
    const bool is_page = page_titles.contains(title);
    if (dock == anchor || !is_page) {continue;}
    main_window->addDockWidget(Qt::LeftDockWidgetArea, dock);
    main_window->tabifyDockWidget(anchor, dock);
  }
  anchor->raise();
}

void DualArmSelectorPanel::embedFeaturePages()
{
  auto * main_window = qobject_cast<QMainWindow *>(window());
  if (!main_window) {return;}

  const auto find_dock = [main_window](const QStringList & titles) -> QDockWidget * {
      for (auto * dock : main_window->findChildren<QDockWidget *>()) {
        for (const auto & title : titles) {
          if (dock->windowTitle() == title || dock->windowTitle().contains(title)) {
            return dock;
          }
        }
      }
      return nullptr;
    };
  const auto detach_dock = [main_window](QDockWidget * dock, QPointer<QWidget> & content,
      QPointer<QWidget> & placeholder) {
      if (!dock || content || !dock->widget()) {return;}
      content = dock->widget();
      // Detach first: QDockWidget may dispose of its old child when setWidget()
      // replaces it. The unified workbench now owns the real panel widget.
      content->setParent(nullptr);
      placeholder = new QWidget(dock);
      dock->setWidget(placeholder);
      dock->hide();
      dock->toggleViewAction()->setVisible(false);
      // A merely hidden tabified dock can leave a stale tab button behind.
      // Remove the empty host dock from QMainWindow; its content stays inside
      // the unified workbench and is restored during orderly teardown.
      main_window->removeDockWidget(dock);
    };

  if (!rebot_control_content_) {
    rebot_control_dock_ = find_dock(
      {QStringLiteral("机械臂控制"), QStringLiteral("reBot 综合控制")});
    detach_dock(rebot_control_dock_, rebot_control_content_, rebot_control_placeholder_);
    if (rebot_control_content_) {
      const int old_index = comprehensive_stack_->indexOf(rebot_loading_page_);
      if (old_index >= 0) {
        comprehensive_stack_->removeWidget(rebot_loading_page_);
        rebot_loading_page_->deleteLater();
        rebot_loading_page_ = nullptr;
      }
      comprehensive_stack_->insertWidget(0, rebot_control_content_);
      shared_control_tabs_ = rebot_control_content_->findChild<QTabWidget *>(
        QStringLiteral("main_tabs"));
      if (shared_control_tabs_ && shared_control_tabs_->count() >= 5) {
        rebot_safety_page_ = shared_control_tabs_->widget(0);
        rebot_actions_page_ = shared_control_tabs_->widget(1);
      }
      forbidden_zone_content_ = rebot_control_content_->findChild<QGroupBox *>(
        QStringLiteral("forbidden_zone_box"));
      if (forbidden_zone_content_ && forbidden_zone_content_->parentWidget()) {
        rebot_safety_layout_ = qobject_cast<QVBoxLayout *>(
          forbidden_zone_content_->parentWidget()->layout());
      }
    }
  }
  if (!preset_content_) {
    preset_dock_ = find_dock(
      {QStringLiteral("Piper-H 预设动作"), QStringLiteral("双臂预设动作")});
    detach_dock(preset_dock_, preset_content_, preset_placeholder_);
  }
  if (!vehicle_control_content_) {
    vehicle_control_dock_ = find_dock({QStringLiteral("车辆控制中心")});
    detach_dock(
      vehicle_control_dock_, vehicle_control_content_, vehicle_control_placeholder_);
    if (vehicle_control_content_) {
      if (auto * loading = chassis_tabs_->findChild<QLabel *>(
          QStringLiteral("chassis_loading_page")))
      {
        const int index = chassis_tabs_->indexOf(loading);
        if (index >= 0) {chassis_tabs_->removeTab(index);}
        loading->deleteLater();
      }
      chassis_tabs_->insertTab(0, vehicle_control_content_, QStringLiteral("车辆控制中心"));
    }
  }
  if (!waypoint_content_) {
    waypoint_dock_ = find_dock({QStringLiteral("航点与巡航组")});
    detach_dock(waypoint_dock_, waypoint_content_, waypoint_placeholder_);
    if (waypoint_content_) {
      chassis_tabs_->addTab(waypoint_content_, QStringLiteral("航点与巡航组"));
    }
  }
  // This editor already follows /dual_arm/selected. Move the same widget into
  // the current robot's action page instead of hiding it with reBot-only UI.
  placeRobotControlPages(selected_robot_ != QStringLiteral("piperh"));
  placeForbiddenZonePanel(selected_robot_ != QStringLiteral("piperh"));
  placePresetPanel(selected_robot_ != QStringLiteral("piperh"));
  placePiperSafetyControls(selected_robot_ == QStringLiteral("piperh"));
}

void DualArmSelectorPanel::placeForbiddenZonePanel(bool rebot_selected)
{
  Q_UNUSED(rebot_selected);
  if (!forbidden_zone_content_) {return;}
  if (rebot_safety_layout_) {rebot_safety_layout_->removeWidget(forbidden_zone_content_);}
  if (piper_safety_layout_) {piper_safety_layout_->removeWidget(forbidden_zone_content_);}
  if (rebot_safety_layout_) {
    rebot_safety_layout_->insertWidget(1, forbidden_zone_content_);
  }
  forbidden_zone_content_->show();
  piper_reload_zones_button_->setVisible(!forbidden_zone_content_);
}

void DualArmSelectorPanel::placeRobotControlPages(bool rebot_selected)
{
  if (!shared_control_tabs_ || !rebot_safety_page_ || !rebot_actions_page_) {return;}
  QWidget * current_page = shared_control_tabs_->currentWidget();
  const auto remove_page = [](QTabWidget * tabs, QWidget * page) {
      if (!tabs || !page) {return;}
      const int index = tabs->indexOf(page);
      if (index >= 0) {tabs->removeTab(index);}
    };
  for (auto * page : {
      rebot_safety_page_.data(), rebot_actions_page_.data(),
      piper_safety_page_, piper_actions_page_})
  {
    remove_page(shared_control_tabs_, page);
  }
  remove_page(piper_control_tabs_, piper_safety_page_);
  remove_page(piper_control_tabs_, piper_actions_page_);

  // Both robots use the exact same complete control pages. Robot selection
  // changes their ROS interfaces and model profile, never their UI modules.
  shared_control_tabs_->insertTab(0, rebot_safety_page_, QStringLiteral("真机与安全"));
  shared_control_tabs_->insertTab(1, rebot_actions_page_, QStringLiteral("演示与动作"));
  piper_control_tabs_->addTab(piper_safety_page_, QStringLiteral("真机与安全"));
  piper_control_tabs_->addTab(piper_actions_page_, QStringLiteral("演示与动作"));
  const int current_index = shared_control_tabs_->indexOf(current_page);
  shared_control_tabs_->setCurrentIndex(current_index >= 0 ? current_index : 0);
  if (auto * demo_panel = qobject_cast<DemoPanel *>(rebot_control_content_.data())) {
    demo_panel->setActiveRobot(
      rebot_selected ? QStringLiteral("rebotarm") : QStringLiteral("piperh"));
  }
}

void DualArmSelectorPanel::placePresetPanel(bool rebot_selected)
{
  Q_UNUSED(rebot_selected);
  if (!preset_content_) {return;}
  auto * rebot_action_tabs = rebot_control_content_ ?
    rebot_control_content_->findChild<QTabWidget *>(
      QStringLiteral("teach_action_mode_tabs")) : nullptr;
  if (piper_actions_layout_) {
    piper_actions_layout_->removeWidget(preset_content_);
  }

  if (rebot_action_tabs && rebot_action_tabs->indexOf(preset_content_) < 0) {
    rebot_action_tabs->addTab(preset_content_, QStringLiteral("关节坐标动作"));
  }
  if (piper_actions_placeholder_) {
    piper_actions_placeholder_->hide();
  }
  preset_content_->show();
}

void DualArmSelectorPanel::placePiperSafetyControls(bool piper_selected)
{
  if (!piper_specific_safety_box_) {return;}
  if (rebot_safety_layout_) {
    rebot_safety_layout_->removeWidget(piper_specific_safety_box_);
  }
  if (piper_safety_layout_) {
    piper_safety_layout_->removeWidget(piper_specific_safety_box_);
  }
  if (piper_selected && rebot_safety_layout_) {
    rebot_safety_layout_->insertWidget(1, piper_specific_safety_box_);
    piper_specific_safety_box_->show();
  } else if (piper_safety_layout_) {
    piper_safety_layout_->insertWidget(1, piper_specific_safety_box_);
  }
}

void DualArmSelectorPanel::restoreEmbeddedPanel(
  QPointer<QDockWidget> & dock, QPointer<QWidget> & content,
  QPointer<QWidget> & placeholder)
{
  if (!dock || !content) {return;}
  content->setParent(nullptr);
  dock->setWidget(content);
  if (auto * main_window = qobject_cast<QMainWindow *>(dock->parentWidget())) {
    main_window->addDockWidget(Qt::LeftDockWidgetArea, dock);
  }
  dock->toggleViewAction()->setVisible(true);
  if (placeholder) {
    placeholder->deleteLater();
  }
  content.clear();
  placeholder.clear();
  dock.clear();
}

void DualArmSelectorPanel::requestSelection()
{
  if (!request_pub_) {return;}
  std_msgs::msg::String msg;
  msg.data = robot_combo_->currentData().toString().toStdString();
  request_pub_->publish(msg);
  select_button_->setEnabled(false);
  updateStatus(QStringLiteral("正在请求安全切换…"));
}

void DualArmSelectorPanel::updateSelected(const QString & robot)
{
  if (robot == QStringLiteral("none")) {
    selected_robot_ = robot;
    current_label_->setText(QStringLiteral("当前控制：切换中（两边锁定）"));
    current_label_->setStyleSheet(QStringLiteral("font-weight: bold; color: #d67c00;"));
    select_button_->setEnabled(false);
    if (getDisplayContext()) {
      // A control handoff must not break the combined chassis + dual-arm
      // overview. Selection affects command ownership, not visualization.
      setRobotModelVisibility(
        getDisplayContext()->getRootDisplayGroup(),
        rebot_model_ready_, piper_model_ready_);
    }
    feature_tabs_->setTabEnabled(1, false);
    feature_tabs_->setTabEnabled(2, true);
    feature_tabs_->setCurrentIndex(0);
    if (rebot_control_content_) {rebot_control_content_->setEnabled(false);}
    if (preset_content_) {preset_content_->setEnabled(false);}
    piper_selected_ = false;
    configurePiperControls(false);
    return;
  }
  if (robot != QStringLiteral("rebotarm") && robot != QStringLiteral("piperh")) {
    updateStatus(QStringLiteral("收到未知机械臂选择：%1").arg(robot));
    return;
  }
  const bool rebot = robot == QStringLiteral("rebotarm");
  selected_robot_ = robot;
  current_label_->setText(
    QStringLiteral("当前控制：%1").arg(rebot ? QStringLiteral("reBotArm") : QStringLiteral("Piper-H")));
  current_label_->setStyleSheet(QStringLiteral("font-weight: bold; color: #16803a;"));
  robot_combo_->setCurrentIndex(rebot ? 0 : 1);
  select_button_->setEnabled(true);
  feature_tabs_->setTabEnabled(1, true);
  feature_tabs_->setTabEnabled(2, true);
  placeRobotControlPages(rebot);
  placeForbiddenZonePanel(rebot);
  placePiperSafetyControls(!rebot);
  comprehensive_stack_->setCurrentIndex(
    rebot || shared_control_tabs_ ? 0 : 1);
  if (feature_tabs_->currentIndex() == 0) {
    feature_tabs_->setCurrentIndex(1);
  }
  if (rebot_control_content_) {rebot_control_content_->setEnabled(true);}
  placePresetPanel(rebot);
  if (preset_content_) {preset_content_->setEnabled(true);}
  piper_selected_ = !rebot;
  configurePiperControls(!rebot);

  // One native MoveIt panel is reused. Changing these two properties makes
  // its planning context and robot model follow the selected hardware stack.
  if (getDisplayContext()) {
    auto * root = getDisplayContext()->getRootDisplayGroup();
    if (root) {
      // Keep both physical arms visible as one chassis-mounted assembly.
      // A model is only revealed after its complete joint state arrives, so
      // disconnected hardware never produces RViz's white error material.
      setRobotModelVisibility(
        root, rebot_model_ready_, piper_model_ready_);
      for (int index = 0; index < root->numDisplays(); ++index) {
        auto * display = root->getDisplayAt(index);
        if (!display || display->getClassId() != QStringLiteral(
            "moveit_rviz_plugin/MotionPlanning"))
        {
          continue;
        }
        if (auto * property = display->subProp(QStringLiteral("Move Group Namespace"))) {
          property->setValue(rebot ? QStringLiteral("/rebotarm") : QStringLiteral("/piperh"));
        }
        if (auto * property = display->subProp(QStringLiteral("Robot Description"))) {
          property->setValue(
            rebot ? QStringLiteral("robot_description") :
            QStringLiteral("piperh_robot_description"));
        }
      }
    }
  }
}

void DualArmSelectorPanel::updateRebotJointState(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (rebot_model_ready_ || !hasCompleteArmState(*message)) {return;}
  rebot_model_ready_ = true;
  if (getDisplayContext()) {
    setRobotModelVisibility(
      getDisplayContext()->getRootDisplayGroup(), true, piper_model_ready_);
  }
}

void DualArmSelectorPanel::updateArmed(const QString & robot, bool armed)
{
  QLabel * label = robot == QStringLiteral("rebotarm") ? rebot_state_label_ : piper_state_label_;
  const QString name = robot == QStringLiteral("rebotarm") ? QStringLiteral("reBotArm") : QStringLiteral("Piper-H");
  label->setText(QStringLiteral("%1：%2").arg(name, armed ? QStringLiteral("ARMED") : QStringLiteral("LOCKED")));
  label->setStyleSheet(armed ? QStringLiteral("color: #c22; font-weight: bold;") : QStringLiteral("color: #16803a;"));
  if (robot == QStringLiteral("piperh")) {
    piper_armed_ = armed;
    piper_armed_known_ = true;
    piper_xbox_request_pending_ = false;
    piper_xbox_visual_state_ = armed ?
      ControlVisualState::Enabled : ControlVisualState::Disabled;
    configurePiperControls(piper_selected_);
  }
}

void DualArmSelectorPanel::updateStatus(const QString & text)
{
  status_label_->setText(text);
}

void DualArmSelectorPanel::configurePiperControls(bool selected)
{
  if (offline_preview_) {
    piper_motor_enable_button_->setEnabled(false);
    piper_motor_disable_button_->setEnabled(false);
    piper_armed_button_->setEnabled(false);
    piper_gravity_button_->setEnabled(false);
    piper_stop_button_->setEnabled(false);
    piper_reload_zones_button_->setEnabled(false);
    piper_armed_button_->setText(QStringLiteral("真机控制（离线禁用）"));
    piper_control_status_label_->setText(QStringLiteral(
      "离线模式只允许 MoveIt/RViz 模型动作，不会向 Piper-H 真机发送命令。"));
    piper_control_status_label_->setStyleSheet(QStringLiteral("color: #5f6b76;"));
    piper_motor_enable_button_->setStyleSheet(QString());
    piper_motor_disable_button_->setStyleSheet(QString());
    piper_armed_button_->setStyleSheet(QString());
    piper_gravity_button_->setStyleSheet(QString());
    return;
  }
  const bool feedback_fresh = last_piper_feedback_ms_ > 0 &&
    steadyMilliseconds() - last_piper_feedback_ms_ <= 1500;
  const bool status_fresh = last_piper_arm_status_ms_ > 0 &&
    steadyMilliseconds() - last_piper_arm_status_ms_ <= 1500;
  piper_motor_enable_button_->setEnabled(
    selected && feedback_fresh && !piper_motor_request_pending_ &&
    !piper_gravity_active_ && !piper_gravity_transition_);
  piper_motor_disable_button_->setEnabled(selected && !piper_motor_request_pending_);
  piper_armed_button_->setEnabled(
    selected && feedback_fresh && piper_armed_known_ && !piper_xbox_request_pending_ &&
    !piper_gravity_active_ && !piper_gravity_transition_);
  const auto gravity_client = piper_gravity_active_ ?
    piper_gravity_stop_client_ : piper_gravity_start_client_;
  piper_gravity_button_->setText(piper_gravity_request_pending_ ?
    QStringLiteral("正在切换 Piper-H 重力补偿…") : piper_gravity_active_ ?
    QStringLiteral("关闭 Piper-H 重力补偿") :
    QStringLiteral("开启 Piper-H 重力补偿"));
  piper_gravity_button_->setEnabled(
    selected && status_fresh && piper_teach_resting_ &&
    !piper_gravity_transition_ && !piper_gravity_request_pending_ &&
    gravity_client && gravity_client->service_is_ready() &&
    (piper_gravity_active_ ||
    (feedback_fresh && piper_motor_enabled_ && piper_armed_known_ && !piper_armed_)));
  QString gravity_reason;
  if (!selected) {
    gravity_reason = QStringLiteral("请先选择 Piper-H");
  } else if (!status_fresh || !feedback_fresh) {
    gravity_reason = QStringLiteral("等待 Piper-H 状态和关节反馈");
  } else if (!piper_motor_enabled_ && !piper_gravity_active_) {
    gravity_reason = QStringLiteral("电机尚未使能：先点击“使能 Piper-H 电机（现场确认）”");
  } else if (!piper_armed_known_ || piper_armed_) {
    gravity_reason = QStringLiteral("等待 Xbox LOCKED 状态");
  } else if (!piper_teach_resting_ || piper_gravity_transition_ ||
    piper_gravity_request_pending_)
  {
    gravity_reason = QStringLiteral("Piper-H 模式或示教状态正在切换");
  } else if (!gravity_client || !gravity_client->service_is_ready()) {
    gravity_reason = QStringLiteral("重力补偿服务尚未就绪");
  }
  piper_gravity_button_->setToolTip(gravity_reason.isEmpty() ?
    QStringLiteral("仅使用普通关节反馈；真实力矩默认关闭，未通过预检时使用失能被动录制") : gravity_reason);
  piper_stop_button_->setEnabled(selected);
  piper_reload_zones_button_->setEnabled(selected);
  piper_motor_enable_button_->setText(
    piper_motor_request_pending_ ? QStringLiteral("正在确认 Piper-H 电机状态…") :
    QStringLiteral("使能 Piper-H 电机（现场确认）"));
  piper_armed_button_->setText(
    !piper_armed_known_ ? QStringLiteral("等待 Xbox 控制器状态") :
    (piper_armed_ ? QStringLiteral("锁定 Piper-H Xbox 控制") :
    QStringLiteral("启用 Piper-H Xbox 控制")));
  applyPiperControlStyles();
}

void DualArmSelectorPanel::applyPiperControlStyles()
{
  const QString enabled_style = QStringLiteral(
    "QPushButton { background-color: #2e7d32; color: white; font-weight: bold; }"
    "QPushButton:disabled { background-color: #8fbc91; color: #f4f4f4; }");
  const QString disabled_style = QStringLiteral(
    "QPushButton { background-color: #546e7a; color: white; font-weight: bold; }"
    "QPushButton:disabled { background-color: #a3afb5; color: #f4f4f4; }");
  const QString pending_style = QStringLiteral(
    "QPushButton { background-color: #ed8b00; color: white; font-weight: bold; }");
  const QString error_style = QStringLiteral(
    "QPushButton { background-color: #c62828; color: white; font-weight: bold; }");

  piper_motor_enable_button_->setStyleSheet(QString());
  piper_motor_disable_button_->setStyleSheet(QString());
  if (piper_motor_visual_state_ == ControlVisualState::Pending) {
    piper_motor_enable_button_->setStyleSheet(pending_style);
    piper_motor_disable_button_->setStyleSheet(pending_style);
  } else if (piper_motor_visual_state_ == ControlVisualState::Enabled) {
    piper_motor_enable_button_->setStyleSheet(enabled_style);
  } else if (piper_motor_visual_state_ == ControlVisualState::Disabled) {
    piper_motor_disable_button_->setStyleSheet(disabled_style);
  } else if (piper_motor_visual_state_ == ControlVisualState::Error) {
    piper_motor_enable_button_->setStyleSheet(error_style);
    piper_motor_disable_button_->setStyleSheet(error_style);
  }

  if (piper_xbox_request_pending_ ||
    piper_xbox_visual_state_ == ControlVisualState::Pending)
  {
    piper_armed_button_->setStyleSheet(pending_style);
  } else if (piper_xbox_visual_state_ == ControlVisualState::Enabled) {
    piper_armed_button_->setStyleSheet(enabled_style);
  } else if (piper_xbox_visual_state_ == ControlVisualState::Disabled) {
    piper_armed_button_->setStyleSheet(disabled_style);
  } else if (piper_xbox_visual_state_ == ControlVisualState::Error) {
    piper_armed_button_->setStyleSheet(error_style);
  } else {
    piper_armed_button_->setStyleSheet(QString());
  }
  piper_gravity_button_->setStyleSheet(
    piper_gravity_request_pending_ || piper_gravity_transition_ ? pending_style :
    piper_gravity_active_ ? enabled_style : QString());
}

void DualArmSelectorPanel::togglePiperGravity()
{
  if (!piper_selected_ || piper_gravity_request_pending_ || !piper_teach_resting_) {return;}
  const bool starting = !piper_gravity_active_;
  auto client = starting ? piper_gravity_start_client_ : piper_gravity_stop_client_;
  if (!client || !client->service_is_ready()) {
    piper_control_status_label_->setText(QStringLiteral("Piper-H 重力补偿服务尚未就绪"));
    return;
  }
  if (starting && QMessageBox::warning(
      this, QStringLiteral("确认开启 Piper-H 重力补偿"),
      QStringLiteral("机械臂将切换到零力拖动，切换时复位可能使机械臂失去支撑。请托住机械臂，"
        "确认安装方向与控制器配置一致、急停可用且工作区无人。是否继续？"),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
  {
    return;
  }
  piper_gravity_request_pending_ = true;
  piper_control_status_label_->setText(starting ?
    QStringLiteral("正在开启 Piper-H 重力补偿…") :
    QStringLiteral("正在关闭 Piper-H 重力补偿…"));
  configurePiperControls(true);
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this, starting](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      QMetaObject::invokeMethod(this, [this, starting, success, detail]() {
        piper_gravity_request_pending_ = false;
        const QString displayed_detail = detail.contains(
          QStringLiteral("calibration is not confirmed"), Qt::CaseInsensitive) ?
          QStringLiteral("六轴 MIT 力矩方向和增益尚未完成现场标定；未向电机发送力矩命令") : detail;
        piper_control_status_label_->setText(success ?
          (starting ? QStringLiteral("Piper-H 重力补偿已开启") :
          QStringLiteral("Piper-H 重力补偿已关闭")) :
          QStringLiteral("重力补偿切换失败：%1").arg(displayed_detail));
        piper_control_status_label_->setStyleSheet(success ?
          QStringLiteral("color: #16803a; font-weight: bold;") :
          QStringLiteral("color: #c22; font-weight: bold;"));
        configurePiperControls(piper_selected_);
      }, Qt::QueuedConnection);
    });
}

void DualArmSelectorPanel::enablePiperMotors()
{
  requestPiperMotorState(true);
}

void DualArmSelectorPanel::disablePiperMotors()
{
  requestPiperMotorState(false);
}

void DualArmSelectorPanel::requestPiperMotorState(bool requested_state)
{
  if (!piper_selected_ || !piper_enable_client_ ||
    !piper_enable_client_->service_is_ready())
  {
    piper_control_status_label_->setText(QStringLiteral("Piper-H 电机使能服务尚未就绪"));
    piper_control_status_label_->setStyleSheet(QStringLiteral("color: #c22; font-weight: bold;"));
    piper_motor_visual_state_ = ControlVisualState::Error;
    applyPiperControlStyles();
    return;
  }
  if (requested_state && QMessageBox::question(
      this, QStringLiteral("确认使能 Piper-H"),
      QStringLiteral("请确认急停可用、机械臂周围无人且运动空间无障碍物。\n\n"
        "是否使能 Piper-H 六个关节电机？"),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
  {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = requested_state;
  piper_motor_request_pending_ = true;
  piper_motor_visual_state_ = ControlVisualState::Pending;
  piper_control_status_label_->setText(
    requested_state ? QStringLiteral("正在使能 Piper-H 电机并等待驱动确认…") :
    QStringLiteral("正在失能 Piper-H 电机并等待驱动确认…"));
  piper_control_status_label_->setStyleSheet(
    QStringLiteral("color: #d67c00; font-weight: bold;"));
  configurePiperControls(true);
  piper_enable_client_->async_send_request(
    request,
    [this, requested_state](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      bool success = false;
      QString message;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {message = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        message = QString::fromLocal8Bit(error.what());
      }
      QMetaObject::invokeMethod(this, [this, requested_state, success, message]() {
        piper_motor_request_pending_ = false;
        if (success) {
          piper_motor_visual_state_ = requested_state ?
            ControlVisualState::Enabled : ControlVisualState::Disabled;
          piper_control_status_label_->setText(
            requested_state ? QStringLiteral("Piper-H 六个关节电机已使能") :
            QStringLiteral("Piper-H 六个关节电机已失能"));
          piper_control_status_label_->setStyleSheet(requested_state ?
            QStringLiteral("color: #16803a; font-weight: bold;") :
            QStringLiteral("color: #456a8a; font-weight: bold;"));
        } else {
          piper_motor_visual_state_ = ControlVisualState::Error;
          piper_control_status_label_->setText(
            QStringLiteral("Piper-H 电机%1失败或等待驱动确认超时")
            .arg(requested_state ? QStringLiteral("使能") : QStringLiteral("失能")) +
            (message.isEmpty() ? QString() : QStringLiteral("：%1").arg(message)));
          piper_control_status_label_->setStyleSheet(
            QStringLiteral("color: #c22; font-weight: bold;"));
        }
        configurePiperControls(piper_selected_);
      }, Qt::QueuedConnection);
    });
}

void DualArmSelectorPanel::togglePiperArmed()
{
  if (!piper_selected_ || !piper_armed_client_ || !piper_armed_client_->service_is_ready()) {
    piper_control_status_label_->setText(QStringLiteral("Piper-H 锁定服务尚未就绪"));
    piper_control_status_label_->setStyleSheet(QStringLiteral("color: #c22; font-weight: bold;"));
    piper_xbox_visual_state_ = ControlVisualState::Error;
    applyPiperControlStyles();
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !piper_armed_;
  piper_xbox_request_pending_ = true;
  piper_xbox_visual_state_ = ControlVisualState::Pending;
  piper_armed_button_->setEnabled(false);
  const bool requested_state = request->data;
  piper_control_status_label_->setText(requested_state ?
    QStringLiteral("正在启用 Piper-H Xbox 控制…") :
    QStringLiteral("正在锁定 Piper-H Xbox 控制…"));
  piper_control_status_label_->setStyleSheet(
    QStringLiteral("color: #d67c00; font-weight: bold;"));
  applyPiperControlStyles();
  piper_armed_client_->async_send_request(
    request, [this, requested_state](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      bool success = false;
      QString message;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {message = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        message = QString::fromLocal8Bit(error.what());
      }
      QMetaObject::invokeMethod(this, [this, success, requested_state, message]() {
        piper_xbox_request_pending_ = false;
        if (success) {
          piper_xbox_visual_state_ = requested_state ?
            ControlVisualState::Enabled : ControlVisualState::Disabled;
          piper_control_status_label_->setText(
            QStringLiteral("Piper-H Xbox 控制已%1：%2").arg(
              requested_state ? QStringLiteral("启用") : QStringLiteral("锁定"), message));
          piper_control_status_label_->setStyleSheet(requested_state ?
            QStringLiteral("color: #16803a; font-weight: bold;") :
            QStringLiteral("color: #456a8a; font-weight: bold;"));
        } else if (message.contains(QStringLiteral("fresh input"), Qt::CaseInsensitive)) {
          piper_xbox_visual_state_ = ControlVisualState::Error;
          piper_control_status_label_->setText(QStringLiteral(
            "未收到 Xbox 手柄输入；动作编排和 MoveIt 仍可使用，只有 Xbox 实时控制不能启用。"));
          piper_control_status_label_->setStyleSheet(
            QStringLiteral("color: #c22; font-weight: bold;"));
        } else {
          piper_xbox_visual_state_ = ControlVisualState::Error;
          piper_control_status_label_->setText(
            QStringLiteral("Xbox 控制状态切换失败：%1").arg(message));
          piper_control_status_label_->setStyleSheet(
            QStringLiteral("color: #c22; font-weight: bold;"));
        }
        configurePiperControls(piper_selected_);
      }, Qt::QueuedConnection);
    });
}

void DualArmSelectorPanel::stopPiperMotion()
{
  if (!piper_selected_) {return;}
  if (piper_gravity_active_) {
    auto gravity_cancel = piper_teach_recording_ ?
      piper_teach_cancel_client_ : piper_gravity_stop_client_;
    if (gravity_cancel && gravity_cancel->service_is_ready()) {
      gravity_cancel->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    }
  }
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  int sent = 0;
  for (const auto & client : {
      piper_move_cancel_client_, piper_execute_cancel_client_, piper_controller_cancel_client_})
  {
    if (client && client->service_is_ready()) {
      client->async_send_request(request);
      ++sent;
    }
  }
  if (piper_armed_client_ && piper_armed_client_->service_is_ready()) {
    auto lock_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    lock_request->data = false;
    piper_armed_client_->async_send_request(lock_request);
  }
  piper_control_status_label_->setText(
    QStringLiteral("已向 %1 个运动接口发送取消，并请求锁定 Piper-H").arg(sent));
}

void DualArmSelectorPanel::reloadPiperZones()
{
  if (!piper_selected_ || !piper_reload_zones_client_ ||
    !piper_reload_zones_client_->service_is_ready())
  {
    piper_control_status_label_->setText(QStringLiteral("Piper-H 禁区服务尚未就绪"));
    return;
  }
  piper_reload_zones_button_->setEnabled(false);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  piper_reload_zones_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      const QString message = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, response, message]() {
        piper_control_status_label_->setText(
          response->success ? QStringLiteral("禁区配置已重新加载：%1").arg(message) :
          QStringLiteral("禁区配置加载失败：%1").arg(message));
        piper_reload_zones_button_->setEnabled(piper_selected_);
      }, Qt::QueuedConnection);
    });
}

void DualArmSelectorPanel::updatePiperJointState(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (!hasCompleteArmState(*message)) {return;}
  last_piper_feedback_ms_ = steadyMilliseconds();
  if (!piper_model_ready_) {
    piper_model_ready_ = true;
    if (getDisplayContext()) {
      setRobotModelVisibility(
        getDisplayContext()->getRootDisplayGroup(), rebot_model_ready_, true);
    }
  }
  std::map<std::string, double> positions;
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    positions[message->name[index]] = message->position[index];
  }
  QStringList values;
  constexpr double radians_to_degrees = 180.0 / M_PI;
  for (int index = 1; index <= 6; ++index) {
    const auto iterator = positions.find("joint" + std::to_string(index));
    if (iterator == positions.end() || !std::isfinite(iterator->second)) {return;}
    values.append(QStringLiteral("J%1 %2°").arg(index).arg(
      iterator->second * radians_to_degrees, 0, 'f', 1));
  }
  piper_joint_label_->setText(values.join(QStringLiteral("    ")));
  if (piper_control_status_label_->text() == QStringLiteral("等待 Piper-H 控制接口…") ||
    piper_control_status_label_->text().startsWith(QStringLiteral("Piper-H 反馈已超时")))
  {
    piper_control_status_label_->setText(QStringLiteral(
      "Piper-H 反馈已连接；动作编排和 MoveIt 可直接使用，Xbox 实时控制需先连接手柄。"));
  }
  configurePiperControls(piper_selected_);
}

}  // namespace rebotarm_demo_rviz

PLUGINLIB_EXPORT_CLASS(rebotarm_demo_rviz::DualArmSelectorPanel, rviz_common::Panel)
