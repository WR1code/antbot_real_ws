#include "vehicle_status_panel.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <map>
#include <string>

#include <QFont>
#include <QFrame>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QKeyEvent>
#include <QMetaObject>
#include <QMessageBox>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>

#include "pluginlib/class_list_macros.hpp"
#include "robotcar_navigation/rviz_chinese_font.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace robotcar_navigation
{
namespace rviz_plugins
{

namespace
{
QLabel * makeValueLabel(QWidget * parent)
{
  auto * label = new QLabel(QStringLiteral("--"), parent);
  QFont font = label->font();
  font.setBold(true);
  font.setPointSize(font.pointSize() + 1);
  label->setFont(font);
  label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  return label;
}

QString ageText(
  const std::chrono::steady_clock::time_point & then,
  const std::chrono::steady_clock::time_point & now)
{
  const double seconds =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count() / 1000.0;
  return QString::number(seconds, 'f', 1);
}

QString chassisKeyLabel(const QString & key)
{
  static const std::map<QString, QString> labels = {
    {"connection", "H743 连接"}, {"connection_detail", "连接详情"},
    {"autonomous_navigation", "自动导航"}, {"operator_requested", "安全请求"},
    {"operator_enabled", "运动许可"}, {"telemetry_fresh", "遥测新鲜"},
    {"chassis_state", "底盘状态"}, {"fault_flags", "总故障标志"},
    {"steering", "RS00 转向"}, {"mini", "MINI 行走驱动"},
    {"can", "CAN 总线"}, {"stm32_tick_ms", "STM32 运行时间 ms"},
    {"enabled", "已使能"}, {"homed", "已回零"}, {"ready", "已就绪"},
    {"fault", "故障"}, {"position_deg", "位置 °"},
    {"feedback_age_ms", "反馈年龄 ms"}, {"speed_erpm", "转速 erpm"},
    {"current_a", "电流 A"}, {"temperature_c", "温度 °C"},
    {"fault_code", "故障码"}, {"fault_bits", "故障码兼容字段"},
    {"valid_mask", "有效位"}, {"safety_flags", "安全标志"},
    {"voltage_v", "母线电压 V"}, {"flags", "CAN 标志"},
    {"tx_count", "发送计数"}, {"rx_count", "接收计数"},
  };
  const auto found = labels.find(key);
  return found == labels.end() ? key : found->second;
}

QString compactJson(const QJsonValue & value)
{
  if (value.isString()) {
    return value.toString();
  }
  if (value.isDouble()) {
    return QString::number(value.toDouble(), 'f', 3);
  }
  if (value.isBool()) {
    return value.toBool() ? QObject::tr("是") : QObject::tr("否");
  }
  if (value.isNull() || value.isUndefined()) {
    return QStringLiteral("--");
  }
  if (value.isArray()) {
    return QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
  }
  return QString();
}

void appendStatusTree(
  QTreeWidget * tree, QTreeWidgetItem * parent,
  const QString & key, const QJsonValue & value)
{
  auto * item = parent ? new QTreeWidgetItem(parent) : new QTreeWidgetItem(tree);
  item->setText(0, chassisKeyLabel(key));
  if (value.isObject()) {
    const auto object = value.toObject();
    for (auto iterator = object.begin(); iterator != object.end(); ++iterator) {
      appendStatusTree(tree, item, iterator.key(), iterator.value());
    }
    item->setExpanded(true);
  } else {
    item->setText(1, compactJson(value));
  }
}
}  // namespace

VehicleStatusPanel::VehicleStatusPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  configureChineseUiFont();
  setMinimumSize(0, 0);
  setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

  auto * outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(0, 0, 0, 0);
  auto * tabs = new QTabWidget(this);
  tabs->setDocumentMode(true);
  tabs->setUsesScrollButtons(true);
  tabs->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

  auto make_page = [tabs](const QString & title) {
    auto * scroll = new QScrollArea(tabs);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setMinimumSize(0, 0);
    scroll->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    scroll->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    auto * content = new QWidget(scroll);
    content->setMinimumSize(0, 0);
    auto * layout = new QVBoxLayout(content);
    scroll->setWidget(content);
    tabs->addTab(scroll, title);
    return layout;
  };
  auto * root = make_page(tr("控制与安全"));
  auto * chassis_root = make_page(tr("底盘信息"));
  auto * operation_root = make_page(tr("建图与遥控"));
  auto * camera_root = make_page(tr("相机"));
  outer_layout->addWidget(tabs, 1);

  auto * safety_box = new QGroupBox(tr("真机安全门禁"), this);
  auto * safety_layout = new QVBoxLayout(safety_box);
  hardware_status_ = new QLabel(tr("底盘未连接 · 控制已锁定"), safety_box);
  hardware_status_->setWordWrap(true);
  hardware_status_->setStyleSheet(QStringLiteral("color: #d32f2f; font-weight: bold;"));
  auto * safety_buttons = new QHBoxLayout();
  safety_enable_button_ = new QPushButton(tr("确认安全并启用"), safety_box);
  safety_enable_button_->setStyleSheet(
    QStringLiteral("QPushButton { background: #2e7d32; color: white; padding: 8px; }"));
  safety_stop_button_ = new QPushButton(tr("停止并锁定"), safety_box);
  safety_stop_button_->setStyleSheet(
    QStringLiteral("QPushButton { background: #c62828; color: white; padding: 8px; }"));
  safety_buttons->addWidget(safety_enable_button_);
  safety_buttons->addWidget(safety_stop_button_);
  safety_layout->addWidget(hardware_status_);
  safety_layout->addLayout(safety_buttons);
  root->addWidget(safety_box);
  connect(
    safety_enable_button_, &QPushButton::clicked,
    this, &VehicleStatusPanel::requestSafetyEnable);
  connect(
    safety_stop_button_, &QPushButton::clicked,
    this, &VehicleStatusPanel::requestSafetyStop);

  auto * intent_box = new QGroupBox(tr("机器人动作意图"), this);
  auto * intent_layout = new QVBoxLayout(intent_box);
  intent_value_ = new QLabel(tr("等待意图分析…"), intent_box);
  QFont intent_font = intent_value_->font();
  intent_font.setBold(true);
  intent_font.setPointSize(intent_font.pointSize() + 4);
  intent_value_->setFont(intent_font);
  intent_value_->setAlignment(Qt::AlignCenter);
  intent_value_->setWordWrap(true);
  intent_value_->setMinimumHeight(54);
  intent_detail_ = new QLabel(
    tr("接口：/antbot/robot_intent · 灯带：/antbot/led_intent"), intent_box);
  intent_detail_->setWordWrap(true);
  intent_layout->addWidget(intent_value_);
  intent_layout->addWidget(intent_detail_);
  root->addWidget(intent_box);

  auto * speed_box = new QGroupBox(tr("小车实际速度（里程计反馈）"), this);
  auto * speed_layout = new QGridLayout(speed_box);
  speed_value_ = makeValueLabel(speed_box);
  velocity_x_value_ = makeValueLabel(speed_box);
  velocity_y_value_ = makeValueLabel(speed_box);
  angular_z_value_ = makeValueLabel(speed_box);
  speed_layout->addWidget(new QLabel(tr("平面合速度"), speed_box), 0, 0);
  speed_layout->addWidget(speed_value_, 0, 1);
  speed_layout->addWidget(new QLabel(tr("前后速度 vx"), speed_box), 1, 0);
  speed_layout->addWidget(velocity_x_value_, 1, 1);
  speed_layout->addWidget(new QLabel(tr("横向速度 vy"), speed_box), 2, 0);
  speed_layout->addWidget(velocity_y_value_, 2, 1);
  speed_layout->addWidget(new QLabel(tr("角速度 wz"), speed_box), 3, 0);
  speed_layout->addWidget(angular_z_value_, 3, 1);
  odometry_status_ = new QLabel(tr("等待 /odometry/filtered…"), speed_box);
  odometry_status_->setWordWrap(true);
  speed_layout->addWidget(odometry_status_, 4, 0, 1, 2);
  root->addWidget(speed_box);

  auto * state_box = new QGroupBox(tr("小车状态与扩展参数"), this);
  auto * state_layout = new QVBoxLayout(state_box);
  battery_bar_ = new QProgressBar(state_box);
  battery_bar_->setRange(0, 100);
  battery_bar_->setValue(0);
  battery_bar_->setFormat(tr("电量：等待 /battery…"));
  battery_detail_ = new QLabel(
    tr("电池接口已预留：电压、电流、温度和充电状态"), state_box);
  battery_detail_->setWordWrap(true);
  extended_status_ = new QTreeWidget(state_box);
  extended_status_->setHeaderLabels({tr("底盘查询项"), tr("值")});
  extended_status_->setRootIsDecorated(true);
  extended_status_->setMinimumHeight(320);
  auto * placeholder = new QTreeWidgetItem(extended_status_);
  placeholder->setText(0, tr("接口"));
  placeholder->setText(1, tr("等待 /antbot/vehicle_status…"));
  state_layout->addWidget(battery_bar_);
  state_layout->addWidget(battery_detail_);
  state_layout->addWidget(extended_status_);
  chassis_root->addWidget(state_box);

  auto * teleop_box = new QGroupBox(tr("常驻遥控方式"), this);
  auto * teleop_layout = new QVBoxLayout(teleop_box);
  xbox_mode_button_ = new QRadioButton(tr("Xbox 手柄"), teleop_box);
  keyboard_mode_button_ = new QRadioButton(tr("RViz 键盘 / 屏幕按钮"), teleop_box);
  xbox_mode_button_->setChecked(true);
  auto * teleop_choices = new QHBoxLayout();
  teleop_choices->addWidget(xbox_mode_button_);
  teleop_choices->addWidget(keyboard_mode_button_);
  teleop_status_ = new QLabel(tr("等待 /antbot/operator_ui_status…"), teleop_box);
  teleop_status_->setWordWrap(true);
  teleop_layout->addLayout(teleop_choices);
  teleop_layout->addWidget(teleop_status_);
  operation_root->addWidget(teleop_box);
  connect(xbox_mode_button_, &QRadioButton::clicked, this, [this]() {
    selectTeleopMode(true);
  });
  connect(keyboard_mode_button_, &QRadioButton::clicked, this, [this]() {
    selectTeleopMode(false);
  });

  control_stack_ = new QStackedWidget(this);

  auto * xbox_box = new QGroupBox(tr("Xbox 手柄操作说明"), control_stack_);
  auto * xbox_layout = new QVBoxLayout(xbox_box);
  auto * xbox_help = new QLabel(
    tr("① 确认界面显示“手柄连接正常”\n"
       "② 保持左摇杆回中，按 A 解锁/锁定手柄输出\n"
       "③ 左摇杆前后控制前进/后退，左右控制横移\n"
       "④ 按下左/右摇杆降低/提高速度档位\n\n"
       "真机还必须在“控制与安全”页通过 H743 安全门禁。\n"
       "当前底盘不支持 angular.z，LT/RT 旋转指令会被丢弃；"
       "地图请使用下方 RViz 建图按钮保存。"), xbox_box);
  xbox_help->setWordWrap(true);
  xbox_help->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  xbox_help->setStyleSheet(QStringLiteral(
    "QLabel { border: 1px solid #90a4ae; border-radius: 5px; padding: 10px; }"));
  xbox_layout->addWidget(xbox_help);
  control_stack_->addWidget(xbox_box);

  auto * keyboard_box = new QGroupBox(tr("键盘全向控制"), control_stack_);
  auto * keyboard_layout = new QVBoxLayout(keyboard_box);
  keyboard_capture_ = new QLabel(
    tr("点击这里取得键盘焦点，然后按 Q/W/E/A/D/Z/X/C；松键即停车，空格急停。"),
    keyboard_box);
  keyboard_capture_->setWordWrap(true);
  keyboard_capture_->setAlignment(Qt::AlignCenter);
  keyboard_capture_->setMinimumHeight(54);
  keyboard_capture_->setFocusPolicy(Qt::StrongFocus);
  keyboard_capture_->installEventFilter(this);
  keyboard_capture_->setStyleSheet(QStringLiteral(
    "QLabel { border: 2px solid #607d8b; border-radius: 5px; padding: 8px; }"
    "QLabel:focus { border-color: #1976d2; background: #e3f2fd; color: #0d47a1; }"));
  keyboard_layout->addWidget(keyboard_capture_);

  auto * drive_grid = new QGridLayout();
  auto add_drive_button = [this, drive_grid, keyboard_box](
    const QString & text, int row, int column, int forward, int left)
    {
      auto * button = new QPushButton(text, keyboard_box);
      button->setMinimumHeight(36);
      drive_grid->addWidget(button, row, column);
      connect(button, &QPushButton::pressed, this, [this, forward, left]() {
        setButtonMotion(forward, left);
      });
      connect(button, &QPushButton::released, this, [this]() {
        clearButtonMotion();
      });
    };
  add_drive_button(tr("↖ Q"), 0, 0, 1, 1);
  add_drive_button(tr("↑ W"), 0, 1, 1, 0);
  add_drive_button(tr("↗ E"), 0, 2, 1, -1);
  add_drive_button(tr("← A"), 1, 0, 0, 1);
  add_drive_button(tr("停止"), 1, 1, 0, 0);
  add_drive_button(tr("→ D"), 1, 2, 0, -1);
  add_drive_button(tr("↙ Z"), 2, 0, -1, 1);
  add_drive_button(tr("↓ X"), 2, 1, -1, 0);
  add_drive_button(tr("↘ C"), 2, 2, -1, -1);
  keyboard_layout->addLayout(drive_grid);
  auto * speed_layout_row = new QHBoxLayout();
  speed_layout_row->addWidget(new QLabel(tr("键盘速度"), keyboard_box));
  keyboard_speed_ = new QDoubleSpinBox(keyboard_box);
  keyboard_speed_->setRange(0.01, 0.10);
  keyboard_speed_->setSingleStep(0.01);
  keyboard_speed_->setValue(0.05);
  keyboard_speed_->setSuffix(tr(" m/s"));
  speed_layout_row->addWidget(keyboard_speed_);
  keyboard_layout->addLayout(speed_layout_row);
  control_stack_->addWidget(keyboard_box);
  control_stack_->setCurrentWidget(xbox_box);
  operation_root->addWidget(control_stack_);

  auto * mapping_box = new QGroupBox(tr("二维建图（当前 RViz 内显示）"), this);
  auto * mapping_layout = new QVBoxLayout(mapping_box);
  mapping_status_ = new QLabel(
    tr("等待建图管理器；开始前需要 /scan_0 雷达数据。"), mapping_box);
  mapping_status_->setWordWrap(true);
  auto * mapping_buttons = new QHBoxLayout();
  mapping_start_button_ = new QPushButton(tr("开始建图"), mapping_box);
  mapping_stop_button_ = new QPushButton(tr("停止建图"), mapping_box);
  mapping_save_button_ = new QPushButton(tr("保存当前地图"), mapping_box);
  mapping_buttons->addWidget(mapping_start_button_);
  mapping_buttons->addWidget(mapping_stop_button_);
  mapping_buttons->addWidget(mapping_save_button_);
  mapping_layout->addWidget(mapping_status_);
  mapping_layout->addLayout(mapping_buttons);
  operation_root->addWidget(mapping_box);
  connect(mapping_start_button_, &QPushButton::clicked, this, [this]() {
    setMappingEnabled(true);
  });
  connect(mapping_stop_button_, &QPushButton::clicked, this, [this]() {
    setMappingEnabled(false);
  });
  connect(mapping_save_button_, &QPushButton::clicked, this, &VehicleStatusPanel::saveMapping);

  auto * camera_box = new QGroupBox(tr("车载 RGB 摄像头"), this);
  auto * camera_layout = new QVBoxLayout(camera_box);
  camera_view_ = new QLabel(tr("等待摄像头画面…"), camera_box);
  camera_view_->setAlignment(Qt::AlignCenter);
  camera_view_->setMinimumSize(160, 120);
  camera_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  camera_view_->setFrameShape(QFrame::StyledPanel);
  camera_view_->setStyleSheet(QStringLiteral("QLabel { background: #151515; color: #cfcfcf; }"));
  camera_status_ = new QLabel(tr("话题：/antbot/camera/color/image_raw"), camera_box);
  camera_status_->setWordWrap(true);
  camera_layout->addWidget(camera_view_, 1);
  camera_layout->addWidget(camera_status_);
  camera_root->addWidget(camera_box, 1);
  root->addStretch();
  chassis_root->addStretch();
  operation_root->addStretch();
  camera_root->addStretch();

  auto * timer = new QTimer(this);
  timer->setInterval(500);
  connect(timer, &QTimer::timeout, this, &VehicleStatusPanel::updateDataStatus);
  timer->start();

  auto * keyboard_timer = new QTimer(this);
  keyboard_timer->setInterval(50);
  connect(
    keyboard_timer, &QTimer::timeout,
    this, &VehicleStatusPanel::publishKeyboardCommand);
  keyboard_timer->start();
}

void VehicleStatusPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    hardware_status_->setText(tr("无法取得 RViz ROS 节点 · 控制已锁定"));
    safety_enable_button_->setEnabled(false);
    safety_stop_button_->setEnabled(false);
    odometry_status_->setText(tr("无法取得 RViz ROS 节点"));
    odometry_status_->setStyleSheet(QStringLiteral("color: #d32f2f;"));
    camera_status_->setText(tr("无法取得 RViz ROS 节点"));
    camera_status_->setStyleSheet(QStringLiteral("color: #d32f2f;"));
    teleop_status_->setText(tr("无法取得 RViz ROS 节点"));
    mapping_status_->setText(tr("无法取得 RViz ROS 节点"));
    xbox_mode_button_->setEnabled(false);
    keyboard_mode_button_->setEnabled(false);
    mapping_start_button_->setEnabled(false);
    mapping_stop_button_->setEnabled(false);
    mapping_save_button_->setEnabled(false);
    return;
  }

  node_ = abstraction->get_raw_node();
  operator_enable_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/antbot/operator_enable");
  teleop_mode_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/antbot/teleop/use_xbox");
  mapping_enable_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/antbot/mapping/set_enabled");
  mapping_save_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/antbot/mapping/save");
  keyboard_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/antbot/cmd_vel/keyboard", 10);
  odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", rclcpp::SensorDataQoS(),
    std::bind(&VehicleStatusPanel::handleOdometry, this, std::placeholders::_1));
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/antbot/camera/color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&VehicleStatusPanel::handleImage, this, std::placeholders::_1));
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery", rclcpp::SensorDataQoS(),
    std::bind(&VehicleStatusPanel::handleBattery, this, std::placeholders::_1));
  intent_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/antbot/robot_intent", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&VehicleStatusPanel::handleIntent, this, std::placeholders::_1));
  extended_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/antbot/vehicle_status", 10,
    std::bind(&VehicleStatusPanel::handleExtendedStatus, this, std::placeholders::_1));
  operator_ui_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/antbot/operator_ui_status", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&VehicleStatusPanel::handleOperatorUiStatus, this, std::placeholders::_1));
}

bool VehicleStatusPanel::eventFilter(QObject * watched, QEvent * event)
{
  if (watched != keyboard_capture_) {
    return rviz_common::Panel::eventFilter(watched, event);
  }
  if (event->type() == QEvent::FocusOut) {
    pressed_keys_.clear();
    return false;
  }
  if (event->type() != QEvent::KeyPress && event->type() != QEvent::KeyRelease) {
    return false;
  }
  auto * key_event = static_cast<QKeyEvent *>(event);
  const int key = key_event->key();
  const QSet<int> drive_keys = {
    Qt::Key_Q, Qt::Key_W, Qt::Key_E, Qt::Key_A, Qt::Key_D,
    Qt::Key_Z, Qt::Key_X, Qt::Key_C, Qt::Key_Space,
  };
  if (!drive_keys.contains(key)) {
    return false;
  }
  if (event->type() == QEvent::KeyPress) {
    pressed_keys_.insert(key);
  } else {
    pressed_keys_.remove(key);
  }
  key_event->accept();
  return true;
}

void VehicleStatusPanel::setButtonMotion(int forward, int left)
{
  button_forward_ = forward;
  button_left_ = left;
  button_motion_active_ = true;
}

void VehicleStatusPanel::clearButtonMotion()
{
  button_forward_ = 0;
  button_left_ = 0;
  button_motion_active_ = false;
  if (keyboard_cmd_pub_) {
    keyboard_cmd_pub_->publish(geometry_msgs::msg::Twist());
  }
}

void VehicleStatusPanel::publishKeyboardCommand()
{
  if (!keyboard_cmd_pub_ || !keyboard_mode_button_->isChecked()) {
    return;
  }
  int forward = 0;
  int left = 0;
  if (button_motion_active_) {
    forward = button_forward_;
    left = button_left_;
  } else if (!pressed_keys_.contains(Qt::Key_Space)) {
    if (pressed_keys_.contains(Qt::Key_Q) || pressed_keys_.contains(Qt::Key_W) ||
      pressed_keys_.contains(Qt::Key_E))
    {
      ++forward;
    }
    if (pressed_keys_.contains(Qt::Key_Z) || pressed_keys_.contains(Qt::Key_X) ||
      pressed_keys_.contains(Qt::Key_C))
    {
      --forward;
    }
    if (pressed_keys_.contains(Qt::Key_Q) || pressed_keys_.contains(Qt::Key_A) ||
      pressed_keys_.contains(Qt::Key_Z))
    {
      ++left;
    }
    if (pressed_keys_.contains(Qt::Key_E) || pressed_keys_.contains(Qt::Key_D) ||
      pressed_keys_.contains(Qt::Key_C))
    {
      --left;
    }
  }
  const double magnitude = std::hypot(forward, left);
  geometry_msgs::msg::Twist message;
  if (magnitude > 0.0) {
    const double speed = keyboard_speed_->value();
    message.linear.x = speed * forward / magnitude;
    message.linear.y = speed * left / magnitude;
  }
  keyboard_cmd_pub_->publish(message);
}

void VehicleStatusPanel::selectTeleopMode(bool use_xbox)
{
  pressed_keys_.clear();
  clearButtonMotion();
  control_stack_->setCurrentIndex(use_xbox ? 0 : 1);
  if (!teleop_mode_client_ || !teleop_mode_client_->service_is_ready()) {
    teleop_status_->setText(tr("遥控方式服务未就绪；底盘不会接受新的控制来源"));
    teleop_status_->setStyleSheet(QStringLiteral("color: #d32f2f;"));
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = use_xbox;
  teleop_mode_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      const QString text = QString::fromStdString(response->message);
      const bool success = response->success;
      QMetaObject::invokeMethod(this, [this, text, success]() {
        teleop_status_->setText(text);
        teleop_status_->setStyleSheet(
          success ? QStringLiteral("color: #2e7d32;") :
          QStringLiteral("color: #d32f2f;"));
      }, Qt::QueuedConnection);
    });
}

void VehicleStatusPanel::setMappingEnabled(bool enabled)
{
  if (!mapping_enable_client_ || !mapping_enable_client_->service_is_ready()) {
    mapping_status_->setText(tr("建图服务未就绪"));
    mapping_status_->setStyleSheet(QStringLiteral("color: #d32f2f;"));
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  mapping_start_button_->setEnabled(false);
  mapping_stop_button_->setEnabled(false);
  mapping_enable_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      const QString text = QString::fromStdString(response->message);
      const bool success = response->success;
      QMetaObject::invokeMethod(this, [this, text, success]() {
        mapping_status_->setText(text);
        mapping_status_->setStyleSheet(
          success ? QStringLiteral("color: #2e7d32;") :
          QStringLiteral("color: #d32f2f;"));
        mapping_start_button_->setEnabled(true);
        mapping_stop_button_->setEnabled(true);
      }, Qt::QueuedConnection);
    });
}

void VehicleStatusPanel::saveMapping()
{
  if (!mapping_save_client_ || !mapping_save_client_->service_is_ready()) {
    mapping_status_->setText(tr("地图保存服务未就绪"));
    mapping_status_->setStyleSheet(QStringLiteral("color: #d32f2f;"));
    return;
  }
  mapping_save_button_->setEnabled(false);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  mapping_save_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      const QString text = QString::fromStdString(response->message);
      const bool success = response->success;
      QMetaObject::invokeMethod(this, [this, text, success]() {
        mapping_status_->setText(text);
        mapping_status_->setStyleSheet(
          success ? QStringLiteral("color: #2e7d32;") :
          QStringLiteral("color: #d32f2f;"));
        mapping_save_button_->setEnabled(true);
      }, Qt::QueuedConnection);
    });
}

void VehicleStatusPanel::requestSafetyEnable()
{
  if (!operator_enable_client_ || !operator_enable_client_->service_is_ready()) {
    hardware_status_->setText(tr("底层安全服务未就绪 · 控制保持锁定"));
    hardware_status_->setStyleSheet(QStringLiteral("color: #d32f2f; font-weight: bold;"));
    return;
  }
  const auto answer = QMessageBox::warning(
    this,
    tr("确认真机安全"),
    tr("请确认：\n\n"
       "• 实体急停可用\n"
       "• 转向标定已完成\n"
       "• 轮旁和车辆运动范围内无人\n"
       "• 当前速度上限适合测试\n\n"
       "Xbox 模式仍需按 A 键；键盘模式松键即停车。是否继续？"),
    QMessageBox::Yes | QMessageBox::No,
    QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  safety_enable_button_->setEnabled(false);
  hardware_status_->setText(tr("正在请求 H743 安全使能…"));
  hardware_status_->setStyleSheet(QStringLiteral("color: #ef6c00; font-weight: bold;"));
  operator_enable_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      const QString text = QString::fromStdString(response->message);
      const bool success = response->success;
      QMetaObject::invokeMethod(this, [this, text, success]() {
        hardware_status_->setText(text);
        hardware_status_->setStyleSheet(
          success ? QStringLiteral("color: #ef6c00; font-weight: bold;") :
          QStringLiteral("color: #d32f2f; font-weight: bold;"));
        safety_enable_button_->setEnabled(true);
      }, Qt::QueuedConnection);
    });
}

void VehicleStatusPanel::requestSafetyStop()
{
  if (!operator_enable_client_ || !operator_enable_client_->service_is_ready()) {
    hardware_status_->setText(tr("底层安全服务未就绪 · 无运动权限"));
    hardware_status_->setStyleSheet(QStringLiteral("color: #d32f2f; font-weight: bold;"));
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;
  safety_stop_button_->setEnabled(false);
  operator_enable_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      const QString text = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, text]() {
        hardware_status_->setText(text);
        hardware_status_->setStyleSheet(
          QStringLiteral("color: #d32f2f; font-weight: bold;"));
        safety_stop_button_->setEnabled(true);
        safety_enable_button_->setEnabled(true);
      }, Qt::QueuedConnection);
    });
}

void VehicleStatusPanel::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  const double vx = message->twist.twist.linear.x;
  const double vy = message->twist.twist.linear.y;
  const double wz = message->twist.twist.angular.z;
  const double speed = std::hypot(vx, vy);

  QMetaObject::invokeMethod(
    this,
    [this, speed, vx, vy, wz]() {
      speed_value_->setText(tr("%1 m/s").arg(speed, 0, 'f', 3));
      velocity_x_value_->setText(tr("%1 m/s").arg(vx, 0, 'f', 3));
      velocity_y_value_->setText(tr("%1 m/s").arg(vy, 0, 'f', 3));
      angular_z_value_->setText(tr("%1 rad/s").arg(wz, 0, 'f', 3));
      has_odometry_ = true;
      last_odometry_time_ = std::chrono::steady_clock::now();
      odometry_status_->setText(tr("反馈正常 · /odometry/filtered"));
      odometry_status_->setStyleSheet(QStringLiteral("color: #2e7d32;"));
    },
    Qt::QueuedConnection);
}

QImage VehicleStatusPanel::convertImage(const sensor_msgs::msg::Image & message)
{
  if (message.width == 0 || message.height == 0 || message.step == 0) {
    return {};
  }
  const std::size_t required =
    static_cast<std::size_t>(message.step) * static_cast<std::size_t>(message.height);
  if (message.data.size() < required) {
    return {};
  }

  const auto * data = message.data.data();
  const int width = static_cast<int>(message.width);
  const int height = static_cast<int>(message.height);
  const int step = static_cast<int>(message.step);
  if (message.encoding == "rgb8") {
    return QImage(data, width, height, step, QImage::Format_RGB888).copy();
  }
  if (message.encoding == "bgr8") {
    return QImage(data, width, height, step, QImage::Format_RGB888).rgbSwapped();
  }
  if (message.encoding == "mono8") {
    return QImage(data, width, height, step, QImage::Format_Grayscale8).copy();
  }
  if (message.encoding == "rgba8") {
    return QImage(data, width, height, step, QImage::Format_RGBA8888).copy();
  }
  return {};
}

void VehicleStatusPanel::handleImage(const sensor_msgs::msg::Image::SharedPtr message)
{
  const QImage image = convertImage(*message);
  const QString encoding = QString::fromStdString(message->encoding);
  if (image.isNull()) {
    QMetaObject::invokeMethod(
      this,
      [this, encoding]() {
        camera_status_->setText(tr("不支持或无效的图像格式：%1").arg(encoding));
        camera_status_->setStyleSheet(QStringLiteral("color: #d32f2f;"));
      },
      Qt::QueuedConnection);
    return;
  }

  QMetaObject::invokeMethod(
    this,
    [this, image, encoding]() {showCameraImage(image, encoding);},
    Qt::QueuedConnection);
}

void VehicleStatusPanel::handleBattery(
  const sensor_msgs::msg::BatteryState::SharedPtr message)
{
  const bool percentage_valid = std::isfinite(message->percentage) && message->percentage >= 0.0;
  const int percentage = percentage_valid ?
    std::clamp(static_cast<int>(std::round(message->percentage * 100.0)), 0, 100) : 0;
  const double voltage = message->voltage;
  const double current = message->current;
  const double temperature = message->temperature;
  const int supply_status = message->power_supply_status;
  const bool charging = supply_status ==
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  QMetaObject::invokeMethod(
    this,
    [this, percentage_valid, percentage, voltage, current, temperature, charging, supply_status]() {
      battery_bar_->setValue(percentage);
      battery_bar_->setFormat(
        percentage_valid ? tr("电量估算：%1%").arg(percentage) : tr("电量：未知"));
      battery_bar_->setStyleSheet(
        charging ? "QProgressBar::chunk { background-color: #2eaf62; }" :
        percentage <= 20 ? "QProgressBar::chunk { background-color: #d9534f; }" : "");
      QString supply_text = tr("供电状态未知");
      if (charging) {
        supply_text = tr("正在充电");
      } else if (supply_status ==
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING)
      {
        supply_text = tr("正在放电");
      }
      const QString current_text = std::isfinite(current) ?
        tr("%1 A").arg(current, 0, 'f', 2) : tr("未测量");
      const QString temperature_text = std::isfinite(temperature) ?
        tr("%1 °C").arg(temperature, 0, 'f', 1) : tr("未测量");
      battery_detail_->setText(
        tr("%1 · MINI 母线平均电压 %2 V · 总电池电流 %3 · 最高温度 %4\n"
           "电量百分比按配置的 18–30 V 区间估算，并非库仑计读数。")
        .arg(supply_text).arg(voltage, 0, 'f', 2).arg(current_text, temperature_text));
      has_battery_ = true;
      last_battery_time_ = std::chrono::steady_clock::now();
    }, Qt::QueuedConnection);
}

void VehicleStatusPanel::handleIntent(const std_msgs::msg::String::SharedPtr message)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(
    QByteArray::fromStdString(message->data), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    return;
  }
  const auto object = document.object();
  const QString text = object.value("text").toString(tr("意图未知"));
  const QString code = object.value("code").toString();
  const QString source = object.value("source").toString();
  const QString led = object.value("led_pattern").toString();
  QMetaObject::invokeMethod(this, [this, text, code, source, led]() {
    intent_value_->setText(text);
    QString color = "#1976d2";
    if (code == "turn_left" || code == "turn_right" || code.startsWith("passing")) {
      color = "#ef6c00";
    } else if (code == "starting" || code == "u_turn" || code == "reversing") {
      color = "#d84315";
    } else if (code == "idle") {
      color = "#607d8b";
    } else if (code == "charging") {
      color = "#2e7d32";
    }
    intent_value_->setStyleSheet(
      QString("QLabel { color: white; background: %1; border-radius: 5px; padding: 8px; }")
      .arg(color));
    intent_detail_->setText(tr("来源：%1 · 灯带动作：%2").arg(source, led));
  }, Qt::QueuedConnection);
}

void VehicleStatusPanel::handleExtendedStatus(
  const std_msgs::msg::String::SharedPtr message)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(
    QByteArray::fromStdString(message->data), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    return;
  }
  const auto object = document.object();
  QMetaObject::invokeMethod(this, [this, object]() {
    const QString connection = object.value("connection").toString();
    const bool requested = object.value("operator_requested").toBool(false);
    const bool enabled = object.value("operator_enabled").toBool(false);
    if (!connection.isEmpty()) {
      has_hardware_status_ = true;
      last_hardware_status_time_ = std::chrono::steady_clock::now();
      if (connection != "connected") {
        hardware_status_->setText(tr("H743 未连接 · 控制已锁定\n%1")
          .arg(object.value("connection_detail").toString()));
        hardware_status_->setStyleSheet(
          QStringLiteral("color: #d32f2f; font-weight: bold;"));
      } else if (enabled) {
        hardware_status_->setText(
          xbox_mode_button_->isChecked() ?
          tr("H743 已连接 · 安全门禁已启用 · Xbox 仍需按 A 键") :
          tr("H743 已连接 · 安全门禁已启用 · RViz 键盘控制可用"));
        hardware_status_->setStyleSheet(
          QStringLiteral("color: #2e7d32; font-weight: bold;"));
      } else if (requested) {
        hardware_status_->setText(tr("H743 已连接 · 正在等待 ready · 控制仍锁定"));
        hardware_status_->setStyleSheet(
          QStringLiteral("color: #ef6c00; font-weight: bold;"));
      } else {
        hardware_status_->setText(tr("H743 已连接 · 请确认安全后启用"));
        hardware_status_->setStyleSheet(
          QStringLiteral("color: #1565c0; font-weight: bold;"));
      }
      safety_enable_button_->setEnabled(!enabled);
      safety_stop_button_->setEnabled(true);
    }
    extended_status_->clear();
    for (auto iterator = object.begin(); iterator != object.end(); ++iterator) {
      appendStatusTree(extended_status_, nullptr, iterator.key(), iterator.value());
    }
    extended_status_->resizeColumnToContents(0);
  }, Qt::QueuedConnection);
}

void VehicleStatusPanel::handleOperatorUiStatus(
  const std_msgs::msg::String::SharedPtr message)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(
    QByteArray::fromStdString(message->data), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    return;
  }
  const auto object = document.object();
  QMetaObject::invokeMethod(this, [this, object]() {
    const QString mode = object.value("teleop_mode").toString("xbox");
    const int xbox_publishers = object.value("xbox_publishers").toInt(0);
    const int joy_publishers = object.value("joy_publishers").toInt(0);
    const int scan_publishers = object.value("scan_publishers").toInt(0);
    const bool mapping_running = object.value("mapping_running").toBool(false);
    const QString mapping_error = object.value("mapping_error").toString();
    xbox_mode_button_->blockSignals(true);
    keyboard_mode_button_->blockSignals(true);
    xbox_mode_button_->setChecked(mode == "xbox");
    keyboard_mode_button_->setChecked(mode == "keyboard");
    xbox_mode_button_->blockSignals(false);
    keyboard_mode_button_->blockSignals(false);
    control_stack_->setCurrentIndex(mode == "xbox" ? 0 : 1);
    teleop_status_->setText(
      mode == "xbox" ?
      tr("当前：Xbox 手柄 · 手柄连接 %1 · 映射节点 %2 · A 键解锁后运动")
      .arg(joy_publishers > 0 ? tr("正常") : tr("未检测到")).arg(xbox_publishers) :
      tr("当前：RViz 键盘 · 点击键盘控制框取得焦点"));
    teleop_status_->setStyleSheet(
      mode == "xbox" && (xbox_publishers == 0 || joy_publishers == 0) ?
      QStringLiteral("color: #ef6c00;") : QStringLiteral("color: #2e7d32;"));

    const QString scan_topic = object.value("scan_topic").toString("/scan_0");
    const QString output = object.value("mapping_output_prefix").toString();
    if (!mapping_error.isEmpty()) {
      mapping_status_->setText(tr("%1\n日志/输出：%2").arg(mapping_error, output));
      mapping_status_->setStyleSheet(QStringLiteral("color: #d32f2f;"));
    } else if (mapping_running) {
      mapping_status_->setText(
        tr("建图运行中 · %1 发布者 %2\n保存目标：%3.yaml")
        .arg(scan_topic).arg(scan_publishers).arg(output));
      mapping_status_->setStyleSheet(QStringLiteral("color: #2e7d32;"));
    } else {
      mapping_status_->setText(
        tr("建图未启动 · %1 发布者 %2\n保存目标：%3.yaml")
        .arg(scan_topic).arg(scan_publishers).arg(output));
      mapping_status_->setStyleSheet(
        scan_publishers > 0 ? QStringLiteral("color: #1565c0;") :
        QStringLiteral("color: #ef6c00;"));
    }
    mapping_start_button_->setEnabled(!mapping_running && scan_publishers > 0);
    mapping_stop_button_->setEnabled(mapping_running);
    mapping_save_button_->setEnabled(mapping_running);
  }, Qt::QueuedConnection);
}

void VehicleStatusPanel::showCameraImage(const QImage & image, const QString & encoding)
{
  const QSize target = camera_view_->contentsRect().size();
  camera_view_->setPixmap(
    QPixmap::fromImage(image).scaled(target, Qt::KeepAspectRatio, Qt::FastTransformation));
  has_camera_ = true;
  last_camera_time_ = std::chrono::steady_clock::now();
  camera_status_->setText(
    tr("画面正常 · %1 × %2 · %3 · /antbot/camera/color/image_raw")
    .arg(image.width()).arg(image.height()).arg(encoding));
  camera_status_->setStyleSheet(QStringLiteral("color: #2e7d32;"));
}

void VehicleStatusPanel::updateDataStatus()
{
  const auto now = std::chrono::steady_clock::now();
  if (has_odometry_ && now - last_odometry_time_ > std::chrono::seconds(2)) {
    odometry_status_->setText(
      tr("里程计数据超时 %1 s · /odometry/filtered")
      .arg(ageText(last_odometry_time_, now)));
    odometry_status_->setStyleSheet(QStringLiteral("color: #ef6c00;"));
  }
  if (has_camera_ && now - last_camera_time_ > std::chrono::seconds(2)) {
    camera_status_->setText(
      tr("摄像头画面超时 %1 s · /antbot/camera/color/image_raw")
      .arg(ageText(last_camera_time_, now)));
    camera_status_->setStyleSheet(QStringLiteral("color: #ef6c00;"));
  }
  if (has_battery_ && now - last_battery_time_ > std::chrono::seconds(3)) {
    battery_detail_->setText(tr("电池数据已超时 · /battery"));
    battery_detail_->setStyleSheet(QStringLiteral("color: #ef6c00;"));
  } else if (has_battery_) {
    battery_detail_->setStyleSheet("");
  }
  if (has_hardware_status_ &&
    now - last_hardware_status_time_ > std::chrono::seconds(3))
  {
    hardware_status_->setText(tr("底层状态超时 · 控制状态未知，请按“停止并锁定”"));
    hardware_status_->setStyleSheet(QStringLiteral("color: #d32f2f; font-weight: bold;"));
    safety_enable_button_->setEnabled(false);
  }
}

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

PLUGINLIB_EXPORT_CLASS(
  robotcar_navigation::rviz_plugins::VehicleStatusPanel,
  rviz_common::Panel)
