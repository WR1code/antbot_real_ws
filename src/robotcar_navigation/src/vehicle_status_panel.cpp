#include "vehicle_status_panel.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <map>
#include <string>

#include <QFont>
#include <QFrame>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
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
#include <QStringList>
#include <QTabWidget>
#include <QTableWidget>
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

QString arrayValue(
  const QJsonObject & object, const QString & key, int index,
  int decimals = 1, const QString & suffix = QString())
{
  const auto array = object.value(key).toArray();
  if (index < 0 || index >= array.size() || !array.at(index).isDouble()) {
    return QStringLiteral("--");
  }
  return QStringLiteral("%1%2").arg(array.at(index).toDouble(), 0, 'f', decimals).arg(suffix);
}

QString faultFlagsText(int flags)
{
  if (flags == 0) {
    return QObject::tr("无故障");
  }
  QStringList names;
  if (flags & 0x01) {names << QObject::tr("转向");}
  if (flags & 0x02) {names << QObject::tr("行走");}
  if (flags & 0x04) {names << QObject::tr("CAN");}
  if (flags & 0x08) {names << QObject::tr("CAN Bus-Off");}
  if (flags & 0x10) {names << QObject::tr("串口");}
  return QObject::tr("0x%1 · %2")
         .arg(flags, 4, 16, QLatin1Char('0')).arg(names.join(QObject::tr("、")));
}

void setTableValue(QTableWidget * table, int row, int column, const QString & value)
{
  auto * item = table->item(row, column);
  if (!item) {
    item = new QTableWidgetItem();
    table->setItem(row, column, item);
  }
  item->setText(value);
  item->setTextAlignment(Qt::AlignCenter);
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
  system_reset_button_ = new QPushButton(tr("RESET"), safety_box);
  system_reset_button_->setToolTip(tr("锁定运动并重新启动 H743 底盘控制器"));
  system_reset_button_->setStyleSheet(QStringLiteral(
    "QPushButton { background: #ef6c00; color: white; padding: 8px; font-weight: bold; }"));
  safety_buttons->addWidget(safety_enable_button_);
  safety_buttons->addWidget(safety_stop_button_);
  safety_buttons->addWidget(system_reset_button_);
  safety_layout->addWidget(hardware_status_);
  safety_layout->addLayout(safety_buttons);
  root->addWidget(safety_box);
  connect(
    safety_enable_button_, &QPushButton::clicked,
    this, &VehicleStatusPanel::requestSafetyEnable);
  connect(
    safety_stop_button_, &QPushButton::clicked,
    this, &VehicleStatusPanel::requestSafetyStop);
  connect(
    system_reset_button_, &QPushButton::clicked,
    this, &VehicleStatusPanel::requestSystemReset);

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

  auto * state_box = new QGroupBox(tr("底盘实时状态"), this);
  auto * state_layout = new QVBoxLayout(state_box);
  chassis_summary_ = new QLabel(tr("等待底盘遥测…"), state_box);
  chassis_summary_->setAlignment(Qt::AlignCenter);
  chassis_summary_->setMinimumHeight(44);
  chassis_summary_->setStyleSheet(QStringLiteral(
    "QLabel { color: #37474f; background: #eceff1; border: 1px solid #cfd8dc; "
    "border-radius: 7px; padding: 8px; font-weight: bold; }"));
  battery_bar_ = new QProgressBar(state_box);
  battery_bar_->setRange(0, 100);
  battery_bar_->setValue(0);
  battery_bar_->setFormat(tr("电量：等待 /battery…"));
  battery_detail_ = new QLabel(
    tr("电池接口已预留：电压、电流、温度和充电状态"), state_box);
  battery_detail_->setWordWrap(true);

  auto * summary_grid = new QGridLayout();
  auto add_summary = [state_box, summary_grid](
    const QString & title, QLabel ** value, int row, int column)
    {
      auto * card = new QFrame(state_box);
      card->setFrameShape(QFrame::StyledPanel);
      card->setStyleSheet(QStringLiteral(
        "QFrame { background: #fafafa; border: 1px solid #dfe3e6; border-radius: 6px; }"
        "QLabel { border: none; background: transparent; }"));
      auto * layout = new QVBoxLayout(card);
      layout->setContentsMargins(8, 5, 8, 5);
      auto * caption = new QLabel(title, card);
      caption->setStyleSheet(QStringLiteral("color: #607d8b; font-size: 9pt;"));
      *value = new QLabel(QStringLiteral("--"), card);
      (*value)->setWordWrap(true);
      (*value)->setStyleSheet(QStringLiteral("font-weight: bold; color: #263238;"));
      layout->addWidget(caption);
      layout->addWidget(*value);
      summary_grid->addWidget(card, row, column);
    };
  add_summary(tr("H743 连接"), &connection_value_, 0, 0);
  add_summary(tr("底盘状态"), &chassis_state_value_, 0, 1);
  add_summary(tr("故障诊断"), &fault_value_, 1, 0);
  add_summary(tr("CAN 通信"), &can_value_, 1, 1);
  add_summary(tr("手柄速度档位"), &xbox_speed_level_value_, 2, 0);
  QLabel * speed_switch_hint = nullptr;
  add_summary(tr("手柄换档方式"), &speed_switch_hint, 2, 1);
  speed_switch_hint->setText(tr("按左摇杆降档 · 按右摇杆升档"));

  auto make_table = [state_box](const QStringList & headers) {
    auto * table = new QTableWidget(4, headers.size(), state_box);
    table->setHorizontalHeaderLabels(headers);
    table->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setSelectionMode(QAbstractItemView::NoSelection);
    table->setAlternatingRowColors(true);
    table->verticalHeader()->setVisible(false);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    table->horizontalHeader()->setStretchLastSection(true);
    table->setMinimumHeight(145);
    const QStringList wheels = {QObject::tr("左前"), QObject::tr("右前"),
      QObject::tr("左后"), QObject::tr("右后")};
    for (int row = 0; row < wheels.size(); ++row) {
      setTableValue(table, row, 0, wheels.at(row));
    }
    return table;
  };
  auto * steering_title = new QLabel(tr("RS00 转向电机"), state_box);
  steering_title->setStyleSheet(QStringLiteral("font-weight: bold; color: #37474f;"));
  steering_table_ = make_table({tr("车轮"), tr("角度"), tr("反馈"), tr("状态")});
  auto * drive_title = new QLabel(tr("MINI 行走驱动"), state_box);
  drive_title->setStyleSheet(QStringLiteral("font-weight: bold; color: #37474f;"));
  drive_table_ = make_table(
    {tr("车轮"), tr("转速"), tr("电流"), tr("电压"), tr("温度"), tr("反馈"), tr("状态")});

  extended_status_ = new QTreeWidget(state_box);
  extended_status_->setHeaderLabels({tr("高级原始参数"), tr("值")});
  extended_status_->setRootIsDecorated(true);
  extended_status_->setMinimumHeight(260);
  extended_status_->setVisible(false);
  auto * placeholder = new QTreeWidgetItem(extended_status_);
  placeholder->setText(0, tr("接口"));
  placeholder->setText(1, tr("等待 /antbot/vehicle_status…"));
  auto * raw_toggle = new QPushButton(tr("显示高级原始参数"), state_box);
  raw_toggle->setCheckable(true);
  connect(raw_toggle, &QPushButton::toggled, this, [this, raw_toggle](bool checked) {
    extended_status_->setVisible(checked);
    raw_toggle->setText(checked ? tr("隐藏高级原始参数") : tr("显示高级原始参数"));
  });
  state_layout->addWidget(chassis_summary_);
  state_layout->addWidget(battery_bar_);
  state_layout->addWidget(battery_detail_);
  state_layout->addLayout(summary_grid);
  state_layout->addWidget(steering_title);
  state_layout->addWidget(steering_table_);
  state_layout->addWidget(drive_title);
  state_layout->addWidget(drive_table_);
  state_layout->addWidget(raw_toggle);
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
  xbox_layout->setSpacing(8);
  auto * xbox_quick_start = new QLabel(tr("手柄操控 · 快速上手"), xbox_box);
  xbox_quick_start->setAlignment(Qt::AlignCenter);
  xbox_quick_start->setMinimumHeight(38);
  xbox_quick_start->setStyleSheet(QStringLiteral(
    "QLabel { color: white; background: #1976d2; border-radius: 7px; "
    "padding: 8px; font-size: 12pt; font-weight: bold; }"));
  auto * xbox_help = new QLabel(
    tr("①  连接手柄，等待上方状态显示“手柄在线”\n"
       "②  保持左摇杆回中，按 A 键解锁或锁定输出\n"
       "③  推动左摇杆：前后行驶 / 左右横移\n"
       "④  按下左摇杆降档，按下右摇杆升档\n"
       "     可选速度：10% / 25% / 50% / 75% / 100%"), xbox_box);
  xbox_help->setWordWrap(true);
  xbox_help->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  xbox_help->setStyleSheet(QStringLiteral(
    "QLabel { color: #263238; background: #f7fafc; border: 1px solid #cfd8dc; "
    "border-radius: 7px; padding: 12px; }"));
  auto * xbox_notice = new QLabel(
    tr("安全提示\n真机操控前，还需在“控制与安全”页通过 H743 安全门禁。\n"
       "当前底盘不支持旋转，LT / RT 的旋转指令不会执行。"), xbox_box);
  xbox_notice->setWordWrap(true);
  xbox_notice->setStyleSheet(QStringLiteral(
    "QLabel { color: #7a4b00; background: #fff8e1; border-left: 4px solid #f9a825; "
    "border-radius: 5px; padding: 10px; }"));
  xbox_layout->addWidget(xbox_quick_start);
  xbox_layout->addWidget(xbox_help);
  xbox_layout->addWidget(xbox_notice);
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
  keyboard_speed_->setRange(0.01, 0.30);
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
  auto * camera_controls = new QHBoxLayout();
  camera_topic_box_ = new QComboBox(camera_box);
  camera_topic_box_->setEditable(true);
  camera_topic_box_->addItem(QStringLiteral("/antbot/camera/color/image_raw"));
  camera_topic_box_->setToolTip(tr("选择任意 sensor_msgs/msg/Image 图像话题"));
  camera_toggle_button_ = new QPushButton(tr("打开画面"), camera_box);
  camera_refresh_button_ = new QPushButton(tr("刷新"), camera_box);
  camera_controls->addWidget(camera_topic_box_, 1);
  camera_controls->addWidget(camera_toggle_button_);
  camera_controls->addWidget(camera_refresh_button_);
  camera_view_ = new QLabel(tr("等待摄像头画面…"), camera_box);
  camera_view_->setAlignment(Qt::AlignCenter);
  camera_view_->setMinimumSize(160, 120);
  camera_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  camera_view_->setFrameShape(QFrame::StyledPanel);
  camera_view_->setStyleSheet(QStringLiteral("QLabel { background: #151515; color: #cfcfcf; }"));
  camera_status_ = new QLabel(tr("等待 ROS 图像话题…"), camera_box);
  camera_status_->setWordWrap(true);
  camera_layout->addLayout(camera_controls);
  camera_layout->addWidget(camera_view_, 1);
  camera_layout->addWidget(camera_status_);
  connect(camera_toggle_button_, &QPushButton::clicked, this, &VehicleStatusPanel::toggleCamera);
  connect(camera_refresh_button_, &QPushButton::clicked, this, &VehicleStatusPanel::refreshCameraTopics);
  connect(camera_topic_box_, &QComboBox::currentTextChanged, this, [this]() {
    if (camera_open_ && node_) {subscribeCameraTopic();}
  });
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
    system_reset_button_->setEnabled(false);
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
  system_reset_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/antbot/system_reset");
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
  refreshCameraTopics();
  subscribeCameraTopic();
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

void VehicleStatusPanel::refreshCameraTopics()
{
  if (!node_) {
    return;
  }
  const QString current = camera_topic_box_->currentText().trimmed();
  QStringList topics;
  for (const auto & entry : node_->get_topic_names_and_types()) {
    const auto & types = entry.second;
    if (std::find(types.begin(), types.end(), "sensor_msgs/msg/Image") != types.end()) {
      topics << QString::fromStdString(entry.first);
    }
  }
  topics.removeDuplicates();
  topics.sort();
  camera_topic_box_->blockSignals(true);
  camera_topic_box_->clear();
  camera_topic_box_->addItems(topics);
  if (!current.isEmpty() && camera_topic_box_->findText(current) < 0) {
    camera_topic_box_->insertItem(0, current);
  }
  camera_topic_box_->setCurrentText(
    current.isEmpty() ? QStringLiteral("/antbot/camera/color/image_raw") : current);
  camera_topic_box_->blockSignals(false);
  camera_status_->setText(
    topics.isEmpty() ? tr("暂未发现图像话题；可手动输入话题后打开") :
    tr("发现 %1 个原始图像话题，请选择后打开").arg(topics.size()));
}

void VehicleStatusPanel::subscribeCameraTopic()
{
  image_sub_.reset();
  has_camera_ = false;
  const QString topic = camera_topic_box_->currentText().trimmed();
  if (!camera_open_ || !node_ || topic.isEmpty()) {
    camera_toggle_button_->setText(tr("打开画面"));
    return;
  }
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    topic.toStdString(), rclcpp::SensorDataQoS(),
    std::bind(&VehicleStatusPanel::handleImage, this, std::placeholders::_1));
  camera_view_->clear();
  camera_view_->setText(tr("正在打开摄像头画面…"));
  camera_status_->setText(tr("已打开：%1 · 等待首帧").arg(topic));
  camera_status_->setStyleSheet(QStringLiteral("color: #1565c0;"));
  camera_toggle_button_->setText(tr("关闭画面"));
  camera_toggle_button_->setStyleSheet(
    QStringLiteral("QPushButton { background: #c62828; color: white; padding: 6px; }"));
}

void VehicleStatusPanel::toggleCamera()
{
  camera_open_ = !camera_open_;
  if (!camera_open_) {
    image_sub_.reset();
    has_camera_ = false;
    camera_view_->clear();
    camera_view_->setText(tr("摄像头画面已关闭"));
    camera_status_->setText(tr("已关闭 · 可选择其他话题后重新打开"));
    camera_status_->setStyleSheet(QStringLiteral("color: #607d8b;"));
    camera_toggle_button_->setText(tr("打开画面"));
    camera_toggle_button_->setStyleSheet(
      QStringLiteral("QPushButton { background: #2e7d32; color: white; padding: 6px; }"));
    return;
  }
  subscribeCameraTopic();
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
        safety_enable_button_->setText(
          success ? tr("… 等待底盘就绪") : tr("启用失败 · 可重试"));
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
        safety_enable_button_->setText(tr("确认安全并启用"));
        safety_stop_button_->setText(tr("停止并锁定"));
      }, Qt::QueuedConnection);
    });
}

void VehicleStatusPanel::requestSystemReset()
{
  if (!system_reset_client_ || !system_reset_client_->service_is_ready()) {
    hardware_status_->setText(tr("底盘 RESET 服务未就绪 · 未执行复位"));
    hardware_status_->setStyleSheet(QStringLiteral("color: #d32f2f; font-weight: bold;"));
    return;
  }
  const auto answer = QMessageBox::warning(
    this,
    tr("确认 RESET 底盘"),
    tr("RESET 将立即：\n\n"
       "• 锁定所有运动输出并发送零速\n"
       "• 重新启动 H743 底盘控制器\n"
       "• 清除当前安全使能，重启后需要重新确认\n\n"
       "请确保车辆周围安全。是否继续？"),
    QMessageBox::Yes | QMessageBox::No,
    QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }

  safety_enable_button_->setEnabled(false);
  safety_stop_button_->setEnabled(false);
  system_reset_button_->setEnabled(false);
  hardware_status_->setText(tr("正在锁定运动并 RESET H743…"));
  hardware_status_->setStyleSheet(QStringLiteral("color: #ef6c00; font-weight: bold;"));
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  system_reset_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      const QString text = QString::fromStdString(response->message);
      const bool success = response->success;
      QMetaObject::invokeMethod(this, [this, text, success]() {
        hardware_status_->setText(text);
        hardware_status_->setStyleSheet(
          success ? QStringLiteral("color: #ef6c00; font-weight: bold;") :
          QStringLiteral("color: #d32f2f; font-weight: bold;"));
        safety_enable_button_->setText(tr("确认安全并启用"));
        safety_enable_button_->setEnabled(!success);
        safety_stop_button_->setEnabled(true);
        system_reset_button_->setEnabled(true);
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
    const QString chassis_state = object.value("chassis_state").toString(tr("未知"));
    const int fault_flags = object.value("fault_flags").toInt(0);
    const auto can = object.value("can").toObject();
    const auto steering = object.value("steering").toObject();
    const auto drive = object.value("mini").toObject();

    connection_value_->setText(
      connection == "connected" ? tr("● 在线") : tr("● 离线"));
    connection_value_->setStyleSheet(
      connection == "connected" ? QStringLiteral("font-weight: bold; color: #2e7d32;") :
      QStringLiteral("font-weight: bold; color: #c62828;"));
    chassis_state_value_->setText(chassis_state);
    chassis_state_value_->setStyleSheet(
      chassis_state == "READY" || chassis_state == "IDLE" || enabled ?
      QStringLiteral("font-weight: bold; color: #2e7d32;") :
      chassis_state == "FAULT" ? QStringLiteral("font-weight: bold; color: #c62828;") :
      QStringLiteral("font-weight: bold; color: #ef6c00;"));
    fault_value_->setText(faultFlagsText(fault_flags));
    fault_value_->setStyleSheet(
      fault_flags == 0 ? QStringLiteral("font-weight: bold; color: #2e7d32;") :
      QStringLiteral("font-weight: bold; color: #c62828;"));
    can_value_->setText(tr("收 %1 · 发 %2 · 标志 0x%3")
      .arg(can.value("rx_count").toInt()).arg(can.value("tx_count").toInt())
      .arg(can.value("flags").toInt(), 0, 16));

    const QString summary_text = connection != "connected" ? tr("底盘离线 · 控制已锁定") :
      enabled ? tr("✓ 底盘已安全启用 · 可接收当前控制源") :
      requested ? tr("正在等待底盘 READY · 运动仍被锁定") :
      fault_flags ? tr("检测到底盘故障 · 请先排障，运动已锁定") :
      tr("底盘在线 · 等待人工安全确认");
    const QString summary_color = connection != "connected" || fault_flags ? "#c62828" :
      enabled ? "#2e7d32" : requested ? "#ef6c00" : "#1565c0";
    chassis_summary_->setText(summary_text);
    chassis_summary_->setStyleSheet(QString(
      "QLabel { color: white; background: %1; border-radius: 7px; "
      "padding: 9px; font-weight: bold; }").arg(summary_color));

    for (int row = 0; row < 4; ++row) {
      setTableValue(steering_table_, row, 1, arrayValue(steering, "position_deg", row, 1, tr("°")));
      setTableValue(steering_table_, row, 2, arrayValue(steering, "feedback_age_ms", row, 0, tr(" ms")));
      QString steering_status = tr("正常");
      if (steering.value("fault").toBool()) {steering_status = tr("故障");}
      else if (!steering.value("homed").toBool()) {steering_status = tr("未回零");}
      else if (!steering.value("ready").toBool()) {steering_status = tr("未就绪");}
      setTableValue(steering_table_, row, 3, steering_status);

      const auto speed_values = drive.value("speed_erpm").toArray();
      if (row < speed_values.size() && speed_values.at(row).isDouble()) {
        const double erpm = speed_values.at(row).toDouble();
        setTableValue(
          drive_table_, row, 1,
          tr("%1 rpm · %2 erpm").arg(erpm / 10.0, 0, 'f', 1).arg(erpm, 0, 'f', 0));
      } else {
        setTableValue(drive_table_, row, 1, QStringLiteral("--"));
      }
      setTableValue(drive_table_, row, 2, arrayValue(drive, "current_a", row, 2, tr(" A")));
      setTableValue(drive_table_, row, 3, arrayValue(drive, "voltage_v", row, 1, tr(" V")));
      setTableValue(drive_table_, row, 4, arrayValue(drive, "temperature_c", row, 0, tr("°C")));
      setTableValue(drive_table_, row, 5, arrayValue(drive, "feedback_age_ms", row, 0, tr(" ms")));
      const auto fault_codes = drive.value("fault_code").toArray();
      const int drive_fault = row < fault_codes.size() ? fault_codes.at(row).toInt() : 0;
      const auto safety_flags = drive.value("safety_flags").toArray();
      const int safety = row < safety_flags.size() ? safety_flags.at(row).toInt() : 0;
      setTableValue(
        drive_table_, row, 6,
        drive_fault ? tr("故障 0x%1").arg(drive_fault, 0, 16) :
        safety ? tr("保护 0x%1").arg(safety, 0, 16) : tr("正常"));
    }
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
      safety_enable_button_->setText(
        enabled ? tr("✓ 底盘已启用") : requested ? tr("… 等待底盘就绪") :
        fault_flags ? tr("故障未清除 · 无法启用") : tr("确认安全并启用"));
      safety_enable_button_->setEnabled(!enabled && !requested && fault_flags == 0);
      safety_enable_button_->setStyleSheet(
        enabled ? QStringLiteral(
          "QPushButton { background: #2e7d32; color: white; padding: 9px; font-weight: bold; }") :
        fault_flags ? QStringLiteral(
          "QPushButton { background: #b0bec5; color: #455a64; padding: 9px; }") :
        QStringLiteral(
          "QPushButton { background: #2e7d32; color: white; padding: 9px; font-weight: bold; }"));
      safety_stop_button_->setText(enabled || requested ? tr("■ 立即停止并锁定") : tr("停止并锁定"));
      safety_stop_button_->setEnabled(true);
      system_reset_button_->setEnabled(connection == "connected");
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
    const QString joy_device = object.value("joy_device").toString();
    const QString joy_name = object.value("joy_device_name").toString();
    const QString joy_error = object.value("joy_manager_error").toString();
    const bool xbox_armed = object.value("xbox_armed").toBool(false);
    const auto xbox_status = object.value("xbox_status").toObject();
    const int speed_level = xbox_status.value("speed_level").toInt(0);
    const int speed_level_count = xbox_status.value("speed_level_count").toInt(0);
    const double speed_ratio = xbox_status.value("speed_ratio").toDouble(0.0);
    const double speed_limit = xbox_status.value("speed_limit_mps").toDouble(0.0);
    const bool input_fresh = xbox_status.value("input_fresh").toBool(false);
    const double command_vx = object.value("command_vx_mps").toDouble(0.0);
    const double command_vy = object.value("command_vy_mps").toDouble(0.0);
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
    if (speed_level_count > 0 && speed_level > 0) {
      xbox_speed_level_value_->setText(
        tr("第 %1/%2 档  ·  %3%  ·  上限 %4 m/s")
        .arg(speed_level).arg(speed_level_count)
        .arg(qRound(speed_ratio * 100.0)).arg(speed_limit, 0, 'f', 3));
      xbox_speed_level_value_->setStyleSheet(
        joy_publishers > 0 ?
        QStringLiteral("font-weight: bold; color: #1565c0; font-size: 11pt;") :
        QStringLiteral("font-weight: bold; color: #ef6c00; font-size: 11pt;"));
    } else {
      xbox_speed_level_value_->setText(tr("等待手柄档位数据…"));
      xbox_speed_level_value_->setStyleSheet(
        QStringLiteral("font-weight: bold; color: #78909c;"));
    }
    if (mode == "xbox") {
      if (joy_publishers > 0) {
        teleop_status_->setText(
          tr("● 手柄在线 · %1\n设备：%2 · %3")
          .arg(joy_name.isEmpty() ? tr("Xbox/兼容手柄") : joy_name)
          .arg(joy_device)
          .arg(xbox_armed ? tr("A 键已解锁，可以操控") : tr("当前锁定，请按 A 键解锁")));
        if (speed_level_count > 0) {
          teleop_status_->setText(
            teleop_status_->text() +
            tr("\n速度档：%1/%2 · 当前上限 %3 m/s · 输入%4")
            .arg(speed_level).arg(speed_level_count).arg(speed_limit, 0, 'f', 3)
            .arg(input_fresh ? tr("正常") : tr("等待动作")));
        }
        teleop_status_->setText(
          teleop_status_->text() + tr("\n当前指令：前后 vx %1 · 横移 vy %2 m/s")
          .arg(command_vx, 0, 'f', 2).arg(command_vy, 0, 'f', 2));
      } else {
        teleop_status_->setText(
          tr("○ 手柄未连接 · 后台正在自动检测\n%1")
          .arg(joy_error.isEmpty() ? tr("插入手柄后无需重启界面") : joy_error));
      }
    } else {
      teleop_status_->setText(tr("● RViz 键盘模式 · 点击键盘控制框取得焦点"));
    }
    teleop_status_->setStyleSheet(
      mode == "xbox" && (xbox_publishers == 0 || joy_publishers == 0) ?
      QStringLiteral(
        "QLabel { color: #ef6c00; background: #fff3e0; border-radius: 6px; padding: 8px; }") :
      xbox_armed || mode == "keyboard" ? QStringLiteral(
        "QLabel { color: #2e7d32; background: #e8f5e9; border-radius: 6px; padding: 8px; font-weight: bold; }") :
      QStringLiteral(
        "QLabel { color: #1565c0; background: #e3f2fd; border-radius: 6px; padding: 8px; }"));

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
    tr("● 画面正常 · %1 × %2 · %3\n话题：%4")
    .arg(image.width()).arg(image.height()).arg(encoding, camera_topic_box_->currentText()));
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
      tr("摄像头画面超时 %1 s · %2")
      .arg(ageText(last_camera_time_, now), camera_topic_box_->currentText()));
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
