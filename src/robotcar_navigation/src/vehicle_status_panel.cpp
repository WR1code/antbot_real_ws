#include "vehicle_status_panel.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <string>

#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMetaObject>
#include <QPixmap>
#include <QProgressBar>
#include <QSizePolicy>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>

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
}  // namespace

VehicleStatusPanel::VehicleStatusPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  configureChineseUiFont();
  auto * root = new QVBoxLayout(this);

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
  extended_status_->setHeaderLabels({tr("扩展参数"), tr("值")});
  extended_status_->setRootIsDecorated(false);
  extended_status_->setMaximumHeight(130);
  auto * placeholder = new QTreeWidgetItem(extended_status_);
  placeholder->setText(0, tr("接口"));
  placeholder->setText(1, tr("/antbot/vehicle_status（JSON，可后续扩展）"));
  state_layout->addWidget(battery_bar_);
  state_layout->addWidget(battery_detail_);
  state_layout->addWidget(extended_status_);
  root->addWidget(state_box);

  auto * camera_box = new QGroupBox(tr("车载 RGB 摄像头"), this);
  auto * camera_layout = new QVBoxLayout(camera_box);
  camera_view_ = new QLabel(tr("等待摄像头画面…"), camera_box);
  camera_view_->setAlignment(Qt::AlignCenter);
  camera_view_->setMinimumSize(320, 240);
  camera_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  camera_view_->setFrameShape(QFrame::StyledPanel);
  camera_view_->setStyleSheet(QStringLiteral("QLabel { background: #151515; color: #cfcfcf; }"));
  camera_status_ = new QLabel(tr("话题：/antbot/camera/color/image_raw"), camera_box);
  camera_status_->setWordWrap(true);
  camera_layout->addWidget(camera_view_, 1);
  camera_layout->addWidget(camera_status_);
  root->addWidget(camera_box, 1);

  auto * timer = new QTimer(this);
  timer->setInterval(500);
  connect(timer, &QTimer::timeout, this, &VehicleStatusPanel::updateDataStatus);
  timer->start();
}

void VehicleStatusPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    odometry_status_->setText(tr("无法取得 RViz ROS 节点"));
    odometry_status_->setStyleSheet(QStringLiteral("color: #d32f2f;"));
    camera_status_->setText(tr("无法取得 RViz ROS 节点"));
    camera_status_->setStyleSheet(QStringLiteral("color: #d32f2f;"));
    return;
  }

  node_ = abstraction->get_raw_node();
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
  const bool charging =
    message->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  QMetaObject::invokeMethod(
    this, [this, percentage_valid, percentage, voltage, current, temperature, charging]() {
      battery_bar_->setValue(percentage);
      battery_bar_->setFormat(
        percentage_valid ? tr("电量：%1%").arg(percentage) : tr("电量：未知"));
      battery_bar_->setStyleSheet(
        charging ? "QProgressBar::chunk { background-color: #2eaf62; }" :
        percentage <= 20 ? "QProgressBar::chunk { background-color: #d9534f; }" : "");
      battery_detail_->setText(
        tr("%1 · 电压 %2 V · 电流 %3 A · 温度 %4 °C")
        .arg(charging ? tr("正在充电") : tr("未充电"))
        .arg(voltage, 0, 'f', 2).arg(current, 0, 'f', 2).arg(temperature, 0, 'f', 1));
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
    extended_status_->clear();
    for (auto iterator = object.begin(); iterator != object.end(); ++iterator) {
      auto * item = new QTreeWidgetItem(extended_status_);
      item->setText(0, iterator.key());
      if (iterator.value().isString()) {
        item->setText(1, iterator.value().toString());
      } else if (iterator.value().isDouble()) {
        item->setText(1, QString::number(iterator.value().toDouble(), 'f', 3));
      } else if (iterator.value().isBool()) {
        item->setText(1, iterator.value().toBool() ? tr("是") : tr("否"));
      } else {
        item->setText(1, QString::fromUtf8(QJsonDocument(iterator.value().toObject()).toJson(
          QJsonDocument::Compact)));
      }
    }
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
}

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

PLUGINLIB_EXPORT_CLASS(
  robotcar_navigation::rviz_plugins::VehicleStatusPanel,
  rviz_common::Panel)
