#include "rebotarm_demo_rviz/demo_panel.hpp"
#include "rebotarm_demo_rviz/motion_planning_i18n.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <QComboBox>
#include <QCheckBox>
#include <QAbstractItemView>
#include <QAction>
#include <QDoubleSpinBox>
#include <QDir>
#include <QDialog>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMap>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPointer>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QPixmap>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSlider>
#include <QSpinBox>
#include <QStandardPaths>
#include <QStringList>
#include <QTabWidget>
#include <QTableWidget>
#include <QHeaderView>
#include <QTimer>
#include <QVBoxLayout>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/display_group.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace rebotarm_demo_rviz
{
namespace
{

const QString kAutoHandeyeEyeOnBaseSequenceName =
  QStringLiteral("自动手眼标定_DM_12姿态");
const QString kAutoHandeyeEyeInHandSequenceName =
  QStringLiteral("自动手眼标定_DM_眼在手上_12姿态");
const QString kPiperAutoHandeyeEyeOnBaseSequenceName =
  QStringLiteral("自动手眼标定_PiperH_12姿态");
const QString kPiperAutoHandeyeEyeInHandSequenceName =
  QStringLiteral("自动手眼标定_PiperH_眼在手上_Link5_18姿态");
const QString kPiperEyeInHandActionPrefix =
  QStringLiteral("眼在手上Link5标定姿态_");

std::int64_t steadyMilliseconds()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

QString lastNonEmptyLine(const QString & output)
{
  const auto lines = output.split('\n', Qt::SkipEmptyParts);
  return lines.isEmpty() ? QString() : lines.last().trimmed();
}

QImage imageMessageToQImage(const sensor_msgs::msg::Image & message)
{
  if (message.width == 0 || message.height == 0 || message.data.empty()) {
    return QImage();
  }

  QImage::Format format;
  std::uint32_t bytes_per_pixel = 0;
  if (message.encoding == "rgb8") {
    format = QImage::Format_RGB888;
    bytes_per_pixel = 3;
  } else if (message.encoding == "bgr8") {
    format = QImage::Format_BGR888;
    bytes_per_pixel = 3;
  } else if (message.encoding == "rgba8") {
    format = QImage::Format_RGBA8888;
    bytes_per_pixel = 4;
  } else if (message.encoding == "bgra8") {
    format = QImage::Format_ARGB32;
    bytes_per_pixel = 4;
  } else if (message.encoding == "mono8") {
    format = QImage::Format_Grayscale8;
    bytes_per_pixel = 1;
  } else if (message.encoding == "16UC1" || message.encoding == "mono16") {
    const auto required_size = static_cast<std::size_t>(message.step) * message.height;
    if (message.step < message.width * 2 || message.data.size() < required_size) {
      return QImage();
    }
    const bool depth = message.encoding == "16UC1";
    std::uint16_t minimum = 65535;
    std::uint16_t maximum = 0;
    auto sample = [&message](const std::uint8_t * pixel) {
        return message.is_bigendian ?
          static_cast<std::uint16_t>((pixel[0] << 8) | pixel[1]) :
          static_cast<std::uint16_t>(pixel[0] | (pixel[1] << 8));
      };
    for (std::uint32_t y = 0; y < message.height; ++y) {
      const auto * row = message.data.data() + static_cast<std::size_t>(y) * message.step;
      for (std::uint32_t x = 0; x < message.width; ++x) {
        const auto value = sample(row + x * 2);
        if (depth && value == 0) {continue;}
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
      }
    }
    if (maximum <= minimum) {
      minimum = 0;
      maximum = 1;
    }
    QImage image(
      static_cast<int>(message.width), static_cast<int>(message.height),
      QImage::Format_Grayscale8);
    for (std::uint32_t y = 0; y < message.height; ++y) {
      const auto * source = message.data.data() + static_cast<std::size_t>(y) * message.step;
      auto * target = image.scanLine(static_cast<int>(y));
      for (std::uint32_t x = 0; x < message.width; ++x) {
        const auto value = sample(source + x * 2);
        target[x] = depth && value == 0 ? 0 : static_cast<std::uint8_t>(
          std::clamp(
            255.0 * (static_cast<double>(value) - minimum) / (maximum - minimum),
            0.0, 255.0));
      }
    }
    return image;
  } else {
    return QImage();
  }

  const auto minimum_step = message.width * bytes_per_pixel;
  const auto required_size = static_cast<std::size_t>(message.step) * message.height;
  if (message.step < minimum_step || message.data.size() < required_size) {
    return QImage();
  }
  return QImage(
    message.data.data(), static_cast<int>(message.width),
    static_cast<int>(message.height), static_cast<int>(message.step), format).copy();
}

enum TeachState
{
  TEACH_UNKNOWN = 0,
  TEACH_LOCKED,
  TEACH_IDLE,
  TEACH_READY,
  TEACH_GRAVITY_PRECHECK,
  TEACH_STARTING,
  TEACH_RECORDING,
  TEACH_STOPPING,
  TEACH_REPLAY_STARTING,
  TEACH_REPLAYING,
  TEACH_CANCELLING,
  TEACH_SEQUENCE_PLANNING,
  TEACH_SHAPE_PLANNING,
  TEACH_FAULT
};

int parseTeachState(const std::string & status)
{
  if (status.find("\"state\": \"LOCKED\"") != std::string::npos) {return TEACH_LOCKED;}
  if (status.find("\"state\": \"IDLE\"") != std::string::npos) {return TEACH_IDLE;}
  if (status.find("\"state\": \"READY\"") != std::string::npos) {return TEACH_READY;}
  if (status.find("\"state\": \"GRAVITY_PRECHECK\"") != std::string::npos) {
    return TEACH_GRAVITY_PRECHECK;
  }
  if (status.find("\"state\": \"STARTING\"") != std::string::npos) {return TEACH_STARTING;}
  if (status.find("\"state\": \"RECORDING\"") != std::string::npos) {return TEACH_RECORDING;}
  if (status.find("\"state\": \"STOPPING\"") != std::string::npos) {return TEACH_STOPPING;}
  if (status.find("\"state\": \"REPLAY_STARTING\"") != std::string::npos) {
    return TEACH_REPLAY_STARTING;
  }
  if (status.find("\"state\": \"REPLAYING\"") != std::string::npos) {return TEACH_REPLAYING;}
  if (status.find("\"state\": \"CANCELLING\"") != std::string::npos) {
    return TEACH_CANCELLING;
  }
  if (status.find("\"state\": \"SEQUENCE_PLANNING\"") != std::string::npos) {
    return TEACH_SEQUENCE_PLANNING;
  }
  if (status.find("\"state\": \"SHAPE_PLANNING\"") != std::string::npos) {
    return TEACH_SHAPE_PLANNING;
  }
  if (status.find("\"state\": \"FAULT\"") != std::string::npos) {return TEACH_FAULT;}
  return TEACH_UNKNOWN;
}

}  // namespace

DemoPanel::DemoPanel(QWidget * parent)
: rviz_common::Panel(parent),
  process_(new QProcess(this)),
  camera_process_(new QProcess(this)),
  hand_vision_process_(new QProcess(this)),
  handeye_process_(new QProcess(this)),
  handeye_publish_process_(new QProcess(this)),
  readiness_timer_(new QTimer(this))
{
  auto * panel_layout = new QVBoxLayout(this);
  panel_layout->setContentsMargins(0, 0, 0, 0);

  auto * language_row = new QHBoxLayout();
  language_label_ = new QLabel(this);
  language_combo_ = new QComboBox(this);
  language_combo_->addItem(QStringLiteral("中文"));
  language_combo_->addItem(QStringLiteral("English"));
  language_row->addWidget(language_label_);
  language_row->addWidget(language_combo_);
  language_row->addStretch(1);
  panel_layout->addLayout(language_row);

  main_tabs_ = new QTabWidget(this);
  main_tabs_->setObjectName(QStringLiteral("main_tabs"));
  main_tabs_->setTabPosition(QTabWidget::North);
  main_tabs_->setUsesScrollButtons(true);
  main_tabs_->setElideMode(Qt::ElideRight);
  panel_layout->addWidget(main_tabs_, 1);

  auto * overview_page = new QWidget(main_tabs_);
  auto * overview_page_layout = new QVBoxLayout(overview_page);
  overview_page_layout->setContentsMargins(0, 0, 0, 0);
  auto * overview_scroll = new QScrollArea(overview_page);
  overview_scroll->setWidgetResizable(true);
  overview_scroll->setFrameShape(QFrame::NoFrame);
  auto * overview_content = new QWidget(overview_scroll);
  auto * overview_root = new QVBoxLayout(overview_content);
  overview_scroll->setWidget(overview_content);
  overview_page_layout->addWidget(overview_scroll);
  main_tabs_->addTab(overview_page, QString());

  auto * teach_workspace_page = new QWidget(main_tabs_);
  auto * teach_workspace_layout = new QVBoxLayout(teach_workspace_page);
  teach_workspace_layout->setContentsMargins(4, 4, 4, 4);
  main_tabs_->addTab(teach_workspace_page, QString());

  auto * camera_page = new QWidget(main_tabs_);
  auto * camera_page_layout = new QVBoxLayout(camera_page);
  camera_page_layout->setContentsMargins(4, 4, 4, 4);
  auto * camera_topic_row = new QHBoxLayout();
  camera_topic_label_ = new QLabel(camera_page);
  camera_topic_combo_ = new QComboBox(camera_page);
  camera_topic_combo_->setObjectName(QStringLiteral("camera_topic_combo"));
  camera_topic_combo_->setEditable(false);
  camera_topic_refresh_button_ = new QPushButton(camera_page);
  camera_topic_refresh_button_->setObjectName(QStringLiteral("camera_topic_refresh_button"));
  camera_topic_row->addWidget(camera_topic_label_);
  camera_topic_row->addWidget(camera_topic_combo_, 1);
  camera_topic_row->addWidget(camera_topic_refresh_button_);
  camera_page_layout->addLayout(camera_topic_row);
  auto * camera_stream_row = new QHBoxLayout();
  camera_color_checkbox_ = new QCheckBox(camera_page);
  camera_color_checkbox_->setObjectName(QStringLiteral("camera_color_checkbox"));
  camera_color_checkbox_->setChecked(true);
  camera_depth_checkbox_ = new QCheckBox(camera_page);
  camera_depth_checkbox_->setObjectName(QStringLiteral("camera_depth_checkbox"));
  camera_ir_checkbox_ = new QCheckBox(camera_page);
  camera_ir_checkbox_->setObjectName(QStringLiteral("camera_ir_checkbox"));
  camera_stream_row->addWidget(camera_color_checkbox_);
  camera_stream_row->addWidget(camera_depth_checkbox_);
  camera_stream_row->addWidget(camera_ir_checkbox_);
  camera_stream_row->addStretch(1);
  camera_page_layout->addLayout(camera_stream_row);
  camera_tab_status_label_ = new QLabel(camera_page);
  camera_tab_status_label_->setWordWrap(true);
  camera_page_layout->addWidget(camera_tab_status_label_);
  camera_stream_info_label_ = new QLabel(camera_page);
  camera_stream_info_label_->setObjectName(QStringLiteral("camera_stream_info_label"));
  camera_stream_info_label_->setWordWrap(true);
  camera_page_layout->addWidget(camera_stream_info_label_);
  camera_view_label_ = new QLabel(camera_page);
  camera_view_label_->setObjectName(QStringLiteral("camera_view_label"));
  camera_view_label_->setAlignment(Qt::AlignCenter);
  // The image follows the available tab size.  A hard-coded 320x240 minimum
  // made every tab (and therefore the enclosing RViz window) inherit that
  // minimum even while the camera tab was not selected.
  camera_view_label_->setMinimumSize(0, 0);
  camera_view_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  camera_view_label_->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  camera_page_layout->addWidget(camera_view_label_, 1);
  camera_tab_button_ = new QPushButton(camera_page);
  camera_page_layout->addWidget(camera_tab_button_);
  main_tabs_->addTab(camera_page, QString());

  auto * hand_vision_page = new QWidget(main_tabs_);
  auto * hand_vision_layout = new QVBoxLayout(hand_vision_page);
  hand_vision_layout->setContentsMargins(8, 8, 8, 8);
  auto * hand_vision_title = new QLabel(hand_vision_page);
  hand_vision_title->setObjectName(QStringLiteral("hand_vision_title"));
  hand_vision_title->setStyleSheet(QStringLiteral("font-weight: bold; font-size: 14px;"));
  hand_vision_layout->addWidget(hand_vision_title);
  auto * hand_vision_hint = new QLabel(hand_vision_page);
  hand_vision_hint->setObjectName(QStringLiteral("hand_vision_hint"));
  hand_vision_hint->setWordWrap(true);
  hand_vision_layout->addWidget(hand_vision_hint);
  auto * hand_vision_topic_row = new QHBoxLayout();
  hand_vision_topic_label_ = new QLabel(hand_vision_page);
  hand_vision_topic_combo_ = new QComboBox(hand_vision_page);
  hand_vision_topic_combo_->setObjectName(QStringLiteral("hand_vision_topic_combo"));
  hand_vision_topic_refresh_button_ = new QPushButton(hand_vision_page);
  hand_vision_topic_refresh_button_->setObjectName(
    QStringLiteral("hand_vision_topic_refresh_button"));
  hand_vision_topic_row->addWidget(hand_vision_topic_label_);
  hand_vision_topic_row->addWidget(hand_vision_topic_combo_, 1);
  hand_vision_topic_row->addWidget(hand_vision_topic_refresh_button_);
  hand_vision_layout->addLayout(hand_vision_topic_row);
  hand_vision_status_label_ = new QLabel(hand_vision_page);
  hand_vision_status_label_->setObjectName(QStringLiteral("hand_vision_status_label"));
  hand_vision_status_label_->setWordWrap(true);
  hand_vision_layout->addWidget(hand_vision_status_label_);
  pulse_region_label_ = new QLabel(hand_vision_page);
  pulse_region_label_->setObjectName(QStringLiteral("pulse_region_label"));
  pulse_region_label_->setWordWrap(true);
  hand_vision_layout->addWidget(pulse_region_label_);
  hand_vision_view_label_ = new QLabel(hand_vision_page);
  hand_vision_view_label_->setObjectName(QStringLiteral("hand_vision_view_label"));
  hand_vision_view_label_->setAlignment(Qt::AlignCenter);
  hand_vision_view_label_->setMinimumSize(0, 0);
  hand_vision_view_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  hand_vision_view_label_->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  hand_vision_view_label_->setText(QStringLiteral("等待手部视觉图像…"));
  hand_vision_layout->addWidget(hand_vision_view_label_, 1);
  hand_vision_large_button_ = new QPushButton(hand_vision_page);
  hand_vision_large_button_->setObjectName(QStringLiteral("hand_vision_large_button"));
  hand_vision_layout->addWidget(hand_vision_large_button_);
  hand_vision_button_ = new QPushButton(hand_vision_page);
  hand_vision_button_->setObjectName(QStringLiteral("hand_vision_button"));
  hand_vision_layout->addWidget(hand_vision_button_);
  auto * pulse_standoff_row = new QHBoxLayout();
  auto * pulse_standoff_label = new QLabel(hand_vision_page);
  pulse_standoff_label->setObjectName(QStringLiteral("pulse_standoff_label"));
  pulse_standoff_spin_ = new QDoubleSpinBox(hand_vision_page);
  pulse_standoff_spin_->setObjectName(QStringLiteral("pulse_standoff_spin"));
  pulse_standoff_spin_->setRange(20.0, 150.0);
  pulse_standoff_spin_->setDecimals(0);
  pulse_standoff_spin_->setSuffix(QStringLiteral(" mm"));
  pulse_standoff_spin_->setValue(60.0);
  pulse_standoff_row->addWidget(pulse_standoff_label);
  pulse_standoff_row->addWidget(pulse_standoff_spin_);
  pulse_standoff_row->addStretch(1);
  hand_vision_layout->addLayout(pulse_standoff_row);
  pulse_approach_button_ = new QPushButton(hand_vision_page);
  pulse_approach_button_->setObjectName(QStringLiteral("pulse_approach_button"));
  hand_vision_layout->addWidget(pulse_approach_button_);
  hand_vision_layout->addStretch(1);
  main_tabs_->addTab(hand_vision_page, QString());

  auto * handeye_page = new QWidget(main_tabs_);
  auto * handeye_layout = new QVBoxLayout(handeye_page);
  handeye_layout->setContentsMargins(8, 8, 8, 8);
  auto * handeye_title = new QLabel(handeye_page);
  handeye_title->setObjectName(QStringLiteral("handeye_title"));
  handeye_title->setStyleSheet(QStringLiteral("font-weight: bold; font-size: 14px;"));
  handeye_layout->addWidget(handeye_title);
  auto * handeye_hint = new QLabel(handeye_page);
  handeye_hint->setObjectName(QStringLiteral("handeye_hint"));
  handeye_hint->setWordWrap(true);
  handeye_layout->addWidget(handeye_hint);
  auto * handeye_form = new QFormLayout();
  handeye_backend_combo_ = new QComboBox(handeye_page);
  handeye_backend_combo_->setObjectName(QStringLiteral("handeye_backend_combo"));
  handeye_backend_combo_->addItem(
    QStringLiteral("easy_handeye2"), QStringLiteral("easy_handeye2"));
  handeye_backend_combo_->addItem(
    QStringLiteral("MoveIt Calibration"), QStringLiteral("moveit_calibration"));
  handeye_mode_combo_ = new QComboBox(handeye_page);
  handeye_mode_combo_->setObjectName(QStringLiteral("handeye_mode_combo"));
  handeye_mode_combo_->addItem(
    QStringLiteral("眼在手外（固定相机）"), QStringLiteral("eye_on_base"));
  handeye_mode_combo_->addItem(
    QStringLiteral("眼在手上（末端相机）"), QStringLiteral("eye_in_hand"));
  handeye_target_combo_ = new QComboBox(handeye_page);
  handeye_target_combo_->setObjectName(QStringLiteral("handeye_target_combo"));
  handeye_target_combo_->addItem(
    QStringLiteral("单个 AprilTag"), QStringLiteral("single_tag"));
  handeye_target_combo_->addItem(
    QStringLiteral("A4 四标签板（ID 0–3）"), QStringLiteral("a4_4tag_board"));
  handeye_target_combo_->addItem(
    QStringLiteral("A4 ChArUco 板（5×7）"), QStringLiteral("charuco_a4_5x7"));
  handeye_name_edit_ = new QLineEdit(QStringLiteral("rebotarm_camera"), handeye_page);
  handeye_robot_base_edit_ = new QLineEdit(QStringLiteral("base_link"), handeye_page);
  handeye_robot_effector_edit_ = new QLineEdit(QStringLiteral("gripper_tcp"), handeye_page);
  handeye_name_edit_->setObjectName(QStringLiteral("handeye_name_edit"));
  handeye_robot_base_edit_->setObjectName(QStringLiteral("handeye_robot_base_edit"));
  handeye_robot_effector_edit_->setObjectName(QStringLiteral("handeye_robot_effector_edit"));
  handeye_tracking_base_edit_ = new QLineEdit(
    QStringLiteral("camera_color_optical_frame"), handeye_page);
  handeye_tracking_marker_edit_ = new QLineEdit(QStringLiteral("marker_frame"), handeye_page);
  handeye_tag_family_combo_ = new QComboBox(handeye_page);
  handeye_tag_family_combo_->setObjectName(QStringLiteral("handeye_tag_family_combo"));
  handeye_tag_family_combo_->addItems({
      QStringLiteral("16h5"),
      QStringLiteral("25h9"),
      QStringLiteral("36h11"),
      QStringLiteral("Circle21h7"),
      QStringLiteral("Circle49h12"),
      QStringLiteral("Custom48h12"),
      QStringLiteral("Standard41h12"),
      QStringLiteral("Standard52h13")});
  handeye_tag_family_combo_->setCurrentText(QStringLiteral("36h11"));
  handeye_tag_id_spin_ = new QSpinBox(handeye_page);
  handeye_tag_id_spin_->setObjectName(QStringLiteral("handeye_tag_id_spin"));
  handeye_tag_id_spin_->setRange(0, 586);
  handeye_tag_id_spin_->setValue(0);
  handeye_tag_size_spin_ = new QDoubleSpinBox(handeye_page);
  handeye_tag_size_spin_->setObjectName(QStringLiteral("handeye_tag_size_spin"));
  handeye_tag_size_spin_->setRange(10.0, 300.0);
  handeye_tag_size_spin_->setDecimals(1);
  handeye_tag_size_spin_->setSuffix(QStringLiteral(" mm"));
  handeye_tag_size_spin_->setValue(40.0);
  auto add_handeye_field = [handeye_form, handeye_page](const QString & object_name,
      const QString & text, QWidget * edit) {
      auto * label = new QLabel(text, handeye_page);
      label->setObjectName(object_name);
      handeye_form->addRow(label, edit);
    };
  add_handeye_field(QStringLiteral("handeye_backend_label"), QStringLiteral("标定包"), handeye_backend_combo_);
  add_handeye_field(QStringLiteral("handeye_mode_label"), QStringLiteral("标定模式"), handeye_mode_combo_);
  add_handeye_field(QStringLiteral("handeye_target_label"), QStringLiteral("标定目标"), handeye_target_combo_);
  add_handeye_field(QStringLiteral("handeye_name_label"), QStringLiteral("名称"), handeye_name_edit_);
  add_handeye_field(QStringLiteral("handeye_robot_base_label"), QStringLiteral("机器人基座 frame"), handeye_robot_base_edit_);
  add_handeye_field(QStringLiteral("handeye_robot_effector_label"), QStringLiteral("机器人末端 frame"), handeye_robot_effector_edit_);
  add_handeye_field(QStringLiteral("handeye_tracking_base_label"), QStringLiteral("跟踪基座 frame"), handeye_tracking_base_edit_);
  add_handeye_field(QStringLiteral("handeye_tracking_marker_label"), QStringLiteral("跟踪标记 frame"), handeye_tracking_marker_edit_);
  add_handeye_field(QStringLiteral("handeye_tag_family_label"), QStringLiteral("AprilTag 家族"), handeye_tag_family_combo_);
  add_handeye_field(QStringLiteral("handeye_tag_id_label"), QStringLiteral("AprilTag ID"), handeye_tag_id_spin_);
  add_handeye_field(QStringLiteral("handeye_tag_size_label"), QStringLiteral("AprilTag 边长"), handeye_tag_size_spin_);
  handeye_layout->addLayout(handeye_form);
  const auto update_tag_id_range = [this](const QString & family) {
      const QMap<QString, int> maximum_ids = {
        {QStringLiteral("16h5"), 29},
        {QStringLiteral("25h9"), 34},
        {QStringLiteral("36h11"), 586},
        {QStringLiteral("Circle21h7"), 37},
        {QStringLiteral("Circle49h12"), 65534},
        {QStringLiteral("Custom48h12"), 42210},
        {QStringLiteral("Standard41h12"), 2114},
        {QStringLiteral("Standard52h13"), 48713}};
      handeye_tag_id_spin_->setMaximum(maximum_ids.value(family, 586));
    };
  connect(
    handeye_tag_family_combo_, &QComboBox::currentTextChanged, this, update_tag_id_range);
  update_tag_id_range(handeye_tag_family_combo_->currentText());
  connect(
    handeye_target_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    [this](int) {
      const QString target = handeye_target_combo_->currentData().toString();
      const bool board = target == QStringLiteral("a4_4tag_board");
      const bool charuco = target == QStringLiteral("charuco_a4_5x7");
      if (board) {
        handeye_tag_family_combo_->setCurrentText(QStringLiteral("36h11"));
        handeye_tag_id_spin_->setValue(0);
        handeye_tag_size_spin_->setValue(40.0);
      } else if (charuco) {
        handeye_tag_id_spin_->setValue(0);
        handeye_tag_size_spin_->setValue(26.0);
      }
      handeye_tag_family_combo_->setEnabled(!board && !charuco);
      handeye_tag_id_spin_->setEnabled(!board && !charuco);
      handeye_tag_size_spin_->setEnabled(!charuco);
      retranslateUi();
    });
  connect(
    handeye_backend_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    [this](int) {
      const bool moveit = handeyeUsesMoveItCalibration();
      if (moveit) {
        const int charuco_index = handeye_target_combo_->findData(
          QStringLiteral("charuco_a4_5x7"));
        if (charuco_index >= 0) {handeye_target_combo_->setCurrentIndex(charuco_index);}
      }
      handeye_target_combo_->setEnabled(!moveit);
      handeye_name_edit_->setEnabled(!moveit);
      retranslateUi();
    });
  connect(
    handeye_mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    [this](int) {
      const QString current_name = handeye_name_edit_->text().trimmed();
      const QString prefix = active_robot_ == QStringLiteral("piperh") ?
        QStringLiteral("piperh_camera") : QStringLiteral("rebotarm_camera");
      if (handeyeEyeInHand()) {
        if (current_name == prefix) {
          handeye_name_edit_->setText(prefix + QStringLiteral("_eye_in_hand"));
        }
      } else if (current_name == prefix + QStringLiteral("_eye_in_hand")) {
        handeye_name_edit_->setText(prefix);
      }
      if (initialized_) {
        if (handeye_publish_process_->state() != QProcess::NotRunning) {
          handeye_publish_process_->terminate();
          handeye_publish_process_->waitForFinished(1000);
        }
        QTimer::singleShot(0, this, &DemoPanel::restartHandeyePublisher);
        updateReadiness();
      }
      retranslateUi();
    });
  handeye_status_label_ = new QLabel(handeye_page);
  handeye_status_label_->setObjectName(QStringLiteral("handeye_status_label"));
  handeye_status_label_->setWordWrap(true);
  handeye_layout->addWidget(handeye_status_label_);
  handeye_button_ = new QPushButton(handeye_page);
  handeye_button_->setObjectName(QStringLiteral("handeye_button"));
  handeye_layout->addWidget(handeye_button_);
  handeye_layout->addStretch(1);
  main_tabs_->addTab(handeye_page, QString());

  teach_tabs_ = new QTabWidget(teach_workspace_page);
  teach_tabs_->setObjectName(QStringLiteral("teach_tabs"));
  teach_tabs_->setTabPosition(QTabWidget::North);
  teach_tabs_->setUsesScrollButtons(true);
  teach_tabs_->setElideMode(Qt::ElideRight);
  auto * teach_action_page = new QWidget(teach_tabs_);
  auto * teach_action_page_layout = new QVBoxLayout(teach_action_page);
  teach_action_page_layout->setContentsMargins(0, 0, 0, 0);
  auto * teach_action_scroll = new QScrollArea(teach_action_page);
  teach_action_scroll->setWidgetResizable(true);
  teach_action_scroll->setFrameShape(QFrame::NoFrame);
  auto * teach_action_content = new QWidget(teach_action_scroll);
  teach_action_content->setObjectName(QStringLiteral("teach_action_content"));
  auto * teach_action_root = new QVBoxLayout(teach_action_content);
  teach_action_scroll->setWidget(teach_action_content);
  teach_action_page_layout->addWidget(teach_action_scroll);
  teach_tabs_->addTab(teach_action_page, QString());

  auto * operation_page = new QWidget(teach_tabs_);
  operation_page->setObjectName(QStringLiteral("robot_demo_operation_page"));
  auto * operation_page_layout = new QVBoxLayout(operation_page);
  operation_page_layout->setContentsMargins(0, 0, 0, 0);
  auto * operation_scroll = new QScrollArea(operation_page);
  operation_scroll->setWidgetResizable(true);
  operation_scroll->setFrameShape(QFrame::NoFrame);
  auto * operation_content = new QWidget(operation_scroll);
  auto * operation_root = new QVBoxLayout(operation_content);
  operation_scroll->setWidget(operation_content);
  operation_page_layout->addWidget(operation_scroll);
  teach_tabs_->addTab(operation_page, QString());

  auto * sequence_page = new QWidget(teach_tabs_);
  auto * sequence_page_layout = new QVBoxLayout(sequence_page);
  sequence_page_layout->setContentsMargins(0, 0, 0, 0);
  auto * sequence_scroll = new QScrollArea(sequence_page);
  sequence_scroll->setWidgetResizable(true);
  sequence_scroll->setFrameShape(QFrame::NoFrame);
  auto * sequence_content = new QWidget(sequence_scroll);
  auto * sequence_root = new QVBoxLayout(sequence_content);
  sequence_scroll->setWidget(sequence_content);
  sequence_page_layout->addWidget(sequence_scroll);
  teach_tabs_->addTab(sequence_page, QString());

  planning_page_ = new QWidget(teach_tabs_);
  planning_page_->setObjectName(QStringLiteral("free_motion_planning_page"));
  planning_layout_ = new QVBoxLayout(planning_page_);
  planning_layout_->setContentsMargins(0, 0, 0, 0);
  planning_placeholder_ = new QLabel(planning_page_);
  planning_placeholder_->setAlignment(Qt::AlignCenter);
  planning_placeholder_->setWordWrap(true);
  planning_layout_->addWidget(planning_placeholder_);
  planning_error_label_ = new QLabel(planning_page_);
  planning_error_label_->setObjectName(QStringLiteral("planning_error_label"));
  planning_error_label_->setWordWrap(true);
  planning_error_label_->setStyleSheet(QStringLiteral("color: #c43b32; font-weight: bold;"));
  planning_error_label_->hide();
  planning_layout_->addWidget(planning_error_label_);
  teach_tabs_->addTab(planning_page_, QString());

  connection_box_ = new QGroupBox(this);
  auto * connection_layout = new QVBoxLayout(connection_box_);
  model_label_ = new QLabel(connection_box_);
  feedback_label_ = new QLabel(connection_box_);
  validity_label_ = new QLabel(connection_box_);
  xbox_label_ = new QLabel(connection_box_);
  camera_label_ = new QLabel(connection_box_);
  connection_layout->addWidget(model_label_);
  connection_layout->addWidget(feedback_label_);
  connection_layout->addWidget(validity_label_);
  connection_layout->addWidget(xbox_label_);
  connection_layout->addWidget(camera_label_);
  auto * device_control_row = new QHBoxLayout();
  xbox_mode_button_ = new QPushButton(connection_box_);
  camera_button_ = new QPushButton(connection_box_);
  xbox_mode_button_->setEnabled(false);
  camera_button_->setEnabled(false);
  device_control_row->addWidget(xbox_mode_button_);
  device_control_row->addWidget(camera_button_);
  connection_layout->addLayout(device_control_row);
  xbox_help_button_ = new QPushButton(connection_box_);
  xbox_help_button_->setCheckable(true);
  xbox_help_button_->setChecked(false);
  connection_layout->addWidget(xbox_help_button_);
  xbox_help_container_ = new QWidget(connection_box_);
  auto * xbox_help_layout = new QVBoxLayout(xbox_help_container_);
  xbox_help_layout->setContentsMargins(0, 0, 0, 0);
  xbox_help_table_ = new QTableWidget(xbox_help_container_);
  xbox_help_table_->setColumnCount(3);
  xbox_help_table_->setRowCount(12);
  xbox_help_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  xbox_help_table_->setSelectionMode(QAbstractItemView::NoSelection);
  xbox_help_table_->setFocusPolicy(Qt::NoFocus);
  xbox_help_table_->setWordWrap(true);
  xbox_help_table_->verticalHeader()->setVisible(false);
  xbox_help_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  xbox_help_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  xbox_help_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  xbox_help_layout->addWidget(xbox_help_table_);
  connection_layout->addWidget(xbox_help_container_);
  xbox_help_container_->hide();
  overview_root->addWidget(connection_box_);

  forbidden_zone_box_ = new QGroupBox(this);
  forbidden_zone_box_->setObjectName(QStringLiteral("forbidden_zone_box"));
  auto * forbidden_zone_layout = new QVBoxLayout(forbidden_zone_box_);
  forbidden_zone_state_label_ = new QLabel(forbidden_zone_box_);
  forbidden_zone_state_label_->setObjectName(QStringLiteral("forbidden_zone_state_label"));
  forbidden_zone_state_label_->setWordWrap(true);
  forbidden_zone_areas_label_ = new QLabel(forbidden_zone_box_);
  forbidden_zone_areas_label_->setWordWrap(true);
  forbidden_zone_groups_label_ = new QLabel(forbidden_zone_box_);
  forbidden_zone_groups_label_->setWordWrap(true);
  forbidden_zone_config_label_ = new QLabel(forbidden_zone_box_);
  forbidden_zone_config_label_->setWordWrap(true);
  forbidden_zone_config_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  forbidden_zone_hint_label_ = new QLabel(forbidden_zone_box_);
  forbidden_zone_hint_label_->setWordWrap(true);
  forbidden_zone_reload_button_ = new QPushButton(forbidden_zone_box_);
  forbidden_zone_reload_button_->setObjectName(
    QStringLiteral("forbidden_zone_reload_button"));
  forbidden_zone_reload_button_->setEnabled(false);
  forbidden_zone_layout->addWidget(forbidden_zone_state_label_);
  forbidden_zone_layout->addWidget(forbidden_zone_areas_label_);
  forbidden_zone_layout->addWidget(forbidden_zone_groups_label_);
  forbidden_zone_layout->addWidget(forbidden_zone_config_label_);
  forbidden_zone_layout->addWidget(forbidden_zone_hint_label_);
  forbidden_zone_layout->addWidget(forbidden_zone_reload_button_);

  auto * forbidden_zone_editor = new QGroupBox(forbidden_zone_box_);
  forbidden_zone_editor->setObjectName(QStringLiteral("forbidden_zone_editor"));
  auto * editor_layout = new QVBoxLayout(forbidden_zone_editor);
  auto * editor_form = new QFormLayout();
  forbidden_zone_name_edit_ = new QLineEdit(forbidden_zone_editor);
  forbidden_zone_name_edit_->setMaxLength(64);
  forbidden_zone_name_edit_->setObjectName(QStringLiteral("forbidden_zone_name_edit"));
  forbidden_zone_shape_combo_ = new QComboBox(forbidden_zone_editor);
  forbidden_zone_shape_combo_->setObjectName(QStringLiteral("forbidden_zone_shape_combo"));
  forbidden_zone_shape_combo_->addItem(QString(), QStringLiteral("box"));
  forbidden_zone_shape_combo_->addItem(QString(), QStringLiteral("sphere"));
  forbidden_zone_shape_combo_->addItem(QString(), QStringLiteral("cylinder"));
  forbidden_zone_shape_combo_->addItem(QString(), QStringLiteral("cone"));
  forbidden_zone_shape_combo_->addItem(QString(), QStringLiteral("mesh"));
  editor_form->addRow(forbidden_zone_name_edit_, forbidden_zone_shape_combo_);
  editor_layout->addLayout(editor_form);
  forbidden_zone_dimensions_label_ = new QLabel(forbidden_zone_editor);
  editor_layout->addWidget(forbidden_zone_dimensions_label_);
  auto * dimensions_row = new QHBoxLayout();
  auto make_zone_distance = [forbidden_zone_editor]() {
      auto * spin = new QDoubleSpinBox(forbidden_zone_editor);
      spin->setRange(0.001, 5.0);
      spin->setDecimals(3);
      spin->setSingleStep(0.01);
      spin->setValue(0.10);
      spin->setSuffix(QStringLiteral(" m"));
      return spin;
    };
  forbidden_zone_dimension_1_spin_ = make_zone_distance();
  forbidden_zone_dimension_2_spin_ = make_zone_distance();
  forbidden_zone_dimension_3_spin_ = make_zone_distance();
  dimensions_row->addWidget(forbidden_zone_dimension_1_spin_);
  dimensions_row->addWidget(forbidden_zone_dimension_2_spin_);
  dimensions_row->addWidget(forbidden_zone_dimension_3_spin_);
  editor_layout->addLayout(dimensions_row);
  forbidden_zone_pose_label_ = new QLabel(forbidden_zone_editor);
  editor_layout->addWidget(forbidden_zone_pose_label_);
  auto * zone_pose_grid = new QGridLayout();
  auto make_zone_position = [forbidden_zone_editor]() {
      auto * spin = new QDoubleSpinBox(forbidden_zone_editor);
      spin->setRange(-5.0, 5.0);
      spin->setDecimals(3);
      spin->setSingleStep(0.01);
      spin->setSuffix(QStringLiteral(" m"));
      return spin;
    };
  auto make_zone_angle = [forbidden_zone_editor]() {
      auto * spin = new QDoubleSpinBox(forbidden_zone_editor);
      spin->setRange(-180.0, 180.0);
      spin->setDecimals(1);
      spin->setSingleStep(5.0);
      spin->setSuffix(QStringLiteral("°"));
      return spin;
    };
  forbidden_zone_x_spin_ = make_zone_position();
  forbidden_zone_y_spin_ = make_zone_position();
  forbidden_zone_z_spin_ = make_zone_position();
  forbidden_zone_roll_spin_ = make_zone_angle();
  forbidden_zone_pitch_spin_ = make_zone_angle();
  forbidden_zone_yaw_spin_ = make_zone_angle();
  forbidden_zone_x_spin_->setPrefix(QStringLiteral("X "));
  forbidden_zone_y_spin_->setPrefix(QStringLiteral("Y "));
  forbidden_zone_z_spin_->setPrefix(QStringLiteral("Z "));
  forbidden_zone_roll_spin_->setPrefix(QStringLiteral("R "));
  forbidden_zone_pitch_spin_->setPrefix(QStringLiteral("P "));
  forbidden_zone_yaw_spin_->setPrefix(QStringLiteral("Y "));
  zone_pose_grid->addWidget(forbidden_zone_x_spin_, 0, 0);
  zone_pose_grid->addWidget(forbidden_zone_y_spin_, 0, 1);
  zone_pose_grid->addWidget(forbidden_zone_z_spin_, 0, 2);
  zone_pose_grid->addWidget(forbidden_zone_roll_spin_, 1, 0);
  zone_pose_grid->addWidget(forbidden_zone_pitch_spin_, 1, 1);
  zone_pose_grid->addWidget(forbidden_zone_yaw_spin_, 1, 2);
  editor_layout->addLayout(zone_pose_grid);
  forbidden_zone_mesh_row_widget_ = new QWidget(forbidden_zone_editor);
  auto * mesh_row = new QHBoxLayout(forbidden_zone_mesh_row_widget_);
  mesh_row->setContentsMargins(0, 0, 0, 0);
  forbidden_zone_mesh_button_ = new QPushButton(forbidden_zone_mesh_row_widget_);
  forbidden_zone_mesh_path_label_ = new QLabel(forbidden_zone_mesh_row_widget_);
  forbidden_zone_mesh_path_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  forbidden_zone_mesh_path_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  mesh_row->addWidget(forbidden_zone_mesh_button_);
  mesh_row->addWidget(forbidden_zone_mesh_path_label_, 1);
  editor_layout->addWidget(forbidden_zone_mesh_row_widget_);
  auto * mesh_scale_row = new QHBoxLayout();
  forbidden_zone_mesh_scale_label_ = new QLabel(forbidden_zone_editor);
  forbidden_zone_mesh_scale_spin_ = new QDoubleSpinBox(forbidden_zone_editor);
  forbidden_zone_mesh_scale_spin_->setRange(0.000001, 1000.0);
  forbidden_zone_mesh_scale_spin_->setDecimals(6);
  forbidden_zone_mesh_scale_spin_->setValue(1.0);
  forbidden_zone_mesh_scale_spin_->setSingleStep(0.001);
  mesh_scale_row->addWidget(forbidden_zone_mesh_scale_label_);
  mesh_scale_row->addWidget(forbidden_zone_mesh_scale_spin_);
  mesh_scale_row->addStretch(1);
  editor_layout->addLayout(mesh_scale_row);
  forbidden_zone_apply_button_ = new QPushButton(forbidden_zone_editor);
  forbidden_zone_apply_button_->setObjectName(QStringLiteral("forbidden_zone_apply_button"));
  editor_layout->addWidget(forbidden_zone_apply_button_);
  auto * delete_row = new QHBoxLayout();
  forbidden_zone_user_combo_ = new QComboBox(forbidden_zone_editor);
  forbidden_zone_remove_button_ = new QPushButton(forbidden_zone_editor);
  delete_row->addWidget(forbidden_zone_user_combo_, 1);
  delete_row->addWidget(forbidden_zone_remove_button_);
  editor_layout->addLayout(delete_row);
  forbidden_zone_layout->addWidget(forbidden_zone_editor);

  auto * forbidden_zone_group_editor = new QGroupBox(forbidden_zone_box_);
  forbidden_zone_group_editor->setObjectName(QStringLiteral("forbidden_zone_group_editor"));
  auto * group_editor_layout = new QVBoxLayout(forbidden_zone_group_editor);
  forbidden_zone_group_name_edit_ = new QLineEdit(forbidden_zone_group_editor);
  forbidden_zone_group_name_edit_->setMaxLength(64);
  forbidden_zone_group_name_edit_->setObjectName(
    QStringLiteral("forbidden_zone_group_name_edit"));
  group_editor_layout->addWidget(forbidden_zone_group_name_edit_);
  forbidden_zone_group_members_list_ = new QListWidget(forbidden_zone_group_editor);
  forbidden_zone_group_members_list_->setObjectName(
    QStringLiteral("forbidden_zone_group_members_list"));
  forbidden_zone_group_members_list_->setSelectionMode(QAbstractItemView::MultiSelection);
  forbidden_zone_group_members_list_->setMaximumHeight(110);
  group_editor_layout->addWidget(forbidden_zone_group_members_list_);
  forbidden_zone_group_save_button_ = new QPushButton(forbidden_zone_group_editor);
  forbidden_zone_group_save_button_->setObjectName(
    QStringLiteral("forbidden_zone_group_save_button"));
  group_editor_layout->addWidget(forbidden_zone_group_save_button_);
  auto * group_delete_row = new QHBoxLayout();
  forbidden_zone_group_combo_ = new QComboBox(forbidden_zone_group_editor);
  forbidden_zone_group_combo_->setObjectName(QStringLiteral("forbidden_zone_group_combo"));
  forbidden_zone_group_remove_button_ = new QPushButton(forbidden_zone_group_editor);
  forbidden_zone_group_remove_button_->setObjectName(
    QStringLiteral("forbidden_zone_group_remove_button"));
  group_delete_row->addWidget(forbidden_zone_group_combo_, 1);
  group_delete_row->addWidget(forbidden_zone_group_remove_button_);
  group_editor_layout->addLayout(group_delete_row);
  forbidden_zone_layout->addWidget(forbidden_zone_group_editor);
  overview_root->addWidget(forbidden_zone_box_);

  safety_box_ = new QGroupBox(this);
  auto * safety_layout = new QVBoxLayout(safety_box_);
  warning_label_ = new QLabel(safety_box_);
  warning_label_->setWordWrap(true);
  warning_label_->setStyleSheet(QStringLiteral("color: #d08020; font-weight: bold;"));
  safety_layout->addWidget(warning_label_);

  auto * speed_row = new QHBoxLayout();
  speed_label_ = new QLabel(safety_box_);
  speed_row->addWidget(speed_label_);
  speed_spin_ = new QDoubleSpinBox(safety_box_);
  speed_spin_->setRange(1.0, 20.0);
  speed_spin_->setValue(5.0);
  speed_spin_->setDecimals(0);
  speed_spin_->setSuffix(QStringLiteral("%"));
  speed_row->addWidget(speed_spin_);
  speed_row->addStretch(1);
  overview_root->addWidget(safety_box_);

  // Keep motion commands and their speed parameter together inside the Demos & Actions tab.
  // The Robot & Safety tab is reserved for connection/interlock and forbidden-zone status.
  auto * operation_box = new QGroupBox(operation_content);
  operation_box->setObjectName(QStringLiteral("demo_operation_box"));
  auto * operation_layout = new QVBoxLayout(operation_box);
  operation_layout->addLayout(speed_row);
  home_button_ = new QPushButton(operation_box);
  singularity_escape_box_ = new QGroupBox(operation_box);
  singularity_escape_box_->setObjectName(QStringLiteral("singularity_escape_box"));
  auto * singularity_escape_layout = new QVBoxLayout(singularity_escape_box_);
  singularity_escape_hint_label_ = new QLabel(singularity_escape_box_);
  singularity_escape_hint_label_->setObjectName(
    QStringLiteral("singularity_escape_hint_label"));
  singularity_escape_hint_label_->setWordWrap(true);
  singularity_escape_layout->addWidget(singularity_escape_hint_label_);
  auto * singularity_escape_grid = new QGridLayout();
  const std::array<double, 6> dm_escape_defaults = {0.0, -1.60, -0.90, -0.90, 0.50, 0.0};
  for (int index = 0; index < 6; ++index) {
    auto * label = new QLabel(QStringLiteral("J%1").arg(index + 1), singularity_escape_box_);
    auto * spin = new QDoubleSpinBox(singularity_escape_box_);
    spin->setObjectName(QStringLiteral("singularity_escape_joint_%1_spin").arg(index + 1));
    spin->setRange(-3.14, 3.14);
    spin->setDecimals(3);
    spin->setSingleStep(0.05);
    spin->setSuffix(QStringLiteral(" rad"));
    spin->setValue(dm_escape_defaults[static_cast<std::size_t>(index)]);
    singularity_escape_joint_spins_[static_cast<std::size_t>(index)] = spin;
    const int column = (index % 3) * 2;
    const int row = index / 3;
    singularity_escape_grid->addWidget(label, row, column);
    singularity_escape_grid->addWidget(spin, row, column + 1);
  }
  singularity_escape_layout->addLayout(singularity_escape_grid);
  auto * singularity_escape_buttons = new QHBoxLayout();
  singularity_escape_reset_button_ = new QPushButton(singularity_escape_box_);
  singularity_escape_reset_button_->setObjectName(
    QStringLiteral("singularity_escape_reset_button"));
  singularity_escape_button_ = new QPushButton(singularity_escape_box_);
  singularity_escape_button_->setObjectName(QStringLiteral("singularity_escape_button"));
  singularity_escape_button_->setEnabled(false);
  singularity_escape_buttons->addWidget(singularity_escape_reset_button_);
  singularity_escape_buttons->addWidget(singularity_escape_button_, 1);
  singularity_escape_layout->addLayout(singularity_escape_buttons);
  sync_button_ = new QPushButton(operation_box);
  pick_place_button_ = new QPushButton(operation_box);
  stop_button_ = new QPushButton(operation_box);
  home_button_->setEnabled(false);
  sync_button_->setEnabled(false);
  pick_place_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  operation_layout->addWidget(home_button_);
  operation_layout->addWidget(singularity_escape_box_);
  operation_layout->addWidget(stop_button_);
  // These legacy shortcuts are intentionally removed from the UI.  Keep the
  // controls allocated for compatibility with the existing slots/readiness
  // logic, but never expose them in the operation layout.
  sync_button_->hide();
  pick_place_button_->hide();
  operation_root->addWidget(operation_box);
  operation_root->addStretch(1);

  teach_box_ = new QGroupBox(teach_action_content);
  teach_box_->setObjectName(QStringLiteral("teach_action_editor_box"));
  teach_box_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  auto * teach_layout = new QVBoxLayout(teach_box_);
  teach_action_mode_tabs_ = new QTabWidget(teach_box_);
  teach_action_mode_tabs_->setObjectName(QStringLiteral("teach_action_mode_tabs"));
  teach_action_mode_tabs_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  auto * teach_editor_page = new QWidget(teach_action_mode_tabs_);
  auto * teach_editor_page_layout = new QVBoxLayout(teach_editor_page);
  teach_editor_page_layout->setContentsMargins(0, 0, 0, 0);
  auto * teach_editor_scroll = new QScrollArea(teach_editor_page);
  teach_editor_scroll->setObjectName(QStringLiteral("teach_shape_editor_scroll"));
  teach_editor_scroll->setWidgetResizable(true);
  teach_editor_scroll->setFrameShape(QFrame::NoFrame);
  auto * teach_editor_content = new QWidget(teach_editor_scroll);
  teach_editor_content->setObjectName(QStringLiteral("teach_shape_editor_content"));
  auto * teach_editor_layout = new QVBoxLayout(teach_editor_content);
  teach_editor_layout->setContentsMargins(4, 4, 4, 4);
  // QTabWidget's minimum-size hint does not include the full current page.  Without
  // a scrollable page, RViz can therefore shrink this long form below its layout
  // minimum and several rows paint on top of each other.  Apply the layout's
  // minimum size to the scroll area's content so a small dock gets a scrollbar
  // instead of compressed controls.
  teach_editor_layout->setSizeConstraint(QLayout::SetMinAndMaxSize);
  teach_editor_scroll->setWidget(teach_editor_content);
  teach_editor_page_layout->addWidget(teach_editor_scroll);
  auto * teach_playback_page = new QWidget(teach_action_mode_tabs_);
  auto * teach_playback_layout = new QVBoxLayout(teach_playback_page);
  teach_playback_layout->setContentsMargins(4, 4, 4, 4);
  teach_action_mode_tabs_->addTab(teach_editor_page, QString());
  teach_action_mode_tabs_->addTab(teach_playback_page, QString());
  teach_layout->addWidget(teach_action_mode_tabs_);
  auto * action_row = new QHBoxLayout();
  teach_action_combo_ = new QComboBox(teach_action_content);
  teach_action_combo_->setObjectName(QStringLiteral("teach_action_combo"));
  teach_refresh_button_ = new QPushButton(teach_action_content);
  teach_copy_button_ = new QPushButton(teach_action_content);
  teach_copy_button_->setObjectName(QStringLiteral("teach_copy_button"));
  teach_rename_button_ = new QPushButton(teach_action_content);
  teach_delete_button_ = new QPushButton(teach_action_content);
  teach_delete_button_->setObjectName(QStringLiteral("teach_delete_button"));
  action_row->addWidget(teach_action_combo_, 1);
  action_row->addWidget(teach_copy_button_);
  action_row->addWidget(teach_rename_button_);
  action_row->addWidget(teach_delete_button_);
  action_row->addWidget(teach_refresh_button_);
  teach_action_root->addLayout(action_row);
  teach_workspace_layout->addWidget(teach_tabs_, 1);
  // Give the embedded editor (including the Piper-H joint-coordinate tab)
  // the remaining dock height instead of reserving it as empty space below.
  teach_action_root->addWidget(teach_box_, 1);

  auto * shape_edit_row = new QHBoxLayout();
  teach_shape_edit_status_label_ = new QLabel(teach_editor_content);
  teach_shape_edit_status_label_->setObjectName(
    QStringLiteral("teach_shape_edit_status_label"));
  teach_shape_edit_status_label_->setWordWrap(true);
  teach_shape_new_button_ = new QPushButton(teach_editor_content);
  teach_shape_new_button_->setObjectName(QStringLiteral("teach_shape_new_button"));
  shape_edit_row->addWidget(teach_shape_edit_status_label_, 1);
  shape_edit_row->addWidget(teach_shape_new_button_);
  teach_editor_layout->addLayout(shape_edit_row);

  teach_shape_box_ = new QGroupBox(teach_editor_content);
  auto * shape_layout = new QVBoxLayout(teach_shape_box_);
  auto * shape_row = new QHBoxLayout();
  teach_shape_combo_ = new QComboBox(teach_shape_box_);
  teach_shape_combo_->addItem(QString(), QStringLiteral("rectangle"));
  teach_shape_combo_->addItem(QString(), QStringLiteral("triangle"));
  teach_shape_combo_->addItem(QString(), QStringLiteral("circle"));
  teach_shape_combo_->addItem(QString(), QStringLiteral("star"));
  teach_shape_combo_->addItem(QString(), QStringLiteral("heart"));
  teach_shape_combo_->addItem(QString(), QStringLiteral("text"));
  teach_shape_combo_->addItem(QString(), QStringLiteral("image"));
  teach_shape_combo_->addItem(QString(), QStringLiteral("freehand"));
  teach_shape_create_button_ = new QPushButton(teach_shape_box_);
  teach_shape_create_button_->setEnabled(false);
  shape_row->addWidget(teach_shape_combo_, 1);
  shape_row->addWidget(teach_shape_create_button_);
  shape_layout->addLayout(shape_row);
  auto * shape_source_row = new QHBoxLayout();
  teach_shape_source_edit_ = new QLineEdit(teach_shape_box_);
  teach_shape_source_edit_->setMaxLength(24);
  teach_shape_source_edit_->setText(QStringLiteral("HELLO"));
  teach_shape_image_button_ = new QPushButton(teach_shape_box_);
  teach_shape_source_label_ = new QLabel(teach_shape_box_);
  teach_shape_source_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  teach_shape_source_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  shape_source_row->addWidget(teach_shape_source_edit_, 1);
  shape_source_row->addWidget(teach_shape_image_button_);
  shape_source_row->addWidget(teach_shape_source_label_, 1);
  shape_layout->addLayout(shape_source_row);
  teach_shape_selected_label_ = new QLabel(teach_shape_box_);
  teach_shape_selected_label_->setStyleSheet(
    QStringLiteral("color: #d08020; font-weight: bold;"));
  shape_layout->addWidget(teach_shape_selected_label_);
  teach_shape_preview_label_ = new QLabel(teach_shape_box_);
  teach_shape_preview_label_->setFixedHeight(105);
  teach_shape_preview_label_->setAlignment(Qt::AlignCenter);
  teach_shape_preview_label_->setStyleSheet(
    QStringLiteral("background: #303238; border: 1px solid #686b72; border-radius: 3px;"));
  shape_layout->addWidget(teach_shape_preview_label_);
  auto * pose_grid = new QGridLayout();
  teach_shape_position_label_ = new QLabel(teach_shape_box_);
  teach_shape_orientation_label_ = new QLabel(teach_shape_box_);
  pose_grid->addWidget(teach_shape_position_label_, 0, 0, 1, 3);
  pose_grid->addWidget(teach_shape_orientation_label_, 2, 0, 1, 3);
  auto make_position_spin = [this]() {
      auto * spin = new QDoubleSpinBox(teach_shape_box_);
      spin->setRange(-2.0, 2.0);
      spin->setDecimals(3);
      spin->setSingleStep(0.01);
      spin->setSuffix(QStringLiteral(" m"));
      return spin;
    };
  auto make_angle_spin = [this]() {
      auto * spin = new QDoubleSpinBox(teach_shape_box_);
      spin->setRange(-180.0, 180.0);
      spin->setDecimals(1);
      spin->setSingleStep(5.0);
      spin->setSuffix(QStringLiteral("°"));
      return spin;
    };
  teach_shape_x_spin_ = make_position_spin();
  teach_shape_y_spin_ = make_position_spin();
  teach_shape_z_spin_ = make_position_spin();
  teach_shape_roll_spin_ = make_angle_spin();
  teach_shape_pitch_spin_ = make_angle_spin();
  teach_shape_yaw_spin_ = make_angle_spin();
  teach_shape_size_label_ = new QLabel(teach_shape_box_);
  teach_shape_pen_label_ = new QLabel(teach_shape_box_);
  teach_shape_width_spin_ = make_position_spin();
  teach_shape_height_spin_ = make_position_spin();
  teach_shape_width_spin_->setObjectName(QStringLiteral("teach_shape_width_spin"));
  teach_shape_height_spin_->setObjectName(QStringLiteral("teach_shape_height_spin"));
  teach_shape_width_spin_->setRange(0.0, 0.50);
  teach_shape_height_spin_->setRange(0.0, 0.50);
  teach_shape_width_spin_->setValue(0.06);
  teach_shape_height_spin_->setValue(0.08);
  teach_shape_pen_length_spin_ = new QDoubleSpinBox(teach_shape_box_);
  teach_shape_pen_length_spin_->setRange(0.0, 500.0);
  teach_shape_pen_length_spin_->setDecimals(1);
  teach_shape_pen_length_spin_->setSingleStep(1.0);
  teach_shape_pen_length_spin_->setSuffix(QStringLiteral(" mm"));
  teach_shape_pen_lift_spin_ = new QDoubleSpinBox(teach_shape_box_);
  teach_shape_pen_lift_spin_->setObjectName(QStringLiteral("teach_shape_pen_lift_spin"));
  teach_shape_pen_lift_spin_->setRange(0.0, 100.0);
  teach_shape_pen_lift_spin_->setDecimals(1);
  teach_shape_pen_lift_spin_->setSingleStep(1.0);
  teach_shape_pen_lift_spin_->setValue(12.0);
  teach_shape_pen_lift_spin_->setSuffix(QStringLiteral(" mm"));
  teach_shape_x_spin_->setPrefix(QStringLiteral("X "));
  teach_shape_y_spin_->setPrefix(QStringLiteral("Y "));
  teach_shape_z_spin_->setPrefix(QStringLiteral("Z "));
  teach_shape_roll_spin_->setPrefix(QStringLiteral("R "));
  teach_shape_pitch_spin_->setPrefix(QStringLiteral("P "));
  teach_shape_yaw_spin_->setPrefix(QStringLiteral("Y "));
  teach_shape_width_spin_->setPrefix(QStringLiteral("W "));
  teach_shape_height_spin_->setPrefix(QStringLiteral("H "));
  teach_shape_pen_length_spin_->setPrefix(QStringLiteral("L "));
  teach_shape_pen_lift_spin_->setPrefix(QStringLiteral("↑ "));
  teach_shape_x_spin_->setValue(0.28);
  teach_shape_z_spin_->setValue(0.12);
  pose_grid->addWidget(teach_shape_x_spin_, 1, 0);
  pose_grid->addWidget(teach_shape_y_spin_, 1, 1);
  pose_grid->addWidget(teach_shape_z_spin_, 1, 2);
  pose_grid->addWidget(teach_shape_roll_spin_, 3, 0);
  pose_grid->addWidget(teach_shape_pitch_spin_, 3, 1);
  pose_grid->addWidget(teach_shape_yaw_spin_, 3, 2);
  pose_grid->addWidget(teach_shape_size_label_, 4, 0, 1, 3);
  pose_grid->addWidget(teach_shape_width_spin_, 5, 0);
  pose_grid->addWidget(teach_shape_height_spin_, 5, 1);
  pose_grid->addWidget(teach_shape_pen_label_, 6, 0, 1, 3);
  pose_grid->addWidget(teach_shape_pen_length_spin_, 7, 0);
  pose_grid->addWidget(teach_shape_pen_lift_spin_, 7, 1);
  shape_layout->addLayout(pose_grid);
  for (auto * spin : {
      teach_shape_x_spin_, teach_shape_y_spin_, teach_shape_z_spin_,
      teach_shape_roll_spin_, teach_shape_pitch_spin_, teach_shape_yaw_spin_,
      teach_shape_width_spin_, teach_shape_height_spin_,
      teach_shape_pen_length_spin_, teach_shape_pen_lift_spin_})
  {
    connect(
      spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
      [this](double) {
        if (!shape_pose_syncing_) {shape_pose_dirty_ = true;}
      });
  }
  teach_shape_apply_pose_button_ = new QPushButton(teach_shape_box_);
  teach_shape_apply_pose_button_->setEnabled(false);
  shape_layout->addWidget(teach_shape_apply_pose_button_);
  teach_shape_reachability_button_ = new QPushButton(teach_shape_box_);
  teach_shape_reachability_button_->setObjectName(
    QStringLiteral("teach_shape_reachability_button"));
  teach_shape_reachability_button_->setEnabled(false);
  shape_layout->addWidget(teach_shape_reachability_button_);
  teach_shape_reachability_label_ = new QLabel(teach_shape_box_);
  teach_shape_reachability_label_->setObjectName(
    QStringLiteral("teach_shape_reachability_label"));
  teach_shape_reachability_label_->setWordWrap(true);
  shape_layout->addWidget(teach_shape_reachability_label_);
  auto * marker_row = new QHBoxLayout();
  teach_shape_marker_button_ = new QPushButton(teach_shape_box_);
  teach_shape_reset_button_ = new QPushButton(teach_shape_box_);
  teach_shape_marker_button_->setEnabled(false);
  teach_shape_reset_button_->setEnabled(false);
  marker_row->addWidget(teach_shape_marker_button_);
  marker_row->addWidget(teach_shape_reset_button_);
  shape_layout->addLayout(marker_row);
  teach_shape_marker_hint_label_ = new QLabel(teach_shape_box_);
  teach_shape_marker_hint_label_->setWordWrap(true);
  shape_layout->addWidget(teach_shape_marker_hint_label_);
  teach_trace_legend_label_ = new QLabel(teach_shape_box_);
  teach_trace_legend_label_->setWordWrap(true);
  shape_layout->addWidget(teach_trace_legend_label_);
  auto * trace_width_row = new QHBoxLayout();
  teach_trace_width_label_ = new QLabel(teach_shape_box_);
  teach_trace_width_spin_ = new QDoubleSpinBox(teach_shape_box_);
  teach_trace_width_spin_->setRange(0.5, 30.0);
  teach_trace_width_spin_->setDecimals(1);
  teach_trace_width_spin_->setSingleStep(0.5);
  teach_trace_width_spin_->setValue(4.0);
  teach_trace_width_spin_->setSuffix(QStringLiteral(" mm"));
  teach_trace_width_spin_->setEnabled(false);
  teach_trace_width_timer_ = new QTimer(this);
  teach_trace_width_timer_->setSingleShot(true);
  teach_trace_width_timer_->setInterval(300);
  trace_width_row->addWidget(teach_trace_width_label_);
  trace_width_row->addWidget(teach_trace_width_spin_);
  trace_width_row->addStretch(1);
  shape_layout->addLayout(trace_width_row);
  teach_editor_layout->addWidget(teach_shape_box_);

  teach_recording_widget_ = new QWidget(teach_editor_content);
  teach_recording_widget_->setObjectName(QStringLiteral("teach_recording_widget"));
  auto * recording_row = new QHBoxLayout(teach_recording_widget_);
  recording_row->setContentsMargins(0, 0, 0, 0);
  teach_start_button_ = new QPushButton(teach_recording_widget_);
  teach_stop_button_ = new QPushButton(teach_recording_widget_);
  recording_row->addWidget(teach_start_button_);
  recording_row->addWidget(teach_stop_button_);
  teach_editor_layout->addWidget(teach_recording_widget_);
  teach_editor_layout->addStretch(1);

  auto * teach_speed_row = new QHBoxLayout();
  teach_replay_speed_label_ = new QLabel(teach_box_);
  teach_replay_speed_spin_ = new QDoubleSpinBox(teach_box_);
  teach_replay_speed_spin_->setRange(0.1, 2.0);
  teach_replay_speed_spin_->setSingleStep(0.1);
  teach_replay_speed_spin_->setDecimals(1);
  teach_replay_speed_spin_->setValue(1.0);
  teach_replay_speed_spin_->setSuffix(QStringLiteral("×"));
  teach_speed_row->addWidget(teach_replay_speed_label_);
  teach_speed_row->addWidget(teach_replay_speed_spin_);
  teach_speed_row->addStretch(1);
  teach_playback_layout->addLayout(teach_speed_row);

  auto * playback_row = new QHBoxLayout();
  teach_preview_button_ = new QPushButton(teach_box_);
  teach_replay_button_ = new QPushButton(teach_box_);
  teach_pause_button_ = new QPushButton(teach_box_);
  teach_cancel_button_ = new QPushButton(teach_box_);
  playback_row->addWidget(teach_preview_button_);
  playback_row->addWidget(teach_replay_button_);
  playback_row->addWidget(teach_pause_button_);
  playback_row->addWidget(teach_cancel_button_);
  teach_playback_layout->addLayout(playback_row);

  teach_hardware_action_progress_label_ = new QLabel(teach_box_);
  teach_hardware_action_progress_bar_ = new QProgressBar(teach_box_);
  teach_hardware_action_progress_bar_->setRange(0, 1000);
  teach_hardware_action_progress_bar_->setValue(0);
  teach_hardware_action_progress_bar_->setFormat(QStringLiteral("0.0%"));
  teach_playback_layout->addWidget(teach_hardware_action_progress_label_);
  teach_playback_layout->addWidget(teach_hardware_action_progress_bar_);
  teach_hardware_overall_progress_label_ = new QLabel(teach_box_);
  teach_hardware_overall_progress_bar_ = new QProgressBar(teach_box_);
  teach_hardware_overall_progress_bar_->setRange(0, 1000);
  teach_hardware_overall_progress_bar_->setValue(0);
  teach_hardware_overall_progress_bar_->setFormat(QStringLiteral("0.0%"));
  teach_playback_layout->addWidget(teach_hardware_overall_progress_label_);
  teach_playback_layout->addWidget(teach_hardware_overall_progress_bar_);

  auto * slider_title_row = new QHBoxLayout();
  teach_slider_label_ = new QLabel(teach_box_);
  teach_progress_label_ = new QLabel(QStringLiteral("0%"), teach_box_);
  slider_title_row->addWidget(teach_slider_label_);
  slider_title_row->addStretch(1);
  slider_title_row->addWidget(teach_progress_label_);
  teach_playback_layout->addLayout(slider_title_row);
  teach_slider_ = new QSlider(Qt::Horizontal, teach_box_);
  teach_slider_->setRange(0, 1000);
  teach_slider_->setSingleStep(10);
  teach_slider_->setPageStep(50);
  teach_playback_layout->addWidget(teach_slider_);
  teach_playback_layout->addStretch(1);

  teach_sequence_box_ = new QGroupBox(sequence_content);
  auto * sequence_layout = new QVBoxLayout(teach_sequence_box_);
  auto * sequence_select_row = new QHBoxLayout();
  teach_sequence_combo_ = new QComboBox(teach_sequence_box_);
  teach_sequence_refresh_button_ = new QPushButton(teach_sequence_box_);
  sequence_select_row->addWidget(teach_sequence_combo_, 1);
  sequence_select_row->addWidget(teach_sequence_refresh_button_);
  sequence_layout->addLayout(sequence_select_row);
  teach_sequence_list_ = new QListWidget(teach_sequence_box_);
  teach_sequence_list_->setObjectName(QStringLiteral("teach_sequence_list"));
  teach_sequence_list_->setMinimumHeight(90);
  sequence_layout->addWidget(teach_sequence_list_);
  teach_sequence_selection_hint_label_ = new QLabel(teach_sequence_box_);
  teach_sequence_selection_hint_label_->setWordWrap(true);
  sequence_layout->addWidget(teach_sequence_selection_hint_label_);
  auto * sequence_edit_row = new QHBoxLayout();
  teach_sequence_add_button_ = new QPushButton(teach_sequence_box_);
  teach_sequence_remove_button_ = new QPushButton(teach_sequence_box_);
  teach_sequence_up_button_ = new QPushButton(teach_sequence_box_);
  teach_sequence_down_button_ = new QPushButton(teach_sequence_box_);
  sequence_edit_row->addWidget(teach_sequence_add_button_);
  sequence_edit_row->addWidget(teach_sequence_remove_button_);
  sequence_edit_row->addWidget(teach_sequence_up_button_);
  sequence_edit_row->addWidget(teach_sequence_down_button_);
  sequence_layout->addLayout(sequence_edit_row);
  auto * sequence_speed_row = new QHBoxLayout();
  teach_sequence_replay_speed_label_ = new QLabel(teach_sequence_box_);
  teach_sequence_replay_speed_spin_ = new QDoubleSpinBox(teach_sequence_box_);
  teach_sequence_replay_speed_spin_->setObjectName(
    QStringLiteral("teach_sequence_replay_speed_spin"));
  teach_sequence_replay_speed_spin_->setRange(0.1, 2.0);
  teach_sequence_replay_speed_spin_->setSingleStep(0.1);
  teach_sequence_replay_speed_spin_->setDecimals(1);
  teach_sequence_replay_speed_spin_->setValue(1.0);
  teach_sequence_replay_speed_spin_->setSuffix(QStringLiteral("×"));
  sequence_speed_row->addWidget(teach_sequence_replay_speed_label_);
  sequence_speed_row->addWidget(teach_sequence_replay_speed_spin_);
  sequence_speed_row->addStretch(1);
  sequence_layout->addLayout(sequence_speed_row);
  teach_sequence_save_button_ = new QPushButton(teach_sequence_box_);
  sequence_layout->addWidget(teach_sequence_save_button_);
  auto * sequence_action_row = new QHBoxLayout();
  teach_sequence_play_button_ = new QPushButton(teach_sequence_box_);
  teach_sequence_play_button_->setObjectName(QStringLiteral("teach_sequence_play_button"));
  teach_sequence_replay_button_ = new QPushButton(teach_sequence_box_);
  teach_sequence_pause_button_ = new QPushButton(teach_sequence_box_);
  teach_sequence_cancel_button_ = new QPushButton(teach_sequence_box_);
  teach_sequence_pause_button_->setObjectName(
    QStringLiteral("teach_sequence_pause_button"));
  teach_sequence_cancel_button_->setObjectName(
    QStringLiteral("teach_sequence_cancel_button"));
  sequence_action_row->addWidget(teach_sequence_play_button_);
  sequence_action_row->addWidget(teach_sequence_replay_button_);
  sequence_action_row->addWidget(teach_sequence_pause_button_);
  sequence_action_row->addWidget(teach_sequence_cancel_button_);
  sequence_layout->addLayout(sequence_action_row);
  teach_sequence_hardware_action_progress_label_ = new QLabel(teach_sequence_box_);
  teach_sequence_hardware_action_progress_bar_ = new QProgressBar(teach_sequence_box_);
  teach_sequence_hardware_action_progress_bar_->setRange(0, 1000);
  teach_sequence_hardware_action_progress_bar_->setValue(0);
  teach_sequence_hardware_action_progress_bar_->setFormat(QStringLiteral("0.0%"));
  sequence_layout->addWidget(teach_sequence_hardware_action_progress_label_);
  sequence_layout->addWidget(teach_sequence_hardware_action_progress_bar_);
  teach_sequence_hardware_overall_progress_label_ = new QLabel(teach_sequence_box_);
  teach_sequence_hardware_overall_progress_bar_ = new QProgressBar(teach_sequence_box_);
  teach_sequence_hardware_overall_progress_bar_->setRange(0, 1000);
  teach_sequence_hardware_overall_progress_bar_->setValue(0);
  teach_sequence_hardware_overall_progress_bar_->setFormat(QStringLiteral("0.0%"));
  sequence_layout->addWidget(teach_sequence_hardware_overall_progress_label_);
  sequence_layout->addWidget(teach_sequence_hardware_overall_progress_bar_);
  auto * sequence_slider_title_row = new QHBoxLayout();
  teach_sequence_slider_label_ = new QLabel(teach_sequence_box_);
  teach_sequence_progress_label_ = new QLabel(QStringLiteral("0.0%"), teach_sequence_box_);
  sequence_slider_title_row->addWidget(teach_sequence_slider_label_);
  sequence_slider_title_row->addStretch(1);
  sequence_slider_title_row->addWidget(teach_sequence_progress_label_);
  sequence_layout->addLayout(sequence_slider_title_row);
  teach_sequence_slider_ = new QSlider(Qt::Horizontal, teach_sequence_box_);
  teach_sequence_slider_->setObjectName(QStringLiteral("teach_sequence_slider"));
  teach_sequence_slider_->setRange(0, 1000);
  teach_sequence_slider_->setSingleStep(10);
  teach_sequence_slider_->setPageStep(50);
  sequence_layout->addWidget(teach_sequence_slider_);
  sequence_root->addWidget(teach_sequence_box_);
  sequence_root->addStretch(1);

  teach_status_label_ = new QLabel(teach_workspace_page);
  teach_status_label_->setWordWrap(true);
  teach_workspace_layout->addWidget(teach_status_label_);

  for (auto * button : {
      teach_refresh_button_, teach_copy_button_, teach_rename_button_, teach_delete_button_,
      teach_start_button_, teach_stop_button_,
      teach_preview_button_, teach_replay_button_, teach_pause_button_, teach_cancel_button_,
      teach_sequence_refresh_button_, teach_sequence_add_button_,
      teach_sequence_remove_button_, teach_sequence_up_button_,
      teach_sequence_down_button_, teach_sequence_save_button_,
      teach_sequence_play_button_,
      teach_sequence_replay_button_, teach_sequence_pause_button_,
      teach_sequence_cancel_button_})
  {
    button->setEnabled(false);
  }
  teach_action_combo_->setEnabled(false);
  teach_sequence_combo_->setEnabled(false);
  teach_sequence_list_->setEnabled(false);
  teach_slider_->setEnabled(false);
  teach_replay_speed_spin_->setEnabled(false);
  teach_sequence_slider_->setEnabled(false);
  teach_sequence_replay_speed_spin_->setEnabled(false);
  status_label_ = new QLabel(this);
  status_label_->setWordWrap(true);
  overview_root->addWidget(status_label_);
  overview_root->addStretch(1);

  process_->setProcessChannelMode(QProcess::MergedChannels);
  camera_process_->setProcessChannelMode(QProcess::MergedChannels);
  hand_vision_process_->setProcessChannelMode(QProcess::MergedChannels);
  handeye_process_->setProcessChannelMode(QProcess::MergedChannels);
  handeye_publish_process_->setProcessChannelMode(QProcess::MergedChannels);
  readiness_timer_->setInterval(100);

  connect(
    language_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    [this](int index) {
      chinese_ = index == 0;
      retranslateUi();
      if (initialized_) {
        updateReadiness();
      }
    });
  connect(home_button_, &QPushButton::clicked, this, &DemoPanel::goHome);
  connect(
    singularity_escape_button_, &QPushButton::clicked,
    this, &DemoPanel::startSingularityEscape);
  connect(
    singularity_escape_reset_button_, &QPushButton::clicked,
    this, &DemoPanel::resetSingularityEscapeTarget);
  connect(sync_button_, &QPushButton::clicked, this, &DemoPanel::syncPlanningState);
  connect(pick_place_button_, &QPushButton::clicked, this, &DemoPanel::startPickPlace);
  connect(stop_button_, &QPushButton::clicked, this, &DemoPanel::stopDemo);
  connect(xbox_mode_button_, &QPushButton::clicked, this, &DemoPanel::toggleXboxControl);
  connect(camera_button_, &QPushButton::clicked, this, &DemoPanel::toggleCamera);
  connect(camera_tab_button_, &QPushButton::clicked, this, &DemoPanel::toggleCamera);
  connect(hand_vision_button_, &QPushButton::clicked, this, &DemoPanel::toggleHandVision);
  connect(
    hand_vision_large_button_, &QPushButton::clicked,
    this, &DemoPanel::showHandVisionLargeView);
  connect(pulse_approach_button_, &QPushButton::clicked, this, &DemoPanel::startPulseApproach);
  connect(
    hand_vision_topic_refresh_button_, &QPushButton::clicked,
    this, &DemoPanel::refreshHandVisionTopics);
  connect(
    hand_vision_topic_combo_, &QComboBox::currentTextChanged,
    this, &DemoPanel::selectHandVisionTopic);
  connect(handeye_button_, &QPushButton::clicked, this, &DemoPanel::toggleHandeyeCalibration);
  connect(camera_topic_refresh_button_, &QPushButton::clicked, this, &DemoPanel::refreshCameraTopics);
  connect(camera_topic_combo_, &QComboBox::currentTextChanged, this, &DemoPanel::selectCameraTopic);
  const auto select_stream = [this](QCheckBox * checkbox, const QString & topic) {
      connect(checkbox, &QCheckBox::toggled, this, [this, topic](bool enabled) {
        if (!initialized_) {return;}
        if (enabled) {
          subscribeCameraTopic(topic);
        } else if (camera_image_topic_ == topic) {
          if (camera_color_checkbox_->isChecked()) {
            subscribeCameraTopic(QStringLiteral("/camera/color/image_raw"));
          } else if (camera_depth_checkbox_->isChecked()) {
            subscribeCameraTopic(QStringLiteral("/camera/depth/image_raw"));
          } else if (camera_ir_checkbox_->isChecked()) {
            subscribeCameraTopic(QStringLiteral("/camera/ir/image_raw"));
          }
        }
      });
    };
  select_stream(camera_color_checkbox_, QStringLiteral("/camera/color/image_raw"));
  select_stream(camera_depth_checkbox_, QStringLiteral("/camera/depth/image_raw"));
  select_stream(camera_ir_checkbox_, QStringLiteral("/camera/ir/image_raw"));
  connect(
    forbidden_zone_reload_button_, &QPushButton::clicked,
    this, &DemoPanel::reloadForbiddenZones);
  connect(
    forbidden_zone_shape_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, [this](int) {updateForbiddenZoneEditor();});
  connect(
    forbidden_zone_mesh_button_, &QPushButton::clicked,
    this, &DemoPanel::selectForbiddenZoneMesh);
  connect(
    forbidden_zone_apply_button_, &QPushButton::clicked,
    this, &DemoPanel::applyForbiddenZone);
  connect(
    forbidden_zone_remove_button_, &QPushButton::clicked,
    this, &DemoPanel::removeForbiddenZone);
  connect(
    forbidden_zone_group_save_button_, &QPushButton::clicked,
    this, &DemoPanel::saveForbiddenZoneGroup);
  connect(
    forbidden_zone_group_remove_button_, &QPushButton::clicked,
    this, &DemoPanel::removeForbiddenZoneGroup);
  connect(
    forbidden_zone_group_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, [this](int) {
      const QString name = forbidden_zone_group_combo_->currentText();
      forbidden_zone_group_name_edit_->setText(name);
      const auto members = forbidden_zone_user_group_members_.value(name);
      for (int index = 0; index < forbidden_zone_group_members_list_->count(); ++index) {
        auto * item = forbidden_zone_group_members_list_->item(index);
        item->setSelected(members.contains(item->text()));
      }
    });
  connect(
    xbox_help_button_, &QPushButton::toggled, this,
    [this, overview_scroll](bool expanded) {
      xbox_help_container_->setVisible(expanded);
      retranslateUi();
      if (expanded) {
        // The overview page is a scroll area.  The mapping table is below the
        // toggle, so bring it into view immediately after the layout updates.
        QTimer::singleShot(0, overview_scroll, [this, overview_scroll]() {
          if (xbox_help_container_->isVisible()) {
            overview_scroll->ensureWidgetVisible(xbox_help_container_);
          }
        });
      }
    });
  connect(
    teach_refresh_button_, &QPushButton::clicked,
    this, &DemoPanel::refreshTeachActions);
  connect(
    teach_shape_create_button_, &QPushButton::clicked,
    this, &DemoPanel::createShapeAction);
  connect(
    teach_shape_new_button_, &QPushButton::clicked,
    this, &DemoPanel::startNewShapeAction);
  connect(
    teach_action_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, [this](int) {
      loadSelectedShapeParameters();
      if (!initialized_) {return;}
      if (selectedTeachAction().isEmpty()) {
        clearTeachActionSelection();
      } else {
        previewTeachAction();
      }
    });
  connect(
    teach_shape_reachability_button_, &QPushButton::clicked,
    this, &DemoPanel::checkShapeReachability);
  connect(
    teach_shape_image_button_, &QPushButton::clicked,
    this, &DemoPanel::selectShapeImage);
  connect(
    teach_shape_source_edit_, &QLineEdit::editingFinished, this,
    [this]() {
      updateShapePreview();
      retranslateUi();
      if (initialized_ &&
        teach_shape_combo_->currentData().toString() == QStringLiteral("text") &&
        !teach_shape_source_edit_->text().trimmed().isEmpty())
      {
        shape_marker_visible_.store(true);
        configureShapeMarker(true, false, false);
      }
    });
  connect(
    teach_shape_marker_button_, &QPushButton::clicked,
    this, &DemoPanel::toggleShapeMarker);
  connect(
    teach_shape_reset_button_, &QPushButton::clicked,
    this, &DemoPanel::resetShapeMarker);
  connect(
    teach_shape_apply_pose_button_, &QPushButton::clicked,
    this, &DemoPanel::applyShapePose);
  connect(
    teach_trace_width_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, [this](double) {teach_trace_width_timer_->start();});
  connect(
    teach_trace_width_timer_, &QTimer::timeout,
    this, &DemoPanel::setTraceLineWidth);
  connect(
    teach_shape_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, [this](int) {
      const QString drawing = teach_shape_combo_->currentData().toString();
      const bool marker_should_be_visible = drawing != QStringLiteral("freehand") &&
        (drawing != QStringLiteral("image") || !teach_shape_image_path_.isEmpty());
      // Selecting a built-in shape is also an explicit request to edit that shape.
      // Do not carry a hidden state over from freehand mode or a previously hidden
      // shape, otherwise changing the combo appears to make the new shape vanish.
      shape_marker_visible_.store(marker_should_be_visible);
      updateShapePreview();
      retranslateUi();
      if (initialized_) {
        configureShapeMarker(marker_should_be_visible, false, false);
      }
    });
  connect(
    main_tabs_, &QTabWidget::currentChanged, this,
    [this](int) {
      if (initialized_) {
        updateFreePlanningVisibility();
      }
    });
  connect(
    teach_tabs_, &QTabWidget::currentChanged, this,
    [this](int) {
      if (initialized_) {
        updateFreePlanningVisibility();
      }
    });
  connect(teach_copy_button_, &QPushButton::clicked, this, &DemoPanel::copyTeachAction);
  connect(teach_rename_button_, &QPushButton::clicked, this, &DemoPanel::renameTeachAction);
  connect(teach_delete_button_, &QPushButton::clicked, this, &DemoPanel::deleteTeachAction);
  connect(teach_start_button_, &QPushButton::clicked, this, &DemoPanel::startTeaching);
  connect(teach_stop_button_, &QPushButton::clicked, this, &DemoPanel::stopTeaching);
  connect(
    teach_preview_button_, &QPushButton::clicked,
    this, &DemoPanel::previewTeachAction);
  connect(teach_replay_button_, &QPushButton::clicked, this, &DemoPanel::replayTeachAction);
  connect(teach_pause_button_, &QPushButton::clicked, this, &DemoPanel::pauseTeachPreview);
  connect(teach_cancel_button_, &QPushButton::clicked, this, &DemoPanel::cancelTeaching);
  connect(
    teach_sequence_refresh_button_, &QPushButton::clicked,
    this, &DemoPanel::refreshTeachSequences);
  connect(
    teach_sequence_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, [this](int) {loadSelectedTeachSequence();});
  connect(
    teach_sequence_list_, &QListWidget::itemChanged,
    this, [this](QListWidgetItem *) {updateReadiness();});
  connect(
    teach_sequence_add_button_, &QPushButton::clicked,
    this, &DemoPanel::addTeachSequenceAction);
  connect(
    teach_sequence_remove_button_, &QPushButton::clicked,
    this, &DemoPanel::removeTeachSequenceAction);
  connect(
    teach_sequence_up_button_, &QPushButton::clicked,
    this, &DemoPanel::moveTeachSequenceActionUp);
  connect(
    teach_sequence_down_button_, &QPushButton::clicked,
    this, &DemoPanel::moveTeachSequenceActionDown);
  connect(
    teach_sequence_save_button_, &QPushButton::clicked,
    this, &DemoPanel::saveTeachSequence);
  connect(
    teach_sequence_play_button_, &QPushButton::clicked,
    this, &DemoPanel::playTeachSequence);
  connect(
    teach_sequence_replay_button_, &QPushButton::clicked,
    this, &DemoPanel::replayTeachSequence);
  connect(
    teach_sequence_pause_button_, &QPushButton::clicked,
    this, &DemoPanel::pauseTeachPreview);
  connect(
    teach_sequence_cancel_button_, &QPushButton::clicked,
    this, &DemoPanel::cancelTeaching);
  connect(
    teach_slider_, &QSlider::valueChanged, this,
    [this](int value) {
      teach_progress_label_->setText(QStringLiteral("%1%").arg(value / 10.0, 0, 'f', 1));
    });
  connect(teach_slider_, &QSlider::sliderReleased, this, &DemoPanel::previewTeachPosition);
  connect(
    teach_sequence_slider_, &QSlider::valueChanged, this,
    [this](int value) {
      teach_sequence_progress_label_->setText(
        QStringLiteral("%1%").arg(value / 10.0, 0, 'f', 1));
    });
  connect(
    teach_sequence_slider_, &QSlider::sliderReleased,
    this, &DemoPanel::previewTeachSequencePosition);
  connect(readiness_timer_, &QTimer::timeout, this, &DemoPanel::updateReadiness);
  connect(process_, &QProcess::readyReadStandardOutput, this, &DemoPanel::readProcessOutput);
  connect(
    process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
    [this](int exit_code, QProcess::ExitStatus) {processFinished(exit_code);});
  connect(
    camera_process_, &QProcess::readyReadStandardOutput, this,
    [this]() {
      const QString line = lastNonEmptyLine(
        QString::fromLocal8Bit(camera_process_->readAllStandardOutput()));
      if (!line.isEmpty()) {camera_button_->setToolTip(line);}
    });
  connect(
    camera_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
    [this](int exit_code, QProcess::ExitStatus) {
      const bool requested = camera_stop_requested_.exchange(false);
      camera_detected_.store(cameraNodeRunning());
      setStatus(
        (requested || exit_code == 0) ? QStringLiteral("Gemini 2 摄像头已关闭。") :
        QStringLiteral("Gemini 2 摄像头进程已退出，代码 %1；详情见按钮提示。")
        .arg(exit_code),
        (requested || exit_code == 0) ? QStringLiteral("Gemini 2 camera stopped.") :
        QStringLiteral("Gemini 2 camera process exited with code %1; see the button tooltip.")
        .arg(exit_code),
        !requested && exit_code != 0);
      updateReadiness();
    });
  connect(
    camera_process_, &QProcess::errorOccurred, this,
    [this](QProcess::ProcessError error) {
      if (error != QProcess::FailedToStart) {return;}
      camera_stop_requested_.store(false);
      setStatus(
        QStringLiteral("无法启动摄像头：%1").arg(camera_process_->errorString()),
        QStringLiteral("Could not start camera: %1").arg(camera_process_->errorString()), true);
      updateReadiness();
    });
  connect(
    hand_vision_process_, &QProcess::readyReadStandardOutput, this,
    [this]() {
      const QString line = lastNonEmptyLine(
        QString::fromLocal8Bit(hand_vision_process_->readAllStandardOutput()));
      if (!line.isEmpty()) {hand_vision_status_label_->setToolTip(line);}
    });
  connect(
    hand_vision_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
    [this](int exit_code, QProcess::ExitStatus) {
      hand_vision_status_label_->setText(exit_code == 0 ?
        localized(QStringLiteral("手部视觉已停止。"), QStringLiteral("Hand vision stopped.")) :
        localized(QStringLiteral("手部视觉已退出，代码 %1。"), QStringLiteral("Hand vision exited with code %1.")).arg(exit_code));
      hand_vision_button_->setText(localized(QStringLiteral("启动手部视觉"), QStringLiteral("Start Hand Vision")));
      hand_vision_button_->setEnabled(true);
      last_hand_vision_image_ms_.store(0);
      last_pulse_point_ms_.store(0);
      last_piper_pulse_point_ms_.store(0);
      updateReadiness();
    });
  connect(
    handeye_process_, &QProcess::readyReadStandardOutput, this,
    [this]() {
      const QString line = lastNonEmptyLine(
        QString::fromLocal8Bit(handeye_process_->readAllStandardOutput()));
      if (!line.isEmpty()) {handeye_status_label_->setToolTip(line);}
    });
  connect(
    handeye_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
    [this](int exit_code, QProcess::ExitStatus) {
      const bool moveit_backend = handeyeUsesMoveItCalibration();
      if (moveit_backend) {removeMoveItCalibrationDisplay();}
      const bool saved = QFileInfo::exists(handeyeCalibrationFile());
      handeye_status_label_->setText(moveit_backend ? localized(
        QStringLiteral("MoveIt Calibration 已停止；其结果需在插件中另行保存和加载。"),
        QStringLiteral("MoveIt Calibration stopped; save and load its result from the plugin.")) :
        saved ? localized(
        QStringLiteral("标定结果已保存，正在发布标定 TF。"),
        QStringLiteral("Calibration saved; publishing the calibration TF.")) :
        exit_code == 0 ? localized(
        QStringLiteral("标定窗口已关闭，但尚未发现保存结果。"),
        QStringLiteral("Calibration window closed, but no saved result was found.")) :
        localized(QStringLiteral("手眼标定进程已退出，代码 %1。"),
        QStringLiteral("Hand-eye calibration exited with code %1.")).arg(exit_code));
      handeye_button_->setText(localized(QStringLiteral("启动手眼标定"), QStringLiteral("Start Hand-eye Calibration")));
      handeye_button_->setEnabled(true);
      handeye_backend_combo_->setEnabled(true);
      handeye_mode_combo_->setEnabled(true);
      handeye_target_combo_->setEnabled(!moveit_backend);
      handeye_name_edit_->setEnabled(!moveit_backend);
      if (!moveit_backend && saved) {restartHandeyePublisher();}
    });
  connect(
    handeye_publish_process_, &QProcess::readyReadStandardOutput, this,
    [this]() {
      const QString line = lastNonEmptyLine(
        QString::fromLocal8Bit(handeye_publish_process_->readAllStandardOutput()));
      if (!line.isEmpty()) {handeye_status_label_->setToolTip(line);}
    });
  connect(
    handeye_publish_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
    [this](int exit_code, QProcess::ExitStatus) {
      if (exit_code != 0 && handeye_process_->state() == QProcess::NotRunning) {
        handeye_status_label_->setText(localized(
          QStringLiteral("标定 TF 发布失败（退出码 %1）。").arg(exit_code),
          QStringLiteral("Calibration TF publisher failed (exit code %1).").arg(exit_code)));
      }
    });

  setStatus(
    QStringLiteral("面板尚未连接 ROS。"),
    QStringLiteral("The panel is not connected to ROS yet."));
  setTeachStatus(
    QStringLiteral("等待示教服务和动作列表。"),
    QStringLiteral("Waiting for teaching services and action list."));
  retranslateUi();
}

void DemoPanel::setActiveRobot(const QString & robot)
{
  if (robot != QStringLiteral("rebotarm") && robot != QStringLiteral("piperh")) {return;}
  active_robot_ = robot;
  const bool piper = robot == QStringLiteral("piperh");
  configureSingularityEscapeControls();
  const QString name = piper ? QStringLiteral("piperh_camera") :
    QStringLiteral("rebotarm_camera");
  const QString eye_in_hand_name = name + QStringLiteral("_eye_in_hand");
  const QStringList known_names{
    QStringLiteral("rebotarm_camera"), QStringLiteral("rebotarm_camera_eye_in_hand"),
    QStringLiteral("piperh_camera"), QStringLiteral("piperh_camera_eye_in_hand")};
  if (known_names.contains(handeye_name_edit_->text().trimmed())) {
    QString selected_name = handeyeEyeInHand() ? eye_in_hand_name : name;
    const QString alternative_name = handeyeEyeInHand() ? name : eye_in_hand_name;
    const QString calibration_directory =
      qEnvironmentVariable("EASY_HANDEYE2_CALIBRATIONS_DIRECTORY", QDir::homePath() + QStringLiteral("/.ros2/easy_handeye2/calibrations")) + QStringLiteral("/");
    if (!QFileInfo::exists(calibration_directory + selected_name + QStringLiteral(".calib")) &&
      QFileInfo::exists(calibration_directory + alternative_name + QStringLiteral(".calib")))
    {
      const QSignalBlocker blocker(handeye_mode_combo_);
      const QString alternative_mode = handeyeEyeInHand() ?
        QStringLiteral("eye_on_base") : QStringLiteral("eye_in_hand");
      const int alternative_index = handeye_mode_combo_->findData(alternative_mode);
      if (alternative_index >= 0) {
        handeye_mode_combo_->setCurrentIndex(alternative_index);
        selected_name = alternative_name;
      }
    }
    handeye_name_edit_->setText(selected_name);
  }
  const QStringList known_bases{
    QStringLiteral("base_link"), QStringLiteral("piperh/base_link")};
  if (known_bases.contains(handeye_robot_base_edit_->text().trimmed())) {
    handeye_robot_base_edit_->setText(
      piper ? QStringLiteral("piperh/base_link") : QStringLiteral("base_link"));
  }
  const QStringList known_effectors{
    QStringLiteral("gripper_tcp"), QStringLiteral("piperh/Link5"),
    QStringLiteral("piperh/Link6")};
  if (known_effectors.contains(handeye_robot_effector_edit_->text().trimmed())) {
    handeye_robot_effector_edit_->setText(
      piper ? QStringLiteral("piperh/Link5") : QStringLiteral("gripper_tcp"));
  }
  forbidden_zone_status_seen_ = false;
  forbidden_zone_config_path_.clear();
  forbidden_zone_user_config_path_.clear();
  forbidden_zone_loaded_areas_.clear();
  forbidden_zone_defined_areas_.clear();
  forbidden_zone_enabled_areas_.clear();
  forbidden_zone_active_groups_.clear();
  forbidden_zone_defined_groups_.clear();
  forbidden_zone_user_areas_.clear();
  forbidden_zone_draggable_areas_.clear();
  forbidden_zone_user_groups_.clear();
  forbidden_zone_user_group_members_.clear();
  last_feedback_ms_.store(0);
  xbox_state_known_.store(false);
  xbox_armed_.store(true);
  validity_state_->valid.store(false);
  validity_state_->response_ms.store(0);
  teach_state_.store(0);
  teach_hardware_allowed_.store(false);
  if (initialized_) {
    // The pulse target transformer needs the saved camera-to-robot TF before it
    // can publish /piperh/pulse/target.  Robot selection also changes the
    // default calibration name, so replace any publisher for the previous arm.
    if (handeye_publish_process_->state() != QProcess::NotRunning) {
      handeye_publish_process_->terminate();
      handeye_publish_process_->waitForFinished(1000);
    }
    configureRobotInterfaces();
    configureForbiddenZoneInterfaces();
    ensureShapeMarkerDisplay();
    ensureForbiddenZoneMarkerDisplay();
    configureRobotDisplays();
    teach_action_combo_->clear();
    teach_sequence_combo_->clear();
    teach_sequence_list_->clear();
    QTimer::singleShot(250, this, &DemoPanel::refreshTeachActions);
    QTimer::singleShot(350, this, &DemoPanel::refreshTeachSequences);
    QTimer::singleShot(0, this, &DemoPanel::restartHandeyePublisher);
  }
  retranslateUi();
  updateReadiness();
}

DemoPanel::~DemoPanel()
{
  restoreMotionPlanningPanel();
  if (camera_process_->state() != QProcess::NotRunning) {
    camera_process_->terminate();
    if (!camera_process_->waitForFinished(3000)) {
      camera_process_->kill();
      camera_process_->waitForFinished(1000);
    }
  }
  for (auto * auxiliary : {hand_vision_process_, handeye_process_, handeye_publish_process_}) {
    if (auxiliary->state() != QProcess::NotRunning) {
      auxiliary->terminate();
      if (!auxiliary->waitForFinished(1000)) {auxiliary->kill(); auxiliary->waitForFinished(500);}
    }
  }
  if (process_->state() != QProcess::NotRunning) {
    process_->terminate();
    if (!process_->waitForFinished(1000)) {
      process_->kill();
      process_->waitForFinished(1000);
    }
  }
}

void DemoPanel::integrateMotionPlanningPanel()
{
  if (!integrate_motion_planning_) {
    return;
  }
  if (embedded_motion_planning_widget_) {
    return;
  }
  auto * dock = motionPlanningDockWidget();
  if (!dock || !dock->widget() || dock->widget() == this) {
    return;
  }

  auto * content = dock->widget();
  // A parent workbench may already contain this DemoPanel. Never detach that
  // whole workbench merely because a misplaced MotionPlanning widget is one
  // of its descendants; doing so creates a cyclic layout and leaves this page
  // on the loading placeholder forever.
  if (content->isAncestorOf(this)) {
    return;
  }
  auto * placeholder = new QWidget(dock);
  dock->setWidget(placeholder);
  content->setParent(planning_page_);
  planning_placeholder_->hide();
  planning_layout_->addWidget(content, 1);
  // QWidget::setParent() hides an already-visible widget.  MotionPlanning is
  // usually created after RViz has shown the panel, so explicitly reveal it
  // after moving it into our planning tab.
  content->show();
  dock->hide();
  dock->toggleViewAction()->setVisible(false);
  motion_planning_dock_ = dock;
  embedded_motion_planning_widget_ = content;
  motion_planning_dock_placeholder_ = placeholder;
  translateMotionPlanningUi(chinese_);
  setMotionPlanningIntegratedMode(true);
  last_motion_planning_result_.clear();
}

void DemoPanel::restoreMotionPlanningPanel()
{
  if (!motion_planning_dock_ || !embedded_motion_planning_widget_) {
    return;
  }
  setMotionPlanningIntegratedMode(false);
  planning_layout_->removeWidget(embedded_motion_planning_widget_);
  motion_planning_dock_->setWidget(embedded_motion_planning_widget_);
  embedded_motion_planning_widget_->show();
  motion_planning_dock_->toggleViewAction()->setVisible(true);
  motion_planning_dock_->show();
  embedded_motion_planning_widget_.clear();
  motion_planning_dock_placeholder_.clear();
  motion_planning_dock_.clear();
}

void DemoPanel::setMoveItGoalMarkerVisible(bool visible)
{
  if (!getDisplayContext()) {return;}
  auto * root = getDisplayContext()->getRootDisplayGroup();
  if (!root) {return;}
  for (int index = 0; index < root->numDisplays(); ++index) {
    auto * display = root->getDisplayAt(index);
    if (!display || display->getClassId() != QStringLiteral(
        "moveit_rviz_plugin/MotionPlanning"))
    {
      continue;
    }
    auto * query_goal = display->subProp(
      QStringLiteral("Planning Request"))->subProp(QStringLiteral("Query Goal State"));
    if (query_goal->getValue().toBool() != visible) {
      query_goal->setValue(visible);
    }
  }
}

void DemoPanel::configureRobotDisplays()
{
  if (!getDisplayContext()) {return;}
  auto * root = getDisplayContext()->getRootDisplayGroup();
  if (!root) {return;}
  const bool preview_active = teach_preview_active_.load();
  for (int index = 0; index < root->numDisplays(); ++index) {
    auto * display = root->getDisplayAt(index);
    if (!display) {continue;}
    const QString class_id = display->getClassId();
    if (class_id == QStringLiteral("moveit_rviz_plugin/PlanningScene")) {
      // MotionPlanning already renders the monitored scene.  A second scene
      // display renders another full robot at the same or a stale pose.
      if (display->isEnabled()) {display->setEnabled(false);}
    } else if (class_id == QStringLiteral("moveit_rviz_plugin/RobotState") &&
      display->subProp(QStringLiteral("Robot State Topic"))->getValue().toString().contains(
        QStringLiteral("display_robot_state")))
    {
      display->subProp(QStringLiteral("Robot State Topic"))->setValue(
        active_robot_ == QStringLiteral("piperh") ?
        QStringLiteral("/piperh/display_robot_state") :
        QStringLiteral("/rebotarm/display_robot_state"));
      display->subProp(QStringLiteral("Robot Description"))->setValue(
        active_robot_ == QStringLiteral("piperh") ?
        QStringLiteral("piperh_robot_description") : QStringLiteral("robot_description"));
      // This display is dedicated to teaching animation and otherwise retains
      // its last message indefinitely, which looks like an unexplained grey arm.
      if (display->isEnabled() != preview_active) {
        display->setEnabled(preview_active);
      }
    } else if (class_id == QStringLiteral("moveit_rviz_plugin/MotionPlanning")) {
      auto * show_trail = display->subProp(QStringLiteral("Planned Path"))->subProp(
        QStringLiteral("Show Trail"));
      if (show_trail->getValue().toBool()) {show_trail->setValue(false);}
      auto * show_planned_robot = display->subProp(QStringLiteral("Planned Path"))->subProp(
        QStringLiteral("Show Robot Visual"));
      // MoveIt retains the last Planned Path robot even after leaving the free
      // planning page, and an empty DisplayTrajectory does not reliably remove
      // that cached frame.  Only panel-launched demos need this layer for their
      // pre-execution preview.  Free planning already has the live scene robot
      // and complete orange goal, while teaching uses its dedicated RobotState.
      const bool planned_robot_visible =
        process_->state() != QProcess::NotRunning && !preview_active;
      if (show_planned_robot->getValue().toBool() != planned_robot_visible) {
        show_planned_robot->setValue(planned_robot_visible);
      }
      auto * goal_alpha = display->subProp(QStringLiteral("Planning Request"))->subProp(
        QStringLiteral("Goal State Alpha"));
      if (goal_alpha->getValue().toDouble() != 0.5) {goal_alpha->setValue(0.5);}
    }
  }
}

void DemoPanel::updateFreePlanningVisibility()
{
  const bool visible = main_tabs_->currentIndex() == 1 &&
    teach_tabs_->currentWidget() == planning_page_;
  if (free_planning_visible_ && !visible && !teach_preview_active_.load()) {
    clearPlannedPathDisplay();
  }
  free_planning_visible_ = visible;
  setMoveItGoalMarkerVisible(visible);
  configureRobotDisplays();
}

void DemoPanel::updateMotionPlanningResult()
{
  if (!embedded_motion_planning_widget_) {return;}
  auto * result = embedded_motion_planning_widget_->findChild<QLabel *>(
    QStringLiteral("result_label"));
  if (!result) {return;}
  const QString normalized = result->text().trimmed();
  if (normalized == last_motion_planning_result_) {return;}
  last_motion_planning_result_ = normalized;
  const bool failed = normalized == QStringLiteral("Failed") ||
    normalized == QStringLiteral("失败");
  if (failed) {
    const QString zones = forbidden_zone_enabled_areas_.isEmpty() ?
      localized(QStringLiteral("当前没有已启用禁区"), QStringLiteral("no enabled forbidden zones")) :
      localized(QStringLiteral("已启用禁区：%1").arg(forbidden_zone_enabled_areas_.join(QStringLiteral(", "))),
      QStringLiteral("enabled zones: %1").arg(forbidden_zone_enabled_areas_.join(QStringLiteral(", "))));
    planning_error_label_->setText(localized(
      QStringLiteral("自由规划失败：MoveIt 未找到满足碰撞约束的路径。可能原因：轨迹与禁区相交（%1）、起点/目标碰撞、关节超限或无 IK 解。请调整起点/目标或检查禁区配置。")
      .arg(zones),
      QStringLiteral("Free planning failed: MoveIt found no collision-free path. Possible causes: the path intersects forbidden zones (%1), the start/goal is in collision, joint limits, or no IK solution. Adjust the start/goal or inspect forbidden-zone configuration.")
      .arg(zones)));
    planning_error_label_->show();
  } else if (normalized == QStringLiteral("Planning...") || normalized == QStringLiteral("正在规划…") ||
    normalized == QStringLiteral("Executed") || normalized == QStringLiteral("已执行") ||
    normalized == QStringLiteral("Stopped") || normalized == QStringLiteral("已停止")) {
    planning_error_label_->clear();
    planning_error_label_->hide();
  }
  if (!teach_preview_active_.load() &&
    (normalized == QStringLiteral("Executed") ||
    normalized == QStringLiteral("Failed") ||
    normalized == QStringLiteral("Stopped") ||
    normalized == QStringLiteral("已执行") ||
    normalized == QStringLiteral("失败") ||
    normalized == QStringLiteral("已停止")))
  {
    clearPlannedPathDisplay();
  }
}

void DemoPanel::clearPlannedPathDisplay()
{
  if (!display_trajectory_pub_) {
    return;
  }
  ++free_planning_display_generation_;
  display_trajectory_pub_->publish(moveit_msgs::msg::DisplayTrajectory());
  configureRobotDisplays();
}

void DemoPanel::ensureShapeMarkerDisplay()
{
  if (!getDisplayContext()) {return;}
  auto * root = getDisplayContext()->getRootDisplayGroup();
  if (!root) {return;}
  rviz_common::Display * marker_display = nullptr;
  for (int index = 0; index < root->numDisplays(); ++index) {
    auto * display = root->getDisplayAt(index);
    if (display && display->getClassId() == QStringLiteral(
        "rviz_default_plugins/InteractiveMarkers") &&
      display->getName() == QStringLiteral("绘图形状空间定位"))
    {
      marker_display = display;
      break;
    }
  }
  if (!marker_display) {
    auto * manager = dynamic_cast<rviz_common::VisualizationManager *>(
      getDisplayContext());
    if (!manager) {return;}
    marker_display = manager->createDisplay(
      QStringLiteral("rviz_default_plugins/InteractiveMarkers"),
      QStringLiteral("绘图形状空间定位"), true);
  }
  if (!marker_display) {return;}
  marker_display->subProp(
    QStringLiteral("Interactive Markers Namespace"))->setValue(
    active_robot_ == QStringLiteral("piperh") ?
    QStringLiteral("/piperh/teach/shape_marker") :
    QStringLiteral("/rebotarm/teach/shape_marker"));
  marker_display->subProp(QStringLiteral("Show Descriptions"))->setValue(true);
  marker_display->subProp(QStringLiteral("Enable Transparency"))->setValue(true);
  marker_display->setEnabled(true);
}

void DemoPanel::ensureForbiddenZoneMarkerDisplay()
{
  if (!getDisplayContext()) {return;}
  auto * root = getDisplayContext()->getRootDisplayGroup();
  if (!root) {return;}
  rviz_common::Display * marker_display = nullptr;
  for (int index = 0; index < root->numDisplays(); ++index) {
    auto * display = root->getDisplayAt(index);
    if (display && display->getClassId() == QStringLiteral(
        "rviz_default_plugins/InteractiveMarkers") &&
      display->getName() == QStringLiteral("禁区拖动手柄"))
    {
      marker_display = display;
      break;
    }
  }
  if (!marker_display) {
    auto * manager = dynamic_cast<rviz_common::VisualizationManager *>(
      getDisplayContext());
    if (!manager) {return;}
    marker_display = manager->createDisplay(
      QStringLiteral("rviz_default_plugins/InteractiveMarkers"),
      QStringLiteral("禁区拖动手柄"), true);
  }
  if (!marker_display) {return;}
  marker_display->subProp(
    QStringLiteral("Interactive Markers Namespace"))->setValue(
    active_robot_ == QStringLiteral("piperh") ?
    QStringLiteral("/piperh/forbidden_zones/interactive") :
    QStringLiteral("/rebotarm/forbidden_zones/interactive"));
  marker_display->subProp(QStringLiteral("Show Descriptions"))->setValue(true);
  marker_display->subProp(QStringLiteral("Enable Transparency"))->setValue(true);
  marker_display->setEnabled(true);
}

void DemoPanel::ensurePulseMarkerDisplay()
{
  if (!getDisplayContext()) {return;}
  auto * root = getDisplayContext()->getRootDisplayGroup();
  if (!root) {return;}
  rviz_common::Display * marker_display = nullptr;
  for (int index = 0; index < root->numDisplays(); ++index) {
    auto * display = root->getDisplayAt(index);
    if (display && display->getClassId() == QStringLiteral("rviz_default_plugins/Marker") &&
      display->getName() == QStringLiteral("把脉区域"))
    {
      marker_display = display;
      break;
    }
  }
  if (!marker_display) {
    auto * manager = dynamic_cast<rviz_common::VisualizationManager *>(getDisplayContext());
    if (!manager) {return;}
    marker_display = manager->createDisplay(
      QStringLiteral("rviz_default_plugins/Marker"), QStringLiteral("把脉区域"), true);
  }
  if (!marker_display) {return;}
  marker_display->subProp(QStringLiteral("Topic"))->setValue(
    active_robot_ == QStringLiteral("piperh") ?
    QStringLiteral("/piperh/pulse/target_marker") :
    QStringLiteral("/meridian_hand_vision/pulse_marker"));
  marker_display->setEnabled(true);

  rviz_common::Display * axis_display = nullptr;
  for (int index = 0; index < root->numDisplays(); ++index) {
    auto * display = root->getDisplayAt(index);
    if (display &&
      display->getClassId() == QStringLiteral("rviz_default_plugins/MarkerArray") &&
      display->getName() == QStringLiteral("Piper-H 三轴对齐"))
    {
      axis_display = display;
      break;
    }
  }
  if (!axis_display) {
    auto * manager = dynamic_cast<rviz_common::VisualizationManager *>(getDisplayContext());
    if (!manager) {return;}
    axis_display = manager->createDisplay(
      QStringLiteral("rviz_default_plugins/MarkerArray"),
      QStringLiteral("Piper-H 三轴对齐"), true);
  }
  if (!axis_display) {return;}
  axis_display->subProp(QStringLiteral("Topic"))->setValue(
    QStringLiteral("/piperh/pulse/axis_alignment_markers"));
  axis_display->setEnabled(true);
}

void DemoPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus(
      QStringLiteral("无法取得 RViz ROS 节点。"),
      QStringLiteral("Could not access the RViz ROS node."), true);
    return;
  }

  node_ = abstraction->get_raw_node();
  integrate_motion_planning_ = node_->has_parameter("rebot_demo.integrate_motion_planning") ?
    node_->get_parameter("rebot_demo.integrate_motion_planning").as_bool() :
    node_->declare_parameter<bool>("rebot_demo.integrate_motion_planning", true);
  model_ = detectedModel();
  configureSingularityEscapeControls();
  initialized_ = true;
  ensureShapeMarkerDisplay();
  ensureForbiddenZoneMarkerDisplay();
  ensurePulseMarkerDisplay();
  retranslateUi();

  subscribeCameraTopic(camera_image_topic_);
  refreshCameraTopics();
  refreshHandVisionTopics();
  hand_vision_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/meridian_hand_vision/annotated_image", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::SharedPtr message) {
      last_hand_vision_image_ms_.store(steadyMilliseconds());
      const QImage image = imageMessageToQImage(*message);
      const QString encoding = QString::fromStdString(message->encoding);
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(this, [self, image, encoding]() {
        if (!self) {return;}
        if (image.isNull()) {
          self->hand_vision_status_label_->setText(self->localized(
            QStringLiteral("收到手部视觉图像，但编码不支持：%1").arg(encoding),
            QStringLiteral("Hand vision image received, but encoding is unsupported: %1").arg(encoding)));
          return;
        }
        self->showHandVisionImage(image);
      }, Qt::QueuedConnection);
    });
  pulse_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/meridian_hand_vision/pulse_point", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(pulse_point_mutex_);
        latest_pulse_point_ = message;
      }
      last_pulse_point_ms_.store(steadyMilliseconds());
      const QString frame = QString::fromStdString(message->header.frame_id);
      const double x = message->point.x;
      const double y = message->point.y;
      const double z = message->point.z;
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(this, [self, frame, x, y, z]() {
        if (!self) {return;}
        self->pulse_region_label_->setText(self->localized(
          QStringLiteral("把脉区域已稳定：%1  X=%2, Y=%3, Z=%4 m")
          .arg(frame).arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(z, 0, 'f', 4),
          QStringLiteral("Stable pulse region: %1  X=%2, Y=%3, Z=%4 m")
          .arg(frame).arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(z, 0, 'f', 4)));
        self->pulse_region_label_->setStyleSheet(QStringLiteral("color: #3a9d23;"));
        self->updateReadiness();
      }, Qt::QueuedConnection);
    });
  pulse_base_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/rebotarm/pulse/target", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
      if (active_robot_ != QStringLiteral("rebotarm")) {return;}
      const QString frame = QString::fromStdString(message->header.frame_id);
      const double x = message->point.x;
      const double y = message->point.y;
      const double z = message->point.z;
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(this, [self, frame, x, y, z]() {
        if (!self) {return;}
        self->pulse_region_label_->setText(self->localized(
          QStringLiteral("把脉区域（机械臂坐标）：%1  X=%2, Y=%3, Z=%4 m")
          .arg(frame).arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(z, 0, 'f', 4),
          QStringLiteral("Pulse region (robot frame): %1  X=%2, Y=%3, Z=%4 m")
          .arg(frame).arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(z, 0, 'f', 4)));
      }, Qt::QueuedConnection);
    });
  piper_pulse_base_point_sub_ =
    node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/piperh/pulse/target", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(pulse_point_mutex_);
        latest_piper_pulse_point_ = message;
      }
      const auto acquisition_age_ms = std::max<std::int64_t>(
        0, (node_->now() - rclcpp::Time(message->header.stamp)).nanoseconds() / 1000000);
      last_piper_pulse_point_ms_.store(steadyMilliseconds() - acquisition_age_ms);
      if (active_robot_ != QStringLiteral("piperh")) {return;}
      const QString frame = QString::fromStdString(message->header.frame_id);
      const double x = message->point.x;
      const double y = message->point.y;
      const double z = message->point.z;
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(this, [self, frame, x, y, z]() {
        if (!self || self->active_robot_ != QStringLiteral("piperh")) {return;}
        self->pulse_region_label_->setText(self->localized(
          QStringLiteral("把脉区域（Piper-H 规划坐标）：%1  X=%2, Y=%3, Z=%4 m")
          .arg(frame).arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(z, 0, 'f', 4),
          QStringLiteral("Pulse region (Piper-H planning frame): %1  X=%2, Y=%3, Z=%4 m")
          .arg(frame).arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(z, 0, 'f', 4)));
        self->pulse_region_label_->setStyleSheet(QStringLiteral("color: #3a9d23;"));
        self->updateReadiness();
      }, Qt::QueuedConnection);
    });

  teach_status_handler_ =
    [this](const std_msgs::msg::String::SharedPtr message) {
      const int teach_state = parseTeachState(message->data);
      teach_state_.store(teach_state);
      const auto document = QJsonDocument::fromJson(
        QByteArray::fromStdString(message->data));
      if (document.isObject()) {
        const auto object = document.object();
        teach_hardware_allowed_.store(
          object.value(QStringLiteral("allow_hardware")).toBool(false));
        teach_offline_preview_.store(
          object.value(QStringLiteral("preview_only")).toBool(false));
        teach_normal_feedback_ready_.store(
          object.value(QStringLiteral("normal_feedback_ready")).toBool(false));
        teach_passive_recording_ready_.store(
          object.value(QStringLiteral("passive_recording_ready")).toBool(false));
        const QString state_message = object.value(QStringLiteral("message")).toString();
        const bool active = object.value(QStringLiteral("preview_active")).toBool(false);
        const bool paused = object.value(QStringLiteral("preview_paused")).toBool(false);
        const auto generation = static_cast<std::int64_t>(
          object.value(QStringLiteral("preview_generation")).toDouble(0.0));
        const double trajectory_duration =
        object.value(QStringLiteral("trajectory_duration_sec")).toDouble(0.0);
        const auto duration_ms = static_cast<std::int64_t>(std::llround(
            object.value(QStringLiteral("preview_duration_sec")).toDouble(
              trajectory_duration) * 1000.0));
        const auto offset_milli = static_cast<std::int64_t>(std::llround(
            object.value(QStringLiteral("preview_progress_offset")).toDouble(0.0) * 1000.0));
        const auto previous_generation = teach_preview_generation_.exchange(generation);
        teach_preview_duration_ms_.store(duration_ms);
        teach_preview_offset_milli_.store(std::clamp<std::int64_t>(offset_milli, 0, 1000));
        teach_preview_active_.store(active);
        teach_preview_paused_.store(paused);
        if (active && !paused &&
        (generation != previous_generation || teach_preview_started_ms_.load() == 0))
        {
          teach_preview_started_ms_.store(steadyMilliseconds());
        } else if (!active || paused) {
          teach_preview_started_ms_.store(0);
        }
        const bool replay_active = object.value(QStringLiteral("replay_active")).toBool(false);
        shape_marker_visible_.store(
          object.value(QStringLiteral("shape_marker_visible")).toBool(true));
        const auto reachability = object.value(
          QStringLiteral("shape_reachability")).toObject();
        const bool reachability_available = reachability.value(
          QStringLiteral("available")).toBool(false);
        const bool reachability_feasible = reachability.value(
          QStringLiteral("feasible")).toBool(false);
        const int reachability_sampled = reachability.value(
          QStringLiteral("sampled_points")).toInt(0);
        const int reachability_green = reachability.value(
          QStringLiteral("reachable_points")).toInt(0);
        const int reachability_yellow = reachability.value(
          QStringLiteral("near_limit_points")).toInt(0);
        const int reachability_purple = reachability.value(
          QStringLiteral("collision_points")).toInt(0);
        const int reachability_red = reachability.value(
          QStringLiteral("no_ik_points")).toInt(0);
        QPointer<DemoPanel> reachability_self(this);
        QMetaObject::invokeMethod(
          this, [reachability_self, reachability_available, reachability_feasible,
            reachability_sampled, reachability_green, reachability_yellow,
            reachability_purple, reachability_red]() {
            if (!reachability_self) {return;}
            reachability_self->shape_reachability_available_ = reachability_available;
            reachability_self->shape_reachability_feasible_ = reachability_feasible;
            if (!reachability_available) {
              reachability_self->teach_shape_reachability_label_->setText(
                reachability_self->localized(
                  QStringLiteral("尚未预检当前笔尖轨迹。"),
                  QStringLiteral("Current pen-tip path has not been checked.")));
              reachability_self->teach_shape_reachability_label_->setStyleSheet(
                QStringLiteral("color: #d08020;"));
            } else {
              reachability_self->teach_shape_reachability_label_->setText(
                reachability_self->localized(
                  QStringLiteral("预检 %1 点：绿色可达 %2，黄色近限位 %3，紫色碰撞 %4，红色无 IK %5。")
                  .arg(reachability_sampled).arg(reachability_green)
                  .arg(reachability_yellow).arg(reachability_purple)
                  .arg(reachability_red),
                  QStringLiteral("Checked %1 points: green reachable %2, yellow near-limit %3, "
                    "purple collision %4, red no-IK %5.")
                  .arg(reachability_sampled).arg(reachability_green)
                  .arg(reachability_yellow).arg(reachability_purple)
                  .arg(reachability_red)));
              reachability_self->teach_shape_reachability_label_->setStyleSheet(
                reachability_feasible ? QStringLiteral("color: #3a9d23; font-weight: bold;") :
                QStringLiteral("color: #c43b32; font-weight: bold;"));
            }
            reachability_self->updateReadiness();
          }, Qt::QueuedConnection);
        const double trace_width_mm = 1000.0 * object.value(
          QStringLiteral("tcp_trace_line_width_m")).toDouble(0.004);
        QPointer<DemoPanel> trace_self(this);
        QMetaObject::invokeMethod(
          this, [trace_self, trace_width_mm]() {
            if (!trace_self) {return;}
            if (trace_self->teach_trace_width_spin_->hasFocus() ||
              trace_self->teach_trace_width_timer_->isActive() ||
              trace_self->trace_width_request_pending_.load())
            {
              return;
            }
            const QSignalBlocker blocker(trace_self->teach_trace_width_spin_);
            trace_self->teach_trace_width_spin_->setValue(trace_width_mm);
          }, Qt::QueuedConnection);
        const auto marker_pose = object.value(QStringLiteral("shape_marker_pose")).toObject();
        const auto marker_position = marker_pose.value(QStringLiteral("position")).toArray();
        const auto marker_orientation = marker_pose.value(QStringLiteral("orientation")).toArray();
        if (marker_position.size() == 3 && marker_orientation.size() == 4) {
          const double x = marker_position[0].toDouble();
          const double y = marker_position[1].toDouble();
          const double z = marker_position[2].toDouble();
          const double qx = marker_orientation[0].toDouble();
          const double qy = marker_orientation[1].toDouble();
          const double qz = marker_orientation[2].toDouble();
          const double qw = marker_orientation[3].toDouble();
          const double width = object.value(
            QStringLiteral("shape_marker_width")).toDouble(0.06);
          const double height = object.value(
            QStringLiteral("shape_marker_height")).toDouble(0.08);
          const double pen_length = object.value(
            QStringLiteral("shape_pen_length_m")).toDouble(0.0);
          const double pen_lift = object.value(
            QStringLiteral("shape_pen_lift_m")).toDouble(0.012);
          QPointer<DemoPanel> shape_self(this);
          QMetaObject::invokeMethod(
            this, [shape_self, x, y, z, qx, qy, qz, qw, width, height,
              pen_length, pen_lift]() {
              if (shape_self) {
                shape_self->updateShapePoseControls(
                  x, y, z, qx, qy, qz, qw, width, height, pen_length, pen_lift);
              }
            }, Qt::QueuedConnection);
        }
        {
          std::lock_guard<std::mutex> lock(teach_preview_source_mutex_);
          teach_preview_sequence_name_ =
            object.value(QStringLiteral("preview_sequence_name")).toString().toStdString();
        }
        const double replay_progress = std::clamp(
          object.value(QStringLiteral("replay_progress")).toDouble(0.0), 0.0, 1.0);
        const QString replay_action =
          object.value(QStringLiteral("replay_action_name")).toString();
        const QString replay_sequence =
          object.value(QStringLiteral("replay_sequence_name")).toString();
        const int replay_index =
          object.value(QStringLiteral("replay_action_index")).toInt(0);
        const int replay_total =
          object.value(QStringLiteral("replay_total_actions")).toInt(0);
        QPointer<DemoPanel> self(this);
        QMetaObject::invokeMethod(
          this,
          [self, replay_action, replay_sequence, replay_index, replay_total,
            replay_progress, replay_active, teach_state, state_message]() {
            if (!self) {return;}
            self->updateHardwareReplayProgress(
              replay_action, replay_sequence, replay_index, replay_total,
              replay_progress, replay_active, teach_state, state_message);
            if (self->return_home_after_teach_cancel_pending_.load()) {
              if (!replay_active &&
                (teach_state == TEACH_READY || teach_state == TEACH_IDLE))
              {
                if (self->return_home_after_teach_cancel_pending_.exchange(false)) {
                  self->startDemo(
                    QStringLiteral("go_home"),
                    self->operationTitle(QStringLiteral("go_home"), self->chinese_),
                    false);
                }
              } else if (teach_state == TEACH_FAULT || teach_state == TEACH_LOCKED) {
                self->return_home_after_teach_cancel_pending_.store(false);
                self->setTeachStatus(
                  QStringLiteral("真机回放已停止；当前状态不安全，已跳过自动回原点。"),
                  QStringLiteral(
                    "Robot replay stopped; automatic return home was skipped because "
                    "the current state is unsafe."),
                  true);
              }
            }
          },
          Qt::QueuedConnection);
      }
    };
  auto_handeye_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/rebotarm/handeye/auto_status",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](const std_msgs::msg::String::SharedPtr message) {
      const auto document = QJsonDocument::fromJson(
        QByteArray::fromStdString(message->data));
      if (!document.isObject()) {return;}
      const auto object = document.object();
      const QString state = object.value(QStringLiteral("state")).toString();
      const QString detail = object.value(QStringLiteral("message")).toString();
      const QString calibration_type =
        object.value(QStringLiteral("calibration_type")).toString();
      const int sampled = object.value(QStringLiteral("sampled_poses")).toInt(0);
      const int required = object.value(QStringLiteral("required_poses")).toInt(12);
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(
        this, [self, state, detail, calibration_type, sampled, required]() {
          if (!self || self->handeye_process_->state() == QProcess::NotRunning) {return;}
          if (calibration_type != self->handeye_mode_combo_->currentData().toString()) {return;}
          if (state == QStringLiteral("failed")) {
            self->handeye_status_label_->setText(self->localized(
              QStringLiteral("自动标定已停止：%1").arg(detail),
              QStringLiteral("Automatic calibration stopped: %1").arg(detail)));
            self->handeye_status_label_->setStyleSheet(QStringLiteral("color: #c43b32;"));
            return;
          }
          if (state == QStringLiteral("saved")) {
            self->handeye_status_label_->setStyleSheet(QString());
            self->handeye_status_label_->setText(self->localized(
              QStringLiteral("自动标定已保存，正在结束标定并发布 TF。"),
              QStringLiteral("Automatic calibration saved; stopping calibration and publishing TF.")));
            self->handeye_process_->terminate();
            QTimer::singleShot(1500, self->handeye_process_, [self]() {
                if (self && self->handeye_process_->state() != QProcess::NotRunning) {
                  self->handeye_process_->kill();
                }
              });
            self->handeye_button_->setEnabled(false);
            return;
          }
          if (state != QStringLiteral("idle")) {
            self->handeye_status_label_->setStyleSheet(QString());
            self->handeye_status_label_->setText(self->localized(
              QStringLiteral("自动标定：%1（%2/%3）").arg(detail).arg(sampled).arg(required),
              QStringLiteral("Automatic calibration: %1 (%2/%3)")
              .arg(detail).arg(sampled).arg(required)));
          }
        }, Qt::QueuedConnection);
    });
  forbidden_zone_status_handler_ =
    [this](const QString & robot, const std_msgs::msg::String::SharedPtr message) {
      if (active_robot_ != robot) {return;}
      const auto document = QJsonDocument::fromJson(
        QByteArray::fromStdString(message->data));
      if (!document.isObject()) {
        QPointer<DemoPanel> self(this);
        QMetaObject::invokeMethod(
          this, [self, robot]() {
            if (!self || self->active_robot_ != robot) {return;}
            self->forbidden_zone_status_seen_ = false;
            self->forbidden_zone_state_label_->setText(self->localized(
                QStringLiteral("禁区管理器返回了无效状态。"),
                QStringLiteral("The forbidden-zone manager returned invalid status.")));
            self->forbidden_zone_state_label_->setStyleSheet(
              QStringLiteral("color: #c43b32; font-weight: bold;"));
          }, Qt::QueuedConnection);
        return;
      }
      const auto object = document.object();
      const auto string_list = [](const QJsonValue & value) {
          QStringList result;
          for (const auto & item : value.toArray()) {
            const auto text = item.toString().trimmed();
            if (!text.isEmpty()) {result.append(text);}
          }
          return result;
        };
      const QString config = object.value(QStringLiteral("config_file")).toString();
      const QString user_config = object.value(
        QStringLiteral("user_config_file")).toString();
      const QStringList loaded = string_list(
        object.value(QStringLiteral("loaded_areas")));
      const QStringList defined = string_list(
        object.value(QStringLiteral("defined_areas")));
      const QStringList enabled = string_list(
        object.value(QStringLiteral("enabled_areas")));
      const QStringList active_groups = string_list(
        object.value(QStringLiteral("active_groups")));
      const QStringList defined_groups = string_list(
        object.value(QStringLiteral("defined_groups")));
      const QStringList user_areas = string_list(
        object.value(QStringLiteral("user_areas")));
      const QStringList draggable_areas = string_list(
        object.value(QStringLiteral("draggable_areas")));
      const QStringList user_groups = string_list(
        object.value(QStringLiteral("user_groups")));
      QMap<QString, QStringList> user_group_members;
      const auto group_members_object = object.value(
        QStringLiteral("user_group_members")).toObject();
      for (auto iterator = group_members_object.begin();
        iterator != group_members_object.end(); ++iterator)
      {
        user_group_members.insert(iterator.key(), string_list(iterator.value()));
      }
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(
        this,
        [self, robot, config, user_config, loaded, defined, enabled, active_groups,
          defined_groups, user_areas, draggable_areas, user_groups,
          user_group_members]() {
          if (!self || self->active_robot_ != robot) {return;}
          self->forbidden_zone_config_path_ = config;
          self->forbidden_zone_user_config_path_ = user_config;
          self->forbidden_zone_loaded_areas_ = loaded;
          self->forbidden_zone_defined_areas_ = defined;
          self->forbidden_zone_enabled_areas_ = enabled;
          self->forbidden_zone_active_groups_ = active_groups;
          self->forbidden_zone_defined_groups_ = defined_groups;
          self->forbidden_zone_user_areas_ = user_areas;
          self->forbidden_zone_draggable_areas_ = draggable_areas;
          self->forbidden_zone_user_groups_ = user_groups;
          self->forbidden_zone_user_group_members_ = user_group_members;
          const QString previous = self->forbidden_zone_user_combo_->currentText();
          self->forbidden_zone_user_combo_->clear();
          self->forbidden_zone_user_combo_->addItems(user_areas);
          const int previous_index = self->forbidden_zone_user_combo_->findText(previous);
          if (previous_index >= 0) {
            self->forbidden_zone_user_combo_->setCurrentIndex(previous_index);
          }
          const QString previous_group = self->forbidden_zone_group_combo_->currentText();
          {
            const QSignalBlocker group_blocker(self->forbidden_zone_group_combo_);
            self->forbidden_zone_group_combo_->clear();
            self->forbidden_zone_group_combo_->addItems(user_groups);
            const int group_index = self->forbidden_zone_group_combo_->findText(
              previous_group);
            if (group_index >= 0) {
              self->forbidden_zone_group_combo_->setCurrentIndex(group_index);
            }
          }
          const QString selected_group = self->forbidden_zone_group_combo_->currentText();
          const auto selected_members = user_group_members.value(selected_group);
          if (!selected_group.isEmpty()) {
            self->forbidden_zone_group_name_edit_->setText(selected_group);
          }
          self->forbidden_zone_group_members_list_->clear();
          self->forbidden_zone_group_members_list_->addItems(user_areas);
          for (int index = 0;
            index < self->forbidden_zone_group_members_list_->count(); ++index)
          {
            auto * item = self->forbidden_zone_group_members_list_->item(index);
            item->setSelected(selected_members.contains(item->text()));
          }
          self->forbidden_zone_status_seen_ = true;
          self->updateForbiddenZoneUi();
        }, Qt::QueuedConnection);
    };
  update_start_state_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
    "/rviz/moveit/update_start_state", 1);
  update_goal_state_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
    "/rviz/moveit/update_goal_state", 1);
  rviz_stop_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
    "/rviz/moveit/stop", 1);
  display_trajectory_handler_ =
    [this](moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr message) {
      const std::uint64_t generation = ++free_planning_display_generation_;
      if (!message || message->trajectory.empty()) {return;}
      double duration_seconds = 0.0;
      for (const auto & trajectory : message->trajectory) {
        if (trajectory.joint_trajectory.points.empty()) {continue;}
        const auto & duration = trajectory.joint_trajectory.points.back().time_from_start;
        duration_seconds += static_cast<double>(duration.sec) +
          static_cast<double>(duration.nanosec) * 1.0e-9;
      }
      const int delay_ms = std::clamp(
        static_cast<int>(std::ceil(duration_seconds * 1000.0)) + 300,
        500, 600000);
      QMetaObject::invokeMethod(
        this,
        [this, generation, delay_ms]() {
          if (!free_planning_visible_ || teach_preview_active_.load()) {return;}
          QTimer::singleShot(
            delay_ms, this,
            [this, generation]() {
              if (generation == free_planning_display_generation_.load() &&
                free_planning_visible_ && !teach_preview_active_.load())
              {
                clearPlannedPathDisplay();
              }
            });
        },
        Qt::QueuedConnection);
    };
  configureRobotInterfaces();
  configureForbiddenZoneInterfaces();
  readiness_timer_->start();
  restartHandeyePublisher();
  QTimer::singleShot(0, this, &DemoPanel::integrateMotionPlanningPanel);
  QTimer::singleShot(750, this, &DemoPanel::refreshTeachActions);
  QTimer::singleShot(900, this, &DemoPanel::refreshTeachSequences);
  setStatus(
    QStringLiteral("已连接；等待新鲜关节反馈。"),
    QStringLiteral("Connected; waiting for fresh joint feedback."));
}

void DemoPanel::configureRobotInterfaces()
{
  if (!node_) {return;}
  const QString prefix = active_robot_ == QStringLiteral("piperh") ?
    QStringLiteral("/piperh") : QStringLiteral("/rebotarm");
  const auto endpoint = [&prefix](const QString & suffix) {
      return (prefix + suffix).toStdString();
    };
  const QString robot = active_robot_;

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    endpoint(QStringLiteral("/joint_states")), rclcpp::SensorDataQoS(),
    [this, robot](const sensor_msgs::msg::JointState::SharedPtr message) {
      if (active_robot_ != robot) {return;}
      const auto found = std::find(message->name.begin(), message->name.end(), "joint6");
      if (found != message->name.end() && message->position.size() == message->name.size()) {
        {
          std::lock_guard<std::mutex> lock(joint_state_mutex_);
          latest_joint_state_ = message;
        }
        last_feedback_ms_.store(steadyMilliseconds());
      }
    });
  xbox_armed_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    endpoint(QStringLiteral("/xbox/armed")),
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this, robot](const std_msgs::msg::Bool::SharedPtr message) {
      if (active_robot_ != robot) {return;}
      xbox_armed_.store(message->data);
      xbox_state_known_.store(true);
    });
  if (teach_status_handler_) {
    teach_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      endpoint(QStringLiteral("/teach/status")),
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this, robot](const std_msgs::msg::String::SharedPtr message) {
        if (active_robot_ != robot) {return;}
        if (teach_status_handler_) {teach_status_handler_(message);}
      });
  }
  if (display_trajectory_handler_) {
    display_trajectory_sub_ =
      node_->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      endpoint(QStringLiteral("/display_planned_path")), 10,
      [this, robot](moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr message) {
        if (active_robot_ != robot) {return;}
        if (display_trajectory_handler_) {display_trajectory_handler_(message);}
      });
  }

  validity_client_ = node_->create_client<GetStateValidity>(
    endpoint(QStringLiteral("/check_state_validity")));
  xbox_mode_client_ = node_->create_client<SetBool>(
    endpoint(QStringLiteral("/xbox/set_armed")));
  teach_list_client_ = node_->create_client<ListActionGroups>(
    endpoint(QStringLiteral("/teach/list_action_groups")));
  teach_shape_marker_client_ = node_->create_client<ConfigureShapeMarker>(
    endpoint(QStringLiteral("/teach/configure_shape_marker")));
  teach_trace_client_ = node_->create_client<ConfigureTrace>(
    endpoint(QStringLiteral("/teach/configure_trace")));
  teach_shape_client_ = node_->create_client<CreateShapeAction>(
    endpoint(QStringLiteral("/teach/create_shape_action")));
  teach_shape_reachability_client_ = node_->create_client<CheckShapeReachability>(
    endpoint(QStringLiteral("/teach/check_shape_reachability")));
  teach_sequence_list_client_ = node_->create_client<ListActionSequences>(
    endpoint(QStringLiteral("/teach/list_action_sequences")));
  teach_copy_client_ = node_->create_client<CopyActionGroup>(
    endpoint(QStringLiteral("/teach/copy_action_group")));
  teach_rename_client_ = node_->create_client<RenameActionGroup>(
    endpoint(QStringLiteral("/teach/rename_action_group")));
  teach_delete_client_ = node_->create_client<DeleteActionGroup>(
    endpoint(QStringLiteral("/teach/delete_action_group")));
  teach_select_client_ = node_->create_client<SelectActionGroup>(
    endpoint(QStringLiteral("/teach/select_action_group")));
  teach_preview_client_ = node_->create_client<PreviewActionGroup>(
    endpoint(QStringLiteral("/teach/preview_action_group")));
  teach_sequence_preview_client_ = node_->create_client<PreviewActionSequence>(
    endpoint(QStringLiteral("/teach/preview_action_sequence")));
  teach_replay_client_ = node_->create_client<ReplayActionGroup>(
    endpoint(QStringLiteral("/teach/replay_action_group")));
  teach_sequence_save_client_ = node_->create_client<SaveActionSequence>(
    endpoint(QStringLiteral("/teach/save_action_sequence")));
  teach_sequence_replay_client_ = node_->create_client<ReplayActionSequence>(
    endpoint(QStringLiteral("/teach/replay_action_sequence")));
  teach_start_client_ = node_->create_client<Trigger>(
    endpoint(QStringLiteral("/teach/start_recording")));
  teach_stop_client_ = node_->create_client<Trigger>(
    endpoint(QStringLiteral("/teach/stop_recording")));
  teach_cancel_client_ = node_->create_client<Trigger>(
    endpoint(QStringLiteral("/teach/cancel")));
  teach_reset_client_ = node_->create_client<Trigger>(
    endpoint(QStringLiteral("/teach/reset")));
  teach_pause_client_ = node_->create_client<Trigger>(
    endpoint(QStringLiteral("/teach/pause_preview")));
  teach_clear_selection_client_ = node_->create_client<Trigger>(
    endpoint(QStringLiteral("/teach/clear_action_selection")));
  execute_client_ = rclcpp_action::create_client<ExecuteTrajectory>(
    node_, endpoint(QStringLiteral("/execute_trajectory")));
  display_trajectory_pub_ =
    node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    endpoint(QStringLiteral("/display_planned_path")), 1);
}

void DemoPanel::configureForbiddenZoneInterfaces()
{
  if (!node_) {return;}
  const QString prefix = active_robot_ == QStringLiteral("piperh") ?
    QStringLiteral("/piperh") : QStringLiteral("/rebotarm");
  forbidden_zone_status_sub_.reset();
  if (forbidden_zone_status_handler_) {
    const QString robot = active_robot_;
    const auto forbidden_zone_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    forbidden_zone_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      (prefix + QStringLiteral("/forbidden_zone_manager/status")).toStdString(),
      forbidden_zone_qos,
      [this, robot](const std_msgs::msg::String::SharedPtr message) {
        if (forbidden_zone_status_handler_) {
          forbidden_zone_status_handler_(robot, message);
        }
      });
  }
  forbidden_zone_reload_client_ = node_->create_client<Trigger>(
    (prefix + QStringLiteral("/forbidden_zone_manager/reload")).toStdString());
  forbidden_zone_configure_client_ = node_->create_client<ConfigureForbiddenZone>(
    (prefix + QStringLiteral("/forbidden_zone_manager/configure")).toStdString());
}

void DemoPanel::startPickPlace()
{
  startDemo(
    QStringLiteral("pick_place"), operationTitle(QStringLiteral("pick_place"), chinese_));
}

void DemoPanel::goHome()
{
  startDemo(QStringLiteral("go_home"), operationTitle(QStringLiteral("go_home"), chinese_));
}

void DemoPanel::configureSingularityEscapeControls()
{
  const std::array<double, 6> dm_lower = {-2.8, -3.14, -3.14, -1.87, -1.57, -3.14};
  const std::array<double, 6> dm_upper = {2.8, 0.005, 0.005, 1.57, 1.57, 3.14};
  const std::array<double, 6> dm_target = {0.0, -1.60, -0.90, -0.90, 0.50, 0.0};
  const std::array<double, 6> rs_lower = {-2.8, 0.0, 0.0, -1.57, -1.57, -3.14};
  const std::array<double, 6> rs_upper = {2.8, 3.14, 3.14, 1.57, 1.57, 3.14};
  const std::array<double, 6> rs_target = {0.0, 1.40, 0.70, -0.70, 0.30, 0.0};
  const std::array<double, 6> piper_lower = {
    -2.618, 0.0, -2.96706, -2.356195, -1.56207, -3.14};
  const std::array<double, 6> piper_upper = {
    2.618, 3.14, 0.0, 2.356195, 1.56207, 3.14};
  const std::array<double, 6> piper_target = {0.0, 1.40, -1.00, 0.0, 0.50, 0.0};
  const bool piper = active_robot_ == QStringLiteral("piperh");
  const bool rs = model_ == QStringLiteral("rs");
  const auto & lower = piper ? piper_lower : rs ? rs_lower : dm_lower;
  const auto & upper = piper ? piper_upper : rs ? rs_upper : dm_upper;
  const auto & target = piper ? piper_target : rs ? rs_target : dm_target;
  for (std::size_t index = 0; index < singularity_escape_joint_spins_.size(); ++index) {
    auto * spin = singularity_escape_joint_spins_[index];
    if (!spin) {continue;}
    spin->setRange(lower[index], upper[index]);
    spin->setValue(target[index]);
  }
}

void DemoPanel::resetSingularityEscapeTarget()
{
  configureSingularityEscapeControls();
  setStatus(
    QStringLiteral("已恢复当前型号的默认脱离奇异位形关节目标。"),
    QStringLiteral("Restored the default singularity-escape joint target for this model."));
}

void DemoPanel::startSingularityEscape()
{
  if (!xbox_state_known_.load() || xbox_armed_.load()) {
    setStatus(
      QStringLiteral("拒绝启动：请先将 Xbox 控制切换为 LOCKED。"),
      QStringLiteral("Start rejected: switch Xbox control to LOCKED first."), true);
    return;
  }
  if (!feedbackFresh() || !stateValidityFresh()) {
    setStatus(
      QStringLiteral("拒绝启动：需要新鲜关节反馈和有效的 MoveIt 当前状态。"),
      QStringLiteral("Start rejected: fresh joint feedback and a valid MoveIt state are required."),
      true);
    return;
  }
  if (process_->state() != QProcess::NotRunning) {
    setStatus(
      QStringLiteral("已有操作正在运行。请先停止。"),
      QStringLiteral("Another operation is running. Stop it first."), true);
    return;
  }

  QStringList target_text;
  for (auto * spin : singularity_escape_joint_spins_) {
    target_text.append(QString::number(spin->value(), 'f', 4));
  }
  const QString target_display = QStringLiteral("[%1] rad").arg(target_text.join(", "));
  QMessageBox confirmation(this);
  confirmation.setIcon(QMessageBox::Warning);
  confirmation.setWindowTitle(localized(
    QStringLiteral("确认脱离奇异位形"),
    QStringLiteral("Confirm singularity escape")));
  confirmation.setText(localized(
    QStringLiteral(
      "目标关节角：%1\n\n"
      "程序将使用 MoveIt 从最新真机反馈进行关节空间规划、碰撞检查并在 RViz 预览，"
      "随后以 %2% 速度/加速度执行。目标中的 |J3|、|J5| 必须至少为 0.15 rad。\n\n"
      "确认 Xbox 为 LOCKED、急停可触达、机械臂固定且工作区无人和障碍物。")
    .arg(target_display).arg(speed_spin_->value(), 0, 'f', 0),
    QStringLiteral(
      "Target joints: %1\n\n"
      "MoveIt will plan from fresh robot feedback in joint space, collision-check and preview "
      "the path in RViz, then execute at %2% velocity/acceleration. |J3| and |J5| must each "
      "be at least 0.15 rad.\n\n"
      "Confirm Xbox is LOCKED, the emergency stop is reachable, the arm is secured, and the "
      "workspace is clear.")
    .arg(target_display).arg(speed_spin_->value(), 0, 'f', 0)));
  auto * confirm = confirmation.addButton(
    localized(QStringLiteral("确认规划并执行"), QStringLiteral("Plan and execute")),
    QMessageBox::AcceptRole);
  confirmation.addButton(
    localized(QStringLiteral("取消"), QStringLiteral("Cancel")),
    QMessageBox::RejectRole);
  confirmation.exec();
  if (confirmation.clickedButton() != confirm) {return;}

  const QString ros2 = QStandardPaths::findExecutable(QStringLiteral("ros2"));
  const QString config = jointTargetConfigPath();
  if (ros2.isEmpty() || config.isEmpty()) {
    setStatus(
      QStringLiteral("找不到 ros2 或脱离奇异位形参数文件。"),
      QStringLiteral("Could not find ros2 or the singularity-escape configuration."), true);
    return;
  }
  const QString scale = QString::number(speed_spin_->value() / 100.0, 'f', 3);
  const QString target_parameter = QStringLiteral("target_joint_values:=[%1]")
    .arg(target_text.join(","));
  process_->setProgram(ros2);
  QStringList arguments{
      QStringLiteral("run"), QStringLiteral("rebotarm_pulse"),
      QStringLiteral("joint_target_move"), QStringLiteral("--ros-args"),
      QStringLiteral("--params-file"), config,
      QStringLiteral("-p"), target_parameter,
      QStringLiteral("-p"), QStringLiteral("velocity_scaling:=%1").arg(scale),
      QStringLiteral("-p"), QStringLiteral("acceleration_scaling:=%1").arg(scale)};
  const QString robot_prefix = active_robot_ == QStringLiteral("piperh") ?
    QStringLiteral("/piperh") : QStringLiteral("/rebotarm");
  arguments.append({
      QStringLiteral("-r"), QStringLiteral("/joint_states:=%1/joint_states").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/rebotarm/joint_states:=%1/joint_states").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/rebot_xbox/armed:=%1/xbox/armed").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/execute_trajectory:=%1/execute_trajectory").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/plan_kinematic_path:=%1/plan_kinematic_path").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/display_planned_path:=%1/display_planned_path").arg(robot_prefix)});
  process_->setArguments(arguments);
  active_executable_ = QStringLiteral("singularity_escape");
  process_->start();
  if (!process_->waitForStarted(2000)) {
    setStatus(
      QStringLiteral("脱离奇异位形进程启动失败：%1").arg(process_->errorString()),
      QStringLiteral("Failed to start singularity escape: %1").arg(process_->errorString()),
      true);
    active_executable_.clear();
    return;
  }
  setStatus(
    QStringLiteral("正在规划脱离奇异位形的关节路径；可随时点击“停止当前操作”。"),
    QStringLiteral("Planning the joint-space singularity escape; Stop remains available."));
  updateReadiness();
}

void DemoPanel::startPulseApproach()
{
  const bool piper = active_robot_ == QStringLiteral("piperh");
  if (!xbox_state_known_.load() || xbox_armed_.load() || !feedbackFresh() ||
    !stateValidityFresh())
  {
    setStatus(
      QStringLiteral("无法前往把脉区：需要新鲜关节反馈、有效 MoveIt 状态和 Xbox LOCKED。"),
      QStringLiteral(
        "Cannot approach the pulse region: fresh joint feedback, a valid MoveIt state, "
        "and Xbox LOCKED are required."), true);
    return;
  }
  if (process_->state() != QProcess::NotRunning) {
    setStatus(QStringLiteral("已有操作正在运行。请先停止。"),
      QStringLiteral("Another operation is running. Stop it first."), true);
    return;
  }
  const auto target_timestamp = piper ?
    last_piper_pulse_point_ms_.load() : last_pulse_point_ms_.load();
  if (steadyMilliseconds() - target_timestamp >= (piper ? 1500 : 700)) {
    const bool camera_target_fresh =
      steadyMilliseconds() - last_pulse_point_ms_.load() < 700;
    setStatus(piper && camera_target_fresh ?
      QStringLiteral(
        "相机中的把脉区域已经稳定，但尚未取得 Piper-H 规划坐标目标；请检查手眼标定转换日志。") :
      QStringLiteral("把脉区域尚未稳定。请保持掌心朝向相机、手腕伸直并静止。"),
      piper && camera_target_fresh ?
      QStringLiteral(
        "The camera pulse region is stable, but no Piper-H planning-frame target is available; "
        "check the hand-eye transform log.") :
      QStringLiteral(
        "The pulse region is not stable. Face the palm toward the camera, keep the wrist "
        "straight, and hold still."), true);
    return;
  }
  const QString calibration = handeyeCalibrationFile();
  if (calibration.isEmpty() || !QFileInfo::exists(calibration)) {
    setStatus(
      QStringLiteral("未找到已保存的手眼标定；请先在“手眼标定”页完成 Compute 和 Save。"),
      QStringLiteral(
        "No saved hand-eye calibration was found. Complete Compute and Save on the "
        "Hand-Eye Calibration tab first."), true);
    return;
  }
  if (handeye_process_->state() != QProcess::NotRunning) {
    setStatus(
      QStringLiteral("手眼标定仍在运行；请先结束标定窗口再测试预接触规划。"),
      QStringLiteral(
        "Hand-eye calibration is still running; close it before testing pre-contact planning."),
      true);
    return;
  }
  geometry_msgs::msg::PointStamped::SharedPtr target;
  {
    std::lock_guard<std::mutex> lock(pulse_point_mutex_);
    target = piper ? latest_piper_pulse_point_ : latest_pulse_point_;
  }
  if (!target) {
    setStatus(
      QStringLiteral("尚未缓存可用于规划的把脉目标；请保持手静止后重试。"),
      QStringLiteral(
        "No pulse target is cached for planning; hold the hand still and retry."),
      true);
    return;
  }
  const double requested_speed = piper ?
    std::min(speed_spin_->value(), 5.0) : speed_spin_->value();

  QMessageBox confirmation(this);
  confirmation.setIcon(QMessageBox::Warning);
  confirmation.setWindowTitle(localized(
    piper ? QStringLiteral("确认假体触压测试") : QStringLiteral("确认前往把脉预接触位"),
    piper ? QStringLiteral("Confirm dummy pressure-stop test") :
    QStringLiteral("Confirm pulse pre-contact motion")));
  confirmation.setText(piper ? localized(
    QStringLiteral(
      "仅限非人体假体！视觉目标：%1  X=%2, Y=%3, Z=%4 m\n"
      "保持 J6 姿态，末端最多走到视觉目标，不越过目标；XY ≤300 mm、Z ≤200 mm，"
      "笛卡尔速度 ≤5 mm/s，执行超时 90 秒。\n"
      "出发后视觉目标冻结，遮挡不会停；压力变化达到阈值将请求取消。"
      "这不是力控制，取消有延迟。"
      "请确认假体固定、路径无人、急停可触达。")
    .arg(QString::fromStdString(target->header.frame_id))
    .arg(target->point.x, 0, 'f', 4).arg(target->point.y, 0, 'f', 4)
    .arg(target->point.z, 0, 'f', 4),
    QStringLiteral(
      "NON-HUMAN DUMMY ONLY! Visual target: %1  X=%2, Y=%3, Z=%4 m\n"
      "The tip moves no farther than the target with J6 orientation fixed; "
      "XY ≤300 mm, Z ≤200 mm, Cartesian speed ≤5 mm/s, 90 s timeout.\n"
      "The target is frozen after motion starts; visual dropout will not stop it. "
      "Pressure change requests cancellation, not force control; cancellation may lag. "
      "Confirm the dummy is fixed, the path is clear, and emergency stop is reachable.")
    .arg(QString::fromStdString(target->header.frame_id))
    .arg(target->point.x, 0, 'f', 4).arg(target->point.y, 0, 'f', 4)
    .arg(target->point.z, 0, 'f', 4)) : localized(
    QStringLiteral(
      "检测点：%1  X=%2, Y=%3, Z=%4 m\n"
      "机械臂将以 %5% 速度规划并运动到距皮肤约 %6 mm 的预接触位。\n"
      "%7\n\n"
      "本功能不会接触人体，也不会施加把脉压力。请确认探头长度与安装方向、手保持静止、"
      "急停可触达且路径无人和障碍物。")
    .arg(QString::fromStdString(target->header.frame_id))
    .arg(target->point.x, 0, 'f', 4).arg(target->point.y, 0, 'f', 4)
    .arg(target->point.z, 0, 'f', 4).arg(requested_speed, 0, 'f', 0)
    .arg(pulse_standoff_spin_->value(), 0, 'f', 0)
    .arg(QString()),
    QStringLiteral(
      "Detected point: %1  X=%2, Y=%3, Z=%4 m\n"
      "The robot will plan and move at %5% speed to a pre-contact pose about %6 mm from skin.\n"
      "%7\n\n"
      "This does not contact a person or apply palpation force. Confirm the probe length "
      "and mounting direction, the hand remains still, the emergency stop is reachable, "
      "and the path is clear.")
    .arg(QString::fromStdString(target->header.frame_id))
    .arg(target->point.x, 0, 'f', 4).arg(target->point.y, 0, 'f', 4)
    .arg(target->point.z, 0, 'f', 4).arg(requested_speed, 0, 'f', 0)
    .arg(pulse_standoff_spin_->value(), 0, 'f', 0)
    .arg(QString())));
  auto * confirm = confirmation.addButton(
    piper ? localized(
      QStringLiteral("确认在假体上前往目标"), QStringLiteral("Move to target on dummy")) :
    localized(QStringLiteral("确认前往预接触位"), QStringLiteral("Move to pre-contact")),
    QMessageBox::AcceptRole);
  confirmation.addButton(localized(QStringLiteral("取消"), QStringLiteral("Cancel")),
    QMessageBox::RejectRole);
  confirmation.exec();
  if (confirmation.clickedButton() != confirm) {return;}
  if (piper) {
    geometry_msgs::msg::PointStamped::SharedPtr latest;
    {
      std::lock_guard<std::mutex> lock(pulse_point_mutex_);
      latest = latest_piper_pulse_point_;
    }
    // The planner owns the median+EMA stability window and drift debounce.  The
    // panel only rejects a genuinely stale stream; comparing two raw frames here
    // used to turn ordinary visual jitter into a hard failure.
    if (!latest || steadyMilliseconds() - last_piper_pulse_point_ms_.load() >= 1500 ||
      latest->header.frame_id != target->header.frame_id)
    {
      setStatus(
        QStringLiteral("确认期间腕部目标丢失或已超过 1.5 秒未更新；请保持手静止后重试。"),
        QStringLiteral(
          "The wrist target was lost or stale for over 1.5 s; hold still and retry."),
        true);
      return;
    }
  }

  restartHandeyePublisher();
  const QString ros2 = QStandardPaths::findExecutable(QStringLiteral("ros2"));
  const QString config = pulseConfigPath();
  if (ros2.isEmpty() || config.isEmpty()) {
    setStatus(QStringLiteral("找不到 rebotarm_pulse 或其参数文件。"),
      QStringLiteral("Could not find rebotarm_pulse or its parameter file."), true);
    return;
  }
  const QString scale = QString::number(requested_speed / 100.0, 'f', 3);
  const QString executable = piper ?
    QStringLiteral("piper_pulse_align") : QStringLiteral("pulse_approach");
  const QString robot_prefix = piper ? QStringLiteral("/piperh") : QStringLiteral("/rebotarm");
  process_->setProgram(ros2);
  QStringList arguments{
      QStringLiteral("run"), QStringLiteral("rebotarm_pulse"),
      executable, QStringLiteral("--ros-args"),
      QStringLiteral("--params-file"), config,
      QStringLiteral("-p"), QStringLiteral("velocity_scaling:=%1").arg(scale),
      QStringLiteral("-p"), QStringLiteral("acceleration_scaling:=%1").arg(scale)};
  if (piper) {
    arguments.append({QStringLiteral("-p"), QStringLiteral("execute_motion:=true"),
      QStringLiteral("-p"), QStringLiteral("dummy_contact_mode:=true")});
  } else {
    arguments.append({
        QStringLiteral("-p"), QStringLiteral("standoff_m:=%1").arg(
          pulse_standoff_spin_->value() / 1000.0, 0, 'f', 4),
        QStringLiteral("-p"), QStringLiteral("calibration_name:=%1").arg(
          handeye_name_edit_->text().trimmed())});
  }
  arguments.append({
      QStringLiteral("-r"), QStringLiteral("/joint_states:=%1/joint_states").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/rebotarm/joint_states:=%1/joint_states").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/rebot_xbox/armed:=%1/xbox/armed").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/execute_trajectory:=%1/execute_trajectory").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/compute_ik:=%1/compute_ik").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/check_state_validity:=%1/check_state_validity").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/compute_cartesian_path:=%1/compute_cartesian_path").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/move_group/get_parameters:=%1/move_group/get_parameters").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/plan_kinematic_path:=%1/plan_kinematic_path").arg(robot_prefix),
      QStringLiteral("-r"), QStringLiteral("/display_planned_path:=%1/display_planned_path").arg(robot_prefix)});
  process_->setArguments(arguments);
  active_executable_ = executable;
  pulse_dummy_pressure_stopped_ = false;
  pulse_process_output_tail_.clear();
  process_->start();
  if (!process_->waitForStarted(2000)) {
    setStatus(
      QStringLiteral("把脉区预接触进程启动失败：%1").arg(process_->errorString()),
      QStringLiteral("Failed to start pulse pre-contact: %1").arg(process_->errorString()),
      true);
    active_executable_.clear();
    return;
  }
  setStatus(
    piper ? QStringLiteral("假体触压测试：正在规划并朝视觉目标运动，压力变化会请求停止。") :
    QStringLiteral("正在规划并执行到预接触位，可随时停止。"),
    piper ? QStringLiteral(
      "Dummy pressure-stop test: planning and moving toward the visual target; "
      "pressure change requests a stop.") :
    QStringLiteral("Planning and moving to the pre-contact pose; stop is available."));
  updateReadiness();
}

void DemoPanel::syncPlanningState()
{
  if (!feedbackFresh()) {
    setStatus(
      QStringLiteral("无法同步：关节反馈缺失或超过 1 秒。"),
      QStringLiteral("Cannot sync: joint feedback is missing or older than 1 second."), true);
    return;
  }
  if (!update_start_state_pub_ || !update_goal_state_pub_) {
    setStatus(
      QStringLiteral("无法同步：RViz 同步接口尚未就绪。"),
      QStringLiteral("Cannot sync: the RViz synchronization interface is not ready."), true);
    return;
  }

  const std_msgs::msg::Empty message;
  update_start_state_pub_->publish(message);
  update_goal_state_pub_->publish(message);
  setStatus(
    QStringLiteral("已把规划起点和目标姿态重置为真机当前关节位置。"),
    QStringLiteral(
      "Planning start and goal states were reset to the robot's current joint positions."));
}

void DemoPanel::reloadForbiddenZones()
{
  if (forbidden_zone_request_pending_) {return;}
  if (!forbidden_zone_reload_client_ ||
    !forbidden_zone_reload_client_->service_is_ready())
  {
    forbidden_zone_state_label_->setText(localized(
        QStringLiteral("无法重新加载：禁区管理器服务未启动。"),
        QStringLiteral("Cannot reload: the forbidden-zone manager service is unavailable.")));
    forbidden_zone_state_label_->setStyleSheet(
      QStringLiteral("color: #c43b32; font-weight: bold;"));
    return;
  }
  forbidden_zone_request_pending_ = true;
  updateForbiddenZoneUi();
  auto request = std::make_shared<Trigger::Request>();
  forbidden_zone_reload_client_->async_send_request(
    request, [this](rclcpp::Client<Trigger>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(
        this, [self, success, detail]() {
          if (!self) {return;}
          self->forbidden_zone_request_pending_ = false;
          if (!success) {
            self->forbidden_zone_state_label_->setText(self->localized(
                QStringLiteral("禁区重新加载失败：%1").arg(detail),
                QStringLiteral("Forbidden-zone reload failed: %1").arg(detail)));
            self->forbidden_zone_state_label_->setStyleSheet(
              QStringLiteral("color: #c43b32; font-weight: bold;"));
          } else {
            self->forbidden_zone_state_label_->setText(self->localized(
                QStringLiteral("正在刷新 MoveIt 禁区，请等待状态更新…"),
                QStringLiteral("Refreshing MoveIt forbidden zones; waiting for status…")));
            self->forbidden_zone_state_label_->setStyleSheet(
              QStringLiteral("color: #d08020; font-weight: bold;"));
          }
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::selectForbiddenZoneMesh()
{
  const QString path = QFileDialog::getOpenFileName(
    this, localized(QStringLiteral("选择禁区三维模型"),
      QStringLiteral("Select forbidden-zone 3D model")),
    forbidden_zone_mesh_path_,
    localized(QStringLiteral("三维网格 (*.stl *.obj)"),
      QStringLiteral("3D meshes (*.stl *.obj)")));
  if (path.isEmpty()) {return;}
  forbidden_zone_mesh_path_ = QFileInfo(path).absoluteFilePath();
  forbidden_zone_mesh_path_label_->setText(QFileInfo(path).fileName());
  forbidden_zone_mesh_path_label_->setToolTip(forbidden_zone_mesh_path_);
  if (forbidden_zone_name_edit_->text().trimmed().isEmpty()) {
    forbidden_zone_name_edit_->setText(QFileInfo(path).completeBaseName());
  }
}

void DemoPanel::applyForbiddenZone()
{
  if (forbidden_zone_request_pending_) {return;}
  if (!forbidden_zone_configure_client_ ||
    !forbidden_zone_configure_client_->service_is_ready())
  {
    forbidden_zone_state_label_->setText(localized(
        QStringLiteral("无法导入：禁区编辑服务未启动。"),
        QStringLiteral("Cannot import: forbidden-zone editor service is unavailable.")));
    forbidden_zone_state_label_->setStyleSheet(
      QStringLiteral("color: #c43b32; font-weight: bold;"));
    return;
  }
  const QString name = forbidden_zone_name_edit_->text().trimmed();
  const QString shape = forbidden_zone_shape_combo_->currentData().toString();
  if (name.isEmpty()) {
    QMessageBox::warning(this, localized(QStringLiteral("禁区名称缺失"),
      QStringLiteral("Missing zone name")), localized(
      QStringLiteral("请输入一个禁区名称。"), QStringLiteral("Enter a zone name.")));
    return;
  }
  if (shape == QStringLiteral("mesh") && forbidden_zone_mesh_path_.isEmpty()) {
    QMessageBox::warning(this, localized(QStringLiteral("三维文件缺失"),
      QStringLiteral("Missing 3D file")), localized(
      QStringLiteral("请先选择 .stl 或 .obj 文件。"),
      QStringLiteral("Select an .stl or .obj file first.")));
    return;
  }
  if (forbidden_zone_user_areas_.contains(name) &&
    QMessageBox::question(this, localized(QStringLiteral("覆盖禁区"),
      QStringLiteral("Replace zone")), localized(
      QStringLiteral("用户禁区“%1”已经存在。是否覆盖？").arg(name),
      QStringLiteral("User zone “%1” already exists. Replace it?").arg(name))) !=
    QMessageBox::Yes)
  {
    return;
  }
  auto request = std::make_shared<ConfigureForbiddenZone::Request>();
  request->operation = "upsert";
  request->name = name.toStdString();
  request->shape = shape.toStdString();
  request->frame_id = "base_link";
  if (shape == QStringLiteral("box")) {
    request->dimensions = {
      forbidden_zone_dimension_1_spin_->value(),
      forbidden_zone_dimension_2_spin_->value(),
      forbidden_zone_dimension_3_spin_->value()};
  } else if (shape == QStringLiteral("sphere")) {
    request->dimensions = {forbidden_zone_dimension_1_spin_->value()};
  } else if (shape == QStringLiteral("cylinder") || shape == QStringLiteral("cone")) {
    request->dimensions = {
      forbidden_zone_dimension_1_spin_->value(),
      forbidden_zone_dimension_2_spin_->value()};
  }
  request->mesh_path = forbidden_zone_mesh_path_.toStdString();
  request->mesh_scale = forbidden_zone_mesh_scale_spin_->value();
  request->pose.position.x = forbidden_zone_x_spin_->value();
  request->pose.position.y = forbidden_zone_y_spin_->value();
  request->pose.position.z = forbidden_zone_z_spin_->value();
  constexpr double pi = 3.14159265358979323846;
  const double roll = forbidden_zone_roll_spin_->value() * pi / 180.0;
  const double pitch = forbidden_zone_pitch_spin_->value() * pi / 180.0;
  const double yaw = forbidden_zone_yaw_spin_->value() * pi / 180.0;
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  request->pose.orientation.x = sr * cp * cy - cr * sp * sy;
  request->pose.orientation.y = cr * sp * cy + sr * cp * sy;
  request->pose.orientation.z = cr * cp * sy - sr * sp * cy;
  request->pose.orientation.w = cr * cp * cy + sr * sp * sy;
  forbidden_zone_request_pending_ = true;
  updateForbiddenZoneUi();
  forbidden_zone_configure_client_->async_send_request(
    request, [this, name](rclcpp::Client<ConfigureForbiddenZone>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(
        this, [self, success, detail, name]() {
          if (!self) {return;}
          self->forbidden_zone_request_pending_ = false;
          self->forbidden_zone_state_label_->setText(success ? self->localized(
              QStringLiteral("禁区“%1”已保存，正在更新三维场景…").arg(name),
              QStringLiteral("Zone “%1” saved; updating the 3D scene…").arg(name)) :
            self->localized(QStringLiteral("禁区导入失败：%1").arg(detail),
              QStringLiteral("Forbidden-zone import failed: %1").arg(detail)));
          self->forbidden_zone_state_label_->setStyleSheet(success ?
            QStringLiteral("color: #d08020; font-weight: bold;") :
            QStringLiteral("color: #c43b32; font-weight: bold;"));
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::removeForbiddenZone()
{
  if (forbidden_zone_request_pending_ || forbidden_zone_user_combo_->currentIndex() < 0 ||
    !forbidden_zone_configure_client_ ||
    !forbidden_zone_configure_client_->service_is_ready())
  {
    return;
  }
  const QString name = forbidden_zone_user_combo_->currentText();
  if (QMessageBox::question(this, localized(QStringLiteral("删除禁区"),
    QStringLiteral("Remove zone")), localized(
    QStringLiteral("确定删除用户禁区“%1”？").arg(name),
    QStringLiteral("Remove user zone “%1”?").arg(name))) != QMessageBox::Yes)
  {
    return;
  }
  auto request = std::make_shared<ConfigureForbiddenZone::Request>();
  request->operation = "remove";
  request->name = name.toStdString();
  forbidden_zone_request_pending_ = true;
  updateForbiddenZoneUi();
  forbidden_zone_configure_client_->async_send_request(
    request, [this](rclcpp::Client<ConfigureForbiddenZone>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(
        this, [self, success, detail]() {
          if (!self) {return;}
          self->forbidden_zone_request_pending_ = false;
          if (!success) {
            self->forbidden_zone_state_label_->setText(self->localized(
                QStringLiteral("删除禁区失败：%1").arg(detail),
                QStringLiteral("Failed to remove zone: %1").arg(detail)));
            self->forbidden_zone_state_label_->setStyleSheet(
              QStringLiteral("color: #c43b32; font-weight: bold;"));
          }
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::saveForbiddenZoneGroup()
{
  if (forbidden_zone_request_pending_ || !forbidden_zone_configure_client_ ||
    !forbidden_zone_configure_client_->service_is_ready())
  {
    return;
  }
  const QString name = forbidden_zone_group_name_edit_->text().trimmed();
  const auto selected = forbidden_zone_group_members_list_->selectedItems();
  if (name.isEmpty() || selected.isEmpty()) {
    QMessageBox::warning(this, localized(QStringLiteral("禁区组信息不完整"),
      QStringLiteral("Incomplete zone group")), localized(
      QStringLiteral("请输入组名，并至少选择一个用户禁区。"),
      QStringLiteral("Enter a group name and select at least one user zone.")));
    return;
  }
  if (forbidden_zone_user_groups_.contains(name) &&
    QMessageBox::question(this, localized(QStringLiteral("覆盖禁区组"),
      QStringLiteral("Replace zone group")), localized(
      QStringLiteral("禁区组“%1”已经存在。是否覆盖其成员？").arg(name),
      QStringLiteral("Zone group “%1” exists. Replace its members?").arg(name))) !=
    QMessageBox::Yes)
  {
    return;
  }
  auto request = std::make_shared<ConfigureForbiddenZone::Request>();
  request->operation = "upsert_group";
  request->name = name.toStdString();
  for (const auto * item : selected) {
    request->members.push_back(item->text().toStdString());
  }
  forbidden_zone_request_pending_ = true;
  updateForbiddenZoneUi();
  forbidden_zone_configure_client_->async_send_request(
    request, [this, name](rclcpp::Client<ConfigureForbiddenZone>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(
        this, [self, success, detail, name]() {
          if (!self) {return;}
          self->forbidden_zone_request_pending_ = false;
          self->forbidden_zone_state_label_->setText(success ? self->localized(
              QStringLiteral("禁区组“%1”已保存。").arg(name),
              QStringLiteral("Zone group “%1” saved.").arg(name)) :
            self->localized(QStringLiteral("禁区组保存失败：%1").arg(detail),
              QStringLiteral("Failed to save zone group: %1").arg(detail)));
          self->forbidden_zone_state_label_->setStyleSheet(success ?
            QStringLiteral("color: #d08020; font-weight: bold;") :
            QStringLiteral("color: #c43b32; font-weight: bold;"));
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::removeForbiddenZoneGroup()
{
  if (forbidden_zone_request_pending_ || forbidden_zone_group_combo_->currentIndex() < 0 ||
    !forbidden_zone_configure_client_ ||
    !forbidden_zone_configure_client_->service_is_ready())
  {
    return;
  }
  const QString name = forbidden_zone_group_combo_->currentText();
  if (QMessageBox::question(this, localized(QStringLiteral("删除禁区组"),
    QStringLiteral("Remove zone group")), localized(
    QStringLiteral("确定删除禁区组“%1”？组内禁区本身不会被删除。").arg(name),
    QStringLiteral("Remove zone group “%1”? Its zones will be kept.").arg(name))) !=
    QMessageBox::Yes)
  {
    return;
  }
  auto request = std::make_shared<ConfigureForbiddenZone::Request>();
  request->operation = "remove_group";
  request->name = name.toStdString();
  forbidden_zone_request_pending_ = true;
  updateForbiddenZoneUi();
  forbidden_zone_configure_client_->async_send_request(
    request, [this](rclcpp::Client<ConfigureForbiddenZone>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(
        this, [self, success, detail]() {
          if (!self) {return;}
          self->forbidden_zone_request_pending_ = false;
          if (!success) {
            self->forbidden_zone_state_label_->setText(self->localized(
                QStringLiteral("删除禁区组失败：%1").arg(detail),
                QStringLiteral("Failed to remove zone group: %1").arg(detail)));
            self->forbidden_zone_state_label_->setStyleSheet(
              QStringLiteral("color: #c43b32; font-weight: bold;"));
          }
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::updateForbiddenZoneEditor()
{
  const QString shape = forbidden_zone_shape_combo_->currentData().toString();
  const bool mesh = shape == QStringLiteral("mesh");
  const bool sphere = shape == QStringLiteral("sphere");
  const bool box = shape == QStringLiteral("box");
  forbidden_zone_dimension_1_spin_->setVisible(!mesh);
  forbidden_zone_dimension_2_spin_->setVisible(!mesh && !sphere);
  forbidden_zone_dimension_3_spin_->setVisible(box);
  forbidden_zone_dimensions_label_->setVisible(!mesh);
  forbidden_zone_mesh_row_widget_->setVisible(mesh);
  forbidden_zone_mesh_scale_label_->setVisible(mesh);
  forbidden_zone_mesh_scale_spin_->setVisible(mesh);
  if (box) {
    forbidden_zone_dimension_1_spin_->setPrefix(QStringLiteral("X "));
    forbidden_zone_dimension_2_spin_->setPrefix(QStringLiteral("Y "));
    forbidden_zone_dimension_3_spin_->setPrefix(QStringLiteral("Z "));
    forbidden_zone_dimensions_label_->setText(localized(
        QStringLiteral("长方体尺寸"), QStringLiteral("Box dimensions")));
  } else if (sphere) {
    forbidden_zone_dimension_1_spin_->setPrefix(localized(
        QStringLiteral("半径 "), QStringLiteral("Radius ")));
    forbidden_zone_dimensions_label_->setText(localized(
        QStringLiteral("球体尺寸"), QStringLiteral("Sphere dimensions")));
  } else {
    forbidden_zone_dimension_1_spin_->setPrefix(localized(
        QStringLiteral("高度 "), QStringLiteral("Height ")));
    forbidden_zone_dimension_2_spin_->setPrefix(localized(
        QStringLiteral("半径 "), QStringLiteral("Radius ")));
    forbidden_zone_dimensions_label_->setText(localized(
        QStringLiteral("高度与半径"), QStringLiteral("Height and radius")));
  }
}

void DemoPanel::updateForbiddenZoneUi()
{
  const bool service_ready = forbidden_zone_reload_client_ &&
    forbidden_zone_reload_client_->service_is_ready();
  forbidden_zone_reload_button_->setEnabled(
    service_ready && !forbidden_zone_request_pending_);
  const bool edit_ready = forbidden_zone_configure_client_ &&
    forbidden_zone_configure_client_->service_is_ready();
  forbidden_zone_apply_button_->setEnabled(
    edit_ready && !forbidden_zone_request_pending_);
  forbidden_zone_remove_button_->setEnabled(
    edit_ready && !forbidden_zone_request_pending_ &&
    forbidden_zone_user_combo_->currentIndex() >= 0);
  forbidden_zone_user_combo_->setEnabled(
    edit_ready && !forbidden_zone_request_pending_ &&
    forbidden_zone_user_combo_->count() > 0);
  forbidden_zone_group_name_edit_->setEnabled(
    edit_ready && !forbidden_zone_request_pending_);
  forbidden_zone_group_members_list_->setEnabled(
    edit_ready && !forbidden_zone_request_pending_ &&
    forbidden_zone_group_members_list_->count() > 0);
  forbidden_zone_group_save_button_->setEnabled(
    edit_ready && !forbidden_zone_request_pending_ &&
    forbidden_zone_group_members_list_->count() > 0);
  forbidden_zone_group_combo_->setEnabled(
    edit_ready && !forbidden_zone_request_pending_ &&
    forbidden_zone_group_combo_->count() > 0);
  forbidden_zone_group_remove_button_->setEnabled(
    edit_ready && !forbidden_zone_request_pending_ &&
    forbidden_zone_group_combo_->currentIndex() >= 0);
  if (forbidden_zone_request_pending_) {
    forbidden_zone_state_label_->setText(localized(
        QStringLiteral("正在重新加载禁区…"),
        QStringLiteral("Reloading forbidden zones…")));
    forbidden_zone_state_label_->setStyleSheet(
      QStringLiteral("color: #d08020; font-weight: bold;"));
  } else if (!forbidden_zone_status_seen_) {
    forbidden_zone_state_label_->setText(service_ready ? localized(
        QStringLiteral("禁区管理器在线，正在等待区域状态…"),
        QStringLiteral("Forbidden-zone manager online; waiting for area status…")) : localized(
        QStringLiteral("禁区保护未启动。"),
        QStringLiteral("Forbidden-zone protection is not running.")));
    forbidden_zone_state_label_->setStyleSheet(service_ready ?
      QStringLiteral("color: #d08020; font-weight: bold;") :
      QStringLiteral("color: #c43b32; font-weight: bold;"));
  } else if (forbidden_zone_loaded_areas_.isEmpty()) {
    forbidden_zone_state_label_->setText(localized(
        QStringLiteral("禁区管理器在线，但当前加载了 0 个禁止通行区域。"),
        QStringLiteral("Forbidden-zone manager online, but 0 areas are loaded.")));
    forbidden_zone_state_label_->setStyleSheet(
      QStringLiteral("color: #d08020; font-weight: bold;"));
  } else {
    forbidden_zone_state_label_->setText(localized(
        QStringLiteral("禁区保护生效：%1 个区域已加入 MoveIt。")
        .arg(forbidden_zone_loaded_areas_.size()),
        QStringLiteral("Forbidden-zone protection active: %1 area(s) are in MoveIt.")
        .arg(forbidden_zone_loaded_areas_.size())));
    forbidden_zone_state_label_->setStyleSheet(
      QStringLiteral("color: #3a9d23; font-weight: bold;"));
  }
  const auto display_list = [this](const QStringList & values) {
      return values.isEmpty() ? localized(QStringLiteral("无"), QStringLiteral("none")) :
             values.join(QStringLiteral(", "));
    };
  forbidden_zone_areas_label_->setText(localized(
      QStringLiteral("已加载：%1　已启用：%2　可拖动：%3　配置中定义：%4")
      .arg(display_list(forbidden_zone_loaded_areas_))
      .arg(display_list(forbidden_zone_enabled_areas_))
      .arg(display_list(forbidden_zone_draggable_areas_))
      .arg(display_list(forbidden_zone_defined_areas_)),
      QStringLiteral("Loaded: %1  Enabled: %2  Draggable: %3  Defined: %4")
      .arg(display_list(forbidden_zone_loaded_areas_))
      .arg(display_list(forbidden_zone_enabled_areas_))
      .arg(display_list(forbidden_zone_draggable_areas_))
      .arg(display_list(forbidden_zone_defined_areas_))));
  forbidden_zone_groups_label_->setText(localized(
      QStringLiteral("活动区域组：%1　用户禁区组：%2　配置中区域组：%3")
      .arg(display_list(forbidden_zone_active_groups_))
      .arg(display_list(forbidden_zone_user_groups_))
      .arg(display_list(forbidden_zone_defined_groups_)),
      QStringLiteral("Active groups: %1  User groups: %2  Defined groups: %3")
      .arg(display_list(forbidden_zone_active_groups_))
      .arg(display_list(forbidden_zone_user_groups_))
      .arg(display_list(forbidden_zone_defined_groups_))));
  forbidden_zone_config_label_->setText(localized(
      QStringLiteral("基础配置：%1\n用户配置：%2").arg(
        forbidden_zone_config_path_.isEmpty() ? QStringLiteral("—") :
        forbidden_zone_config_path_).arg(
        forbidden_zone_user_config_path_.isEmpty() ? QStringLiteral("—") :
        forbidden_zone_user_config_path_),
      QStringLiteral("Base configuration: %1\nUser configuration: %2").arg(
        forbidden_zone_config_path_.isEmpty() ? QStringLiteral("—") :
        forbidden_zone_config_path_).arg(
        forbidden_zone_user_config_path_.isEmpty() ? QStringLiteral("—") :
        forbidden_zone_user_config_path_)));
  forbidden_zone_config_label_->setToolTip(
    forbidden_zone_config_path_ + QStringLiteral("\n") + forbidden_zone_user_config_path_);
}

void DemoPanel::startDemo(
  const QString & executable, const QString & title, bool require_confirmation)
{
  if (!xbox_state_known_.load() || xbox_armed_.load()) {
    setStatus(
      QStringLiteral("拒绝启动：请先按 A 将 Xbox 控制切换为 LOCKED。"),
      QStringLiteral("Start rejected: press A to switch Xbox control to LOCKED first."), true);
    return;
  }
  if (!feedbackFresh()) {
    setStatus(
      QStringLiteral("拒绝启动：关节反馈缺失或超过 1 秒。"),
      QStringLiteral("Start rejected: joint feedback is missing or older than 1 second."), true);
    return;
  }
  if (!stateValidityFresh()) {
    setStatus(
      QStringLiteral("拒绝启动：MoveIt 当前状态无效、存在碰撞或检查已超时。"),
      QStringLiteral(
        "Start rejected: the current MoveIt state is invalid, in collision, or stale."),
      true);
    return;
  }
  if (process_->state() != QProcess::NotRunning) {
    setStatus(
      QStringLiteral("已有操作正在运行。请先停止。"),
      QStringLiteral("Another operation is running. Stop it first."), true);
    return;
  }

  if (require_confirmation) {
    QMessageBox confirmation(this);
    confirmation.setIcon(QMessageBox::Warning);
    confirmation.setWindowTitle(localized(
      QStringLiteral("确认驱动真机"), QStringLiteral("Confirm physical robot motion")));
    confirmation.setText(
      localized(
        QStringLiteral(
          "将以 %1% 速度/加速度启动“%2”。\n\n"
          "确认 Xbox 控制为 LOCKED、急停可触达、机械臂固定、工作区无人且无障碍物。"),
        QStringLiteral(
          "Start “%2” at %1% velocity/acceleration.\n\n"
          "Confirm Xbox control is LOCKED, the emergency stop is reachable, the arm is secured, "
          "and the workspace is clear."))
      .arg(speed_spin_->value(), 0, 'f', 0).arg(title));
    auto * confirm_button = confirmation.addButton(
      localized(QStringLiteral("确认启动"), QStringLiteral("Start")),
      QMessageBox::AcceptRole);
    confirmation.addButton(
      localized(QStringLiteral("取消"), QStringLiteral("Cancel")),
      QMessageBox::RejectRole);
    confirmation.exec();
    if (confirmation.clickedButton() != confirm_button) {
      setStatus(QStringLiteral("已取消启动。"), QStringLiteral("Start cancelled."));
      return;
    }
  }

  const QString ros2 = QStandardPaths::findExecutable(QStringLiteral("ros2"));
  const QString config = configPath(executable);
  if (ros2.isEmpty() || config.isEmpty()) {
    setStatus(
      QStringLiteral("找不到 ros2 或操作参数文件。"),
      QStringLiteral("Could not find ros2 or the operation parameter file."), true);
    return;
  }

  const double scale = speed_spin_->value() / 100.0;
  const QString scale_arg = QString::number(scale, 'f', 3);
  QStringList arguments = {
    QStringLiteral("run"), QStringLiteral("rebotarm_moveit_demos"), executable,
    QStringLiteral("--ros-args"), QStringLiteral("--params-file"), config,
    QStringLiteral("-p"), QStringLiteral("velocity_scaling:=%1").arg(scale_arg),
    QStringLiteral("-p"), QStringLiteral("acceleration_scaling:=%1").arg(scale_arg)};
  if (active_robot_ == QStringLiteral("piperh")) {
    arguments.append({
        QStringLiteral("-r"), QStringLiteral("/joint_states:=/piperh/joint_states"),
        QStringLiteral("-r"), QStringLiteral("/rebotarm/joint_states:=/piperh/joint_states"),
        QStringLiteral("-r"), QStringLiteral("/execute_trajectory:=/piperh/execute_trajectory"),
        QStringLiteral("-r"), QStringLiteral("/compute_ik:=/piperh/compute_ik"),
        QStringLiteral("-r"), QStringLiteral("/plan_kinematic_path:=/piperh/plan_kinematic_path"),
        QStringLiteral("-r"), QStringLiteral("/apply_planning_scene:=/piperh/apply_planning_scene"),
        QStringLiteral("-r"), QStringLiteral("/get_planning_scene:=/piperh/get_planning_scene"),
        QStringLiteral("-r"), QStringLiteral("/display_planned_path:=/piperh/display_planned_path")});
  }

  active_executable_ = executable;
  process_->start(ros2, arguments);
  if (!process_->waitForStarted(2000)) {
    setStatus(
      QStringLiteral("操作进程启动失败：%1").arg(process_->errorString()),
      QStringLiteral("Failed to start the operation process: %1").arg(process_->errorString()),
      true);
    active_executable_.clear();
    return;
  }
  home_button_->setEnabled(false);
  sync_button_->setEnabled(false);
  pick_place_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  speed_spin_->setEnabled(false);
  setStatus(
    QStringLiteral("%1正在运行；可随时点击“停止当前操作”。")
    .arg(operationTitle(executable, true)),
    QStringLiteral("%1 is running; click “Stop current operation” at any time.")
    .arg(operationTitle(executable, false)));
}

void DemoPanel::stopDemo()
{
  if (execute_client_) {
    execute_client_->async_cancel_all_goals();
  }
  if (process_->state() != QProcess::NotRunning) {
    process_->terminate();
    QTimer::singleShot(1500, process_, [this]() {
        if (process_->state() != QProcess::NotRunning) {
          process_->kill();
        }
    });
  }
  stop_button_->setEnabled(false);
  setStatus(
    QStringLiteral("已请求取消 MoveIt 轨迹并停止操作进程。"),
    QStringLiteral("Requested cancellation of the MoveIt trajectory and operation process."));
}

void DemoPanel::toggleXboxControl()
{
  if (!xbox_mode_client_ || !xbox_mode_client_->service_is_ready() ||
    xbox_request_pending_.load() || !xbox_state_known_.load())
  {
    setStatus(
      QStringLiteral("Xbox 模式切换服务或状态尚未就绪。"),
      QStringLiteral("Xbox mode service or state is not ready."), true);
    return;
  }

  const bool enable = !xbox_armed_.load();
  if (enable) {
    const auto answer = QMessageBox::warning(
      this,
      localized(QStringLiteral("启用 Xbox 控制"), QStringLiteral("Enable Xbox Control")),
      localized(
        QStringLiteral(
          "将把机械臂运动控制权交给 Xbox。确认手柄在线，所有摇杆回中、扳机松开，"
          "急停可触达且工作区无人。"),
        QStringLiteral(
          "This transfers robot motion control to the Xbox. Confirm the controller is online, "
          "all sticks are centered, triggers released, the emergency stop is reachable, and "
          "the workspace is clear.")),
      QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer != QMessageBox::Ok) {return;}
  }

  auto request = std::make_shared<SetBool::Request>();
  request->data = enable;
  xbox_request_pending_.store(true);
  updateReadiness();
  QPointer<DemoPanel> self(this);
  xbox_mode_client_->async_send_request(
    request, [self, enable](rclcpp::Client<SetBool>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, enable, success, detail]() {
          if (!self) {return;}
          self->xbox_request_pending_.store(false);
          QString chinese_detail = detail;
          if (detail.startsWith(QStringLiteral("trigger axes are not initialized"))) {
            chinese_detail = QStringLiteral(
              "手柄扳机轴尚未初始化；请把 LT、RT 分别完全按下再完全松开，然后重新启用。%1")
              .arg(detail.mid(detail.indexOf(QStringLiteral("(LT="))));
          } else if (detail == QStringLiteral("release both Xbox triggers before enabling control")) {
            chinese_detail = QStringLiteral("请完全松开 LT 和 RT 后再启用 Xbox 控制");
          }
          self->setStatus(
            success ? (enable ? QStringLiteral("Xbox 控制已启用：%1").arg(chinese_detail) :
            QStringLiteral("Xbox 控制已锁定：%1").arg(chinese_detail)) :
            QStringLiteral("Xbox 模式切换被拒绝：%1").arg(chinese_detail),
            success ? (enable ? QStringLiteral("Xbox control enabled: %1").arg(detail) :
            QStringLiteral("Xbox control locked: %1").arg(detail)) :
            QStringLiteral("Xbox mode change rejected: %1").arg(detail),
            !success);
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

bool DemoPanel::cameraNodeRunning() const
{
  if (!node_) {return false;}
  try {
    for (const auto & entry :
      node_->get_node_graph_interface()->get_node_names_and_namespaces())
    {
      if (entry.first == "camera" && entry.second == "/camera") {return true;}
    }
  } catch (const std::exception &) {
    return false;
  }
  return false;
}

bool DemoPanel::cameraStreamPublished() const
{
  if (!node_ || camera_image_topic_.isEmpty()) {return false;}
  try {
    return !node_->get_publishers_info_by_topic(camera_image_topic_.toStdString()).empty();
  } catch (const std::exception &) {
    return false;
  }
}

void DemoPanel::toggleCamera()
{
  if (camera_process_->state() != QProcess::NotRunning) {
    camera_stop_requested_.store(true);
    camera_button_->setEnabled(false);
    camera_label_->setText(localized(
      QStringLiteral("Gemini 2：正在关闭…"),
      QStringLiteral("Gemini 2: stopping…")));
    camera_process_->terminate();
    return;
  }
  if (cameraNodeRunning()) {
    camera_detected_.store(true);
    camera_stream_published_.store(cameraStreamPublished());
    setStatus(
      camera_stream_published_.load() ?
      QStringLiteral("检测到外部启动的 Gemini 2 图像流；面板不会关闭非本面板创建的进程。") :
      QStringLiteral("检测到外部 Gemini 2 驱动，但摄像头设备离线或尚无图像流。"),
      camera_stream_published_.load() ?
      QStringLiteral(
        "An external Gemini 2 stream is running; the panel will not stop a process it does not own.") :
      QStringLiteral(
        "An external Gemini 2 driver exists, but the camera is offline or has no image stream."));
    updateReadiness();
    return;
  }

  const bool enable_color = camera_color_checkbox_->isChecked();
  const bool enable_depth = camera_depth_checkbox_->isChecked();
  const bool enable_ir = camera_ir_checkbox_->isChecked();
  if (!enable_color && !enable_depth && !enable_ir) {
    setStatus(
      QStringLiteral("请至少勾选彩色图、深度图或红外图中的一路。"),
      QStringLiteral("Select at least one of color, depth, or infrared."), true);
    return;
  }

  camera_stop_requested_.store(false);
  camera_button_->setEnabled(false);
  camera_process_->setProgram(QStringLiteral("ros2"));
  camera_process_->setArguments({
      QStringLiteral("launch"), QStringLiteral("orbbec_camera"),
      QStringLiteral("gemini2.launch.py"),
      QStringLiteral("enable_color:=%1").arg(enable_color ? "true" : "false"),
      QStringLiteral("color_width:=1280"),
      QStringLiteral("color_height:=720"),
      QStringLiteral("color_fps:=30"),
      QStringLiteral("color_qos:=SENSOR_DATA"),
      QStringLiteral("color_camera_info_qos:=SENSOR_DATA"),
      QStringLiteral("enable_depth:=%1").arg(enable_depth ? "true" : "false"),
      QStringLiteral("depth_qos:=SENSOR_DATA"),
      QStringLiteral("enable_ir:=%1").arg(enable_ir ? "true" : "false"),
      QStringLiteral("ir_qos:=SENSOR_DATA"),
      QStringLiteral("depth_registration:=%1").arg(
        enable_color && enable_depth ? "true" : "false"),
      QStringLiteral("enable_frame_sync:=false")});
  camera_process_->start();
  QStringList streams;
  if (enable_color) {streams.append(localized(QStringLiteral("彩色"), QStringLiteral("color")));}
  if (enable_depth) {streams.append(localized(QStringLiteral("深度"), QStringLiteral("depth")));}
  if (enable_ir) {streams.append(localized(QStringLiteral("红外"), QStringLiteral("infrared")));}
  setStatus(
    QStringLiteral("正在启动 Gemini 2：%1…").arg(streams.join(QStringLiteral("、"))),
    QStringLiteral("Starting Gemini 2: %1…").arg(streams.join(QStringLiteral(", "))));
  updateReadiness();
}

void DemoPanel::toggleHandVision()
{
  if (hand_vision_process_->state() != QProcess::NotRunning) {
    hand_vision_process_->terminate();
    QTimer::singleShot(1500, hand_vision_process_, [this]() {
        if (hand_vision_process_->state() != QProcess::NotRunning) {hand_vision_process_->kill();}
      });
    hand_vision_status_label_->setText(localized(
      QStringLiteral("正在停止手部视觉…"), QStringLiteral("Stopping hand vision…")));
    hand_vision_button_->setEnabled(false);
    return;
  }

  QString script;
  try {
    const QString prefix = QString::fromStdString(
      ament_index_cpp::get_package_prefix("meridian_hand_vision"));
    script = QStringLiteral("%1/lib/meridian_hand_vision/run_hand_depth_viewer.sh").arg(prefix);
  } catch (const std::exception & error) {
    hand_vision_status_label_->setText(localized(
      QStringLiteral("找不到手部视觉 ROS 包：%1").arg(QString::fromUtf8(error.what())),
      QStringLiteral("Hand vision ROS package was not found: %1").arg(
        QString::fromUtf8(error.what()))));
    return;
  }
  if (!QFileInfo::exists(script)) {
    hand_vision_status_label_->setText(localized(
      QStringLiteral("找不到手部视觉启动脚本：%1").arg(script),
      QStringLiteral("Hand vision launcher not found: %1").arg(script)));
    return;
  }
  // Start the saved hand-eye TF before vision can produce a robot-frame
  // target.  Previously this happened only after the pulse button was clicked,
  // while the button itself required that transformed target, creating a
  // readiness deadlock after switching to Piper-H.
  restartHandeyePublisher();
  hand_vision_process_->setProgram(QStringLiteral("bash"));
  QString depth_topic = QStringLiteral("/camera/depth/image_raw");
  QString camera_info_topic = QStringLiteral("/camera/color/camera_info");
  const QString marker = QStringLiteral("/color/");
  const int marker_index = hand_vision_color_topic_.indexOf(marker);
  if (marker_index >= 0) {
    depth_topic = hand_vision_color_topic_.left(marker_index) +
      QStringLiteral("/depth/") + hand_vision_color_topic_.mid(marker_index + marker.size());
    camera_info_topic = hand_vision_color_topic_.left(marker_index) +
      QStringLiteral("/color/camera_info");
  }
  hand_vision_process_->setArguments({
      script, QStringLiteral("--color-topic"), hand_vision_color_topic_,
      QStringLiteral("--depth-topic"), depth_topic,
      QStringLiteral("--camera-info-topic"), camera_info_topic,
      QStringLiteral("--num-hands"), QStringLiteral("1"),
      QStringLiteral("--no-window")});
  last_hand_vision_image_ms_.store(0);
  hand_vision_process_->start();
  if (!hand_vision_process_->waitForStarted(1500)) {
    hand_vision_status_label_->setText(localized(
      QStringLiteral("手部视觉启动失败：%1").arg(hand_vision_process_->errorString()),
      QStringLiteral("Could not start hand vision: %1").arg(hand_vision_process_->errorString())));
    return;
  }
  hand_vision_status_label_->setText(localized(
    QStringLiteral("手部视觉已启动（仅在本面板显示）。"),
    QStringLiteral("Hand vision started in this panel only.")));
  hand_vision_button_->setText(localized(QStringLiteral("停止手部视觉"), QStringLiteral("Stop Hand Vision")));
  updateReadiness();
}

void DemoPanel::refreshHandVisionTopics()
{
  if (!node_ || !hand_vision_topic_combo_) {return;}
  QStringList topics;
  for (const auto & entry : node_->get_topic_names_and_types()) {
    const bool image_type = std::find(
      entry.second.begin(), entry.second.end(), "sensor_msgs/msg/Image") != entry.second.end();
    if (image_type) {topics.append(QString::fromStdString(entry.first));}
  }
  topics.sort();
  if (!topics.contains(hand_vision_color_topic_)) {topics.prepend(hand_vision_color_topic_);}
  {
    const QSignalBlocker blocker(hand_vision_topic_combo_);
    hand_vision_topic_combo_->clear();
    hand_vision_topic_combo_->addItems(topics);
    const int index = hand_vision_topic_combo_->findText(hand_vision_color_topic_);
    hand_vision_topic_combo_->setCurrentIndex(index >= 0 ? index : 0);
  }
  hand_vision_topic_refresh_button_->setToolTip(localized(
    QStringLiteral("已检索 %1 个图像话题").arg(topics.size()),
    QStringLiteral("Found %1 image topics").arg(topics.size())));
}

void DemoPanel::selectHandVisionTopic(const QString & topic)
{
  const QString selected = topic.trimmed();
  if (selected.isEmpty() || selected == hand_vision_color_topic_) {return;}
  hand_vision_color_topic_ = selected;
  if (hand_vision_process_->state() != QProcess::NotRunning) {
    hand_vision_status_label_->setText(localized(
      QStringLiteral("摄像头已切换为 %1；请重启手部视觉使其生效。"),
      QStringLiteral("Camera changed to %1; restart hand vision to apply it.")).arg(selected));
  } else {
    hand_vision_status_label_->setText(localized(
      QStringLiteral("已选择摄像头：%1。点击启动手部视觉。"),
      QStringLiteral("Selected camera: %1. Click Start Hand Vision.")).arg(selected));
  }
}

void DemoPanel::toggleHandeyeCalibration()
{
  if (handeye_process_->state() != QProcess::NotRunning) {
    removeMoveItCalibrationDisplay();
    handeye_process_->terminate();
    QTimer::singleShot(1500, handeye_process_, [this]() {
        if (handeye_process_->state() != QProcess::NotRunning) {handeye_process_->kill();}
      });
    handeye_status_label_->setText(localized(
      QStringLiteral("正在停止手眼标定…"), QStringLiteral("Stopping hand-eye calibration…")));
    handeye_button_->setEnabled(false);
    return;
  }

  const bool eye_in_hand = handeyeEyeInHand();
  const bool moveit_backend = handeyeUsesMoveItCalibration();
  if (moveit_backend && active_robot_ != QStringLiteral("rebotarm")) {
    handeye_status_label_->setText(localized(
      QStringLiteral("当前 MoveIt Calibration 模式仅接入 reBotArm；Piper-H 请先使用 easy_handeye2。"),
      QStringLiteral("MoveIt Calibration is currently integrated for reBotArm only; use easy_handeye2 for Piper-H.")));
    return;
  }
  const bool board_target = handeye_target_combo_->currentData().toString() ==
    QStringLiteral("a4_4tag_board");
  const bool charuco_target = handeye_target_combo_->currentData().toString() ==
    QStringLiteral("charuco_a4_5x7");
  const QString calibration_type = eye_in_hand ?
    QStringLiteral("eye_in_hand") : QStringLiteral("eye_on_base");
  const bool automatic_sequence_supported =
    !moveit_backend && (active_robot_ == QStringLiteral("rebotarm") ||
    active_robot_ == QStringLiteral("piperh"));
  const QString auto_sequence_name = automaticHandeyeSequenceName();
  const QString auto_action_prefix =
    eye_in_hand && active_robot_ == QStringLiteral("piperh") ?
    kPiperEyeInHandActionPrefix :
    (eye_in_hand ? QStringLiteral("眼在手上标定姿态_") : QStringLiteral("手眼标定姿态_"));
  QStringList fields = {
    handeye_name_edit_->text().trimmed(), handeye_robot_base_edit_->text().trimmed(),
    handeye_robot_effector_edit_->text().trimmed(), handeye_tracking_base_edit_->text().trimmed(),
    handeye_tracking_marker_edit_->text().trimmed()};
  if (fields[2] == QStringLiteral("griper_tcp")) {
    fields[2] = QStringLiteral("gripper_tcp");
    handeye_robot_effector_edit_->setText(fields[2]);
  }
  if (std::any_of(fields.cbegin(), fields.cend(), [](const QString & value) {return value.isEmpty();})) {
    handeye_status_label_->setText(localized(
      QStringLiteral("请填写完整的标定名称和 4 个 TF frame。"),
      QStringLiteral("Fill in the calibration name and all four TF frames.")));
    return;
  }
  if (!cameraNodeRunning() || !cameraStreamPublished()) {
    handeye_status_label_->setText(localized(
      QStringLiteral("请先连接 Gemini 2，并确认“摄像头”页已收到实时图像，再开始标定。"),
      QStringLiteral("Connect Gemini 2 and confirm a live image on the Camera tab before calibrating.")));
    return;
  }
  QMessageBox guide(this);
  guide.setIcon(QMessageBox::Information);
  guide.setWindowTitle(localized(
    QStringLiteral("手眼标定向导"), QStringLiteral("Hand-eye calibration guide")));
  const bool piper = active_robot_ == QStringLiteral("piperh");
  const QString target_zh = charuco_target ?
    QStringLiteral("A4 ChArUco 板（DICT_4X4_50、5 × 7、方格 35 mm、Marker 26 mm、板面 175 × 245 mm）") :
    board_target ?
    QStringLiteral("A4 四标签板（36h11、ID 0–3、黑框边长 %1 mm、中心距 100 × 120 mm）")
      .arg(handeye_tag_size_spin_->value(), 0, 'f', 1) :
    QStringLiteral("%1 家族 ID %2、黑色方框边长 %3 mm 的 AprilTag")
      .arg(handeye_tag_family_combo_->currentText()).arg(handeye_tag_id_spin_->value()).arg(
      handeye_tag_size_spin_->value(), 0, 'f', 1);
  const QString target_en = charuco_target ?
    QStringLiteral("A4 ChArUco board (DICT_4X4_50, 5 × 7, 35 mm squares, 26 mm markers, 175 × 245 mm board)") :
    board_target ?
    QStringLiteral("A4 four-tag board (36h11 IDs 0–3, %1 mm black-square edge, 100 × 120 mm centre spacing)")
      .arg(handeye_tag_size_spin_->value(), 0, 'f', 1) :
    QStringLiteral("%1 AprilTag (ID %2, %3 mm black-square edge)")
      .arg(handeye_tag_family_combo_->currentText()).arg(handeye_tag_id_spin_->value()).arg(
      handeye_tag_size_spin_->value(), 0, 'f', 1);
  const QString mounting_zh = eye_in_hand ?
    QStringLiteral(
      "这是“眼在手上”模式：把相机刚性固定到机器人末端，把%1刚性固定在环境中。"
      "相机和标定目标在整个标定过程中都不能松动。") :
    (piper ? QStringLiteral(
      "这是“眼在手外”模式：把相机刚性固定在机器人外部，把%1刚性固定到 Piper-H "
      "末端法兰或已标定工具。相机和标定目标在整个标定过程中都不能松动。") : QStringLiteral(
      "这是“眼在手外”模式：把相机刚性固定在机器人外部，把%1刚性固定到夹爪。"
      "相机和标定目标在整个标定过程中都不能松动。"));
  const QString mounting_en = eye_in_hand ?
    QStringLiteral(
      "Eye-in-hand mode: rigidly mount the camera on the robot effector and rigidly fix the %1 "
      "in the environment. Neither mount may shift.") :
    (piper ? QStringLiteral(
      "Eye-on-base mode: rigidly fix the camera outside the robot and rigidly mount the %1 "
      "on the Piper-H flange or calibrated tool. "
      "Neither mount may shift.") : QStringLiteral(
      "Eye-on-base mode: rigidly fix the camera outside the robot and rigidly mount the %1 "
      "on the gripper. Neither mount may shift."));
  const QString mounting_details_zh = mounting_zh.arg(target_zh);
  const QString mounting_details_en = mounting_en.arg(target_en);
  guide.setText(localized(
    moveit_backend ? QStringLiteral(
      "%1\n\n将打开 MoveIt Calibration 插件并预置本项目的 ChArUco 参数。"
      "在插件中确认 Target 图像、Context 的 4 个 frame 和 Planning Group=arm，"
      "然后在 Calibrate 页手动采集至少 12 个覆盖多轴旋转的姿态并保存结果。\n\n"
      "MoveIt Calibration 模式暂不接入本项目的自动采样动作组。")
    .arg(mounting_details_zh) : automatic_sequence_supported ? QStringLiteral(
      "%1\n\n启动后：\n"
      "1. 确认标定窗口能持续看到 marker_frame；\n"
      "2. 手动采集至少 12 个覆盖多轴旋转的姿态，或使用匹配当前模式的自动动作组；\n"
      "3. 自动流程请选择“%2”，先完整预览，再勾选全部姿态执行；\n"
      "4. 自动流程会采样、Compute 和 Save；手动流程需自行点击 Compute/Save。\n\n"
      "“启动手眼标定”本身不会驱动机械臂；只有确认真机回放动作组后才会运动。")
    .arg(mounting_details_zh).arg(auto_sequence_name) : QStringLiteral(
      "%1\n\n启动后确认标定窗口持续看到 marker_frame，再通过 Piper-H MoveIt 逐个规划"
      "至少 12 个覆盖多轴旋转的安全姿态，并在 easy_handeye2 中手动采样、Compute 和 Save。\n\n"
      "启动标定不会驱动机械臂；每个姿态都必须单独规划、碰撞检查并由现场人员确认。")
    .arg(mounting_details_zh),
    moveit_backend ? QStringLiteral(
      "%1\n\nThe MoveIt Calibration plugin will open with this project's ChArUco parameters. "
      "Verify the target image, all four Context frames, and Planning Group=arm, then collect at "
      "least 12 multi-axis poses manually on the Calibrate tab and save the result.\n\n"
      "Project automatic sampling is not connected to the MoveIt Calibration backend yet.")
    .arg(mounting_details_en) : automatic_sequence_supported ? QStringLiteral(
      "%1\n\nAfter launch, keep marker_frame visible and collect at least 12 multi-axis poses "
      "manually, or preview and execute all poses in the matching robot sequence “%2”. The automatic "
      "workflow samples, computes, and saves; a manual workflow requires Compute and Save.\n\n"
      "Starting calibration alone never moves the robot; motion begins only after robot replay is confirmed.")
    .arg(mounting_details_en).arg(auto_sequence_name) : QStringLiteral(
      "%1\n\nAfter launch, keep marker_frame visible. Use Piper-H MoveIt to plan at least "
      "12 safe multi-axis poses, then sample, Compute, and Save manually in easy_handeye2.\n\n"
      "Starting calibration does not move the robot; every pose requires planning, collision checking, "
      "and on-site confirmation.")
    .arg(mounting_details_en)));
  auto * start = guide.addButton(
    localized(QStringLiteral("开始标定"), QStringLiteral("Start calibration")),
    QMessageBox::AcceptRole);
  guide.addButton(localized(QStringLiteral("取消"), QStringLiteral("Cancel")),
    QMessageBox::RejectRole);
  guide.exec();
  if (guide.clickedButton() != start) {return;}

  if (handeye_publish_process_->state() != QProcess::NotRunning) {
    handeye_publish_process_->terminate();
    handeye_publish_process_->waitForFinished(1000);
  }
  QString camera_info_topic = QStringLiteral("/camera/color/camera_info");
  const QString color_marker = QStringLiteral("/color/");
  const int color_index = hand_vision_color_topic_.indexOf(color_marker);
  if (color_index >= 0) {
    camera_info_topic = hand_vision_color_topic_.left(color_index) +
      QStringLiteral("/color/camera_info");
  }
  handeye_process_->setProgram(QStringLiteral("ros2"));
  const QString handeye_robot_prefix = active_robot_ == QStringLiteral("piperh") ?
    QStringLiteral("/piperh") : QStringLiteral("/rebotarm");
  handeye_process_->setArguments({
      QStringLiteral("launch"), QStringLiteral("rebotarm_pulse"),
      QStringLiteral("handeye_calibrate.launch.py"),
      QStringLiteral("name:=%1").arg(fields[0]),
      QStringLiteral("calibration_type:=%1").arg(calibration_type),
      QStringLiteral("calibration_backend:=%1").arg(
        moveit_backend ? QStringLiteral("moveit_calibration") : QStringLiteral("easy_handeye2")),
      QStringLiteral("robot_base_frame:=%1").arg(fields[1]),
      QStringLiteral("robot_effector_frame:=%1").arg(fields[2]),
      QStringLiteral("tracking_base_frame:=%1").arg(fields[3]),
      QStringLiteral("tracking_marker_frame:=%1").arg(fields[4]),
      QStringLiteral("target_type:=%1").arg(
        handeye_target_combo_->currentData().toString()),
      QStringLiteral("tag_family:=%1").arg(handeye_tag_family_combo_->currentText()),
      QStringLiteral("tag_id:=%1").arg(handeye_tag_id_spin_->value()),
      QStringLiteral("tag_size_m:=%1").arg(
        handeye_tag_size_spin_->value() / 1000.0, 0, 'f', 4),
      QStringLiteral("auto_sequence_name:=%1").arg(auto_sequence_name),
      QStringLiteral("auto_action_prefix:=%1").arg(auto_action_prefix),
      QStringLiteral("auto_minimum_samples:=%1").arg(
        active_robot_ == QStringLiteral("piperh") && eye_in_hand ? 15 : 12),
      QStringLiteral("use_auto_sequence:=%1").arg(
        automatic_sequence_supported ? QStringLiteral("true") : QStringLiteral("false")),
      QStringLiteral("teach_status_topic:=%1/teach/status").arg(handeye_robot_prefix),
      QStringLiteral("joint_state_topic:=%1/joint_states").arg(handeye_robot_prefix),
      QStringLiteral("teach_cancel_service:=%1/teach/cancel").arg(handeye_robot_prefix),
      QStringLiteral("color_topic:=%1").arg(hand_vision_color_topic_),
      QStringLiteral("camera_info_topic:=%1").arg(camera_info_topic)});
  handeye_process_->start();
  if (!handeye_process_->waitForStarted(1500)) {
    handeye_status_label_->setText(localized(
      QStringLiteral("手眼标定启动失败：%1").arg(handeye_process_->errorString()),
      QStringLiteral("Could not start hand-eye calibration: %1").arg(handeye_process_->errorString())));
    handeye_backend_combo_->setEnabled(true);
    handeye_mode_combo_->setEnabled(true);
    handeye_target_combo_->setEnabled(!moveit_backend);
    return;
  }
  if (moveit_backend && !showMoveItCalibrationDisplay(fields, eye_in_hand)) {
    handeye_process_->terminate();
    handeye_process_->waitForFinished(1000);
    handeye_status_label_->setText(localized(
      QStringLiteral("无法加载 MoveIt Calibration RViz 插件；请确认 moveit_calibration_gui 已构建并重新 source。"),
      QStringLiteral("Could not load the MoveIt Calibration RViz plugin; build moveit_calibration_gui and source the workspace.")));
    handeye_backend_combo_->setEnabled(true);
    handeye_mode_combo_->setEnabled(true);
    return;
  }
  handeye_backend_combo_->setEnabled(false);
  handeye_mode_combo_->setEnabled(false);
  handeye_target_combo_->setEnabled(false);
  handeye_status_label_->setText(localized(
    (moveit_backend ?
    QStringLiteral("MoveIt Calibration 已打开；请在其三个页签中确认目标、frame 并手动采样。") :
    automatic_sequence_supported ?
    QStringLiteral("%1模式已启动；请使用匹配的动作组“%2”或手动采样。") :
    QStringLiteral("%1模式已启动；请用 Piper-H MoveIt 规划安全姿态并手动采样。"))
    .arg(eye_in_hand ? QStringLiteral("眼在手上") : QStringLiteral("眼在手外"))
    .arg(auto_sequence_name),
    (moveit_backend ?
    QStringLiteral("MoveIt Calibration opened; verify target and frames, then sample manually in its three tabs.") :
    automatic_sequence_supported ?
    QStringLiteral("%1 calibration started; use matching sequence “%2” or sample manually.") :
    QStringLiteral("%1 calibration started; plan safe Piper-H poses and sample manually."))
    .arg(eye_in_hand ? QStringLiteral("Eye-in-hand") : QStringLiteral("Eye-on-base"))
    .arg(auto_sequence_name)));
  handeye_button_->setText(localized(
    moveit_backend ? QStringLiteral("关闭 MoveIt Calibration") : QStringLiteral("停止手眼标定"),
    moveit_backend ? QStringLiteral("Close MoveIt Calibration") : QStringLiteral("Stop Hand-eye Calibration")));
}

bool DemoPanel::handeyeEyeInHand() const
{
  return handeye_mode_combo_ &&
         handeye_mode_combo_->currentData().toString() == QStringLiteral("eye_in_hand");
}

bool DemoPanel::handeyeUsesMoveItCalibration() const
{
  return handeye_backend_combo_ &&
         handeye_backend_combo_->currentData().toString() == QStringLiteral("moveit_calibration");
}

bool DemoPanel::showMoveItCalibrationDisplay(const QStringList & fields, bool eye_in_hand)
{
  if (!getDisplayContext()) {return false;}
  auto * root = getDisplayContext()->getRootDisplayGroup();
  auto * manager = dynamic_cast<rviz_common::VisualizationManager *>(getDisplayContext());
  if (!root || !manager) {return false;}
  for (int index = 0; index < root->numDisplays(); ++index) {
    auto * display = root->getDisplayAt(index);
    if (display && display->getClassId() == QStringLiteral(
        "moveit_rviz_plugin/HandEyeCalibration"))
    {
      moveit_calibration_display_ = display;
      break;
    }
  }
  if (!moveit_calibration_display_) {
    moveit_calibration_display_ = manager->createDisplay(
      QStringLiteral("moveit_rviz_plugin/HandEyeCalibration"),
      QStringLiteral("MoveIt 手眼标定"), true);
  }
  if (!moveit_calibration_display_) {return false;}
  configureMoveItCalibrationDisplay(fields, eye_in_hand);
  QTimer::singleShot(2000, this, [this, fields, eye_in_hand]() {
      configureMoveItCalibrationDisplay(fields, eye_in_hand);
    });
  return true;
}

void DemoPanel::configureMoveItCalibrationDisplay(
  const QStringList & fields, bool eye_in_hand)
{
  if (!moveit_calibration_display_ || fields.size() < 5) {return;}
  moveit_calibration_display_->subProp(QStringLiteral("Move Group Namespace"))->setValue(
    QStringLiteral("/rebotarm"));
  moveit_calibration_display_->subProp(QStringLiteral("Planning Scene Topic"))->setValue(
    QStringLiteral("/rebotarm/planning_scene"));
  rviz_common::Config config;
  config.mapSetValue(QStringLiteral("target_type"), QStringLiteral("HandEyeTarget/Charuco"));
  config.mapSetValue(QStringLiteral("squares, X"), 5);
  config.mapSetValue(QStringLiteral("squares, Y"), 7);
  config.mapSetValue(QStringLiteral("marker size (px)"), 260);
  config.mapSetValue(QStringLiteral("square size (px)"), 350);
  config.mapSetValue(QStringLiteral("margin size (px)"), 0);
  config.mapSetValue(QStringLiteral("marker border (bits)"), 1);
  config.mapSetValue(QStringLiteral("ArUco dictionary"), QStringLiteral("DICT_4X4_50"));
  config.mapSetValue(QStringLiteral("longest board side (m)"), 0.245f);
  config.mapSetValue(QStringLiteral("measured marker size (m)"), 0.026f);
  config.mapSetValue(QStringLiteral("image_topic"), hand_vision_color_topic_);
  config.mapSetValue(QStringLiteral("sensor_mount_type"), eye_in_hand ? 1 : 0);
  config.mapSetValue(QStringLiteral("sensor"), fields[3]);
  config.mapSetValue(QStringLiteral("object"), QStringLiteral("handeye_target"));
  config.mapSetValue(QStringLiteral("eef"), fields[2]);
  config.mapSetValue(QStringLiteral("base"), fields[1]);
  config.mapSetValue(QStringLiteral("solver"), QStringLiteral("OpenCV/Tsai1989"));
  config.mapSetValue(QStringLiteral("group"), QStringLiteral("arm"));
  moveit_calibration_display_->load(config);
  moveit_calibration_display_->setEnabled(true);
}

void DemoPanel::removeMoveItCalibrationDisplay()
{
  if (!moveit_calibration_display_ || !getDisplayContext()) {return;}
  auto * root = getDisplayContext()->getRootDisplayGroup();
  if (!root) {return;}
  if (auto * display = root->takeDisplay(moveit_calibration_display_)) {delete display;}
  moveit_calibration_display_ = nullptr;
}

QString DemoPanel::automaticHandeyeSequenceName() const
{
  if (active_robot_ == QStringLiteral("piperh")) {
    return handeyeEyeInHand() ?
           kPiperAutoHandeyeEyeInHandSequenceName :
           kPiperAutoHandeyeEyeOnBaseSequenceName;
  }
  return handeyeEyeInHand() ? kAutoHandeyeEyeInHandSequenceName :
         kAutoHandeyeEyeOnBaseSequenceName;
}

QString DemoPanel::handeyeCalibrationFile() const
{
  if (!handeye_name_edit_) {return QString();}
  const QString name = handeye_name_edit_->text().trimmed();
  if (name.isEmpty() || name.contains('/') || name.contains(QStringLiteral(".."))) {
    return QString();
  }
  return qEnvironmentVariable("EASY_HANDEYE2_CALIBRATIONS_DIRECTORY", QDir::homePath() + QStringLiteral("/.ros2/easy_handeye2/calibrations")) + QStringLiteral("/") +
    name + QStringLiteral(".calib");
}

void DemoPanel::restartHandeyePublisher()
{
  const QString calibration = handeyeCalibrationFile();
  if (calibration.isEmpty() || !QFileInfo::exists(calibration) ||
    handeye_process_->state() != QProcess::NotRunning)
  {
    return;
  }
  if (handeye_publish_process_->state() != QProcess::NotRunning) {
    return;
  }
  handeye_publish_process_->setProgram(QStringLiteral("ros2"));
  handeye_publish_process_->setArguments({
      QStringLiteral("launch"), QStringLiteral("easy_handeye2"),
      QStringLiteral("publish.launch.py"),
      QStringLiteral("name:=%1").arg(handeye_name_edit_->text().trimmed())});
  handeye_publish_process_->start();
  if (handeye_publish_process_->waitForStarted(1500)) {
    handeye_status_label_->setText(localized(
      QStringLiteral("已加载并发布标定：%1").arg(calibration),
      QStringLiteral("Loaded and publishing calibration: %1").arg(calibration)));
  } else {
    handeye_status_label_->setText(localized(
      QStringLiteral("无法发布标定 TF：%1").arg(handeye_publish_process_->errorString()),
      QStringLiteral("Could not publish calibration TF: %1").arg(
        handeye_publish_process_->errorString())));
  }
}

QString DemoPanel::selectedTeachAction() const
{
  if (!teach_action_combo_ || teach_action_combo_->currentIndex() < 0) {
    return QString();
  }
  return teach_action_combo_->currentData().toString().trimmed();
}

QString DemoPanel::selectedDrawingSource() const
{
  const QString drawing = teach_shape_combo_->currentData().toString();
  if (drawing == QStringLiteral("text")) {
    return teach_shape_source_edit_->text().trimmed();
  }
  if (drawing == QStringLiteral("image")) {
    return teach_shape_image_path_;
  }
  return QString();
}

void DemoPanel::startNewShapeAction()
{
  if (selected_legacy_shape_action_ && !selectedTeachAction().isEmpty()) {
    editing_shape_action_name_ = selectedTeachAction();
    selected_legacy_shape_action_ = false;
    shape_editor_new_mode_ = false;
    setTeachStatus(
      QStringLiteral(
        "已用面板当前值接管旧动作包“%1”；请核对全部参数后再保存。")
      .arg(editing_shape_action_name_),
      QStringLiteral(
        "The current panel values are now attached to legacy action pack “%1”; "
        "verify every parameter before saving.")
      .arg(editing_shape_action_name_));
  } else {
    editing_shape_action_name_.clear();
    shape_editor_new_mode_ = true;
    if (teach_action_combo_ && teach_action_combo_->currentIndex() != 0) {
      teach_action_combo_->setCurrentIndex(0);
    } else if (initialized_) {
      resetNewActionParameters();
      clearTeachActionSelection();
    }
    setTeachStatus(
      QStringLiteral("已切换到新建模式；画布将定位到当前夹爪位姿，真机不会运动。"),
      QStringLiteral(
        "Switched to new-action mode; the canvas will align with the current gripper pose. "
        "Hardware will not move."));
  }
  if (teach_action_mode_tabs_) {teach_action_mode_tabs_->setCurrentIndex(0);}
  retranslateUi();
}

void DemoPanel::resetNewActionParameters()
{
  const QSignalBlocker shape_blocker(teach_shape_combo_);
  const QSignalBlocker source_blocker(teach_shape_source_edit_);
  shape_pose_syncing_ = true;
  const int rectangle_index = teach_shape_combo_->findData(QStringLiteral("rectangle"));
  if (rectangle_index >= 0) {teach_shape_combo_->setCurrentIndex(rectangle_index);}
  teach_shape_source_edit_->clear();
  teach_shape_image_path_.clear();
  // Start from a valid, visible drawing pose.  Resetting every field to zero
  // made W/H and pen lift fail the safety gate, leaving the create button
  // disabled with no actionable explanation for a new Piper-H library.
  teach_shape_x_spin_->setValue(0.28);
  teach_shape_y_spin_->setValue(0.0);
  teach_shape_z_spin_->setValue(0.12);
  teach_shape_roll_spin_->setValue(0.0);
  teach_shape_pitch_spin_->setValue(0.0);
  teach_shape_yaw_spin_->setValue(0.0);
  teach_shape_width_spin_->setValue(0.06);
  teach_shape_height_spin_->setValue(0.08);
  teach_shape_pen_length_spin_->setValue(0.0);
  teach_shape_pen_lift_spin_->setValue(12.0);
  shape_pose_syncing_ = false;
  shape_pose_dirty_ = true;
  shape_reachability_available_ = false;
  shape_reachability_feasible_ = false;
  shape_marker_visible_.store(true);
  updateShapePreview();
  retranslateUi();
  if (initialized_) {configureShapeMarker(true, true, false, false);}
}

void DemoPanel::clearTeachActionSelection()
{
  teach_selection_generation_.fetch_add(1);
  if (teach_request_pending_.load()) {return;}
  if (!teach_clear_selection_client_ ||
    !teach_clear_selection_client_->service_is_ready())
  {
    setTeachStatus(
      QStringLiteral("无法清除旧动作预览：清除选择服务尚未就绪。"),
      QStringLiteral("Cannot clear the old action preview: the clear-selection service is not ready."),
      true);
    return;
  }
  teach_request_pending_.store(true);
  updateReadiness();
  auto request = std::make_shared<Trigger::Request>();
  QPointer<DemoPanel> self(this);
  teach_clear_selection_client_->async_send_request(
    request, [self](rclcpp::Client<Trigger>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, success, detail]() {
          if (!self) {return;}
          self->teach_request_pending_.store(false);
          self->setTeachStatus(
            success ? QStringLiteral("已进入增加动作模式，旧轨迹预览已清除；真机未运动。") :
            QStringLiteral("清除旧动作预览失败：%1").arg(detail),
            success ? QStringLiteral(
              "Add-action mode is active and the old trajectory preview was cleared; "
              "hardware did not move.") :
            QStringLiteral("Failed to clear the old action preview: %1").arg(detail),
            !success);
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::loadSelectedShapeParameters()
{
  editing_shape_action_name_.clear();
  shape_editor_new_mode_ = false;
  selected_legacy_shape_action_ = false;
  if (!teach_action_combo_ || teach_action_combo_->currentIndex() < 0) {
    shape_editor_new_mode_ = true;
    resetNewActionParameters();
    retranslateUi();
    return;
  }
  if (selectedTeachAction().isEmpty()) {
    shape_editor_new_mode_ = true;
    resetNewActionParameters();
    teach_action_mode_tabs_->setCurrentIndex(0);
    retranslateUi();
    return;
  }

  const QString metadata_text = teach_action_combo_->currentData(
    Qt::UserRole + 1).toString().trimmed();
  const QJsonDocument document = QJsonDocument::fromJson(metadata_text.toUtf8());
  if (!document.isObject()) {
    retranslateUi();
    return;
  }
  const QJsonObject metadata = document.object();
  const QJsonObject pose = metadata.value(QStringLiteral("pose")).toObject();
  const QJsonArray position = pose.value(QStringLiteral("position")).toArray();
  const QJsonArray orientation = pose.value(QStringLiteral("orientation")).toArray();
  const QString shape = metadata.value(QStringLiteral("shape")).toString();
  const int shape_index = teach_shape_combo_->findData(shape);
  if (metadata.value(QStringLiteral("kind")).toString() != QStringLiteral("shape") ||
    shape_index < 0 || position.size() != 3 || orientation.size() != 4)
  {
    const QString action_name = selectedTeachAction();
    selected_legacy_shape_action_ =
      action_name.startsWith(QStringLiteral("形状包_")) ||
      action_name.startsWith(QStringLiteral("字符包_")) ||
      action_name.startsWith(QStringLiteral("简笔画_"));
    if (selected_legacy_shape_action_) {
      int inferred_index = -1;
      QString inferred_text;
      if (action_name.startsWith(QStringLiteral("字符包_"))) {
        inferred_index = teach_shape_combo_->findData(QStringLiteral("text"));
        inferred_text = action_name.mid(QStringLiteral("字符包_").size());
      } else if (action_name.startsWith(QStringLiteral("简笔画_"))) {
        inferred_index = teach_shape_combo_->findData(QStringLiteral("image"));
      } else {
        const QMap<QString, QString> shapes = {
          {QStringLiteral("矩形"), QStringLiteral("rectangle")},
          {QStringLiteral("三角形"), QStringLiteral("triangle")},
          {QStringLiteral("圆形"), QStringLiteral("circle")},
          {QStringLiteral("五角星"), QStringLiteral("star")},
          {QStringLiteral("心形"), QStringLiteral("heart")}};
        inferred_index = teach_shape_combo_->findData(
          shapes.value(action_name.mid(QStringLiteral("形状包_").size())));
      }
      if (inferred_index >= 0) {
        const QSignalBlocker shape_blocker(teach_shape_combo_);
        const QSignalBlocker source_blocker(teach_shape_source_edit_);
        teach_shape_combo_->setCurrentIndex(inferred_index);
        if (!inferred_text.isEmpty()) {teach_shape_source_edit_->setText(inferred_text);}
        updateShapePreview();
      }
    }
    teach_action_mode_tabs_->setCurrentIndex(0);
    retranslateUi();
    return;
  }

  {
    const QSignalBlocker shape_blocker(teach_shape_combo_);
    const QSignalBlocker source_blocker(teach_shape_source_edit_);
    teach_shape_combo_->setCurrentIndex(shape_index);
    const QString source = metadata.value(QStringLiteral("source")).toString();
    teach_shape_source_edit_->setText(
      shape == QStringLiteral("text") ? source : QString());
    teach_shape_image_path_ =
      shape == QStringLiteral("image") ? source : QString();
  }
  shape_pose_dirty_ = false;
  updateShapePoseControls(
    position.at(0).toDouble(), position.at(1).toDouble(), position.at(2).toDouble(),
    orientation.at(0).toDouble(), orientation.at(1).toDouble(),
    orientation.at(2).toDouble(), orientation.at(3).toDouble(),
    metadata.value(QStringLiteral("width")).toDouble(),
    metadata.value(QStringLiteral("height")).toDouble(),
    metadata.value(QStringLiteral("pen_length_m")).toDouble(),
    metadata.value(QStringLiteral("pen_lift_m")).toDouble());
  shape_pose_dirty_ = false;
  shape_reachability_available_ = false;
  editing_shape_action_name_ = selectedTeachAction();
  selected_legacy_shape_action_ = false;
  shape_marker_visible_.store(true);
  updateShapePreview();
  teach_action_mode_tabs_->setCurrentIndex(0);
  retranslateUi();
  if (initialized_) {configureShapeMarker(true, false, false, true);}
}

QString DemoPanel::selectedTeachSequence() const
{
  if (!teach_sequence_combo_ || teach_sequence_combo_->currentIndex() < 0) {
    return QString();
  }
  return teach_sequence_combo_->currentText().trimmed();
}

void DemoPanel::loadSelectedTeachSequence()
{
  teach_sequence_list_->clear();
  if (teach_sequence_combo_->currentIndex() < 0) {return;}
  for (const QString & action_name :
    teach_sequence_combo_->currentData().toStringList())
  {
    auto * item = new QListWidgetItem(action_name, teach_sequence_list_);
    item->setData(Qt::UserRole, action_name);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Checked);
  }
  updateTeachSequenceNumbering();
}

void DemoPanel::updateTeachSequenceNumbering()
{
  for (int index = 0; index < teach_sequence_list_->count(); ++index) {
    auto * item = teach_sequence_list_->item(index);
    const QString action_name = item->data(Qt::UserRole).toString();
    item->setText(QStringLiteral("%1.  %2").arg(index + 1).arg(action_name));
  }
}

void DemoPanel::updateHardwareReplayProgress(
  const QString & action_name, const QString & sequence_name,
  int action_index, int total_actions, double progress, bool active,
  int state, const QString & state_message)
{
  const double normalized = std::clamp(progress, 0.0, 1.0);
  const int current_value = static_cast<int>(std::lround(normalized * 1000.0));
  const int safe_total = std::max(1, total_actions);
  const int safe_index = std::clamp(action_index, 1, safe_total);
  const double overall = total_actions > 0 ?
    (static_cast<double>(safe_index - 1) + normalized) / safe_total : normalized;
  const int overall_value = static_cast<int>(std::lround(overall * 1000.0));

  teach_hardware_action_progress_bar_->setValue(current_value);
  teach_hardware_action_progress_bar_->setFormat(
    QStringLiteral("%1%").arg(normalized * 100.0, 0, 'f', 1));
  teach_hardware_overall_progress_bar_->setValue(overall_value);
  teach_hardware_overall_progress_bar_->setFormat(
    QStringLiteral("%1%").arg(overall * 100.0, 0, 'f', 1));
  teach_sequence_hardware_action_progress_bar_->setValue(current_value);
  teach_sequence_hardware_action_progress_bar_->setFormat(
    QStringLiteral("%1%").arg(normalized * 100.0, 0, 'f', 1));
  teach_sequence_hardware_overall_progress_bar_->setValue(overall_value);
  teach_sequence_hardware_overall_progress_bar_->setFormat(
    QStringLiteral("%1%").arg(overall * 100.0, 0, 'f', 1));

  if (!sequence_name.isEmpty() && total_actions > 0) {
    int selected_index = 0;
    for (int row = 0; row < teach_sequence_list_->count(); ++row) {
      auto * item = teach_sequence_list_->item(row);
      QString original_name = item->data(Qt::UserRole).toString();
      if (original_name.isEmpty()) {
        original_name = item->text();
        item->setData(Qt::UserRole, original_name);
      }
      if (item->checkState() == Qt::Checked) {
        ++selected_index;
        const double item_progress = selected_index < safe_index ? 1.0 :
          selected_index == safe_index ? normalized : 0.0;
        item->setText(
          QStringLiteral("[%1%] %2.  %3")
          .arg(item_progress * 100.0, 0, 'f', 1).arg(row + 1).arg(original_name));
      } else {
        item->setText(QStringLiteral("%1.  %2").arg(row + 1).arg(original_name));
      }
    }
  }

  if (!active && state == TEACH_FAULT && !action_name.isEmpty()) {
    teach_hardware_action_progress_label_->setText(localized(
      QStringLiteral("上一真机动作未通过完成检查：%1；%2")
      .arg(action_name, state_message),
      QStringLiteral("Last robot action failed its completion check: %1; %2")
      .arg(action_name, state_message)));
    teach_hardware_action_progress_label_->setStyleSheet(
      QStringLiteral("color: #c43b32;"));
  } else if (active) {
    teach_hardware_action_progress_label_->setText(localized(
      QStringLiteral("当前真机动作 %1/%2：%3")
      .arg(safe_index).arg(safe_total).arg(action_name),
      QStringLiteral("Current robot action %1/%2: %3")
      .arg(safe_index).arg(safe_total).arg(action_name)));
    teach_hardware_action_progress_label_->setStyleSheet(QString());
  } else if (normalized >= 0.999 && !action_name.isEmpty()) {
    teach_hardware_action_progress_label_->setText(localized(
      QStringLiteral("上一真机动作已完成：%1").arg(action_name),
      QStringLiteral("Last robot action completed: %1").arg(action_name)));
    teach_hardware_action_progress_label_->setStyleSheet(QString());
  } else if (normalized > 0.0 && !action_name.isEmpty()) {
    teach_hardware_action_progress_label_->setText(localized(
      QStringLiteral("真机动作已在 %1% 停止：%2")
      .arg(normalized * 100.0, 0, 'f', 1).arg(action_name),
      QStringLiteral("Robot action stopped at %1%: %2")
      .arg(normalized * 100.0, 0, 'f', 1).arg(action_name)));
    teach_hardware_action_progress_label_->setStyleSheet(
      QStringLiteral("color: #c43b32;"));
  } else {
    teach_hardware_action_progress_label_->setText(localized(
      QStringLiteral("当前真机动作进度：等待回放"),
      QStringLiteral("Current robot action: waiting for replay")));
    teach_hardware_action_progress_label_->setStyleSheet(QString());
  }

  if (!sequence_name.isEmpty() || total_actions > 1) {
    teach_hardware_overall_progress_label_->setText(localized(
      QStringLiteral("动作组“%1”总进度：第 %2/%3 个动作")
      .arg(sequence_name).arg(safe_index).arg(safe_total),
      QStringLiteral("Sequence “%1” overall: action %2/%3")
      .arg(sequence_name).arg(safe_index).arg(safe_total)));
  } else {
    teach_hardware_overall_progress_label_->setText(localized(
      QStringLiteral("本次真机回放总进度"),
      QStringLiteral("Overall robot replay progress")));
  }
  teach_sequence_hardware_action_progress_label_->setText(
    teach_hardware_action_progress_label_->text());
  teach_sequence_hardware_action_progress_label_->setStyleSheet(
    teach_hardware_action_progress_label_->styleSheet());
  teach_sequence_hardware_overall_progress_label_->setText(
    teach_hardware_overall_progress_label_->text());
}

void DemoPanel::setTeachStatus(const QString & zh, const QString & en, bool error)
{
  teach_status_zh_ = zh;
  teach_status_en_ = en;
  teach_status_error_ = error;
  teach_status_label_->setText(localized(teach_status_zh_, teach_status_en_));
  teach_status_label_->setStyleSheet(
    teach_status_error_ ? QStringLiteral("color: #c43b32;") : QString());
}

void DemoPanel::refreshTeachActions()
{
  if (!teach_list_client_ || !teach_list_client_->service_is_ready()) {
    setTeachStatus(
      QStringLiteral("示教服务尚未就绪；确认组合 launch 的 use_teach:=true。"),
      QStringLiteral("Teaching service is not ready; confirm use_teach:=true."), true);
    return;
  }
  teach_refresh_button_->setEnabled(false);
  auto request = std::make_shared<ListActionGroups::Request>();
  QPointer<DemoPanel> self(this);
  teach_list_client_->async_send_request(
    request, [self](rclcpp::Client<ListActionGroups>::SharedFuture future) {
      QStringList names;
      QStringList descriptions;
      QStringList metadata_json;
      QString selected;
      QString detail;
      bool success = false;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {
          for (const auto & name : response->names) {
            names.push_back(QString::fromStdString(name));
          }
          for (const auto & description : response->descriptions) {
            descriptions.push_back(QString::fromStdString(description));
          }
          for (const auto & metadata : response->metadata_json) {
            metadata_json.push_back(QString::fromStdString(metadata));
          }
          selected = QString::fromStdString(response->selected_name);
          detail = QString::fromStdString(response->message);
        }
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, names, descriptions, metadata_json, selected, detail, success]() {
          if (!self) {return;}
          const QString previous = self->selectedTeachAction();
          {
            const QSignalBlocker blocker(self->teach_action_combo_);
            self->teach_action_combo_->clear();
            self->teach_action_combo_->addItem(
              self->localized(QStringLiteral("＋ 增加动作"),
              QStringLiteral("+ Add action")), QString());
            self->teach_action_combo_->setItemData(
              0, QStringLiteral("{\"mode\":\"new\"}"), Qt::UserRole + 1);
            for (int index = 0; index < names.size(); ++index) {
              const QString description =
                index < descriptions.size() ? descriptions[index] : QString();
              const QString metadata =
                index < metadata_json.size() ? metadata_json[index] : QStringLiteral("{}");
              self->teach_action_combo_->addItem(names[index], names[index]);
              const int combo_index = index + 1;
              self->teach_action_combo_->setItemData(
                combo_index, description, Qt::ToolTipRole);
              self->teach_action_combo_->setItemData(
                combo_index, metadata, Qt::UserRole + 1);
            }
            const QString wanted = selected.isEmpty() ? previous : selected;
            const int wanted_index = self->teach_action_combo_->findData(wanted);
            if (wanted_index >= 0) {
              self->teach_action_combo_->setCurrentIndex(wanted_index);
            }
          }
          self->loadSelectedShapeParameters();
          self->teach_slider_->setValue(0);
          self->setTeachStatus(
            success ? QStringLiteral("动作列表已刷新：%1 个。可播放动画或拖动进度条。")
            .arg(names.size()) : QStringLiteral("刷新动作列表失败：%1").arg(detail),
            success ? QStringLiteral("Action list refreshed: %1. Play it or drag the timeline.")
            .arg(names.size()) : QStringLiteral("Failed to refresh actions: %1").arg(detail),
            !success);
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::checkShapeReachability()
{
  if (shape_reachability_request_pending_.load()) {
    return;
  }
  if (!teach_shape_reachability_client_ ||
    !teach_shape_reachability_client_->service_is_ready())
  {
    setTeachStatus(
      QStringLiteral("笔尖轨迹预检服务尚未就绪。"),
      QStringLiteral("Pen-tip reachability service is not ready."), true);
    return;
  }
  if (shape_reachability_request_pending_.exchange(true)) {
    return;
  }
  updateReadiness();
  teach_shape_reachability_label_->setText(localized(
      QStringLiteral("正在逐点检查 IK、碰撞和关节余量…"),
      QStringLiteral("Checking IK, collision, and joint margin point by point…")));
  teach_shape_reachability_label_->setStyleSheet(
    QStringLiteral("color: #d08020; font-weight: bold;"));
  auto request = std::make_shared<CheckShapeReachability::Request>();
  QPointer<DemoPanel> self(this);
  teach_shape_reachability_client_->async_send_request(
    request, [self](rclcpp::Client<CheckShapeReachability>::SharedFuture future) {
      bool completed = false;
      bool feasible = false;
      bool has_suggestion = false;
      QString detail;
      double suggestion_distance = 0.0;
      geometry_msgs::msg::Pose suggested_pose;
      try {
        const auto response = future.get();
        completed = response && response->success;
        if (response) {
          feasible = response->feasible;
          has_suggestion = response->has_suggestion;
          suggestion_distance = response->suggestion_distance_m;
          suggested_pose = response->suggested_pose;
          detail = QString::fromStdString(response->message);
        }
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, completed, feasible, has_suggestion, suggestion_distance,
          suggested_pose, detail]() {
          if (!self) {return;}
          self->shape_reachability_request_pending_.store(false);
          self->setTeachStatus(
            completed ? (feasible ? QStringLiteral(
              "笔尖轨迹预检通过；绿色/黄色段可规划，真机未运动。%1")
              .arg(detail.isEmpty() ? QString() : QStringLiteral("（%1）").arg(detail)) :
              QStringLiteral("笔尖轨迹存在不可行点：%1。红色=无 IK，紫色=碰撞（包括禁区），请调整位置/方向后重新预检。")
              .arg(detail.isEmpty() ? QStringLiteral("请查看红色/紫色段") : detail)) :
            QStringLiteral("笔尖轨迹预检失败：%1").arg(detail),
            completed ? (feasible ? QStringLiteral(
              "Pen-tip preflight passed; green/yellow segments are plannable. Hardware did not move.%1")
              .arg(detail.isEmpty() ? QString() : QStringLiteral(" (%1)").arg(detail)) :
              QStringLiteral("Pen-tip path is infeasible: %1. Red=no IK; purple=collision (including forbidden zones). Adjust pose and run preflight again.")
              .arg(detail.isEmpty() ? QStringLiteral("inspect red/purple segments") : detail)) :
            QStringLiteral("Pen-tip preflight failed: %1").arg(detail),
            !completed || !feasible);
          if (completed && !feasible && has_suggestion) {
            const bool apply = QMessageBox::question(
              self, self->localized(QStringLiteral("发现附近可达位置"),
                QStringLiteral("Nearby reachable pose found")),
              self->localized(
                QStringLiteral("搜索到距离当前画布 %1 mm 的候选位置。是否把画布移动到该位置？\n"
                  "只更新 RViz 位置，不会驱动真机；移动后请再次预检。")
                .arg(suggestion_distance * 1000.0, 0, 'f', 1),
                QStringLiteral("A candidate pose %1 mm from the current canvas was found. "
                  "Apply it? This only updates RViz; run preflight again afterward.")
                .arg(suggestion_distance * 1000.0, 0, 'f', 1))) == QMessageBox::Yes;
            if (apply) {
              self->teach_shape_x_spin_->setValue(suggested_pose.position.x);
              self->teach_shape_y_spin_->setValue(suggested_pose.position.y);
              self->teach_shape_z_spin_->setValue(suggested_pose.position.z);
              self->configureShapeMarker(true, false, true, true);
            }
          }
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::createShapeAction()
{
  if (!teach_shape_client_ || !teach_shape_client_->service_is_ready() ||
    teach_shape_combo_->currentIndex() < 0)
  {
    setTeachStatus(
      QStringLiteral("形状动作生成服务尚未就绪。"),
      QStringLiteral("The shape-action generation service is not ready."), true);
    return;
  }
  const QString shape = teach_shape_combo_->currentData().toString();
  if (shape == QStringLiteral("freehand")) {
    setTeachStatus(
      QStringLiteral("当前为“自由拖动录制”，请使用下方的“开始拖动录制”。"),
      QStringLiteral(
        "Freehand recording is selected; use Start Drag Recording below."));
    return;
  }
  const QString source = selectedDrawingSource();
  if ((shape == QStringLiteral("text") || shape == QStringLiteral("image")) &&
    source.isEmpty())
  {
    setTeachStatus(
      shape == QStringLiteral("text") ? QStringLiteral("请输入要绘制的英文、数字或符号。") :
      QStringLiteral("请先选择一张图片。"),
      shape == QStringLiteral("text") ? QStringLiteral(
        "Enter English letters, digits, or symbols to draw.") :
      QStringLiteral("Select an image first."), true);
    return;
  }
  const QString display_name = teach_shape_combo_->currentText();
  const QMap<QString, QString> stored_shape_names = {
    {QStringLiteral("rectangle"), QStringLiteral("矩形")},
    {QStringLiteral("triangle"), QStringLiteral("三角形")},
    {QStringLiteral("circle"), QStringLiteral("圆形")},
    {QStringLiteral("star"), QStringLiteral("五角星")},
    {QStringLiteral("heart"), QStringLiteral("心形")}};
  QString action_name;
  if (shape == QStringLiteral("text")) {
    QString safe_source = source.left(32);
    safe_source.replace(QChar('/'), QChar('_'));
    safe_source.replace(QChar('\\'), QChar('_'));
    action_name = QStringLiteral("字符包_%1").arg(safe_source);
  } else if (shape == QStringLiteral("image")) {
    action_name = QStringLiteral("简笔画_%1").arg(QFileInfo(source).completeBaseName().left(36));
  } else {
    action_name = QStringLiteral("形状包_%1").arg(stored_shape_names.value(shape));
  }
  const bool editing_existing = !editing_shape_action_name_.isEmpty();
  if (editing_existing) {action_name = editing_shape_action_name_;}
  const bool overwrite = teach_action_combo_->findData(action_name) >= 0;
  if (overwrite && QMessageBox::question(
      this,
      localized(
        editing_existing ? QStringLiteral("保存动作参数修改") :
        QStringLiteral("重新生成形状动作"),
        editing_existing ? QStringLiteral("Save action parameter changes") :
        QStringLiteral("Regenerate shape action")),
      localized(
        editing_existing ?
        QStringLiteral("是否使用当前修改的参数重新规划并覆盖动作包“%1”？真机不会运动。")
        .arg(action_name) :
        QStringLiteral("动作“%1”已存在。是否使用当前姿态重新规划并替换？").arg(action_name),
        editing_existing ?
        QStringLiteral(
          "Replan and replace action pack “%1” using the edited parameters? Hardware will not move.")
        .arg(action_name) :
        QStringLiteral(
          "Action “%1” exists. Replan from the current pose and replace it?").arg(action_name)),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
  {
    return;
  }
  auto request = std::make_shared<CreateShapeAction::Request>();
  request->shape = shape.toStdString();
  request->source = source.toStdString();
  request->target_name = editing_existing ? action_name.toStdString() : std::string();
  request->overwrite = overwrite;
  teach_request_pending_.store(true);
  setTeachStatus(
    QStringLiteral("正在用 MoveIt 规划“%1”动作；生成阶段真机不会运动…").arg(display_name),
    QStringLiteral(
      "MoveIt is planning the “%1” action; hardware will not move while generating…")
    .arg(display_name));
  QPointer<DemoPanel> self(this);
  teach_shape_client_->async_send_request(
    request, [self](rclcpp::Client<CreateShapeAction>::SharedFuture future) {
      bool success = false;
      QString detail;
      QString name;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {
          detail = QString::fromStdString(response->message);
          name = QString::fromStdString(response->name);
        }
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, success, detail, name]() {
          if (!self) {return;}
          self->teach_request_pending_.store(false);
          if (success) {
            self->editing_shape_action_name_ = name;
            self->shape_editor_new_mode_ = false;
          }
          self->setTeachStatus(
            success ? QStringLiteral(
              "形状动作“%1”已生成并加入示教动作列表；真机未运动。").arg(name) :
            QStringLiteral("形状动作生成失败：%1").arg(detail),
            success ? QStringLiteral(
              "Shape action “%1” was added to the teaching library; hardware did not move.")
            .arg(name) : QStringLiteral("Shape action generation failed: %1").arg(detail),
            !success);
          if (success) {self->refreshTeachActions();}
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::selectShapeImage()
{
  const QString selected = QFileDialog::getOpenFileName(
    this,
    localized(QStringLiteral("选择要转换为简笔画的图片"), QStringLiteral("Select image to trace")),
    teach_shape_image_path_.isEmpty() ? QStandardPaths::writableLocation(
      QStandardPaths::PicturesLocation) : QFileInfo(teach_shape_image_path_).absolutePath(),
    localized(
      QStringLiteral("图片 (*.png *.jpg *.jpeg *.bmp *.webp);;所有文件 (*)"),
      QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.webp);;All files (*)")));
  if (selected.isEmpty()) {return;}
  QImage image(selected);
  if (image.isNull()) {
    setTeachStatus(
      QStringLiteral("无法读取所选图片：%1").arg(selected),
      QStringLiteral("Could not read the selected image: %1").arg(selected), true);
    return;
  }
  teach_shape_image_path_ = QFileInfo(selected).absoluteFilePath();
  shape_marker_visible_.store(true);
  updateShapePreview();
  retranslateUi();
  if (initialized_) {configureShapeMarker(true, false, true);}
}

void DemoPanel::configureShapeMarker(
  bool visible, bool reset_pose, bool report_success, bool set_pose)
{
  if (!teach_shape_marker_client_ || !teach_shape_marker_client_->service_is_ready() ||
    teach_shape_combo_->currentIndex() < 0)
  {
    if (report_success) {
      setTeachStatus(
        QStringLiteral("形状空间定位服务尚未就绪。"),
        QStringLiteral("The shape-pose service is not ready."), true);
    }
    return;
  }
  auto request = std::make_shared<ConfigureShapeMarker::Request>();
  const QString selected_shape = teach_shape_combo_->currentData().toString();
  const bool requested_visible = visible && selected_shape != QStringLiteral("freehand");
  request->shape = selected_shape.toStdString();
  request->source = selectedDrawingSource().toStdString();
  request->visible = requested_visible;
  request->reset_pose = reset_pose;
  request->set_pose = set_pose;
  request->x = teach_shape_x_spin_->value();
  request->y = teach_shape_y_spin_->value();
  request->z = teach_shape_z_spin_->value();
  const double radians_per_degree = std::acos(-1.0) / 180.0;
  request->roll = teach_shape_roll_spin_->value() * radians_per_degree;
  request->pitch = teach_shape_pitch_spin_->value() * radians_per_degree;
  request->yaw = teach_shape_yaw_spin_->value() * radians_per_degree;
  request->set_size = set_pose;
  request->width = teach_shape_width_spin_->value();
  request->height = teach_shape_height_spin_->value();
  request->set_pen = set_pose;
  request->pen_length = teach_shape_pen_length_spin_->value() / 1000.0;
  request->pen_lift = teach_shape_pen_lift_spin_->value() / 1000.0;
  QPointer<DemoPanel> self(this);
  teach_shape_marker_client_->async_send_request(
    request,
    [self, requested_visible, reset_pose, report_success, set_pose](
      rclcpp::Client<ConfigureShapeMarker>::SharedFuture future)
    {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, success, detail, requested_visible, reset_pose, report_success, set_pose]() {
          if (!self) {return;}
          if (success) {
            self->shape_marker_visible_.store(requested_visible);
            if (reset_pose || set_pose) {self->shape_pose_dirty_ = false;}
          }
          if (report_success || !success) {
            self->setTeachStatus(
              success ? (reset_pose ? QStringLiteral(
                "形状定位已恢复默认位姿；可在 RViz 中继续拖动。") : set_pose ?
                QStringLiteral("已应用选中形状的位置、方向和大小；真机未运动。") :
                (requested_visible ? QStringLiteral("已显示形状空间定位标记。") :
                QStringLiteral("已隐藏形状空间定位标记。"))) :
              QStringLiteral("设置形状空间定位失败：%1").arg(detail),
              success ? (reset_pose ? QStringLiteral(
                "Shape pose reset; it can be dragged again in RViz.") : set_pose ?
                QStringLiteral(
                  "Selected shape position, orientation and size applied; hardware did not move.") :
                (requested_visible ? QStringLiteral("Shape pose marker shown.") :
                QStringLiteral("Shape pose marker hidden."))) :
              QStringLiteral("Failed to configure shape pose: %1").arg(detail),
              !success);
          }
          self->retranslateUi();
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::toggleShapeMarker()
{
  configureShapeMarker(!shape_marker_visible_.load(), false);
}

void DemoPanel::resetShapeMarker()
{
  configureShapeMarker(true, true);
}

void DemoPanel::applyShapePose()
{
  configureShapeMarker(true, false, true, true);
}

void DemoPanel::setTraceLineWidth()
{
  if (!teach_trace_client_ || !teach_trace_client_->service_is_ready()) {
    setTeachStatus(
      QStringLiteral("轨迹显示服务尚未就绪。"),
      QStringLiteral("The trajectory display service is not ready."), true);
    return;
  }
  auto request = std::make_shared<ConfigureTrace::Request>();
  request->set_line_width = true;
  request->line_width_m = teach_trace_width_spin_->value() / 1000.0;
  const auto generation = trace_width_request_generation_.fetch_add(1) + 1;
  trace_width_request_pending_.store(true);
  QPointer<DemoPanel> self(this);
  teach_trace_client_->async_send_request(
    request, [self, generation](rclcpp::Client<ConfigureTrace>::SharedFuture future) {
      bool success = false;
      QString detail;
      double line_width_mm = 4.0;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {
          detail = QString::fromStdString(response->message);
          line_width_mm = response->line_width_m * 1000.0;
        }
      } catch (const std::exception & error) {
        detail = QString::fromUtf8(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, generation, success, detail, line_width_mm]() {
          if (!self) {return;}
          if (generation != self->trace_width_request_generation_.load()) {return;}
          self->trace_width_request_pending_.store(false);
          if (success) {
            const QSignalBlocker blocker(self->teach_trace_width_spin_);
            self->teach_trace_width_spin_->setValue(line_width_mm);
          }
          self->setTeachStatus(
            success ? QStringLiteral("轨迹粗细已更新为 %1 mm；不会驱动真机。")
              .arg(line_width_mm, 0, 'f', 1) :
              QStringLiteral("轨迹粗细更新失败：%1").arg(detail),
            success ? QStringLiteral("Trajectory width updated to %1 mm; hardware unchanged.")
              .arg(line_width_mm, 0, 'f', 1) :
              QStringLiteral("Failed to update trajectory width: %1").arg(detail),
            !success);
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::updateShapePoseControls(
  double x, double y, double z,
  double qx, double qy, double qz, double qw,
  double width, double height, double pen_length_m, double pen_lift_m)
{
  // Status arrives twice a second. Do not overwrite a value the user is still
  // preparing before they press Apply.
  if (shape_pose_dirty_) {return;}
  shape_pose_syncing_ = true;
  teach_shape_x_spin_->setValue(x);
  teach_shape_y_spin_->setValue(y);
  teach_shape_z_spin_->setValue(z);
  const double sin_roll = 2.0 * (qw * qx + qy * qz);
  const double cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double roll = std::atan2(sin_roll, cos_roll);
  const double sin_pitch = std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
  const double pitch = std::asin(sin_pitch);
  const double sin_yaw = 2.0 * (qw * qz + qx * qy);
  const double cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double yaw = std::atan2(sin_yaw, cos_yaw);
  const double degrees_per_radian = 180.0 / std::acos(-1.0);
  teach_shape_roll_spin_->setValue(roll * degrees_per_radian);
  teach_shape_pitch_spin_->setValue(pitch * degrees_per_radian);
  teach_shape_yaw_spin_->setValue(yaw * degrees_per_radian);
  teach_shape_width_spin_->setValue(width);
  teach_shape_height_spin_->setValue(height);
  teach_shape_pen_length_spin_->setValue(pen_length_m * 1000.0);
  teach_shape_pen_lift_spin_->setValue(pen_lift_m * 1000.0);
  shape_pose_syncing_ = false;
}

void DemoPanel::updateShapePreview()
{
  const QString shape = teach_shape_combo_->currentData().toString();
  QPixmap preview(420, 96);
  preview.fill(QColor(QStringLiteral("#303238")));
  QPainter painter(&preview);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.translate(preview.width() * 0.5, preview.height() * 0.5);
  QPen pen(QColor(QStringLiteral("#ff9414")), 4.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  painter.setPen(pen);
  painter.setBrush(QColor(255, 148, 20, 80));
  QPainterPath path;
  bool custom_preview = false;
  if (shape == QStringLiteral("rectangle")) {
    path.addRoundedRect(QRectF(-82.0, -30.0, 164.0, 60.0), 2.0, 2.0);
  } else if (shape == QStringLiteral("triangle")) {
    path.moveTo(0.0, -34.0);
    path.lineTo(-78.0, 31.0);
    path.lineTo(78.0, 31.0);
    path.closeSubpath();
  } else if (shape == QStringLiteral("circle")) {
    path.addEllipse(QRectF(-48.0, -36.0, 96.0, 72.0));
  } else if (shape == QStringLiteral("star")) {
    for (int index = 0; index < 10; ++index) {
      const double radius = index % 2 == 0 ? 38.0 : 17.0;
      const double angle = -std::acos(-1.0) * 0.5 + index * std::acos(-1.0) / 5.0;
      const QPointF point(radius * std::cos(angle), radius * std::sin(angle));
      if (index == 0) {path.moveTo(point);} else {path.lineTo(point);}
    }
    path.closeSubpath();
  } else if (shape == QStringLiteral("heart")) {
    path.moveTo(0.0, 34.0);
    path.cubicTo(-80.0, -8.0, -50.0, -48.0, 0.0, -18.0);
    path.cubicTo(50.0, -48.0, 80.0, -8.0, 0.0, 34.0);
    path.closeSubpath();
  } else if (shape == QStringLiteral("text")) {
    custom_preview = true;
    painter.resetTransform();
    painter.setPen(pen);
    QFont font = painter.font();
    font.setPointSize(38);
    font.setBold(false);
    painter.setFont(font);
    painter.drawText(
      QRect(12, 24, preview.width() - 24, preview.height() - 28),
      Qt::AlignCenter, teach_shape_source_edit_->text());
  } else if (shape == QStringLiteral("image")) {
    custom_preview = true;
    painter.resetTransform();
    const QImage source(teach_shape_image_path_);
    if (!source.isNull()) {
      const QImage scaled = source.scaled(
        preview.width() - 28, preview.height() - 30,
        Qt::KeepAspectRatio, Qt::SmoothTransformation);
      painter.setOpacity(0.85);
      painter.drawImage(
        QPoint((preview.width() - scaled.width()) / 2,
        26 + (preview.height() - 26 - scaled.height()) / 2), scaled);
      painter.setOpacity(1.0);
    }
  } else {
    painter.setBrush(Qt::NoBrush);
    path.moveTo(-100.0, 15.0);
    path.cubicTo(-65.0, -38.0, -25.0, 44.0, 8.0, -5.0);
    path.cubicTo(42.0, -50.0, 70.0, 35.0, 105.0, -15.0);
  }
  if (!custom_preview) {painter.drawPath(path);}
  painter.resetTransform();
  painter.setPen(QColor(QStringLiteral("#f3f3f3")));
  painter.drawText(
    QRect(8, 5, preview.width() - 16, 24), Qt::AlignLeft | Qt::AlignVCenter,
    shape == QStringLiteral("freehand") ? localized(
      QStringLiteral("不添加预设图形：自由拖动录制"),
      QStringLiteral("No preset shape: freehand recording")) : shape == QStringLiteral("image") &&
    teach_shape_image_path_.isEmpty() ? localized(
      QStringLiteral("请选择图片"), QStringLiteral("Select an image")) :
    localized(
      QStringLiteral("落笔内容正视预览（不受三维视角影响）"),
      QStringLiteral("Front preview of pen-down content (independent of 3D view)")));
  teach_shape_preview_label_->setPixmap(preview);
}

void DemoPanel::copyTeachAction()
{
  const QString source_name = selectedTeachAction();
  if (source_name.isEmpty() || !teach_copy_client_ ||
    !teach_copy_client_->service_is_ready())
  {
    setTeachStatus(
      QStringLiteral("请选择动作包，并确认复制服务已就绪。"),
      QStringLiteral("Select an action pack and confirm the copy service is ready."), true);
    return;
  }
  bool accepted = false;
  const QString new_name = QInputDialog::getText(
    this, localized(QStringLiteral("复制动作包"), QStringLiteral("Copy action pack")),
    localized(QStringLiteral("副本名称："), QStringLiteral("Copy name:")),
    QLineEdit::Normal,
    localized(QStringLiteral("%1 副本").arg(source_name),
    QStringLiteral("%1 copy").arg(source_name)), &accepted).trimmed();
  if (!accepted || new_name.isEmpty()) {return;}

  auto request = std::make_shared<CopyActionGroup::Request>();
  request->source_name = source_name.toStdString();
  request->new_name = new_name.toStdString();
  QPointer<DemoPanel> self(this);
  teach_copy_client_->async_send_request(
    request, [self, source_name, new_name](
      rclcpp::Client<CopyActionGroup>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, source_name, new_name, success, detail]() {
          if (!self) {return;}
          self->setTeachStatus(
            success ? QStringLiteral("已将动作包“%1”复制为“%2”。")
            .arg(source_name, new_name) :
            QStringLiteral("复制动作包失败：%1").arg(detail),
            success ? QStringLiteral("Action pack “%1” copied to “%2”.")
            .arg(source_name, new_name) :
            QStringLiteral("Failed to copy action pack: %1").arg(detail), !success);
          if (success) {
            self->refreshTeachActions();
          }
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::renameTeachAction()
{
  const QString old_name = selectedTeachAction();
  if (old_name.isEmpty() || !teach_rename_client_ ||
    !teach_rename_client_->service_is_ready())
  {
    setTeachStatus(
      QStringLiteral("请选择动作，并确认重命名服务已就绪。"),
      QStringLiteral("Select an action and confirm the rename service is ready."), true);
    return;
  }
  bool accepted = false;
  const QString new_name = QInputDialog::getText(
    this, localized(QStringLiteral("重命名动作"), QStringLiteral("Rename action")),
    localized(QStringLiteral("新名称："), QStringLiteral("New name:")),
    QLineEdit::Normal, old_name, &accepted).trimmed();
  if (!accepted || new_name.isEmpty() || new_name == old_name) {return;}

  auto request = std::make_shared<RenameActionGroup::Request>();
  request->old_name = old_name.toStdString();
  request->new_name = new_name.toStdString();
  QPointer<DemoPanel> self(this);
  teach_rename_client_->async_send_request(
    request, [self, new_name](rclcpp::Client<RenameActionGroup>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, new_name, success, detail]() {
          if (!self) {return;}
          self->setTeachStatus(
            success ? QStringLiteral("动作已重命名为“%1”。").arg(new_name) :
            QStringLiteral("重命名失败：%1").arg(detail),
            success ? QStringLiteral("Action renamed to “%1”.").arg(new_name) :
            QStringLiteral("Rename failed: %1").arg(detail), !success);
          if (success) {
            self->refreshTeachActions();
            self->refreshTeachSequences();
          }
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::deleteTeachAction()
{
  const QString name = selectedTeachAction();
  if (name.isEmpty() || !teach_delete_client_ ||
    !teach_delete_client_->service_is_ready())
  {
    setTeachStatus(
      QStringLiteral("请选择动作，并确认删除服务已就绪。"),
      QStringLiteral("Select an action and confirm the delete service is ready."), true);
    return;
  }
  if (QMessageBox::question(
      this, localized(QStringLiteral("删除动作包"), QStringLiteral("Delete action pack")),
      localized(
        QStringLiteral("确定永久删除动作包“%1”？此操作无法撤销。").arg(name),
        QStringLiteral("Permanently delete action pack “%1”? This cannot be undone.").arg(name)),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
  {
    return;
  }

  auto request = std::make_shared<DeleteActionGroup::Request>();
  request->name = name.toStdString();
  QPointer<DemoPanel> self(this);
  teach_delete_client_->async_send_request(
    request, [self, name](rclcpp::Client<DeleteActionGroup>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, name, success, detail]() {
          if (!self) {return;}
          self->setTeachStatus(
            success ? QStringLiteral("动作包“%1”已删除。").arg(name) :
            QStringLiteral("删除动作包失败：%1").arg(detail),
            success ? QStringLiteral("Action pack “%1” deleted.").arg(name) :
            QStringLiteral("Failed to delete action pack: %1").arg(detail), !success);
          if (success) {
            self->editing_shape_action_name_.clear();
            self->shape_editor_new_mode_ = true;
            self->refreshTeachActions();
            self->refreshTeachSequences();
          }
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::refreshTeachSequences()
{
  if (!teach_sequence_list_client_ || !teach_sequence_list_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<ListActionSequences::Request>();
  QPointer<DemoPanel> self(this);
  teach_sequence_list_client_->async_send_request(
    request, [self](rclcpp::Client<ListActionSequences>::SharedFuture future) {
      QStringList names;
      QList<QStringList> action_lists;
      QString detail;
      bool success = false;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {
          detail = QString::fromStdString(response->message);
          for (std::size_t index = 0; index < response->names.size(); ++index) {
            names.push_back(QString::fromStdString(response->names[index]));
            QStringList actions;
            if (index < response->action_lists_json.size()) {
              const auto document = QJsonDocument::fromJson(
                QByteArray::fromStdString(response->action_lists_json[index]));
              for (const auto & value : document.array()) {
                actions.push_back(value.toString());
              }
            }
            action_lists.push_back(actions);
          }
        }
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, names, action_lists, detail, success]() {
          if (!self) {return;}
          const QString previous = self->selectedTeachSequence();
          self->teach_sequence_combo_->clear();
          for (int index = 0; index < names.size(); ++index) {
            self->teach_sequence_combo_->addItem(names[index], action_lists[index]);
          }
          const int wanted = self->teach_sequence_combo_->findText(previous);
          if (wanted >= 0) {self->teach_sequence_combo_->setCurrentIndex(wanted);}
          self->loadSelectedTeachSequence();
          if (!success) {
            self->setTeachStatus(
              QStringLiteral("刷新动作组失败：%1").arg(detail),
              QStringLiteral("Failed to refresh sequences: %1").arg(detail), true);
          }
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::addTeachSequenceAction()
{
  const QString action = selectedTeachAction();
  if (!action.isEmpty()) {
    auto * item = new QListWidgetItem(action, teach_sequence_list_);
    item->setData(Qt::UserRole, action);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Checked);
    teach_sequence_list_->setCurrentRow(teach_sequence_list_->count() - 1);
    updateTeachSequenceNumbering();
  }
}

void DemoPanel::removeTeachSequenceAction()
{
  delete teach_sequence_list_->takeItem(teach_sequence_list_->currentRow());
  updateTeachSequenceNumbering();
}

void DemoPanel::moveTeachSequenceActionUp()
{
  const int row = teach_sequence_list_->currentRow();
  if (row <= 0) {return;}
  auto * item = teach_sequence_list_->takeItem(row);
  teach_sequence_list_->insertItem(row - 1, item);
  teach_sequence_list_->setCurrentRow(row - 1);
  updateTeachSequenceNumbering();
}

void DemoPanel::moveTeachSequenceActionDown()
{
  const int row = teach_sequence_list_->currentRow();
  if (row < 0 || row >= teach_sequence_list_->count() - 1) {return;}
  auto * item = teach_sequence_list_->takeItem(row);
  teach_sequence_list_->insertItem(row + 1, item);
  teach_sequence_list_->setCurrentRow(row + 1);
  updateTeachSequenceNumbering();
}

void DemoPanel::saveTeachSequence()
{
  if (teach_sequence_list_->count() == 0 || !teach_sequence_save_client_ ||
    !teach_sequence_save_client_->service_is_ready())
  {
    setTeachStatus(
      QStringLiteral("动作组至少需要一个动作。"),
      QStringLiteral("A sequence needs at least one action."), true);
    return;
  }
  bool accepted = false;
  const QString name = QInputDialog::getText(
    this, localized(QStringLiteral("保存动作组"), QStringLiteral("Save sequence")),
    localized(QStringLiteral("动作组名称："), QStringLiteral("Sequence name:")),
    QLineEdit::Normal, selectedTeachSequence(), &accepted).trimmed();
  if (!accepted || name.isEmpty()) {return;}
  bool overwrite = teach_sequence_combo_->findText(name) >= 0;
  if (overwrite && QMessageBox::question(
      this, localized(QStringLiteral("覆盖动作组"), QStringLiteral("Overwrite sequence")),
      localized(QStringLiteral("动作组“%1”已存在，是否覆盖？").arg(name),
      QStringLiteral("Sequence “%1” exists. Overwrite it?").arg(name)),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
  {
    return;
  }
  auto request = std::make_shared<SaveActionSequence::Request>();
  request->name = name.toStdString();
  request->overwrite = overwrite;
  for (int index = 0; index < teach_sequence_list_->count(); ++index) {
    auto * item = teach_sequence_list_->item(index);
    const QString original_name = item->data(Qt::UserRole).toString();
    request->action_names.push_back(
      (original_name.isEmpty() ? item->text() : original_name).toStdString());
  }
  QPointer<DemoPanel> self(this);
  teach_sequence_save_client_->async_send_request(
    request, [self, name](rclcpp::Client<SaveActionSequence>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {detail = QString::fromLocal8Bit(error.what());}
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, name, success, detail]() {
          if (!self) {return;}
          self->setTeachStatus(
            success ? QStringLiteral("动作组“%1”已保存。").arg(name) :
            QStringLiteral("保存动作组失败：%1").arg(detail),
            success ? QStringLiteral("Sequence “%1” saved.").arg(name) :
            QStringLiteral("Failed to save sequence: %1").arg(detail), !success);
          if (success) {self->refreshTeachSequences();}
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::playTeachSequence()
{
  const QString name = selectedTeachSequence();
  if (teach_sequence_list_->count() == 0 || !teach_sequence_preview_client_ ||
    !teach_sequence_preview_client_->service_is_ready())
  {
    setTeachStatus(
      QStringLiteral("请先向动作组添加动作，并确认 RViz 预览服务已就绪。"),
      QStringLiteral("Add actions to the sequence and confirm RViz preview is ready."), true);
    return;
  }
  auto request = std::make_shared<PreviewActionSequence::Request>();
  request->name = name.toStdString();
  request->progress = -1.0;
  request->overlay = false;
  for (int index = 0; index < teach_sequence_list_->count(); ++index) {
    auto * item = teach_sequence_list_->item(index);
    if (item->checkState() != Qt::Checked) {continue;}
    request->action_names.push_back(item->data(Qt::UserRole).toString().toStdString());
    request->display_indices.push_back(index + 1);
  }
  const int preview_count = static_cast<int>(request->action_names.size());
  if (preview_count == 0) {
    setTeachStatus(
      QStringLiteral("请至少勾选一个要播放的动作。"),
      QStringLiteral("Select at least one action to play."), true);
    return;
  }
  teach_sequence_slider_->setValue(0);
  QPointer<DemoPanel> self(this);
  teach_sequence_preview_client_->async_send_request(
    request, [self, name, preview_count](
      rclcpp::Client<PreviewActionSequence>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, name, preview_count, success, detail]() {
          if (!self) {return;}
          const QString display_name = name.isEmpty() ?
            self->localized(QStringLiteral("当前编辑内容"),
            QStringLiteral("current editor")) : name;
          self->setTeachStatus(
            success ? QStringLiteral(
              "正在按顺序播放动作组“%1”的 %2 个动作；可暂停或取消，真机未运动。")
            .arg(display_name).arg(preview_count) :
            QStringLiteral("动作组顺序动画失败：%1").arg(detail),
            success ? QStringLiteral(
              "Playing %2 actions from sequence “%1” in order; pause and cancel "
              "are available, and hardware did not move.")
            .arg(display_name).arg(preview_count) :
            QStringLiteral("Sequence animation failed: %1").arg(detail), !success);
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::previewTeachSequencePosition()
{
  if (!teach_sequence_preview_client_ ||
    !teach_sequence_preview_client_->service_is_ready())
  {
    setTeachStatus(
      QStringLiteral("动作组 RViz 预览服务不可用。"),
      QStringLiteral("Sequence RViz preview service is unavailable."), true);
    return;
  }
  auto request = std::make_shared<PreviewActionSequence::Request>();
  request->name = selectedTeachSequence().toStdString();
  request->progress = teach_sequence_slider_->value() / 1000.0;
  request->overlay = false;
  for (int index = 0; index < teach_sequence_list_->count(); ++index) {
    auto * item = teach_sequence_list_->item(index);
    if (item->checkState() != Qt::Checked) {continue;}
    request->action_names.push_back(item->data(Qt::UserRole).toString().toStdString());
    request->display_indices.push_back(index + 1);
  }
  if (request->action_names.empty()) {
    setTeachStatus(
      QStringLiteral("请至少勾选一个要定位预览的动作。"),
      QStringLiteral("Select at least one action before seeking the preview."), true);
    return;
  }
  const double progress = request->progress;
  teach_preview_active_.store(false);
  teach_preview_started_ms_.store(0);
  QPointer<DemoPanel> self(this);
  teach_sequence_preview_client_->async_send_request(
    request, [self, progress](
      rclcpp::Client<PreviewActionSequence>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, progress, success, detail]() {
          if (!self) {return;}
          self->setTeachStatus(
            success ? QStringLiteral("动作组 RViz 动画已定位到 %1%；真机未运动。")
            .arg(progress * 100.0, 0, 'f', 1) :
            QStringLiteral("动作组滑动预览失败：%1").arg(detail),
            success ? QStringLiteral(
              "Sequence RViz animation moved to %1%; hardware did not move.")
            .arg(progress * 100.0, 0, 'f', 1) :
            QStringLiteral("Sequence timeline preview failed: %1").arg(detail),
            !success);
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::handleTriggerResponse(
  rclcpp::Client<Trigger>::SharedFuture future,
  const QString & success_zh, const QString & success_en,
  bool refresh_after)
{
  teach_request_pending_.store(false);
  bool success = false;
  QString detail;
  try {
    const auto response = future.get();
    success = response && response->success;
    if (response) {detail = QString::fromStdString(response->message);}
  } catch (const std::exception & error) {
    detail = QString::fromLocal8Bit(error.what());
  }
  QPointer<DemoPanel> self(this);
  QMetaObject::invokeMethod(
    self, [self, success, detail, success_zh, success_en, refresh_after]() {
      if (!self) {return;}
      self->setTeachStatus(
        success ? QStringLiteral("%1 %2").arg(success_zh, detail) :
        QStringLiteral("操作失败：%1").arg(detail),
        success ? QStringLiteral("%1 %2").arg(success_en, detail) :
        QStringLiteral("Operation failed: %1").arg(detail), !success);
      if (success && refresh_after) {
        QTimer::singleShot(250, self, &DemoPanel::refreshTeachActions);
      }
      self->updateReadiness();
    }, Qt::QueuedConnection);
}

void DemoPanel::startTeaching()
{
  if (teach_request_pending_.load()) {
    return;
  }
  if (!feedbackFresh() || !xbox_state_known_.load() || xbox_armed_.load()) {
    setTeachStatus(
      QStringLiteral("无法开始示教：需要新鲜关节反馈，并将 Xbox 切换为 LOCKED。"),
      QStringLiteral("Cannot teach: fresh feedback and Xbox LOCKED are required."), true);
    return;
  }
  QMessageBox confirmation(this);
  const bool passive_piper = active_robot_ == QStringLiteral("piperh") &&
    teach_passive_recording_ready_.load();
  confirmation.setIcon(QMessageBox::Warning);
  confirmation.setWindowTitle(localized(
    QStringLiteral("确认拖动示教"), QStringLiteral("Confirm drag teaching")));
  confirmation.setText(passive_piper ? localized(
    QStringLiteral("电机保持失能；将只读取 Piper-H 关节反馈并录制人工拖动。请托住机械臂，确认急停可触达。"),
    QStringLiteral("Motors remain disabled. Recording reads Piper-H joint feedback only. "
      "Support the arm and keep the emergency stop reachable.")) : localized(
    QStringLiteral(
      "将从 Piper-H 普通关节反馈开始采样。确认急停可触达、托住机械臂，且工作区无人。"),
    QStringLiteral(
      "Recording will sample normal Piper-H joint feedback. Keep the emergency stop "
      "reachable, support the arm, and clear the workspace.")));
  auto * confirm_button = confirmation.addButton(
    localized(QStringLiteral("开始录制"), QStringLiteral("Start recording")),
    QMessageBox::AcceptRole);
  confirmation.addButton(
    localized(QStringLiteral("取消"), QStringLiteral("Cancel")), QMessageBox::RejectRole);
  confirmation.exec();
  if (confirmation.clickedButton() != confirm_button) {return;}
  if (!teach_start_client_ || !teach_start_client_->service_is_ready()) {
    setTeachStatus(
      QStringLiteral("开始示教服务不可用。"),
      QStringLiteral("Start teaching service is unavailable."), true);
    return;
  }
  teach_request_pending_.store(true);
  setTeachStatus(passive_piper ?
    QStringLiteral("正在被动采集失能状态下的关节反馈……") :
    QStringLiteral("正在采集 Piper-H 普通关节反馈，请等待，不要重复点击……"),
    passive_piper ? QStringLiteral("Passively recording disabled-arm joint feedback…") :
    QStringLiteral("Recording normal Piper-H joint feedback; please wait…"));
  updateReadiness();
  auto request = std::make_shared<Trigger::Request>();
  teach_start_client_->async_send_request(
    request, [this](rclcpp::Client<Trigger>::SharedFuture future) {
      handleTriggerResponse(
        future, QStringLiteral("已开始录制。"), QStringLiteral("Recording started."));
    });
}

void DemoPanel::stopTeaching()
{
  if (teach_request_pending_.load()) {
    return;
  }
  if (!teach_stop_client_ || !teach_stop_client_->service_is_ready()) {
    setTeachStatus(
      QStringLiteral("停止示教服务不可用。"),
      QStringLiteral("Stop teaching service is unavailable."), true);
    return;
  }
  teach_request_pending_.store(true);
  const bool passive_piper = active_robot_ == QStringLiteral("piperh") &&
    teach_passive_recording_ready_.load();
  setTeachStatus(passive_piper ?
    QStringLiteral("正在停止被动采样、校验并保存；电机保持失能……") :
    QStringLiteral("正在停止采样、校验并保存；重力补偿保持开启……"),
    passive_piper ? QStringLiteral("Stopping passive recording; motors remain disabled…") :
    QStringLiteral("Stopping sampling and saving; gravity compensation remains active…"));
  updateReadiness();
  auto request = std::make_shared<Trigger::Request>();
  teach_stop_client_->async_send_request(
    request, [this](rclcpp::Client<Trigger>::SharedFuture future) {
      handleTriggerResponse(
        future, QStringLiteral("录制已保存并发送到 RViz 预览。"),
        QStringLiteral("Recording saved and sent to RViz preview."), true);
    });
}

void DemoPanel::previewTeachAction()
{
  const QString name = selectedTeachAction();
  if (name.isEmpty() || !teach_select_client_ || !teach_select_client_->service_is_ready()) {
    setTeachStatus(
      QStringLiteral("请选择动作，并确认预览服务已就绪。"),
      QStringLiteral("Select an action and confirm the preview service is ready."), true);
    return;
  }
  const std::int64_t generation = teach_selection_generation_.fetch_add(1) + 1;
  auto request = std::make_shared<SelectActionGroup::Request>();
  request->name = name.toStdString();
  setTeachStatus(
    QStringLiteral("正在载入“%1”的 RViz 预览；真机不会运动……").arg(name),
    QStringLiteral("Loading the RViz preview for “%1”; hardware will not move…").arg(name));
  QPointer<DemoPanel> self(this);
  teach_select_client_->async_send_request(
    request, [self, name, generation](
      rclcpp::Client<SelectActionGroup>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, name, generation, success, detail]() {
          if (!self) {return;}
          if (generation != self->teach_selection_generation_.load()) {
            if (self->selectedTeachAction().isEmpty()) {
              self->clearTeachActionSelection();
            }
            return;
          }
          self->setTeachStatus(
            success ? QStringLiteral("正在 RViz 循环预览“%1”；真机未运动。").arg(name) :
            QStringLiteral("预览失败：%1").arg(detail),
            success ?
          QStringLiteral("Loop-previewing “%1” in RViz; hardware did not move.").arg(name) :
            QStringLiteral("Preview failed: %1").arg(detail), !success);
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::previewTeachPosition()
{
  QString sequence_name;
  {
    std::lock_guard<std::mutex> lock(teach_preview_source_mutex_);
    sequence_name = QString::fromStdString(teach_preview_sequence_name_);
  }
  if (!sequence_name.isEmpty()) {
    if (!teach_sequence_preview_client_ ||
      !teach_sequence_preview_client_->service_is_ready())
    {
      setTeachStatus(
        QStringLiteral("动作组 RViz 预览服务不可用。"),
        QStringLiteral("Sequence RViz preview service is unavailable."), true);
      return;
    }
    const double progress = teach_slider_->value() / 1000.0;
    teach_preview_active_.store(false);
    teach_preview_started_ms_.store(0);
    auto request = std::make_shared<PreviewActionSequence::Request>();
    request->name = sequence_name.toStdString();
    request->progress = progress;
    QPointer<DemoPanel> self(this);
    teach_sequence_preview_client_->async_send_request(
      request, [self, sequence_name, progress](
        rclcpp::Client<PreviewActionSequence>::SharedFuture future) {
        bool success = false;
        QString detail;
        try {
          const auto response = future.get();
          success = response && response->success;
          if (response) {detail = QString::fromStdString(response->message);}
        } catch (const std::exception & error) {
          detail = QString::fromLocal8Bit(error.what());
        }
        if (!self) {return;}
        QMetaObject::invokeMethod(
          self, [self, sequence_name, progress, success, detail]() {
            if (!self) {return;}
            self->setTeachStatus(
              success ? QStringLiteral("RViz 已从 %2% 继续预览动作组“%1”；真机未运动。")
              .arg(sequence_name).arg(progress * 100.0, 0, 'f', 1) :
              QStringLiteral("动作组滑动预览失败：%1").arg(detail),
              success ? QStringLiteral(
                "RViz resumed sequence “%1” from %2%; hardware did not move.")
              .arg(sequence_name).arg(progress * 100.0, 0, 'f', 1) :
              QStringLiteral("Sequence timeline preview failed: %1").arg(detail),
              !success);
          }, Qt::QueuedConnection);
      });
    return;
  }
  const QString name = selectedTeachAction();
  if (name.isEmpty() || !teach_preview_client_ || !teach_preview_client_->service_is_ready()) {
    return;
  }
  const double progress = teach_slider_->value() / 1000.0;
  teach_preview_active_.store(false);
  teach_preview_started_ms_.store(0);
  setTeachStatus(
    QStringLiteral("正在将 RViz 动画定位到 %1%……").arg(progress * 100.0, 0, 'f', 1),
    QStringLiteral("Seeking RViz animation to %1%…").arg(progress * 100.0, 0, 'f', 1));
  auto request = std::make_shared<PreviewActionGroup::Request>();
  request->name = name.toStdString();
  request->progress = progress;
  QPointer<DemoPanel> self(this);
  teach_preview_client_->async_send_request(
    request, [self, progress](rclcpp::Client<PreviewActionGroup>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, progress, success, detail]() {
          if (!self) {return;}
          self->setTeachStatus(
            success ? QStringLiteral("RViz 已从 %1% 继续循环预览；真机未运动。")
            .arg(progress * 100.0, 0, 'f', 1) : QStringLiteral("滑动预览失败：%1").arg(detail),
            success ? QStringLiteral("RViz loop resumed from %1%; hardware did not move.")
            .arg(progress * 100.0, 0, 'f',
          1) : QStringLiteral("Timeline preview failed: %1").arg(detail),
            !success);
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::replayTeachAction()
{
  const QString name = selectedTeachAction();
  const double speed_scale = teach_replay_speed_spin_->value();
  if (name.isEmpty() || !teach_replay_client_ || !teach_replay_client_->service_is_ready()) {
    setTeachStatus(
      QStringLiteral("请选择动作，并确认回放服务已就绪。"),
      QStringLiteral("Select an action and confirm replay is ready."), true);
    return;
  }
  if (!feedbackFresh() || !xbox_state_known_.load() || xbox_armed_.load()) {
    setTeachStatus(
      QStringLiteral("无法回放：需要新鲜反馈，并将 Xbox 切换为 LOCKED。"),
      QStringLiteral("Cannot replay: fresh feedback and Xbox LOCKED are required."), true);
    return;
  }
  const auto answer = QMessageBox::warning(
    this, localized(QStringLiteral("确认真机回放"), QStringLiteral("Confirm robot replay")),
    localized(
      QStringLiteral("将以 %2× 驱动真机回放“%1”。确认已在 RViz 检查轨迹，急停可触达且工作区无人。")
      .arg(name).arg(speed_scale, 0, 'f', 1),
      QStringLiteral("The physical robot will replay “%1” at %2×. Confirm the RViz path was "
      "checked, the emergency stop is reachable, and the workspace is clear.")
      .arg(name).arg(speed_scale, 0, 'f', 1)),
    QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
  if (answer != QMessageBox::Ok) {return;}
  auto request = std::make_shared<ReplayActionGroup::Request>();
  request->name = name.toStdString();
  request->speed_scale = speed_scale;
  QPointer<DemoPanel> self(this);
  teach_replay_client_->async_send_request(
    request, [self, name, speed_scale](rclcpp::Client<ReplayActionGroup>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, name, speed_scale, success, detail]() {
          if (!self) {return;}
          self->setTeachStatus(
            success ? QStringLiteral("真机正在以 %2× 回放“%1”。")
            .arg(name).arg(speed_scale, 0, 'f', 1) :
            QStringLiteral("回放失败：%1").arg(detail),
            success ? QStringLiteral("The robot is replaying “%1” at %2×.")
            .arg(name).arg(speed_scale, 0, 'f', 1) :
            QStringLiteral("Replay failed: %1").arg(detail), !success);
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::replayTeachSequence()
{
  const QString name = selectedTeachSequence();
  const bool eye_in_hand_sequence =
    name == kAutoHandeyeEyeInHandSequenceName ||
    name == kPiperAutoHandeyeEyeInHandSequenceName;
  const bool automatic_handeye =
    name == kAutoHandeyeEyeOnBaseSequenceName ||
    name == kPiperAutoHandeyeEyeOnBaseSequenceName || eye_in_hand_sequence;
  const double speed_scale = teach_sequence_replay_speed_spin_->value();
  if (automatic_handeye && handeyeUsesMoveItCalibration()) {
    setTeachStatus(
      QStringLiteral("MoveIt Calibration 当前只支持在插件中手动采样；自动动作组请切换到 easy_handeye2。"),
      QStringLiteral("MoveIt Calibration currently requires manual sampling in its plugin; switch to easy_handeye2 for automatic sequences."),
      true);
    return;
  }
  if (teach_sequence_list_->count() == 0 ||
    !teach_sequence_replay_client_ || !teach_sequence_replay_client_->service_is_ready())
  {
    setTeachStatus(
      QStringLiteral("请向动作组添加动作，并确认回放服务已就绪。"),
      QStringLiteral("Add actions to the sequence and confirm replay is ready."), true);
    return;
  }
  QStringList selected_actions;
  for (int index = 0; index < teach_sequence_list_->count(); ++index) {
    auto * item = teach_sequence_list_->item(index);
    if (item->checkState() == Qt::Checked) {
      selected_actions.push_back(item->data(Qt::UserRole).toString());
    }
  }
  if (selected_actions.isEmpty()) {
    setTeachStatus(
      QStringLiteral("请至少勾选一个要真机回放的动作。"),
      QStringLiteral("Select at least one action for robot replay."), true);
    return;
  }
  QStringList expected_auto_actions;
  const QString expected_prefix = eye_in_hand_sequence ?
    kPiperEyeInHandActionPrefix : QStringLiteral("手眼标定姿态_");
  const int expected_pose_count =
    name == kPiperAutoHandeyeEyeInHandSequenceName ? 18 : 12;
  for (int index = 1; index <= expected_pose_count; ++index) {
    expected_auto_actions.push_back(
      QStringLiteral("%1%2").arg(expected_prefix).arg(index, 2, 10, QLatin1Char('0')));
  }
  if (automatic_handeye && selected_actions != expected_auto_actions) {
    setTeachStatus(
      QStringLiteral("自动手眼标定必须按原顺序勾选动作组内全部 %1 个姿态，请重新加载该动作组。")
      .arg(expected_pose_count),
      QStringLiteral("Automatic hand-eye calibration requires all %1 poses in order; reload the sequence.")
      .arg(expected_pose_count),
      true);
    return;
  }
  if (automatic_handeye && name != automaticHandeyeSequenceName()) {
    setTeachStatus(
      QStringLiteral("当前标定模式与动作组不匹配：请使用“%1”。")
      .arg(automaticHandeyeSequenceName()),
      QStringLiteral("The calibration mode does not match this sequence; use “%1”.")
      .arg(automaticHandeyeSequenceName()), true);
    return;
  }
  if (automatic_handeye && handeye_process_->state() == QProcess::NotRunning) {
    setTeachStatus(
      QStringLiteral("请先在“手眼标定”页启动标定，再执行此动作组。"),
      QStringLiteral("Start calibration on the Hand-eye Calibration tab before replaying this sequence."),
      true);
    return;
  }
  if (!feedbackFresh() || !xbox_state_known_.load() || xbox_armed_.load()) {
    setTeachStatus(
      QStringLiteral("无法回放动作组：需要新鲜反馈，并将 Xbox 切换为 LOCKED。"),
      QStringLiteral("Cannot replay sequence: fresh feedback and Xbox LOCKED are required."),
      true);
    return;
  }
  const auto answer = QMessageBox::warning(
    this, localized(QStringLiteral("确认动作组回放"), QStringLiteral("Confirm sequence replay")),
    automatic_handeye ? localized(
      QStringLiteral(
        "将以 %1× 驱动真机完成 12 个标定姿态，并在每次停稳且 AprilTag TF 有效后自动采样，最后自动 Compute 和 Save。\n\n"
        "%2\n\n请先完整播放 RViz 预览，确认标签在全部姿态中可见；确认急停可触达、Xbox 为 LOCKED 且工作区无人。任何 TF、规划或采样失败都会停止流程。")
      .arg(speed_scale, 0, 'f', 1)
      .arg(eye_in_hand_sequence ?
        QStringLiteral("眼在手上：相机必须刚性安装在末端、标签必须固定在环境中。") :
        QStringLiteral("眼在手外：相机必须固定在外部、标签必须刚性安装在夹爪上。")),
      QStringLiteral(
        "The robot will execute 12 calibration poses at %1x, sample after each stable pose with a valid AprilTag TF, "
        "then Compute and Save automatically.\n\n%2\n\nPreview the complete RViz path first, keep the tag visible in every pose, "
        "keep the emergency stop reachable, Xbox LOCKED, and the workspace clear. Any TF, planning, or sampling failure stops the workflow.")
      .arg(speed_scale, 0, 'f', 1)
      .arg(eye_in_hand_sequence ?
        QStringLiteral("Eye-in-hand: rigidly mount the camera on the effector and fix the tag in the environment.") :
        QStringLiteral("Eye-on-base: fix the camera outside the robot and rigidly mount the tag on the gripper."))) : localized(
      QStringLiteral("将以 %3× 按列表顺序驱动真机执行动作组“%1”中勾选的 %2 个动作。确认急停可触达且工作区无人。")
      .arg(name.isEmpty() ? QStringLiteral("当前编辑内容") : name)
      .arg(selected_actions.size()).arg(speed_scale, 0, 'f', 1),
      QStringLiteral("The robot will execute %2 selected actions from sequence “%1” in list order at %3×. Confirm the "
      "emergency stop is reachable and the workspace is clear.")
      .arg(name.isEmpty() ? QStringLiteral("current editor") : name)
      .arg(selected_actions.size()).arg(speed_scale, 0, 'f', 1)),
    QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
  if (answer != QMessageBox::Ok) {return;}
  auto request = std::make_shared<ReplayActionSequence::Request>();
  request->name = name.toStdString();
  request->speed_scale = speed_scale;
  for (const QString & action : selected_actions) {
    request->action_names.push_back(action.toStdString());
  }
  QPointer<DemoPanel> self(this);
  teach_sequence_replay_client_->async_send_request(
    request, [self, name, speed_scale, selected_actions](
      rclcpp::Client<ReplayActionSequence>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {detail = QString::fromLocal8Bit(error.what());}
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, name, speed_scale, selected_actions, success, detail]() {
          if (!self) {return;}
          self->setTeachStatus(
            success ? QStringLiteral("真机正在以 %3× 按顺序执行动作组“%1”中勾选的 %2 个动作。")
            .arg(name.isEmpty() ? QStringLiteral("当前编辑内容") : name)
            .arg(selected_actions.size()).arg(speed_scale, 0, 'f', 1) :
            QStringLiteral("动作组回放失败：%1").arg(detail),
            success ? QStringLiteral("The robot is executing %2 selected actions from sequence “%1” in order at %3×.")
            .arg(name.isEmpty() ? QStringLiteral("current editor") : name)
            .arg(selected_actions.size()).arg(speed_scale, 0, 'f', 1) :
            QStringLiteral("Sequence replay failed: %1").arg(detail), !success);
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::pauseTeachPreview()
{
  if (!teach_pause_client_ || !teach_pause_client_->service_is_ready()) {return;}
  teach_request_pending_.store(true);
  auto request = std::make_shared<Trigger::Request>();
  teach_pause_client_->async_send_request(
    request, [this](rclcpp::Client<Trigger>::SharedFuture future) {
      handleTriggerResponse(
        future,
        QStringLiteral("RViz 预览的暂停/继续状态已切换。"),
        QStringLiteral("RViz preview pause/resume state changed."));
    });
}

void DemoPanel::cancelTeaching()
{
  if (teach_state_.load() == TEACH_FAULT) {
    if (!teach_reset_client_ || !teach_reset_client_->service_is_ready()) {
      setTeachStatus(
        QStringLiteral("故障复位服务尚未就绪。"),
        QStringLiteral("Fault reset service is not ready."), true);
      return;
    }
    teach_request_pending_.store(true);
    updateReadiness();
    auto request = std::make_shared<Trigger::Request>();
    teach_reset_client_->async_send_request(
      request, [this](rclcpp::Client<Trigger>::SharedFuture future) {
        handleTriggerResponse(
          future,
          QStringLiteral("安全预检通过，示教操作已恢复。"),
          QStringLiteral("Safety preflight passed; teaching controls restored."));
      });
    return;
  }
  if (!teach_cancel_client_ || !teach_cancel_client_->service_is_ready()) {return;}
  const int state = teach_state_.load();
  const bool canceling_hardware_replay =
    state == TEACH_REPLAY_STARTING || state == TEACH_REPLAYING ||
    state == TEACH_CANCELLING;
  return_home_after_teach_cancel_pending_.store(canceling_hardware_replay);
  teach_request_pending_.store(true);
  updateReadiness();
  auto request = std::make_shared<Trigger::Request>();
  QPointer<DemoPanel> self(this);
  teach_cancel_client_->async_send_request(
    request, [self, canceling_hardware_replay](
      rclcpp::Client<Trigger>::SharedFuture future) {
      bool success = false;
      QString detail;
      try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {detail = QString::fromStdString(response->message);}
      } catch (const std::exception & error) {
        detail = QString::fromLocal8Bit(error.what());
      }
      if (!self) {return;}
      QMetaObject::invokeMethod(
        self, [self, success, detail, canceling_hardware_replay]() {
          if (!self) {return;}
          self->teach_request_pending_.store(false);
          if (!success) {
            self->return_home_after_teach_cancel_pending_.store(false);
          }
          self->setTeachStatus(
            success ? (canceling_hardware_replay ? QStringLiteral(
              "已请求停止真机回放；控制器确认停止后将自动规划回原点。") :
              QStringLiteral("已请求取消示教操作。")) :
            QStringLiteral("取消示教操作失败：%1").arg(detail),
            success ? (canceling_hardware_replay ? QStringLiteral(
              "Robot replay stop requested; a collision-checked return home will be planned "
              "after the controller confirms it has stopped.") :
              QStringLiteral("Teaching operation cancellation requested.")) :
            QStringLiteral("Failed to cancel teaching operation: %1").arg(detail),
            !success);
          self->updateReadiness();
        }, Qt::QueuedConnection);
    });
}

void DemoPanel::updateReadiness()
{
  if (!embedded_motion_planning_widget_) {
    integrateMotionPlanningPanel();
  }
  ensureShapeMarkerDisplay();
  ensureForbiddenZoneMarkerDisplay();
  ensurePulseMarkerDisplay();
  configureRobotDisplays();
  updateFreePlanningVisibility();
  translateMotionPlanningUi(chinese_);
  setMotionPlanningIntegratedMode(true);
  updateMotionPlanningResult();
  const bool fresh = feedbackFresh();
  const bool xbox_known = xbox_state_known_.load();
  const bool xbox_is_armed = xbox_known && xbox_armed_.load();
  const bool xbox_locked = xbox_known && !xbox_is_armed;
  const auto now = steadyMilliseconds();
  if (now - last_camera_graph_check_ms_ >= 1000) {
    last_camera_graph_check_ms_ = now;
    camera_detected_.store(cameraNodeRunning());
    camera_stream_published_.store(cameraStreamPublished());
  }
  const auto last_hand_image = last_hand_vision_image_ms_.load();
  if (hand_vision_process_->state() != QProcess::NotRunning &&
    (last_hand_image == 0 || now - last_hand_image >= 1000))
  {
    hand_vision_status_label_->setText(localized(
      QStringLiteral("手部视觉画面已停止更新；当前显示的是最后一帧，请检查彩色/深度图流。"),
      QStringLiteral(
        "Hand-vision frames stopped updating; the displayed image is the last frame. "
        "Check the color/depth streams.")));
    hand_vision_status_label_->setStyleSheet(QStringLiteral("color: #c43b32;"));
  }
  if (fresh && validity_client_ && validity_client_->service_is_ready() &&
    !validity_state_->pending.load() && now - last_validity_request_ms_ >= 500)
  {
    sensor_msgs::msg::JointState::SharedPtr joint_state;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      joint_state = latest_joint_state_;
    }
    if (joint_state) {
      auto request = std::make_shared<GetStateValidity::Request>();
      request->group_name = "arm";
      request->robot_state.joint_state = *joint_state;
      last_validity_request_ms_ = now;
      validity_state_->pending.store(true);
      const auto state = validity_state_;
      validity_client_->async_send_request(
        request,
        [state](rclcpp::Client<GetStateValidity>::SharedFuture future) {
          try {
            const auto response = future.get();
            state->valid.store(response && response->valid);
            state->response_ms.store(steadyMilliseconds());
          } catch (const std::exception &) {
            state->valid.store(false);
            state->response_ms.store(0);
          }
          state->pending.store(false);
        });
    }
  }
  const bool valid = stateValidityFresh();
  const bool hardware_allowed = teach_hardware_allowed_.load();
  const bool offline_preview = teach_offline_preview_.load();
  bool running = process_->state() != QProcess::NotRunning;
  if (fresh && !connection_ready_announced_) {
    connection_ready_announced_ = true;
    setStatus(
      QStringLiteral("已连接，关节反馈正常。"),
      QStringLiteral("Connected; joint feedback is healthy."));
  }
  const bool armed_transition =
    xbox_known && xbox_state_observed_ && !previous_xbox_armed_ && xbox_is_armed;
  if (xbox_known) {
    previous_xbox_armed_ = xbox_is_armed;
    xbox_state_observed_ = true;
  }
  if (armed_transition) {
    if (execute_client_) {
      execute_client_->async_cancel_all_goals();
    }
    if (rviz_stop_pub_) {
      rviz_stop_pub_->publish(std_msgs::msg::Empty());
    }
    if (running) {
      stopDemo();
    }
    setStatus(
      QStringLiteral("Xbox 已切换为 ARMED；已请求取消全部 MoveIt 轨迹，控制权交给手柄。"),
      QStringLiteral(
        "Xbox switched to ARMED; cancellation of all MoveIt trajectories was requested and "
        "control was given to the gamepad."), true);
    running = process_->state() != QProcess::NotRunning;
  }
  feedback_label_->setText(offline_preview ? localized(
    QStringLiteral("运行模式：离线模型预览（未启动真机驱动）"),
    QStringLiteral("Mode: offline model preview (no robot driver)")) : fresh ? localized(
    QStringLiteral("关节反馈：正常（< 1 秒）"),
    QStringLiteral("Joint feedback: OK (< 1 second)")) : localized(
    QStringLiteral("关节反馈：缺失或已超时"),
    QStringLiteral("Joint feedback: missing or stale")));
  feedback_label_->setStyleSheet(offline_preview ?
    QStringLiteral("color: #2b7bb9; font-weight: bold;") :
    fresh ? QStringLiteral("color: #3a9d23;") : QStringLiteral("color: #c43b32;"));
  validity_label_->setText(valid ? localized(
    QStringLiteral("MoveIt 状态：有效"), QStringLiteral("MoveIt state: valid")) : localized(
    QStringLiteral("MoveIt 状态：无效/碰撞或等待检查"),
    QStringLiteral("MoveIt state: invalid/in collision or waiting for check")));
  validity_label_->setStyleSheet(
    valid ? QStringLiteral("color: #3a9d23;") : QStringLiteral("color: #c43b32;"));
  xbox_label_->setText(offline_preview ? localized(
    QStringLiteral("真机输出：已禁用（仅允许 RViz 动画）"),
    QStringLiteral("Robot output: disabled (RViz animation only)")) : !xbox_known ? localized(
    QStringLiteral("Xbox 控制：等待状态"), QStringLiteral("Xbox control: waiting for state")) :
    xbox_locked ? localized(
      QStringLiteral("Xbox 控制：LOCKED（MoveIt 可执行）"),
      QStringLiteral("Xbox control: LOCKED (MoveIt may execute)")) : localized(
      QStringLiteral("Xbox 控制：ARMED（手柄控制）"),
      QStringLiteral("Xbox control: ARMED (gamepad control)")));
  xbox_label_->setStyleSheet(offline_preview ?
    QStringLiteral("color: #2b7bb9; font-weight: bold;") :
    xbox_locked ? QStringLiteral("color: #3a9d23;") : QStringLiteral("color: #d08020;"));
  home_button_->setEnabled(fresh && valid && xbox_locked && !running);
  singularity_escape_button_->setEnabled(fresh && valid && xbox_locked && !running);
  singularity_escape_reset_button_->setEnabled(!running);
  for (auto * spin : singularity_escape_joint_spins_) {
    if (spin) {spin->setEnabled(!running);}
  }
  sync_button_->setEnabled(fresh && !running);
  pick_place_button_->setEnabled(fresh && valid && xbox_locked && !running);
  stop_button_->setEnabled(running);
  speed_spin_->setEnabled(!running);
  const auto pulse_fresh_limit_ms = active_robot_ == QStringLiteral("piperh") ? 1500 : 700;
  const auto pulse_last_ms = active_robot_ == QStringLiteral("piperh") ?
    last_piper_pulse_point_ms_.load() : last_pulse_point_ms_.load();
  const auto pulse_age_ms = pulse_last_ms > 0 ? now - pulse_last_ms : now;
  const bool pulse_fresh = pulse_age_ms < pulse_fresh_limit_ms;
  const bool calibration_saved = QFileInfo::exists(handeyeCalibrationFile());
  const bool calibration_idle = handeye_process_->state() == QProcess::NotRunning;
  const bool pulse_ready = fresh && valid && xbox_locked && pulse_fresh &&
    calibration_saved && !running && calibration_idle;
  // Keep this button clickable as a readiness diagnostic. A disabled Qt button
  // emits no clicked signal, which previously made a missing visual target look
  // like the application had stopped responding. startPulseApproach() repeats
  // every safety check and reports the concrete blocker without starting motion.
  pulse_approach_button_->setEnabled(!running);
  QStringList pulse_blockers_zh;
  QStringList pulse_blockers_en;
  if (!fresh) {
    pulse_blockers_zh.append(QStringLiteral("关节反馈缺失或超时"));
    pulse_blockers_en.append(QStringLiteral("joint feedback missing or stale"));
  }
  if (!valid) {
    pulse_blockers_zh.append(QStringLiteral("MoveIt 状态无效或尚未检查"));
    pulse_blockers_en.append(QStringLiteral("MoveIt state invalid or not checked"));
  }
  if (!xbox_known) {
    pulse_blockers_zh.append(QStringLiteral("尚未收到 Xbox 控制状态"));
    pulse_blockers_en.append(QStringLiteral("Xbox control state not received"));
  } else if (!xbox_locked) {
    pulse_blockers_zh.append(QStringLiteral("Xbox 尚未 LOCKED"));
    pulse_blockers_en.append(QStringLiteral("Xbox is not LOCKED"));
  }
  if (!calibration_saved) {
    pulse_blockers_zh.append(QStringLiteral("未找到当前机械臂的手眼标定文件"));
    pulse_blockers_en.append(QStringLiteral("hand-eye calibration file not found"));
  }
  if (!calibration_idle) {
    pulse_blockers_zh.append(QStringLiteral("手眼标定仍在运行"));
    pulse_blockers_en.append(QStringLiteral("hand-eye calibration is still running"));
  }
  if (!pulse_fresh) {
    pulse_blockers_zh.append(QStringLiteral("Piper-H 规划坐标中的视觉目标缺失或超过工程 freshness 阈值"));
    pulse_blockers_en.append(QStringLiteral(
      "vision target in the Piper-H planning frame is missing or older than its freshness limit"));
  }
  if (running) {
    pulse_blockers_zh.append(QStringLiteral("已有操作正在运行"));
    pulse_blockers_en.append(QStringLiteral("another operation is running"));
  }
  pulse_approach_button_->setToolTip(pulse_ready ? localized(
    QStringLiteral("条件满足；再次确认仅在非人体假体上向视觉目标运动，压力异常请求停止。"),
    QStringLiteral(
      "Ready; confirm dummy-only motion toward the visual target with pressure cancellation.")) : localized(
    QStringLiteral("当前不可用：%1").arg(pulse_blockers_zh.join(QStringLiteral("；"))),
    QStringLiteral("Currently unavailable: %1").arg(
      pulse_blockers_en.join(QStringLiteral("; ")))));
  pulse_standoff_spin_->setEnabled(!running);
  if (active_robot_ == QStringLiteral("piperh") && pulse_age_ms >= 1500) {
    pulse_region_label_->setText(localized(
      QStringLiteral("Piper-H 目标已失效：Target age: %1 ms；需重新跟踪稳定目标")
      .arg(pulse_age_ms),
      QStringLiteral("Piper-H target expired: Target age: %1 ms; fresh tracking required")
      .arg(pulse_age_ms)));
    pulse_region_label_->setStyleSheet(QStringLiteral("color: #c43b32; font-weight: bold;"));
  } else if (active_robot_ == QStringLiteral("piperh") && pulse_age_ms >= 500) {
    pulse_region_label_->setText(localized(
      QStringLiteral("Piper-H 目标 STALE：Target age: %1 ms（暂存目标仅供等待，不生成新下降规划）")
      .arg(pulse_age_ms),
      QStringLiteral(
        "Piper-H target STALE: Target age: %1 ms "
        "(retained only while waiting; no new descent plan)")
      .arg(pulse_age_ms)));
    pulse_region_label_->setStyleSheet(QStringLiteral("color: #d6a000; font-weight: bold;"));
  } else if (!pulse_fresh) {
    pulse_region_label_->setText(localized(
      (active_robot_ == QStringLiteral("piperh") ?
        QStringLiteral("假体目标运动尚未就绪：%1") :
        QStringLiteral("预接触尚未就绪：%1")).arg(
        pulse_blockers_zh.join(QStringLiteral("；"))),
      (active_robot_ == QStringLiteral("piperh") ?
        QStringLiteral("Dummy target motion is not ready: %1") :
        QStringLiteral("Pre-contact is not ready: %1")).arg(
        pulse_blockers_en.join(QStringLiteral("; ")))));
    pulse_region_label_->setStyleSheet(QStringLiteral("color: #d08020;"));
  }
  updateForbiddenZoneUi();

  const int teach_state = teach_state_.load();
  const bool teach_pending = teach_request_pending_.load();
  const bool shape_reachability_pending =
    shape_reachability_request_pending_.load();
  bool sequence_preview_source = false;
  {
    std::lock_guard<std::mutex> lock(teach_preview_source_mutex_);
    sequence_preview_source = !teach_preview_sequence_name_.empty();
  }
  QSlider * active_preview_slider = sequence_preview_source ?
    teach_sequence_slider_ : teach_slider_;
  if (teach_preview_active_.load() && !active_preview_slider->isSliderDown()) {
    const auto duration_ms = teach_preview_duration_ms_.load();
    const double offset = teach_preview_offset_milli_.load() / 1000.0;
    const auto started_ms = teach_preview_started_ms_.load();
    // MotionPlanning is configured for 1x playback, so wall time and the
    // trajectory's time_from_start use the same scale.
    if (teach_preview_paused_.load()) {
      active_preview_slider->setValue(static_cast<int>(std::lround(offset * 1000.0)));
    } else if (duration_ms > 0 && started_ms > 0) {
      const auto elapsed_ms = std::max<std::int64_t>(0, now - started_ms);
      const double progress = std::fmod(
        offset + static_cast<double>(elapsed_ms) / duration_ms, 1.0);
      active_preview_slider->setValue(static_cast<int>(std::lround(progress * 1000.0)));
    }
  }
  const bool teach_resting =
    teach_state == TEACH_LOCKED || teach_state == TEACH_IDLE ||
    teach_state == TEACH_READY;
  const bool xbox_mode_ready = xbox_mode_client_ && xbox_mode_client_->service_is_ready();
  const bool xbox_mode_pending = xbox_request_pending_.load();
  xbox_mode_button_->setText(xbox_is_armed ? localized(
    QStringLiteral("锁定 Xbox 控制"), QStringLiteral("Lock Xbox Control")) : localized(
    QStringLiteral("启用 Xbox 控制"), QStringLiteral("Enable Xbox Control")));
  xbox_mode_button_->setEnabled(
    xbox_known && xbox_mode_ready && !xbox_mode_pending &&
    (xbox_is_armed || (!running && teach_resting)));

  const bool camera_owned = camera_process_->state() != QProcess::NotRunning;
  const bool camera_external = camera_detected_.load() && !camera_owned;
  const bool camera_stream_published = camera_stream_published_.load();
  const bool camera_image_fresh = now - last_camera_image_ms_.load() < 1000;
  const bool camera_settings_editable = !camera_owned && !camera_external &&
    !camera_stop_requested_.load();
  camera_color_checkbox_->setEnabled(camera_settings_editable);
  camera_depth_checkbox_->setEnabled(camera_settings_editable);
  camera_ir_checkbox_->setEnabled(camera_settings_editable);
  QStringList configured_streams;
  if (camera_color_checkbox_->isChecked()) {
    configured_streams.append(localized(QStringLiteral("彩色"), QStringLiteral("color")));
  }
  if (camera_depth_checkbox_->isChecked()) {
    configured_streams.append(localized(QStringLiteral("深度"), QStringLiteral("depth")));
  }
  if (camera_ir_checkbox_->isChecked()) {
    configured_streams.append(localized(QStringLiteral("红外"), QStringLiteral("infrared")));
  }
  if (camera_stop_requested_.load()) {
    camera_label_->setText(localized(
      QStringLiteral("Gemini 2：正在关闭…"), QStringLiteral("Gemini 2: stopping…")));
  } else if (camera_owned && camera_image_fresh) {
    camera_label_->setText(localized(
      QStringLiteral("Gemini 2：%1流运行中").arg(configured_streams.join(QStringLiteral("、"))),
      QStringLiteral("Gemini 2: %1 streams running").arg(
        configured_streams.join(QStringLiteral(", ")))));
  } else if (camera_owned) {
    camera_label_->setText(localized(
      QStringLiteral("Gemini 2：正在启动并等待数据…"),
      QStringLiteral("Gemini 2: starting and waiting for data…")));
  } else if (camera_external && camera_stream_published) {
    camera_label_->setText(localized(
      QStringLiteral("Gemini 2：已由外部进程打开"),
      QStringLiteral("Gemini 2: opened by an external process")));
  } else if (camera_external) {
    camera_label_->setText(localized(
      QStringLiteral("Gemini 2：驱动已启动，但设备离线/无图像流"),
      QStringLiteral("Gemini 2: driver running, but device offline/no image stream")));
  } else {
    camera_label_->setText(localized(
      QStringLiteral("Gemini 2：未打开"), QStringLiteral("Gemini 2: off")));
  }
  camera_button_->setText(camera_owned ? localized(
    QStringLiteral("关闭摄像头"), QStringLiteral("Close Camera")) : camera_external ? localized(
    camera_stream_published ? QStringLiteral("摄像头已打开（外部）") :
    QStringLiteral("摄像头设备离线（外部驱动）"),
    camera_stream_published ? QStringLiteral("Camera Open (External)") :
    QStringLiteral("Camera Offline (External Driver)")) : localized(
    QStringLiteral("打开摄像头"), QStringLiteral("Open Camera")));
  camera_button_->setEnabled(!camera_stop_requested_.load() && !camera_external);
  camera_tab_button_->setText(camera_button_->text());
  camera_tab_button_->setEnabled(camera_button_->isEnabled());
  if (camera_image_fresh) {
    camera_tab_status_label_->setText(localized(
      QStringLiteral("Gemini 2 彩色画面：实时数据正常"),
      QStringLiteral("Gemini 2 color view: live data OK")));
  } else if (camera_external && !camera_stream_published) {
    camera_tab_status_label_->setText(localized(
      QStringLiteral("驱动节点已存在，但设备离线；系统未检测到 Gemini 2 USB 设备。"),
      QStringLiteral("The driver node exists, but the device is offline; no Gemini 2 USB device was detected.")));
  } else if (camera_owned || camera_external) {
    camera_tab_status_label_->setText(localized(
      QStringLiteral("正在等待 %1…").arg(camera_image_topic_),
      QStringLiteral("Waiting for %1…").arg(camera_image_topic_)));
  } else {
    camera_tab_status_label_->setText(localized(
      QStringLiteral("摄像头未打开。点击下方按钮启动 Gemini 2。"),
      QStringLiteral("Camera is off. Use the button below to start Gemini 2.")));
  }
  if (!camera_image_fresh && camera_image_visible_) {
    latest_camera_image_ = QImage();
    camera_view_label_->clear();
    camera_view_label_->setText(localized(
      QStringLiteral("等待摄像头画面"), QStringLiteral("Waiting for camera image")));
    camera_image_visible_ = false;
  }
  const bool has_builtin_shape =
    teach_shape_combo_->currentData().toString() != QStringLiteral("freehand");
  const QString selected_drawing = teach_shape_combo_->currentData().toString();
  const bool drawing_source_ready =
    (selected_drawing != QStringLiteral("text") &&
    selected_drawing != QStringLiteral("image")) || !selectedDrawingSource().isEmpty();
  const bool shape_geometry_ready =
    teach_shape_width_spin_->value() >= 0.01 &&
    teach_shape_height_spin_->value() >= 0.01 &&
    teach_shape_pen_lift_spin_->value() >= 1.0;
  const bool has_action = !selectedTeachAction().isEmpty();
  const bool adding_action = shape_editor_new_mode_ && !has_action;
  const bool list_ready = teach_list_client_ && teach_list_client_->service_is_ready();
  const bool shape_marker_ready = teach_shape_marker_client_ &&
    teach_shape_marker_client_->service_is_ready();
  const bool trace_ready = teach_trace_client_ && teach_trace_client_->service_is_ready();
  const bool shape_ready = teach_shape_client_ && teach_shape_client_->service_is_ready();
  const bool shape_reachability_ready = teach_shape_reachability_client_ &&
    teach_shape_reachability_client_->service_is_ready();
  const bool select_ready = teach_select_client_ && teach_select_client_->service_is_ready();
  const bool preview_ready = teach_preview_client_ && teach_preview_client_->service_is_ready();
  const bool replay_ready = teach_replay_client_ && teach_replay_client_->service_is_ready();
  const bool reset_ready = teach_reset_client_ && teach_reset_client_->service_is_ready();
  const bool copy_ready = teach_copy_client_ && teach_copy_client_->service_is_ready();
  const bool rename_ready = teach_rename_client_ && teach_rename_client_->service_is_ready();
  const bool delete_ready = teach_delete_client_ && teach_delete_client_->service_is_ready();
  const bool sequence_list_ready = teach_sequence_list_client_ &&
    teach_sequence_list_client_->service_is_ready();
  const bool sequence_save_ready = teach_sequence_save_client_ &&
    teach_sequence_save_client_->service_is_ready();
  const bool sequence_preview_ready = teach_sequence_preview_client_ &&
    teach_sequence_preview_client_->service_is_ready();
  const bool sequence_replay_ready = teach_sequence_replay_client_ &&
    teach_sequence_replay_client_->service_is_ready();
  const int sequence_row = teach_sequence_list_->currentRow();
  int checked_sequence_actions = 0;
  for (int index = 0; index < teach_sequence_list_->count(); ++index) {
    if (teach_sequence_list_->item(index)->checkState() == Qt::Checked) {
      ++checked_sequence_actions;
    }
  }
  teach_refresh_button_->setEnabled(list_ready && !teach_pending);
  teach_shape_create_button_->setEnabled(
    (adding_action || !editing_shape_action_name_.isEmpty()) &&
    has_builtin_shape && drawing_source_ready && shape_geometry_ready &&
    shape_ready && !teach_pending &&
    !shape_reachability_pending && teach_resting &&
    fresh && valid && xbox_locked);
  teach_shape_reachability_button_->setText(shape_reachability_pending ? localized(
      QStringLiteral("正在预检，请稍候…"),
      QStringLiteral("Preflight running; please wait…")) : localized(
      QStringLiteral("预检当前笔尖轨迹（不动真机）"),
      QStringLiteral("Preflight Current Pen-tip Path (No Hardware Motion)")));
  teach_shape_reachability_button_->setEnabled(
    has_builtin_shape && drawing_source_ready && shape_geometry_ready &&
    shape_reachability_ready &&
    !teach_pending && !shape_reachability_pending && teach_resting && fresh && xbox_locked);
  teach_shape_combo_->setEnabled(
    !teach_pending && !shape_reachability_pending && teach_resting);
  teach_shape_source_edit_->setEnabled(
    !teach_pending && !shape_reachability_pending && teach_resting);
  teach_shape_image_button_->setEnabled(
    !teach_pending && !shape_reachability_pending && teach_resting);
  teach_shape_marker_button_->setEnabled(
    has_builtin_shape && drawing_source_ready && shape_geometry_ready &&
    shape_marker_ready && !teach_pending &&
    !shape_reachability_pending && teach_resting);
  teach_shape_reset_button_->setEnabled(
    has_builtin_shape && drawing_source_ready && shape_marker_ready && !teach_pending &&
    !shape_reachability_pending && teach_resting);
  teach_shape_apply_pose_button_->setEnabled(
    has_builtin_shape && drawing_source_ready && shape_geometry_ready &&
    shape_marker_ready && !teach_pending &&
    !shape_reachability_pending && teach_resting);
  teach_trace_width_spin_->setEnabled(trace_ready && !teach_pending);
  for (auto * spin : {
      teach_shape_x_spin_, teach_shape_y_spin_, teach_shape_z_spin_,
      teach_shape_roll_spin_, teach_shape_pitch_spin_, teach_shape_yaw_spin_,
      teach_shape_width_spin_, teach_shape_height_spin_,
      teach_shape_pen_length_spin_, teach_shape_pen_lift_spin_})
  {
    spin->setEnabled(
      has_builtin_shape && !teach_pending && !shape_reachability_pending && teach_resting);
  }
  teach_copy_button_->setEnabled(
    !teach_pending && has_action && teach_resting && copy_ready);
  teach_rename_button_->setEnabled(
    !teach_pending && has_action && teach_resting && rename_ready);
  teach_delete_button_->setEnabled(
    !teach_pending && has_action && teach_resting && delete_ready);
  teach_action_combo_->setEnabled(
    list_ready && !teach_pending && teach_resting && teach_action_combo_->count() > 0);
  teach_start_button_->setEnabled(
    hardware_allowed && adding_action && !teach_pending && fresh && xbox_locked &&
    ((active_robot_ == QStringLiteral("piperh") &&
      ((teach_state == TEACH_IDLE || teach_state == TEACH_READY) &&
      teach_normal_feedback_ready_.load())) ||
     (active_robot_ != QStringLiteral("piperh") &&
      (teach_state == TEACH_IDLE || teach_state == TEACH_READY))) &&
    teach_start_client_ && teach_start_client_->service_is_ready());
  teach_stop_button_->setEnabled(
    !teach_pending && teach_state == TEACH_RECORDING &&
    teach_stop_client_ && teach_stop_client_->service_is_ready());
  teach_replay_speed_spin_->setEnabled(!teach_pending && teach_resting);
  teach_preview_button_->setEnabled(
    !teach_pending && has_action && teach_resting && select_ready);
  teach_slider_->setEnabled(
    !teach_pending && has_action && teach_resting && preview_ready);
  teach_replay_button_->setEnabled(
    hardware_allowed && !teach_pending && has_action && fresh && valid && xbox_locked &&
    (teach_state == TEACH_IDLE || teach_state == TEACH_READY) && replay_ready);
  if (offline_preview) {
    teach_replay_button_->setText(localized(
      QStringLiteral("真机回放（离线禁用）"),
      QStringLiteral("Robot replay (disabled offline)")));
    teach_replay_button_->setToolTip(localized(
      QStringLiteral("离线预览入口未启动串口/CAN 驱动，只能播放 RViz 动画。"),
      QStringLiteral("The offline launcher starts no serial/CAN driver; only RViz animation is available.")));
  }
  teach_pause_button_->setEnabled(
    !teach_pending && teach_preview_active_.load() && !sequence_preview_source &&
    teach_pause_client_ && teach_pause_client_->service_is_ready());
  teach_pause_button_->setText(
    teach_preview_paused_.load() ?
    localized(QStringLiteral("继续"), QStringLiteral("Resume")) :
    localized(QStringLiteral("暂停"), QStringLiteral("Pause")));
  teach_cancel_button_->setText(
    teach_state == TEACH_FAULT ? localized(
      QStringLiteral("检查并解除故障"), QStringLiteral("Check and Reset Fault")) :
    teach_state == TEACH_SEQUENCE_PLANNING ? localized(
      QStringLiteral("取消动作组过渡规划"),
      QStringLiteral("Cancel Sequence Transition Planning")) :
    (teach_state == TEACH_REPLAY_STARTING || teach_state == TEACH_REPLAYING ||
    teach_state == TEACH_CANCELLING) ? localized(
      QStringLiteral("取消回放并自动回原点"),
      QStringLiteral("Cancel Replay and Return Home")) : localized(
      QStringLiteral("取消"), QStringLiteral("Cancel")));
  teach_cancel_button_->setEnabled(
    !teach_pending &&
    ((teach_state == TEACH_FAULT && reset_ready) ||
    ((teach_preview_active_.load() || teach_state == TEACH_RECORDING ||
    teach_state == TEACH_SEQUENCE_PLANNING ||
    teach_state == TEACH_REPLAY_STARTING || teach_state == TEACH_REPLAYING ||
    teach_state == TEACH_CANCELLING) &&
    teach_cancel_client_ && teach_cancel_client_->service_is_ready())));
  teach_sequence_refresh_button_->setEnabled(sequence_list_ready && !teach_pending);
  teach_sequence_combo_->setEnabled(
    sequence_list_ready && !teach_pending && teach_resting &&
    teach_sequence_combo_->count() > 0);
  teach_sequence_list_->setEnabled(!teach_pending && teach_resting);
  teach_sequence_add_button_->setEnabled(!teach_pending && teach_resting && has_action);
  teach_sequence_remove_button_->setEnabled(
    !teach_pending && teach_resting && sequence_row >= 0);
  teach_sequence_up_button_->setEnabled(
    !teach_pending && teach_resting && sequence_row > 0);
  teach_sequence_down_button_->setEnabled(
    !teach_pending && teach_resting && sequence_row >= 0 &&
    sequence_row < teach_sequence_list_->count() - 1);
  teach_sequence_save_button_->setEnabled(
    !teach_pending && teach_resting && teach_sequence_list_->count() > 0 &&
    sequence_save_ready);
  teach_sequence_play_button_->setEnabled(
    !teach_pending && checked_sequence_actions > 0 && teach_resting &&
    sequence_preview_ready);
  teach_sequence_slider_->setEnabled(
    !teach_pending && checked_sequence_actions > 0 && teach_resting &&
    sequence_preview_ready);
  teach_sequence_replay_speed_spin_->setEnabled(!teach_pending && teach_resting);
  teach_sequence_replay_button_->setEnabled(
    hardware_allowed && !teach_pending && checked_sequence_actions > 0 &&
    fresh && valid && xbox_locked &&
    (teach_state == TEACH_IDLE || teach_state == TEACH_READY) &&
    sequence_replay_ready);
  if (offline_preview) {
    teach_sequence_replay_button_->setText(localized(
      QStringLiteral("真机回放勾选动作（离线禁用）"),
      QStringLiteral("Robot replay of checked actions (disabled offline)")));
    teach_sequence_replay_button_->setToolTip(localized(
      QStringLiteral("离线预览模式不会向真实机械臂发送任何轨迹。"),
      QStringLiteral("Offline preview never sends trajectories to a physical robot.")));
  }
  teach_sequence_pause_button_->setEnabled(
    !teach_pending && teach_preview_active_.load() && sequence_preview_source &&
    teach_pause_client_ && teach_pause_client_->service_is_ready());
  teach_sequence_pause_button_->setText(
    teach_preview_paused_.load() ?
    localized(QStringLiteral("继续 RViz 动画"), QStringLiteral("Resume RViz animation")) :
    localized(QStringLiteral("暂停 RViz 动画"), QStringLiteral("Pause RViz animation")));
  teach_sequence_cancel_button_->setEnabled(
    !teach_pending &&
    ((teach_state == TEACH_FAULT && reset_ready) ||
    ((teach_preview_active_.load() || teach_state == TEACH_RECORDING ||
    teach_state == TEACH_SEQUENCE_PLANNING ||
    teach_state == TEACH_REPLAY_STARTING || teach_state == TEACH_REPLAYING ||
    teach_state == TEACH_CANCELLING) && teach_cancel_client_ &&
    teach_cancel_client_->service_is_ready())));
  teach_sequence_cancel_button_->setText(
    teach_state == TEACH_FAULT ? localized(
      QStringLiteral("检查并解除故障"), QStringLiteral("Check and Reset Fault")) :
    teach_state == TEACH_SEQUENCE_PLANNING ? localized(
      QStringLiteral("取消过渡规划"),
      QStringLiteral("Cancel Transition Planning")) :
    (teach_state == TEACH_REPLAY_STARTING || teach_state == TEACH_REPLAYING ||
    teach_state == TEACH_CANCELLING) ? localized(
      QStringLiteral("取消动作组回放并回原点"),
      QStringLiteral("Cancel sequence and return Home")) :
    localized(QStringLiteral("取消 RViz 动画"), QStringLiteral("Cancel RViz animation")));
}

void DemoPanel::readProcessOutput()
{
  const QString output = QString::fromLocal8Bit(process_->readAllStandardOutput());
  const QString combined_output = pulse_process_output_tail_ + output;
  if (combined_output.contains(QStringLiteral("result=DUMMY_PRESSURE_STOPPED"))) {
    pulse_dummy_pressure_stopped_ = true;
  }
  pulse_process_output_tail_ = combined_output.right(256);
  const QString line = lastNonEmptyLine(output);
  if (!line.isEmpty()) {
    setStatus(
      QStringLiteral("%1：%2").arg(operationTitle(active_executable_, true), line),
      QStringLiteral("%1: %2").arg(operationTitle(active_executable_, false), line),
      line.contains(QStringLiteral("ERROR"), Qt::CaseInsensitive));
  }
}

void DemoPanel::processFinished(int exit_code)
{
  const QString remaining = QString::fromLocal8Bit(process_->readAllStandardOutput());
  const QString combined_output = pulse_process_output_tail_ + remaining;
  if (combined_output.contains(QStringLiteral("result=DUMMY_PRESSURE_STOPPED"))) {
    pulse_dummy_pressure_stopped_ = true;
  }
  pulse_process_output_tail_ = combined_output.right(256);
  const QString executable = active_executable_;
  active_executable_.clear();
  if (exit_code == 0 && executable == QStringLiteral("piper_pulse_align")) {
    setStatus(pulse_dummy_pressure_stopped_ ?
      QStringLiteral("假体测试：压力变化已触发取消，执行动作已返回。") :
      QStringLiteral("假体测试：已到达视觉目标；未检测到压力停止触发。"),
      pulse_dummy_pressure_stopped_ ?
      QStringLiteral("Dummy test: pressure change cancelled the trajectory and the action returned.") :
      QStringLiteral("Dummy test: visual target reached without a pressure-stop trigger."));
  } else if (exit_code == 0) {
    setStatus(
      QStringLiteral("%1已完成。").arg(operationTitle(executable, true)),
      QStringLiteral("%1 completed.").arg(operationTitle(executable, false)));
  } else {
    setStatus(
      QStringLiteral("%1已停止或失败（退出码 %2）。")
      .arg(operationTitle(executable, true)).arg(exit_code),
      QStringLiteral("%1 stopped or failed (exit code %2).")
      .arg(operationTitle(executable, false)).arg(exit_code), true);
  }
  updateReadiness();
}

void DemoPanel::setStatus(const QString & zh, const QString & en, bool error)
{
  status_zh_ = zh;
  status_en_ = en;
  status_error_ = error;
  status_label_->setText(localized(status_zh_, status_en_));
  status_label_->setStyleSheet(
    status_error_ ? QStringLiteral("color: #c43b32;") : QString());
}

void DemoPanel::showCameraImage(const QImage & image)
{
  latest_camera_image_ = image;
  camera_view_label_->setPixmap(QPixmap::fromImage(latest_camera_image_).scaled(
      camera_view_label_->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
  camera_image_visible_ = true;
}

void DemoPanel::showHandVisionImage(const QImage & image)
{
  if (!hand_vision_view_label_ || image.isNull()) {return;}
  latest_hand_vision_image_ = image;
  hand_vision_view_label_->setPixmap(QPixmap::fromImage(latest_hand_vision_image_).scaled(
      hand_vision_view_label_->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
  if (hand_vision_large_dialog_ && hand_vision_large_dialog_->isVisible() &&
    hand_vision_large_label_)
  {
    hand_vision_large_label_->setPixmap(QPixmap::fromImage(latest_hand_vision_image_).scaled(
        hand_vision_large_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }
  hand_vision_image_visible_ = true;
  hand_vision_status_label_->setStyleSheet(QString());
  hand_vision_status_label_->setText(localized(
    QStringLiteral("已收到手部视觉标注图像。"),
    QStringLiteral("Receiving annotated hand vision images.")));
}

void DemoPanel::showHandVisionLargeView()
{
  if (!hand_vision_large_dialog_) {
    hand_vision_large_dialog_ = new QDialog(this, Qt::Window);
    hand_vision_large_dialog_->setObjectName(QStringLiteral("hand_vision_large_dialog"));
    hand_vision_large_dialog_->setModal(false);
    auto * layout = new QVBoxLayout(hand_vision_large_dialog_);
    layout->setContentsMargins(0, 0, 0, 0);
    hand_vision_large_label_ = new QLabel(hand_vision_large_dialog_);
    hand_vision_large_label_->setObjectName(QStringLiteral("hand_vision_large_label"));
    hand_vision_large_label_->setAlignment(Qt::AlignCenter);
    hand_vision_large_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    hand_vision_large_label_->setStyleSheet(QStringLiteral("background: black; color: white;"));
    layout->addWidget(hand_vision_large_label_);
  }
  hand_vision_large_dialog_->setWindowTitle(localized(
    QStringLiteral("手部视觉标注图像"),
    QStringLiteral("Annotated Hand Vision Image")));
  if (latest_hand_vision_image_.isNull()) {
    hand_vision_large_label_->clear();
    hand_vision_large_label_->setText(localized(
      QStringLiteral("等待手部视觉标注图像…"),
      QStringLiteral("Waiting for annotated hand vision image…")));
  } else {
    hand_vision_large_label_->setPixmap(QPixmap::fromImage(latest_hand_vision_image_).scaled(
        hand_vision_large_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }
  hand_vision_large_dialog_->showMaximized();
  hand_vision_large_dialog_->raise();
  hand_vision_large_dialog_->activateWindow();
  QTimer::singleShot(0, hand_vision_large_dialog_, [this]() {
    if (!latest_hand_vision_image_.isNull() && hand_vision_large_label_) {
      hand_vision_large_label_->setPixmap(QPixmap::fromImage(latest_hand_vision_image_).scaled(
          hand_vision_large_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
  });
}

void DemoPanel::subscribeCameraTopic(const QString & topic)
{
  if (!node_ || topic.trimmed().isEmpty()) {return;}
  camera_image_topic_ = topic.trimmed();
  last_camera_frame_ms_.store(0);
  camera_image_sub_.reset();
  camera_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    camera_image_topic_.toStdString(), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::SharedPtr message) {
      const auto now = steadyMilliseconds();
      last_camera_image_ms_.store(now);
      const auto previous_frame = last_camera_frame_ms_.exchange(now);
      auto previous_render = last_camera_render_ms_.load();
      if (now - previous_render < 66 ||
        !last_camera_render_ms_.compare_exchange_strong(previous_render, now)) {return;}
      const QImage image = imageMessageToQImage(*message);
      const QString encoding = QString::fromStdString(message->encoding);
      const int width = static_cast<int>(message->width);
      const int height = static_cast<int>(message->height);
      const double fps = previous_frame > 0 && now > previous_frame ?
        1000.0 / static_cast<double>(now - previous_frame) : 0.0;
      QPointer<DemoPanel> self(this);
      QMetaObject::invokeMethod(this, [self, image, encoding, width, height, fps]() {
        if (!self) {return;}
        self->camera_stream_info_label_->setText(self->localized(
          QStringLiteral("当前图像：%1 × %2 · 编码 %3 · 约 %4 FPS")
          .arg(width).arg(height).arg(encoding).arg(fps, 0, 'f', 1),
          QStringLiteral("Current image: %1 x %2 · %3 · about %4 FPS")
          .arg(width).arg(height).arg(encoding).arg(fps, 0, 'f', 1)));
        if (image.isNull()) {
          self->camera_tab_status_label_->setText(self->localized(
            QStringLiteral("收到图像，但暂不支持编码：%1").arg(encoding),
            QStringLiteral("Image received, but encoding is unsupported: %1").arg(encoding)));
          return;
        }
        self->showCameraImage(image);
      }, Qt::QueuedConnection);
    });
  if (camera_topic_combo_) {
    const QSignalBlocker blocker(camera_topic_combo_);
    const int index = camera_topic_combo_->findText(camera_image_topic_);
    if (index >= 0) {camera_topic_combo_->setCurrentIndex(index);}
  }
  camera_tab_status_label_->setText(localized(
    QStringLiteral("已选择图像话题：%1").arg(camera_image_topic_),
    QStringLiteral("Selected image topic: %1").arg(camera_image_topic_)));
}

void DemoPanel::refreshCameraTopics()
{
  if (!node_ || !camera_topic_combo_) {return;}
  QStringList topics;
  for (const auto & entry : node_->get_topic_names_and_types()) {
    const bool image_type = std::find(
      entry.second.begin(), entry.second.end(), "sensor_msgs/msg/Image") != entry.second.end();
    if (image_type) {topics.append(QString::fromStdString(entry.first));}
  }
  topics.sort();
  if (!topics.contains(camera_image_topic_)) {topics.prepend(camera_image_topic_);}
  {
    const QSignalBlocker blocker(camera_topic_combo_);
    camera_topic_combo_->clear();
    camera_topic_combo_->addItems(topics);
    const int index = camera_topic_combo_->findText(camera_image_topic_);
    camera_topic_combo_->setCurrentIndex(index >= 0 ? index : 0);
  }
  camera_topic_refresh_button_->setToolTip(localized(
    QStringLiteral("已检索 %1 个 sensor_msgs/Image 话题").arg(topics.size()),
    QStringLiteral("Found %1 sensor_msgs/Image topics").arg(topics.size())));
}

void DemoPanel::selectCameraTopic(const QString & topic)
{
  if (initialized_ && !topic.trimmed().isEmpty() && topic != camera_image_topic_) {
    subscribeCameraTopic(topic);
    last_camera_image_ms_.store(0);
    camera_image_visible_ = false;
    camera_view_label_->clear();
  }
}

void DemoPanel::retranslateUi()
{
  translateMotionPlanningUi(chinese_);
  const bool piper = active_robot_ == QStringLiteral("piperh");
  setWindowTitle(localized(
    piper ? QStringLiteral("Piper-H 综合控制") : QStringLiteral("reBot 综合控制"),
    piper ? QStringLiteral("Piper-H Integrated Control") :
    QStringLiteral("reBot Integrated Control")));
  main_tabs_->setTabText(0, localized(
    QStringLiteral("真机与安全"), QStringLiteral("Robot & Safety")));
  main_tabs_->setTabText(1, localized(
    QStringLiteral("演示与动作"), QStringLiteral("Demos & Actions")));
  main_tabs_->setTabText(2, localized(
    QStringLiteral("摄像头"), QStringLiteral("Camera")));
  main_tabs_->setTabText(3, localized(
    QStringLiteral("手部视觉"), QStringLiteral("Hand Vision")));
  main_tabs_->setTabText(4, localized(
    QStringLiteral("手眼标定"), QStringLiteral("Hand-Eye Calibration")));
  if (auto * title = findChild<QLabel *>(QStringLiteral("hand_vision_title"))) {
    title->setText(localized(QStringLiteral("手部视觉与手势识别"), QStringLiteral("Hand vision and gesture recognition")));
  }
  if (auto * hint = findChild<QLabel *>(QStringLiteral("hand_vision_hint"))) {
    hint->setText(localized(
      piper ? QStringLiteral(
        "仅限非人体假体测试：识别稳定目标后，保持 J6 姿态向目标运动；压力异常请求取消，非力控制。") :
      QStringLiteral("识别 MediaPipe 手部关键点、对齐深度和桡侧腕部近似把脉区域。绿色 PULSE READY 后可经确认前往预接触位；视觉不会命令探头接触人体。"),
      piper ? QStringLiteral(
        "Non-human dummy only: after stable target detection, move toward it with J6 fixed; pressure requests cancellation, not force control.") :
      QStringLiteral("Detects MediaPipe landmarks, registered depth, and an approximate radial-wrist pulse region. After green PULSE READY, confirmed motion may go only to pre-contact; vision never commands human contact.")));
  }
  hand_vision_topic_label_->setText(localized(
    QStringLiteral("彩色摄像头话题："), QStringLiteral("Color camera topic:")));
  hand_vision_topic_refresh_button_->setText(localized(
    QStringLiteral("刷新"), QStringLiteral("Refresh")));
  hand_vision_topic_refresh_button_->setToolTip(localized(
    QStringLiteral("检索可用的 sensor_msgs/Image 话题。切换后需重启手部视觉。"),
    QStringLiteral("Discover sensor_msgs/Image topics. Restart hand vision after switching.")));
  hand_vision_status_label_->setText(hand_vision_process_->state() == QProcess::NotRunning ?
    localized(QStringLiteral("手部视觉未运行。"), QStringLiteral("Hand vision is not running.")) :
    localized(QStringLiteral("手部视觉运行中。"), QStringLiteral("Hand vision is running.")));
  if (!hand_vision_image_visible_) {
    hand_vision_view_label_->setText(localized(
      QStringLiteral("等待手部视觉标注图像…"),
      QStringLiteral("Waiting for annotated hand vision image…")));
  }
  hand_vision_button_->setText(hand_vision_process_->state() == QProcess::NotRunning ?
    localized(QStringLiteral("启动手部视觉"), QStringLiteral("Start Hand Vision")) :
    localized(QStringLiteral("停止手部视觉"), QStringLiteral("Stop Hand Vision")));
  hand_vision_large_button_->setText(localized(
    QStringLiteral("独立大屏显示"), QStringLiteral("Open Large View")));
  if (hand_vision_large_dialog_) {
    hand_vision_large_dialog_->setWindowTitle(localized(
      QStringLiteral("手部视觉标注图像"),
      QStringLiteral("Annotated Hand Vision Image")));
  }
  if ((piper ? last_piper_pulse_point_ms_.load() : last_pulse_point_ms_.load()) == 0) {
    pulse_region_label_->setText(localized(
      QStringLiteral("等待稳定把脉区域。"),
      QStringLiteral("Waiting for a stable pulse region.")));
  }
  if (auto * label = findChild<QLabel *>(QStringLiteral("pulse_standoff_label"))) {
    label->setVisible(!piper);
    label->setText(localized(
      QStringLiteral("预接触安全距离："), QStringLiteral("Pre-contact standoff:")));
  }
  pulse_standoff_spin_->setVisible(!piper);
  pulse_approach_button_->setText(localized(
    piper ? QStringLiteral("假体测试：前往视觉目标（压力停止）") :
    QStringLiteral("识别把脉区并前往预接触位"),
    piper ? QStringLiteral("Dummy Test: Move to Target (Pressure Stop)") :
    QStringLiteral("Recognize Pulse Region & Move to Pre-contact")));
  pulse_approach_button_->setToolTip(localized(
    piper ? QStringLiteral("仅限非人体假体；保持 J6 姿态，XY ≤300 mm、Z ≤200 mm，压力异常请求取消。") :
    QStringLiteral("只执行经 MoveIt 碰撞检查的预接触运动，不接触人体、不施加压力。"),
    piper ? QStringLiteral("Non-human dummy only; J6 fixed, XY ≤300 mm, Z ≤200 mm, pressure requests cancellation.") :
    QStringLiteral("Runs only a collision-checked MoveIt pre-contact motion; no human contact or pressure.")));
  if (auto * title = findChild<QLabel *>(QStringLiteral("handeye_title"))) {
    title->setText(localized(QStringLiteral("手眼标定"), QStringLiteral("Hand-eye calibration")));
  }
  if (auto * hint = findChild<QLabel *>(QStringLiteral("handeye_hint"))) {
    const bool moveit_backend = handeyeUsesMoveItCalibration();
    const QString mode_zh = handeyeEyeInHand() ?
      QStringLiteral("眼在手上（相机装末端、标签固定）") :
      (piper ? QStringLiteral("眼在手外（相机固定、标签装末端法兰）") :
      QStringLiteral("眼在手外（相机固定、标签装夹爪）"));
    const QString mode_en = handeyeEyeInHand() ?
      QStringLiteral("eye-in-hand (camera on effector, fixed tag)") :
      (piper ? QStringLiteral("eye-on-base (fixed camera, tag on effector flange)") :
      QStringLiteral("eye-on-base (fixed camera, tag on gripper)"));
    const QString hint_zh = moveit_backend ? QStringLiteral(
      "当前为%1，求解器为 MoveIt Calibration。启动后将在 RViz 打开独立标定窗口；"
      "请在那里确认 ChArUco 目标、4 个 frame 和 arm 规划组并手动采样。自动动作组暂不可用。")
      .arg(mode_zh) : QStringLiteral(
      "当前为%1。按钮会启动 AprilTag 跟踪和 easy_handeye2；可手动采集至少 12 个多轴姿态，或到“动作组编排”预览并执行“%2”，自动采样、计算、保存并发布 TF。")
      .arg(mode_zh).arg(automaticHandeyeSequenceName());
    const QString hint_en = moveit_backend ? QStringLiteral(
      "Current mode: %1 with MoveIt Calibration. Starting opens its RViz calibration window; "
      "verify the ChArUco target, four frames, and arm planning group there, then sample manually. "
      "Automatic sequences are currently unavailable.")
      .arg(mode_en) : QStringLiteral(
      "Current mode: %1. The button starts AprilTag tracking and easy_handeye2. Collect 12+ multi-axis poses manually, or preview and execute “%2” in Sequence Builder for automatic sampling, computation, saving, and TF publishing.")
      .arg(mode_en).arg(automaticHandeyeSequenceName());
    hint->setText(localized(hint_zh, hint_en));
  }
  const auto set_handeye_label = [this](const QString & object_name, const QString & zh, const QString & en) {
      if (auto * label = findChild<QLabel *>(object_name)) {label->setText(localized(zh, en));}
    };
  set_handeye_label(QStringLiteral("handeye_backend_label"), QStringLiteral("标定包"), QStringLiteral("Calibration package"));
  set_handeye_label(QStringLiteral("handeye_mode_label"), QStringLiteral("标定模式"), QStringLiteral("Calibration mode"));
  set_handeye_label(QStringLiteral("handeye_target_label"), QStringLiteral("标定目标"), QStringLiteral("Calibration target"));
  set_handeye_label(QStringLiteral("handeye_name_label"), QStringLiteral("名称"), QStringLiteral("Name"));
  set_handeye_label(QStringLiteral("handeye_robot_base_label"), QStringLiteral("机器人基座 frame"), QStringLiteral("Robot base frame"));
  set_handeye_label(QStringLiteral("handeye_robot_effector_label"), QStringLiteral("机器人末端 frame"), QStringLiteral("Robot effector frame"));
  set_handeye_label(QStringLiteral("handeye_tracking_base_label"), QStringLiteral("跟踪基座 frame"), QStringLiteral("Tracking base frame"));
  set_handeye_label(QStringLiteral("handeye_tracking_marker_label"), QStringLiteral("跟踪标记 frame"), QStringLiteral("Tracking marker frame"));
  set_handeye_label(QStringLiteral("handeye_tag_family_label"), QStringLiteral("AprilTag 家族"), QStringLiteral("AprilTag family"));
  set_handeye_label(QStringLiteral("handeye_tag_id_label"), QStringLiteral("AprilTag ID"), QStringLiteral("AprilTag ID"));
  set_handeye_label(QStringLiteral("handeye_tag_size_label"), QStringLiteral("AprilTag 黑框边长"), QStringLiteral("AprilTag black-square edge"));
  if (handeye_backend_combo_->count() >= 2) {
    handeye_backend_combo_->setItemText(0, QStringLiteral("easy_handeye2"));
    handeye_backend_combo_->setItemText(1, QStringLiteral("MoveIt Calibration"));
  }
  if (handeye_mode_combo_->count() >= 2) {
    handeye_mode_combo_->setItemText(0, localized(
      QStringLiteral("眼在手外（固定相机）"), QStringLiteral("Eye-on-base (fixed camera)")));
    handeye_mode_combo_->setItemText(1, localized(
      QStringLiteral("眼在手上（末端相机）"), QStringLiteral("Eye-in-hand (camera on effector)")));
  }
  if (handeye_target_combo_->count() >= 3) {
    handeye_target_combo_->setItemText(0, localized(
      QStringLiteral("单个 AprilTag"), QStringLiteral("Single AprilTag")));
    handeye_target_combo_->setItemText(1, localized(
      QStringLiteral("A4 四标签板（ID 0–3）"),
      QStringLiteral("A4 four-tag board (IDs 0–3)")));
    handeye_target_combo_->setItemText(2, localized(
      QStringLiteral("A4 ChArUco 板（5×7 / 35 / 26 mm）"),
      QStringLiteral("A4 ChArUco board (5×7 / 35 / 26 mm)")));
  }
  handeye_status_label_->setText(handeye_process_->state() != QProcess::NotRunning ?
    localized(QStringLiteral("手眼标定运行中。"), QStringLiteral("Hand-eye calibration is running.")) :
    handeye_publish_process_->state() != QProcess::NotRunning ? localized(
    QStringLiteral("已保存标定并正在发布 TF。"),
    QStringLiteral("Saved calibration is being published as TF.")) : localized(
    QStringLiteral("手眼标定未运行。"), QStringLiteral("Hand-eye calibration is not running.")));
  handeye_button_->setText(handeye_process_->state() == QProcess::NotRunning ?
    localized(QStringLiteral("启动手眼标定"), QStringLiteral("Start Hand-eye Calibration")) :
    localized(QStringLiteral("停止手眼标定"), QStringLiteral("Stop Hand-eye Calibration")));
  teach_tabs_->setTabText(0, localized(
    QStringLiteral("单动作与形状"), QStringLiteral("Actions & Shapes")));
  teach_tabs_->setTabText(1, localized(
    QStringLiteral("真机演示操作"), QStringLiteral("Robot Demo Controls")));
  teach_tabs_->setTabText(2, localized(
    QStringLiteral("动作组编排"), QStringLiteral("Sequence Builder")));
  teach_tabs_->setTabText(3, localized(
    QStringLiteral("自由运动规划"), QStringLiteral("Free Motion Planning")));
  teach_action_mode_tabs_->setTabText(0, localized(
    QStringLiteral("新建 / 编辑参数"), QStringLiteral("Create / Edit Parameters")));
  teach_action_mode_tabs_->setTabText(1, localized(
    QStringLiteral("RViz / 真机播放"), QStringLiteral("RViz / Robot Playback")));
  planning_placeholder_->setText(localized(
    QStringLiteral("正在载入 MoveIt 自由运动规划…"),
    QStringLiteral("Loading MoveIt Free Motion Planning…")));
  language_label_->setText(localized(
    QStringLiteral("界面语言："), QStringLiteral("Interface language:")));
  connection_box_->setTitle(localized(
    QStringLiteral("当前控制链路"), QStringLiteral("Current control connection")));
  forbidden_zone_box_->setTitle(localized(
    QStringLiteral("三维禁止通行区域"), QStringLiteral("3-D Forbidden Zones")));
  forbidden_zone_reload_button_->setText(localized(
    QStringLiteral("重新加载禁区"), QStringLiteral("Reload Forbidden Zones")));
  forbidden_zone_reload_button_->setToolTip(localized(
    QStringLiteral("重新读取 YAML 并原子更新 MoveIt Planning Scene；不会移动真机。"),
    QStringLiteral(
      "Re-read YAML and atomically update the MoveIt Planning Scene; hardware will not move.")));
  forbidden_zone_hint_label_->setText(localized(
    QStringLiteral(
      "红色表示已加入 MoveIt 的碰撞禁区；用户禁区的红色模型可用箭头平移、圆环旋转，松开后保存。"
      "区域坐标必须先按现场标定，不能直接启用示例值。"),
    QStringLiteral(
      "Red denotes MoveIt collision keepouts. Red user-zone models can be translated with arrows "
      "or rotated with rings and are saved on release. Calibrate coordinates before enabling "
      "example values.")));
  if (auto * editor = findChild<QGroupBox *>(QStringLiteral("forbidden_zone_editor"))) {
    editor->setTitle(localized(
      QStringLiteral("新建或替换用户禁区"),
      QStringLiteral("Create or replace a user zone")));
  }
  forbidden_zone_name_edit_->setPlaceholderText(localized(
    QStringLiteral("禁区名称（1–64 字符）"),
    QStringLiteral("Zone name (1–64 characters)")));
  const QStringList zone_shapes_zh = {
    QStringLiteral("长方体"), QStringLiteral("球体"), QStringLiteral("圆柱体"),
    QStringLiteral("圆锥体"), QStringLiteral("三维文件（STL/OBJ）")};
  const QStringList zone_shapes_en = {
    QStringLiteral("Box"), QStringLiteral("Sphere"), QStringLiteral("Cylinder"),
    QStringLiteral("Cone"), QStringLiteral("3D file (STL/OBJ)")};
  for (int index = 0; index < forbidden_zone_shape_combo_->count(); ++index) {
    forbidden_zone_shape_combo_->setItemText(
      index, chinese_ ? zone_shapes_zh[index] : zone_shapes_en[index]);
  }
  forbidden_zone_pose_label_->setText(localized(
    QStringLiteral("位置 XYZ（米）与姿态 RPY（度），基座坐标系"),
    QStringLiteral("XYZ position (m) and RPY orientation (deg), base frame")));
  forbidden_zone_mesh_button_->setText(localized(
    QStringLiteral("选择三维文件…"), QStringLiteral("Choose 3D file…")));
  forbidden_zone_mesh_path_label_->setText(
    forbidden_zone_mesh_path_.isEmpty() ? localized(
      QStringLiteral("尚未选择"), QStringLiteral("None selected")) :
    QFileInfo(forbidden_zone_mesh_path_).fileName());
  forbidden_zone_mesh_scale_label_->setText(localized(
    QStringLiteral("模型统一缩放（毫米模型通常填 0.001）："),
    QStringLiteral("Uniform mesh scale (usually 0.001 for mm models):")));
  forbidden_zone_apply_button_->setText(localized(
    QStringLiteral("导入并加入 MoveIt 禁区"),
    QStringLiteral("Import into MoveIt forbidden zones")));
  forbidden_zone_apply_button_->setToolTip(localized(
    QStringLiteral("验证并持久化该区域，然后更新规划场景；不会移动真机。"),
    QStringLiteral("Validate and persist this area, then update the planning scene; no motion.")));
  forbidden_zone_remove_button_->setText(localized(
    QStringLiteral("删除所选用户禁区"), QStringLiteral("Remove selected user zone")));
  if (auto * group_editor = findChild<QGroupBox *>(
      QStringLiteral("forbidden_zone_group_editor")))
  {
    group_editor->setTitle(localized(
      QStringLiteral("命名禁区组"), QStringLiteral("Named zone groups")));
  }
  forbidden_zone_group_name_edit_->setPlaceholderText(localized(
    QStringLiteral("禁区组名称（1–64 字符）"),
    QStringLiteral("Zone group name (1–64 characters)")));
  forbidden_zone_group_members_list_->setToolTip(localized(
    QStringLiteral("按住 Ctrl 可选择多个用户禁区作为组成员。"),
    QStringLiteral("Hold Ctrl to select multiple user zones as members.")));
  forbidden_zone_group_save_button_->setText(localized(
    QStringLiteral("保存或替换命名禁区组"),
    QStringLiteral("Save or replace named zone group")));
  forbidden_zone_group_remove_button_->setText(localized(
    QStringLiteral("删除所选组"), QStringLiteral("Remove selected group")));
  updateForbiddenZoneEditor();
  updateForbiddenZoneUi();
  camera_label_->setText(localized(
    QStringLiteral("Gemini 2：等待状态"), QStringLiteral("Gemini 2: waiting for status")));
  xbox_mode_button_->setText(xbox_armed_.load() ? localized(
    QStringLiteral("锁定 Xbox 控制"), QStringLiteral("Lock Xbox Control")) : localized(
    QStringLiteral("启用 Xbox 控制"), QStringLiteral("Enable Xbox Control")));
  xbox_mode_button_->setToolTip(localized(
    QStringLiteral("启用前强制检查手柄在线、输入新鲜、摇杆回中且扳机松开。"),
    QStringLiteral(
      "Before enabling, verifies the controller is online, input is fresh, sticks centered, "
      "and triggers released.")));
  camera_button_->setText(localized(
    QStringLiteral("打开摄像头"), QStringLiteral("Open Camera")));
  camera_button_->setToolTip(localized(
    QStringLiteral("按上方勾选项启动 Orbbec Gemini 2 图像流；彩色与深度同时启用时自动对齐深度。"),
    QStringLiteral(
      "Start the selected Gemini 2 streams; depth is registered when color and depth are both enabled.")));
  camera_tab_button_->setText(camera_button_->text());
  camera_tab_button_->setToolTip(camera_button_->toolTip());
  camera_topic_label_->setText(localized(
    QStringLiteral("图像话题："), QStringLiteral("Image topic:")));
  camera_color_checkbox_->setText(localized(
    QStringLiteral("彩色图传输"), QStringLiteral("Color stream")));
  camera_depth_checkbox_->setText(localized(
    QStringLiteral("深度图传输"), QStringLiteral("Depth stream")));
  camera_ir_checkbox_->setText(localized(
    QStringLiteral("红外图传输"), QStringLiteral("Infrared stream")));
  camera_topic_refresh_button_->setText(localized(
    QStringLiteral("检索摄像头"), QStringLiteral("Discover cameras")));
  camera_topic_refresh_button_->setToolTip(localized(
    QStringLiteral("检索当前 ROS 图像话题并选择要显示的摄像头。"),
    QStringLiteral("Discover ROS image topics and choose the camera to display.")));
  camera_tab_status_label_->setText(localized(
    QStringLiteral("摄像头未打开。点击下方按钮启动 Gemini 2。"),
    QStringLiteral("Camera is off. Use the button below to start Gemini 2.")));
  if (last_camera_image_ms_.load() == 0) {
    camera_stream_info_label_->setText(localized(
      QStringLiteral("图像信息：等待数据（将显示分辨率、编码和帧率）"),
      QStringLiteral("Image info: waiting (resolution, encoding, and frame rate)")));
  }
  if (!camera_image_visible_) {
    camera_view_label_->setText(localized(
      QStringLiteral("等待摄像头画面"), QStringLiteral("Waiting for camera image")));
  }
  xbox_help_button_->setText(xbox_help_button_->isChecked() ? localized(
    QStringLiteral("▼ 收起 Xbox 按键说明"), QStringLiteral("▼ Hide Xbox Controls")) : localized(
    QStringLiteral("▶ 展开 Xbox 按键说明"), QStringLiteral("▶ Show Xbox Controls")));
  xbox_help_button_->setToolTip(localized(
    QStringLiteral("显示当前 Generic X-Box pad 的实测按键映射和可用状态。"),
    QStringLiteral(
      "Show the measured Generic X-Box pad mapping and the state required for each control.")));
  xbox_help_table_->setHorizontalHeaderLabels(chinese_ ? QStringList{
      QStringLiteral("按键/控制"), QStringLiteral("功能"), QStringLiteral("可用状态")} : QStringList{
      QStringLiteral("Button/Control"), QStringLiteral("Function"), QStringLiteral("Available in")});
  const QStringList xbox_controls = {
    QStringLiteral("A"),
    localized(QStringLiteral("左摇杆"), QStringLiteral("Left stick")),
    localized(QStringLiteral("右摇杆"), QStringLiteral("Right stick")),
    localized(QStringLiteral("十字键 ↑/↓"), QStringLiteral("D-pad ↑/↓")),
    QStringLiteral("LB / RB"), QStringLiteral("LT / RT"),
    localized(QStringLiteral("L3 / R3（按下摇杆）"), QStringLiteral("L3 / R3 (stick press)")),
    QStringLiteral("Back / View"),
    localized(QStringLiteral("按住 X / 松开 X"), QStringLiteral("Hold / release X")),
    QStringLiteral("B"), QStringLiteral("Start"), QStringLiteral("Y / Xbox")};
  QStringList xbox_functions_zh = {
    QStringLiteral("切换 Xbox ARMED / LOCKED；也可使用上方按钮"),
    QStringLiteral("上推=-X，右推=+Y 平移"),
    QStringLiteral("上推=-Z 平移，右推=+Yaw"),
    QStringLiteral("控制末端 Pitch"),
    QStringLiteral("LB=-Roll，RB=+Roll"),
    QStringLiteral("LT 按比例打开夹爪，RT 按比例关闭夹爪"),
    QStringLiteral("降低/提高速度档：25% / 50% / 100%"),
    QStringLiteral("切换基座坐标系 / 末端坐标系"),
    QStringLiteral("开始拖动录制 / 停止保存并预览"),
    QStringLiteral("回放当前选中的示教动作"),
    QStringLiteral("立即锁定 Xbox，并取消录制或回放"),
    QStringLiteral("当前未分配")};
  QStringList xbox_functions_en = {
    QStringLiteral("Toggle Xbox ARMED / LOCKED; the button above does the same"),
    QStringLiteral("Push up=-X and right=+Y translation"),
    QStringLiteral("Push up=-Z translation and right=+Yaw"),
    QStringLiteral("Control tool Pitch"),
    QStringLiteral("LB=-Roll and RB=+Roll"),
    QStringLiteral("LT opens and RT closes the gripper proportionally"),
    QStringLiteral("Decrease/increase speed tier: 25% / 50% / 100%"),
    QStringLiteral("Switch base / end-effector command frame"),
    QStringLiteral("Start drag recording / stop, save, and preview"),
    QStringLiteral("Replay the selected teaching action"),
    QStringLiteral("Immediately lock Xbox and cancel recording or replay"),
    QStringLiteral("Currently unassigned")};
  if (piper) {
    xbox_functions_zh[5] = QStringLiteral("Piper-H 无夹爪，LT / RT 不发送末端命令");
    xbox_functions_en[5] = QStringLiteral("Piper-H has no gripper; LT / RT send no tool command");
    xbox_functions_zh[8] = QStringLiteral("使用面板开始/停止 Piper-H 动作录制");
    xbox_functions_en[8] = QStringLiteral("Start/stop Piper-H action recording from the panel");
  }
  const QStringList xbox_states_zh = {
    QStringLiteral("始终；启用需回中"),
    QStringLiteral("仅 ARMED"), QStringLiteral("仅 ARMED"), QStringLiteral("仅 ARMED"),
    QStringLiteral("仅 ARMED"), QStringLiteral("仅 ARMED"),
    QStringLiteral("ARMED / LOCKED"), QStringLiteral("ARMED / LOCKED"),
    QStringLiteral("仅 LOCKED"), QStringLiteral("仅 LOCKED"),
    QStringLiteral("始终"), QStringLiteral("—")};
  const QStringList xbox_states_en = {
    QStringLiteral("Always; center before arming"),
    QStringLiteral("ARMED only"), QStringLiteral("ARMED only"), QStringLiteral("ARMED only"),
    QStringLiteral("ARMED only"), QStringLiteral("ARMED only"),
    QStringLiteral("ARMED / LOCKED"), QStringLiteral("ARMED / LOCKED"),
    QStringLiteral("LOCKED only"), QStringLiteral("LOCKED only"),
    QStringLiteral("Always"), QStringLiteral("—")};
  const auto & xbox_functions = chinese_ ? xbox_functions_zh : xbox_functions_en;
  const auto & xbox_states = chinese_ ? xbox_states_zh : xbox_states_en;
  for (int row = 0; row < xbox_controls.size(); ++row) {
    xbox_help_table_->setItem(row, 0, new QTableWidgetItem(xbox_controls[row]));
    xbox_help_table_->setItem(row, 1, new QTableWidgetItem(xbox_functions[row]));
    xbox_help_table_->setItem(row, 2, new QTableWidgetItem(xbox_states[row]));
  }
  xbox_help_table_->resizeRowsToContents();
  xbox_help_table_->setFixedHeight(390);
  safety_box_->setTitle(localized(
    QStringLiteral("真机操作"), QStringLiteral("Physical robot operations")));
  warning_label_->setText(localized(
    QStringLiteral(
      "真机运动操作已移到“演示与动作”页。执行前将 Xbox 控制设为 LOCKED，保持急停可触达，"
      "工作区无人且无障碍物。"),
    QStringLiteral(
      "Robot motion controls are in the Demos & Actions page. Set Xbox control to LOCKED, "
      "keep the emergency stop reachable, and keep the workspace clear.")));
  speed_label_->setText(localized(
    QStringLiteral("速度/加速度上限："), QStringLiteral("Velocity/acceleration limit:")));
  if (auto * operation_box = findChild<QGroupBox *>(QStringLiteral("demo_operation_box"))) {
    operation_box->setTitle(localized(
      QStringLiteral("真机演示参数与操作"), QStringLiteral("Robot demo parameters and controls")));
  }
  home_button_->setText(localized(
    QStringLiteral("回原点"), QStringLiteral("Return Home")));
  home_button_->setToolTip(localized(
    QStringLiteral("使用 MoveIt 规划并执行到当前型号的 home 命名姿态。"),
    QStringLiteral("Use MoveIt to plan and execute the named home pose for this model.")));
  singularity_escape_box_->setTitle(localized(
    QStringLiteral("脱离奇异位形（关节空间）"),
    QStringLiteral("Escape Singularity (Joint Space)")));
  singularity_escape_hint_label_->setText(localized(
    piper ? QStringLiteral(
      "自定义 J1–J6 目标角（rad）。默认值与 Piper-H 六轴限位匹配；|J3|、|J5| "
      "小于 0.15 rad 时将拒绝执行。路径须通过 Piper-H MoveIt 全程碰撞检查。") : QStringLiteral(
      "自定义 J1–J6 目标角（rad）。DM 默认值将末端收在基座前方约 0.45 m、桌面上方约 "
      "0.33 m；|J3|、|J5| 小于 0.15 rad 时将拒绝执行。路径仍须通过 MoveIt 全程碰撞检查。"),
    piper ? QStringLiteral(
      "Set custom J1–J6 targets in radians. Defaults match Piper-H limits; execution is "
      "rejected when |J3| or |J5| is below 0.15 rad. Piper-H MoveIt collision-checks the path.") :
    QStringLiteral(
      "Set custom J1–J6 targets in radians. The DM default keeps the TCP about 0.45 m in front "
      "of the base and 0.33 m above the tabletop; execution is rejected when |J3| or |J5| is "
      "below 0.15 rad. MoveIt still collision-checks the full path.")));
  singularity_escape_reset_button_->setText(localized(
    QStringLiteral("恢复默认角度"), QStringLiteral("Restore Defaults")));
  singularity_escape_button_->setText(localized(
    QStringLiteral("规划并脱离奇异位形"), QStringLiteral("Plan Singularity Escape")));
  singularity_escape_button_->setToolTip(localized(
    QStringLiteral("仅在 Xbox LOCKED、关节反馈新鲜且 MoveIt 当前状态有效时可用。"),
    QStringLiteral(
      "Available only while Xbox is LOCKED with fresh feedback and a valid MoveIt state.")));
  sync_button_->setText(localized(
    QStringLiteral("规划机械臂同步到真机"), QStringLiteral("Sync Planned Arm to Robot")));
  sync_button_->setToolTip(localized(
    QStringLiteral("将规划起点和目标更新为最新真机关节反馈；不会驱动电机。"),
    QStringLiteral(
      "Update planning start and goal states from current joint feedback; motors do not move.")));
  pick_place_button_->setText(localized(
    QStringLiteral("抓取放置演示"), QStringLiteral("Pick-and-Place Demo")));
  stop_button_->setText(localized(
    QStringLiteral("停止当前操作"), QStringLiteral("Stop Current Operation")));
  teach_box_->setTitle(localized(
    QStringLiteral("动作编辑与独立播放"),
    QStringLiteral("Action editing and separate playback")));
  teach_refresh_button_->setText(localized(
    QStringLiteral("刷新"), QStringLiteral("Refresh")));
  if (teach_action_combo_->count() > 0 &&
    teach_action_combo_->itemData(0).toString().isEmpty())
  {
    teach_action_combo_->setItemText(0, localized(
      QStringLiteral("＋ 增加动作"), QStringLiteral("+ Add action")));
  }
  teach_shape_box_->setTitle(localized(
    QStringLiteral("绘图形状动作包"), QStringLiteral("Drawing shape action pack")));
  const bool editing_saved_shape = !editing_shape_action_name_.isEmpty();
  const bool show_shape_editor = shape_editor_new_mode_ || editing_saved_shape;
  teach_shape_box_->setVisible(show_shape_editor);
  teach_recording_widget_->setVisible(shape_editor_new_mode_);
  teach_shape_new_button_->setVisible(false);
  teach_shape_combo_->setVisible(show_shape_editor);
  teach_shape_preview_label_->setVisible(show_shape_editor);
  if (!editing_shape_action_name_.isEmpty()) {
    teach_shape_edit_status_label_->setText(localized(
      QStringLiteral("正在编辑动作包“%1”：可更换形状或修改参数，再重新规划并覆盖保存。")
      .arg(editing_shape_action_name_),
      QStringLiteral(
        "Editing action pack “%1”: change its shape or parameters, then replan and replace it.")
      .arg(editing_shape_action_name_)));
    teach_shape_edit_status_label_->setStyleSheet(
      QStringLiteral("color: #2b9e55; font-weight: bold;"));
  } else if (shape_editor_new_mode_) {
    teach_shape_edit_status_label_->setText(localized(
      QStringLiteral("新建模式：调整下方参数后生成新的动作包。"),
      QStringLiteral("New-action mode: adjust the parameters below, then generate a new pack.")));
    teach_shape_edit_status_label_->setStyleSheet(QString());
  } else if (selected_legacy_shape_action_) {
    teach_shape_edit_status_label_->setText(localized(
      QStringLiteral(
        "这是改造前生成的旧版绘图动作包，文件从未保存位姿、尺寸和笔参数，因此不能恢复，"
        "也不会用当前默认值冒充历史值。请在“＋ 增加动作”中重新建立一次。"),
      QStringLiteral(
        "This drawing pack predates parameter storage, so its pose, size, and pen settings do not "
        "exist and current defaults will not be presented as historical values. Re-create it once "
        "through + Add action.")));
    teach_shape_edit_status_label_->setStyleSheet(QStringLiteral("color: #d08020;"));
  } else {
    teach_shape_edit_status_label_->setText(localized(
      QStringLiteral("当前是拖动录制动作，没有图形参数；仍可在播放页使用。"),
      QStringLiteral(
        "This is a drag-recorded action without shape parameters; it remains available on the "
        "playback tab.")));
    teach_shape_edit_status_label_->setStyleSheet(QStringLiteral("color: #d08020;"));
  }
  const QStringList shape_zh = {
    QStringLiteral("矩形"), QStringLiteral("三角形"), QStringLiteral("圆形"),
    QStringLiteral("五角星"), QStringLiteral("心形"),
    QStringLiteral("英文/数字/符号"), QStringLiteral("图片转简笔画"),
    QStringLiteral("不添加图形（自由拖动录制）")};
  const QStringList shape_en = {
    QStringLiteral("Rectangle"), QStringLiteral("Triangle"), QStringLiteral("Circle"),
    QStringLiteral("Five-point star"), QStringLiteral("Heart"),
    QStringLiteral("Letters / digits / symbols"), QStringLiteral("Image to line drawing"),
    QStringLiteral("No shape (freehand recording)")};
  for (int index = 0; index < teach_shape_combo_->count(); ++index) {
    teach_shape_combo_->setItemText(index, chinese_ ? shape_zh[index] : shape_en[index]);
  }
  teach_shape_create_button_->setText(!editing_shape_action_name_.isEmpty() ? localized(
    QStringLiteral("保存修改并重新规划"), QStringLiteral("Save changes and replan")) : localized(
    QStringLiteral("生成新动作并保存"), QStringLiteral("Generate and save new action")));
  teach_shape_create_button_->setToolTip(localized(
    QStringLiteral("使用 RViz 中当前形状位姿进行碰撞检查规划并保存；生成时真机不运动。"),
    QStringLiteral(
      "Plan and save from the current RViz shape pose with collision checking; hardware does not move.")));
  const bool freehand_shape =
    teach_shape_combo_->currentData().toString() == QStringLiteral("freehand");
  const bool text_shape =
    teach_shape_combo_->currentData().toString() == QStringLiteral("text");
  const bool image_shape =
    teach_shape_combo_->currentData().toString() == QStringLiteral("image");
  teach_shape_source_edit_->setVisible(text_shape);
  teach_shape_image_button_->setVisible(image_shape);
  teach_shape_source_label_->setVisible(image_shape);
  teach_shape_source_edit_->setPlaceholderText(localized(
    QStringLiteral("最多 24 个：A-Z、0-9 和常用符号"),
    QStringLiteral("Up to 24: A-Z, 0-9 and common symbols")));
  teach_shape_image_button_->setText(localized(
    QStringLiteral("选择图片…"), QStringLiteral("Choose image…")));
  teach_shape_source_label_->setText(
    teach_shape_image_path_.isEmpty() ? localized(
      QStringLiteral("尚未选择"), QStringLiteral("None selected")) :
    QFileInfo(teach_shape_image_path_).fileName());
  teach_shape_source_label_->setToolTip(teach_shape_image_path_);
  teach_shape_selected_label_->setText(editing_saved_shape ? localized(
    QStringLiteral("当前编辑形状：%1（可从上方更换）").arg(teach_shape_combo_->currentText()),
    QStringLiteral("Editing shape: %1 (change it above)").arg(teach_shape_combo_->currentText())) :
    freehand_shape ? localized(
    QStringLiteral("当前模式：自由拖动录制（不添加图形）"),
    QStringLiteral("Mode: freehand recording (no shape)")) : text_shape ? localized(
    QStringLiteral("当前选中：字符路径"), QStringLiteral("Selected: text path")) :
    image_shape ? localized(
    QStringLiteral("当前选中：图片简笔画路径"),
    QStringLiteral("Selected: image line-art path")) : localized(
    QStringLiteral("当前选中形状：%1").arg(teach_shape_combo_->currentText()),
    QStringLiteral("Selected shape: %1").arg(teach_shape_combo_->currentText())));
  teach_shape_position_label_->setText(localized(
    QStringLiteral("空间位置（基座坐标系）"), QStringLiteral("Position (base frame)")));
  teach_shape_orientation_label_->setText(localized(
    QStringLiteral("空间方向（角度）"), QStringLiteral("Orientation (degrees)")));
  teach_shape_size_label_->setText(localized(
    QStringLiteral("图形大小（实际空间尺寸）"),
    QStringLiteral("Shape size (physical dimensions)")));
  teach_shape_pen_label_->setText(localized(
    QStringLiteral("虚拟笔参数（L：夹爪前端外露长度 / ↑：朝腕部抬笔）"),
    QStringLiteral("Virtual pen (L: exposed beyond gripper / ↑: lift towards wrist)")));
  teach_shape_apply_pose_button_->setText(localized(
    QStringLiteral("应用位置、方向、大小和笔参数"),
    QStringLiteral("Apply pose, size and pen settings")));
  teach_shape_reachability_button_->setText(localized(
    QStringLiteral("预检当前笔尖轨迹（不动真机）"),
    QStringLiteral("Check current pen-tip path (no motion)")));
  teach_shape_reachability_button_->setToolTip(localized(
    QStringLiteral("逐点检查带虚拟笔模型的碰撞 IK、关节余量和无解区域；失败时搜索附近可达位置。"),
    QStringLiteral("Check collision-aware IK with the virtual pen, joint margin, and no-IK regions; "
      "search for a nearby feasible pose on failure.")));
  if (!shape_reachability_available_) {
    teach_shape_reachability_label_->setText(localized(
      QStringLiteral("尚未预检当前笔尖轨迹。"),
      QStringLiteral("Current pen-tip path has not been checked.")));
    teach_shape_reachability_label_->setStyleSheet(QStringLiteral("color: #d08020;"));
  }
  teach_shape_marker_button_->setText(shape_marker_visible_.load() ? localized(
    QStringLiteral("隐藏空间定位标记"), QStringLiteral("Hide pose marker")) : localized(
    QStringLiteral("显示空间定位标记"), QStringLiteral("Show pose marker")));
  teach_shape_reset_button_->setText(localized(
    QStringLiteral("恢复默认位姿"), QStringLiteral("Reset pose")));
  teach_shape_marker_hint_label_->setText(freehand_shape ? localized(
    QStringLiteral("不显示或生成预设图形；直接点击下方“开始拖动录制”，用手拖动机械臂录制任意轨迹。"),
    QStringLiteral(
      "No preset shape is shown or generated. Click Start Drag Recording below and "
      "move the arm by hand to record any path.")) : text_shape ? localized(
    QStringLiteral(
      "输入英文、数字或常用符号；字符会转换为少量单线笔画，不相连笔画间自动抬笔。"
      "三维侧视会压缩平面内容，请用上方正视预览核对实际落笔内容。"),
    QStringLiteral(
      "Enter letters, digits, or common symbols. They become compact single-line strokes, "
      "with automatic pen lifts between disconnected strokes. A side-on 3D view compresses "
      "the planar content; use the front preview above to verify it.")) : image_shape ? localized(
    QStringLiteral(
      "选择本地图片后会提取主要边缘、简化轮廓并限制点数；L 是 gripper_tcp 到笔尖的长度，"
      "空白移动前笔尖会朝机械臂抬起。生成前请在 RViz 检查青色目标笔画和三维笔模型。"),
    QStringLiteral(
      "A local image is reduced to its main edges, simplified, and point-limited. Check the "
      "cyan target strokes and 3D pen model in RViz before generating. Set pen length from gripper_tcp to the tip; "
      "the tool lifts towards the arm before every blank move. Use the front preview above "
      "when the 3D canvas is viewed edge-on.")) : localized(
    QStringLiteral(
      "右侧显示双面青色目标轮廓、三维笔模型和调整控件；XYZ/RPY 调位置方向，W/H 调实际大小。"
      "侧视时请用上方正视预览核对内容；调整不会驱动真机。"),
    QStringLiteral(
      "The 3D view shows two-sided cyan target strokes, the 3D pen, and editing controls. XYZ/RPY change "
      "pose and W/H change physical size. Use the front preview above for edge-on views. "
      "Editing never moves hardware.")));
  updateShapePreview();
  teach_trace_legend_label_->setText(localized(
    QStringLiteral(
      "预检笔尖轨迹：<span style='color:#1fc759'>● 可达</span>　"
      "<span style='color:#ffc70d'>● 近关节限位</span>　"
      "<span style='color:#c71feb'>● 碰撞</span>　"
      "<span style='color:#ef0a0a'>● 无 IK</span>（细半透明虚线为抬笔）<br/>"
      "未预检：<span style='color:#0db8ff'>● 目标笔画</span>　"
      "<span style='color:#ff9414'>● 总运动待执行</span>　"
      "<span style='color:#1fc759'>● 已完成</span>"),
    QStringLiteral(
      "Preflight pen-tip path: <span style='color:#1fc759'>● reachable</span>　"
      "<span style='color:#ffc70d'>● near joint limit</span>　"
      "<span style='color:#c71feb'>● collision</span>　"
      "<span style='color:#ef0a0a'>● no IK</span> "
      "(thin translucent dashes mean pen-up)<br/>"
      "Unchecked: <span style='color:#0db8ff'>● target strokes</span>　"
      "<span style='color:#ff9414'>● total motion upcoming</span>　"
      "<span style='color:#1fc759'>● completed</span>")));
  teach_trace_width_label_->setText(localized(
    QStringLiteral("轨迹/轮廓粗细："), QStringLiteral("Path/outline thickness:")));
  teach_trace_width_spin_->setToolTip(localized(
    QStringLiteral(
      "调整 RViz 中当前形状轮廓和虚拟笔尖轨迹的线宽；范围 0.5–30.0 mm，不影响运动轨迹。"),
    QStringLiteral(
      "Adjust the current shape outline and gripper-tip path width in RViz "
      "(0.5–30.0 mm); motion is unchanged.")));
  teach_copy_button_->setText(localized(
    QStringLiteral("复制"), QStringLiteral("Copy")));
  teach_rename_button_->setText(localized(
    QStringLiteral("重命名"), QStringLiteral("Rename")));
  teach_delete_button_->setText(localized(
    QStringLiteral("删除"), QStringLiteral("Delete")));
  teach_start_button_->setText(localized(
    QStringLiteral("开始拖动录制"), QStringLiteral("Start drag recording")));
  teach_stop_button_->setText(localized(
    QStringLiteral("停止并保存"), QStringLiteral("Stop and save")));
  teach_replay_speed_label_->setText(localized(
    QStringLiteral("真机回放倍速："), QStringLiteral("Robot replay speed:")));
  teach_replay_speed_spin_->setToolTip(localized(
    QStringLiteral("同时用于单动作和命名动作组；安全速度上限始终优先。"),
    QStringLiteral("Used for actions and named sequences; safety velocity limits take priority.")));
  teach_preview_button_->setText(localized(
    QStringLiteral("播放 RViz 动画"), QStringLiteral("Play RViz animation")));
  teach_replay_button_->setText(localized(
    QStringLiteral("真机回放"), QStringLiteral("Robot replay")));
  teach_pause_button_->setText(
    teach_preview_paused_.load() ?
    localized(QStringLiteral("继续"), QStringLiteral("Resume")) :
    localized(QStringLiteral("暂停"), QStringLiteral("Pause")));
  teach_cancel_button_->setText(
    teach_state_.load() == TEACH_FAULT ? localized(
      QStringLiteral("检查并解除故障"), QStringLiteral("Check and Reset Fault")) :
    teach_state_.load() == TEACH_SEQUENCE_PLANNING ? localized(
      QStringLiteral("取消动作组过渡规划"),
      QStringLiteral("Cancel Sequence Transition Planning")) :
    (teach_state_.load() == TEACH_REPLAY_STARTING ||
    teach_state_.load() == TEACH_REPLAYING ||
    teach_state_.load() == TEACH_CANCELLING) ? localized(
      QStringLiteral("取消回放并自动回原点"),
      QStringLiteral("Cancel Replay and Return Home")) : localized(
      QStringLiteral("取消"), QStringLiteral("Cancel")));
  teach_cancel_button_->setToolTip(localized(
    QStringLiteral(
      "取消真机回放时，等待控制器停止后通过 MoveIt 无碰撞规划回到 Home；"
      "故障、急停、反馈异常或非真机预览取消不会自动运动。"),
    QStringLiteral(
      "When cancelling hardware replay, wait for the controller to stop and then use a "
      "collision-checked MoveIt plan to Home. Faults, emergency stops, stale feedback, and "
      "RViz-only cancellation never trigger automatic motion.")));
  teach_slider_label_->setText(localized(
    QStringLiteral("RViz 预览进度（不动真机）："),
    QStringLiteral("RViz preview timeline (no robot motion):")));
  teach_slider_->setToolTip(localized(
    QStringLiteral("拖动后松开，RViz 显示该时刻姿态；不会发送真机轨迹。"),
    QStringLiteral("Drag and release to show that pose in RViz; no hardware command is sent.")));
  teach_sequence_box_->setTitle(localized(
    QStringLiteral("命名动作组（按列表顺序执行）"),
    QStringLiteral("Named sequences (execute in list order)")));
  teach_sequence_refresh_button_->setText(localized(
    QStringLiteral("刷新"), QStringLiteral("Refresh")));
  teach_sequence_selection_hint_label_->setText(localized(
    QStringLiteral(
      "勾选本次要播放的动作；RViz 与真机均按列表顺序执行，并为不连续边界规划过渡。"
      "保存时仍保存完整列表。"),
    QStringLiteral(
      "Check the actions for this run. RViz and robot playback follow list order and plan "
      "disconnected transitions; saving keeps the full list.")));
  teach_sequence_add_button_->setText(localized(
    QStringLiteral("添加当前动作"), QStringLiteral("Add current")));
  teach_sequence_remove_button_->setText(localized(
    QStringLiteral("移除"), QStringLiteral("Remove")));
  teach_sequence_up_button_->setText(localized(
    QStringLiteral("上移"), QStringLiteral("Up")));
  teach_sequence_down_button_->setText(localized(
    QStringLiteral("下移"), QStringLiteral("Down")));
  teach_sequence_save_button_->setText(localized(
    QStringLiteral("命名并保存动作组"), QStringLiteral("Name and save sequence")));
  teach_sequence_replay_speed_label_->setText(localized(
    QStringLiteral("真机回放倍速："), QStringLiteral("Robot replay speed:")));
  teach_sequence_play_button_->setText(localized(
    QStringLiteral("播放勾选动作的 RViz 动画"),
    QStringLiteral("Play checked actions in RViz")));
  teach_sequence_replay_button_->setText(localized(
    QStringLiteral("真机回放勾选动作"), QStringLiteral("Replay checked actions on robot")));
  teach_sequence_pause_button_->setText(
    teach_preview_paused_.load() ?
    localized(QStringLiteral("继续 RViz 动画"), QStringLiteral("Resume RViz animation")) :
    localized(QStringLiteral("暂停 RViz 动画"), QStringLiteral("Pause RViz animation")));
  teach_sequence_cancel_button_->setText(
    teach_state_.load() == TEACH_FAULT ? localized(
      QStringLiteral("检查并解除故障"), QStringLiteral("Check and Reset Fault")) :
    teach_state_.load() == TEACH_SEQUENCE_PLANNING ? localized(
      QStringLiteral("取消过渡规划"),
      QStringLiteral("Cancel Transition Planning")) :
    (teach_state_.load() == TEACH_REPLAY_STARTING ||
    teach_state_.load() == TEACH_REPLAYING ||
    teach_state_.load() == TEACH_CANCELLING) ? localized(
      QStringLiteral("取消动作组回放并回原点"),
      QStringLiteral("Cancel sequence and return Home")) :
    localized(QStringLiteral("取消 RViz 动画"), QStringLiteral("Cancel RViz animation")));
  teach_sequence_slider_label_->setText(localized(
    QStringLiteral("动作组 RViz 预览进度（不动真机）："),
    QStringLiteral("Sequence RViz preview timeline (no robot motion):")));
  teach_sequence_slider_->setToolTip(localized(
    QStringLiteral("拖动后松开，RViz 从勾选动作组合轨迹的对应位置继续；不会发送真机轨迹。"),
    QStringLiteral(
      "Drag and release to resume the checked-action RViz path from that point; no robot command is sent.")));
  if (!initialized_) {
    teach_hardware_action_progress_label_->setText(localized(
      QStringLiteral("当前真机动作进度：等待回放"),
      QStringLiteral("Current robot action: waiting for replay")));
    teach_hardware_overall_progress_label_->setText(localized(
      QStringLiteral("本次真机回放总进度"),
      QStringLiteral("Overall robot replay progress")));
    teach_sequence_hardware_action_progress_label_->setText(localized(
      QStringLiteral("当前真机动作进度：等待回放"),
      QStringLiteral("Current robot action: waiting for replay")));
    teach_sequence_hardware_overall_progress_label_->setText(localized(
      QStringLiteral("本次动作组真机回放总进度"),
      QStringLiteral("Overall sequence robot replay progress")));
  }
  teach_status_label_->setText(localized(teach_status_zh_, teach_status_en_));
  teach_status_label_->setStyleSheet(
    teach_status_error_ ? QStringLiteral("color: #c43b32;") : QString());

  if (initialized_) {
    model_label_->setText(piper ? localized(
      QStringLiteral("型号：Piper-H（CAN 六轴）"),
      QStringLiteral("Model: Piper-H (CAN, 6-axis)")) :
      model_ == QStringLiteral("rs") ? localized(
      QStringLiteral("型号：reBotArm RS"), QStringLiteral("Model: reBotArm RS")) : localized(
      QStringLiteral("型号：reBotArm DM（串口）"),
      QStringLiteral("Model: reBotArm DM (serial)")));
  } else {
    model_label_->setText(localized(
      QStringLiteral("型号：等待 RViz ROS 初始化…"),
      QStringLiteral("Model: waiting for RViz ROS initialization…")));
    feedback_label_->setText(localized(
      QStringLiteral("关节反馈：等待中"), QStringLiteral("Joint feedback: waiting")));
    validity_label_->setText(localized(
      QStringLiteral("MoveIt 状态：等待检查"), QStringLiteral("MoveIt state: waiting for check")));
    xbox_label_->setText(localized(
      QStringLiteral("Xbox 控制：等待状态"), QStringLiteral("Xbox control: waiting for state")));
  }

  status_label_->setText(localized(status_zh_, status_en_));
}

QString DemoPanel::localized(const QString & zh, const QString & en) const
{
  return chinese_ ? zh : en;
}

QString DemoPanel::operationTitle(const QString & executable, bool chinese) const
{
  if (executable == QStringLiteral("go_home")) {
    return chinese ? QStringLiteral("回原点") : QStringLiteral("Return Home");
  }
  if (executable == QStringLiteral("singularity_escape")) {
    return chinese ? QStringLiteral("脱离奇异位形") : QStringLiteral("Singularity Escape");
  }
  if (executable == QStringLiteral("pick_place")) {
    return chinese ? QStringLiteral("抓取放置演示") : QStringLiteral("Pick-and-Place Demo");
  }
  if (executable == QStringLiteral("pulse_approach")) {
    return chinese ? QStringLiteral("把脉区预接触") : QStringLiteral("Pulse Pre-contact");
  }
  if (executable == QStringLiteral("piper_pulse_align")) {
    return chinese ? QStringLiteral("假体压力停止测试") : QStringLiteral("Dummy Pressure-Stop Test");
  }
  return chinese ? QStringLiteral("操作") : QStringLiteral("Operation");
}

bool DemoPanel::feedbackFresh() const
{
  const auto last = last_feedback_ms_.load();
  return last > 0 && steadyMilliseconds() - last < 1000;
}

bool DemoPanel::stateValidityFresh() const
{
  const auto response = validity_state_->response_ms.load();
  return validity_state_->valid.load() && response > 0 &&
         steadyMilliseconds() - response < 1000;
}

QString DemoPanel::detectedModel() const
{
  if (!node_ || !node_->has_parameter("robot_description")) {
    return QStringLiteral("dm");
  }
  try {
    const auto description = node_->get_parameter("robot_description").as_string();
    return description.find("rebotarm_rs") != std::string::npos ||
           description.find("reBotArm-RS") != std::string::npos ?
           QStringLiteral("rs") : QStringLiteral("dm");
  } catch (const std::exception &) {
    return QStringLiteral("dm");
  }
}

QString DemoPanel::configPath(const QString & executable) const
{
  try {
    const QString share = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("rebotarm_moveit_demos"));
    const QString suffix = active_robot_ == QStringLiteral("piperh") ?
      QStringLiteral("_piperh") :
      model_ == QStringLiteral("rs") ? QStringLiteral("_rs") : QString();
    return QStringLiteral("%1/config/%2%3.yaml").arg(share, executable, suffix);
  } catch (const std::exception &) {
    return QString();
  }
}

QString DemoPanel::pulseConfigPath() const
{
  try {
    const QString share = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("rebotarm_pulse"));
    if (active_robot_ == QStringLiteral("piperh")) {
      return QStringLiteral("%1/config/piper_pulse_align.yaml").arg(share);
    }
    const QString suffix = model_ == QStringLiteral("rs") ? QStringLiteral("_rs") : QString();
    return QStringLiteral("%1/config/pulse_approach%2.yaml").arg(share, suffix);
  } catch (const std::exception &) {
    return QString();
  }
}

QString DemoPanel::jointTargetConfigPath() const
{
  try {
    const QString share = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("rebotarm_pulse"));
    const QString suffix = active_robot_ == QStringLiteral("piperh") ?
      QStringLiteral("_piperh") :
      model_ == QStringLiteral("rs") ? QStringLiteral("_rs") : QString();
    return QStringLiteral("%1/config/joint_target%2.yaml").arg(share, suffix);
  } catch (const std::exception &) {
    return QString();
  }
}

}  // namespace rebotarm_demo_rviz

PLUGINLIB_EXPORT_CLASS(rebotarm_demo_rviz::DemoPanel, rviz_common::Panel)
