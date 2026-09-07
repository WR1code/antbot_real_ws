#include "waypoint_manager_panel.h"

#include <algorithm>
#include <utility>

#include <QAbstractItemView>
#include <QBrush>
#include <QColor>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QProgressBar>
#include <QSettings>
#include <QScrollArea>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "robotcar_navigation/rviz_chinese_font.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/tool_manager.hpp"

namespace robotcar_navigation
{
namespace rviz_plugins
{

WaypointManagerPanel::WaypointManagerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  configureChineseUiFont();
  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setFrameShape(QFrame::NoFrame);
  auto * content = new QWidget(scroll);
  auto * root = new QVBoxLayout(content);
  root->setSizeConstraint(QLayout::SetMinAndMaxSize);

  auto * map_box = new QGroupBox(tr("地图"), this);
  auto * map_layout = new QVBoxLayout(map_box);
  active_map_label_ = new QLabel(tr("当前二维地图：启动时地图"), map_box);
  active_map_label_->setWordWrap(true);
  auto * open_map_button = new QPushButton(tr("选择并打开二维地图…"), map_box);
  map_layout->addWidget(active_map_label_);
  map_layout->addWidget(open_map_button);
  root->addWidget(map_box);

  auto * group_box = new QGroupBox(tr("航点组（XML）"), this);
  auto * group_layout = new QVBoxLayout(group_box);
  active_group_label_ = new QLabel(tr("当前航点组：读取中…"), group_box);
  active_group_label_->setWordWrap(true);
  auto * group_buttons = new QHBoxLayout();
  auto * refresh_button = new QPushButton(tr("刷新"), group_box);
  auto * save_group_button = new QPushButton(tr("命名保存…"), group_box);
  auto * open_group_button = new QPushButton(tr("选择打开…"), group_box);
  group_buttons->addWidget(refresh_button);
  group_buttons->addWidget(save_group_button);
  group_buttons->addWidget(open_group_button);
  group_layout->addWidget(active_group_label_);
  group_layout->addLayout(group_buttons);
  root->addWidget(group_box);

  auto * waypoint_box = new QGroupBox(tr("选择与修改航点"), this);
  auto * waypoint_layout = new QVBoxLayout(waypoint_box);
  waypoint_combo_ = new QComboBox(waypoint_box);
  auto * rename_row = new QHBoxLayout();
  rename_edit_ = new QLineEdit(waypoint_box);
  rename_edit_->setPlaceholderText(tr("输入新名称"));
  auto * rename_button = new QPushButton(tr("重命名"), waypoint_box);
  rename_row->addWidget(rename_edit_);
  rename_row->addWidget(rename_button);
  auto * navigate_button = new QPushButton(tr("导航到选中航点"), waypoint_box);
  waypoint_layout->addWidget(waypoint_combo_);
  waypoint_layout->addLayout(rename_row);
  waypoint_layout->addWidget(navigate_button);
  root->addWidget(waypoint_box);

  auto * charger_box = new QGroupBox(tr("一键充电"), this);
  auto * charger_layout = new QVBoxLayout(charger_box);
  charger_combo_ = new QComboBox(charger_box);
  auto * charger_row = new QHBoxLayout();
  auto * refresh_charger_button = new QPushButton(tr("刷新充电点"), charger_box);
  auto * charge_button = new QPushButton(tr("前往并开始充电"), charger_box);
  charger_row->addWidget(refresh_charger_button);
  charger_row->addWidget(charge_button);
  auto * charger_hint = new QLabel(
    tr("到达预停靠位后发布 /antbot/dock_request；充电状态读取 /battery。"), charger_box);
  charger_hint->setWordWrap(true);
  charger_layout->addWidget(charger_combo_);
  charger_layout->addLayout(charger_row);
  charger_layout->addWidget(charger_hint);
  root->addWidget(charger_box);

  auto * keepout_box = new QGroupBox(tr("临时禁区（Nav2 Keepout）"), this);
  auto * keepout_layout = new QVBoxLayout(keepout_box);
  active_keepout_label_ = new QLabel(tr("当前禁区组：读取中…"), keepout_box);
  active_keepout_label_->setWordWrap(true);
  keepout_shape_combo_ = new QComboBox(keepout_box);
  keepout_shape_combo_->addItems({tr("矩形"), tr("正方形"), tr("椭圆"), tr("圆形")});
  keepout_shape_combo_->setToolTip(tr("选择新禁区的绘图形状"));
  auto * add_keepout_button = new QPushButton(tr("输入并绘制禁区"), keepout_box);
  add_keepout_button->setToolTip(
    tr("启用禁区绘制工具；随后可在 Tool Properties 中选择形状并输入名称。"));
  keepout_combo_ = new QComboBox(keepout_box);
  auto * keepout_rename_row = new QHBoxLayout();
  keepout_rename_edit_ = new QLineEdit(keepout_box);
  keepout_rename_edit_->setPlaceholderText(tr("输入选中禁区的新名称"));
  auto * rename_keepout_button = new QPushButton(tr("重命名"), keepout_box);
  keepout_rename_row->addWidget(keepout_rename_edit_);
  keepout_rename_row->addWidget(rename_keepout_button);
  auto * keepout_row = new QHBoxLayout();
  auto * refresh_keepout_button = new QPushButton(tr("刷新"), keepout_box);
  auto * delete_keepout_button = new QPushButton(tr("删除选中"), keepout_box);
  keepout_enable_button_ = new QPushButton(tr("禁区已启用"), keepout_box);
  keepout_enable_button_->setCheckable(true);
  keepout_enable_button_->setChecked(true);
  keepout_row->addWidget(refresh_keepout_button);
  keepout_row->addWidget(delete_keepout_button);
  keepout_row->addWidget(keepout_enable_button_);
  auto * keepout_file_row = new QHBoxLayout();
  auto * save_keepout_button = new QPushButton(tr("命名保存禁区组…"), keepout_box);
  auto * open_keepout_button = new QPushButton(tr("打开禁区组…"), keepout_box);
  keepout_file_row->addWidget(save_keepout_button);
  keepout_file_row->addWidget(open_keepout_button);
  auto * keepout_hint = new QLabel(
    tr("先选矩形/正方形/椭圆/圆形，再像绘图工具一样从一个角拖到对角；拖动时实时预览。"),
    keepout_box);
  keepout_hint->setWordWrap(true);
  keepout_layout->addWidget(active_keepout_label_);
  keepout_layout->addWidget(new QLabel(tr("新禁区形状："), keepout_box));
  keepout_layout->addWidget(keepout_shape_combo_);
  keepout_layout->addWidget(add_keepout_button);
  keepout_layout->addWidget(keepout_combo_);
  keepout_layout->addLayout(keepout_rename_row);
  keepout_layout->addLayout(keepout_row);
  keepout_layout->addLayout(keepout_file_row);
  keepout_layout->addWidget(keepout_hint);
  root->addWidget(keepout_box);

  auto * speed_zone_box = new QGroupBox(tr("区域限速（Nav2 SpeedFilter）"), this);
  auto * speed_zone_layout = new QVBoxLayout(speed_zone_box);
  active_speed_zone_label_ = new QLabel(tr("当前限速区组：读取中…"), speed_zone_box);
  active_speed_zone_label_->setWordWrap(true);
  speed_zone_shape_combo_ = new QComboBox(speed_zone_box);
  speed_zone_shape_combo_->addItems({tr("矩形"), tr("正方形"), tr("椭圆"), tr("圆形")});
  speed_zone_shape_combo_->setToolTip(tr("选择新限速区的绘图形状"));
  auto * add_speed_zone_button = new QPushButton(tr("输入并绘制限速区"), speed_zone_box);
  add_speed_zone_button->setToolTip(
    tr("启用限速区绘制工具；随后可在 Tool Properties 中选择形状并设置最大速度。"));
  speed_zone_combo_ = new QComboBox(speed_zone_box);
  auto * speed_zone_rename_row = new QHBoxLayout();
  speed_zone_rename_edit_ = new QLineEdit(speed_zone_box);
  speed_zone_rename_edit_->setPlaceholderText(tr("输入选中限速区的新名称"));
  auto * rename_speed_zone_button = new QPushButton(tr("重命名"), speed_zone_box);
  speed_zone_rename_row->addWidget(speed_zone_rename_edit_);
  speed_zone_rename_row->addWidget(rename_speed_zone_button);
  auto * speed_zone_row = new QHBoxLayout();
  auto * refresh_speed_zone_button = new QPushButton(tr("刷新"), speed_zone_box);
  auto * delete_speed_zone_button = new QPushButton(tr("删除选中"), speed_zone_box);
  speed_zone_enable_button_ = new QPushButton(tr("限速区已启用"), speed_zone_box);
  speed_zone_enable_button_->setCheckable(true);
  speed_zone_enable_button_->setChecked(true);
  speed_zone_row->addWidget(refresh_speed_zone_button);
  speed_zone_row->addWidget(delete_speed_zone_button);
  speed_zone_row->addWidget(speed_zone_enable_button_);
  auto * speed_zone_file_row = new QHBoxLayout();
  auto * save_speed_zone_button = new QPushButton(tr("命名保存限速组…"), speed_zone_box);
  auto * open_speed_zone_button = new QPushButton(tr("打开限速组…"), speed_zone_box);
  speed_zone_file_row->addWidget(save_speed_zone_button);
  speed_zone_file_row->addWidget(open_speed_zone_button);
  auto * speed_zone_hint = new QLabel(
    tr("先选矩形/正方形/椭圆/圆形和最大速度，再从一个角拖到对角；拖动时实时预览。"),
    speed_zone_box);
  speed_zone_hint->setWordWrap(true);
  speed_zone_layout->addWidget(active_speed_zone_label_);
  speed_zone_layout->addWidget(new QLabel(tr("新限速区形状："), speed_zone_box));
  speed_zone_layout->addWidget(speed_zone_shape_combo_);
  speed_zone_layout->addWidget(add_speed_zone_button);
  speed_zone_layout->addWidget(speed_zone_combo_);
  speed_zone_layout->addLayout(speed_zone_rename_row);
  speed_zone_layout->addLayout(speed_zone_row);
  speed_zone_layout->addLayout(speed_zone_file_row);
  speed_zone_layout->addWidget(speed_zone_hint);
  root->addWidget(speed_zone_box);

  auto * route_box = new QGroupBox(tr("巡航顺序组"), this);
  auto * route_layout = new QVBoxLayout(route_box);
  route_list_ = new QListWidget(route_box);
  route_list_->setDragDropMode(QAbstractItemView::InternalMove);
  route_list_->setDefaultDropAction(Qt::MoveAction);
  route_list_->setSelectionMode(QAbstractItemView::SingleSelection);
  route_layout->addWidget(route_list_);

  auto * route_add_row = new QHBoxLayout();
  auto * add_button = new QPushButton(tr("添加选中"), route_box);
  auto * all_button = new QPushButton(tr("使用全部"), route_box);
  auto * remove_button = new QPushButton(tr("移除"), route_box);
  route_add_row->addWidget(add_button);
  route_add_row->addWidget(all_button);
  route_add_row->addWidget(remove_button);
  route_layout->addLayout(route_add_row);

  auto * route_move_row = new QHBoxLayout();
  auto * up_button = new QPushButton(tr("上移"), route_box);
  auto * down_button = new QPushButton(tr("下移"), route_box);
  auto * save_route_button = new QPushButton(tr("命名保存顺序…"), route_box);
  auto * open_route_button = new QPushButton(tr("打开顺序组…"), route_box);
  route_move_row->addWidget(up_button);
  route_move_row->addWidget(down_button);
  route_move_row->addWidget(save_route_button);
  route_move_row->addWidget(open_route_button);
  route_layout->addLayout(route_move_row);

  auto * progress_box = new QGroupBox(tr("巡航进度"), route_box);
  auto * progress_layout = new QVBoxLayout(progress_box);
  route_progress_label_ = new QLabel(tr("未开始"), progress_box);
  route_progress_label_->setWordWrap(true);
  route_progress_bar_ = new QProgressBar(progress_box);
  route_progress_bar_->setRange(0, 1);
  route_progress_bar_->setValue(0);
  route_progress_bar_->setFormat(tr("0 / 0（0%）"));
  progress_layout->addWidget(route_progress_label_);
  progress_layout->addWidget(route_progress_bar_);
  route_layout->addWidget(progress_box);

  auto * route_run_row = new QHBoxLayout();
  start_button_ = new QPushButton(tr("开始巡航"), route_box);
  pause_button_ = new QPushButton(tr("暂停"), route_box);
  stop_button_ = new QPushButton(tr("停止"), route_box);
  pause_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  route_run_row->addWidget(start_button_);
  route_run_row->addWidget(pause_button_);
  route_run_row->addWidget(stop_button_);
  route_layout->addLayout(route_run_row);
  route_edit_widgets_ = {
    add_button, all_button, remove_button, up_button, down_button,
    save_route_button, open_route_button};
  root->addWidget(route_box, 1);

  auto * history_box = new QGroupBox(tr("巡航日志与任务历史"), this);
  auto * history_layout = new QVBoxLayout(history_box);
  history_tree_ = new QTreeWidget(history_box);
  history_tree_->setHeaderLabels({tr("时间"), tr("任务"), tr("结果"), tr("详情")});
  history_tree_->setRootIsDecorated(false);
  history_tree_->setMaximumHeight(180);
  auto * export_history_button = new QPushButton(tr("导出任务历史…"), history_box);
  history_layout->addWidget(history_tree_);
  history_layout->addWidget(export_history_button);
  root->addWidget(history_box);

  root->addStretch();
  scroll->setWidget(content);
  outer->addWidget(scroll, 1);

  status_label_ = new QLabel(tr("等待 ROS 初始化…"), this);
  status_label_->setWordWrap(true);
  status_label_->setContentsMargins(6, 3, 6, 3);
  outer->addWidget(status_label_);

  connect(refresh_button, &QPushButton::clicked, this, &WaypointManagerPanel::refreshWaypoints);
  connect(rename_button, &QPushButton::clicked, this, &WaypointManagerPanel::renameWaypoint);
  connect(save_group_button, &QPushButton::clicked, this, &WaypointManagerPanel::saveWaypointGroup);
  connect(open_group_button, &QPushButton::clicked, this, &WaypointManagerPanel::openWaypointGroup);
  connect(open_map_button, &QPushButton::clicked, this, &WaypointManagerPanel::openMap);
  connect(add_button, &QPushButton::clicked, this, &WaypointManagerPanel::addSelectedToRoute);
  connect(all_button, &QPushButton::clicked, this, &WaypointManagerPanel::useAllWaypoints);
  connect(remove_button, &QPushButton::clicked, this, &WaypointManagerPanel::removeRouteItem);
  connect(up_button, &QPushButton::clicked, this, &WaypointManagerPanel::moveRouteItemUp);
  connect(down_button, &QPushButton::clicked, this, &WaypointManagerPanel::moveRouteItemDown);
  connect(save_route_button, &QPushButton::clicked, this, &WaypointManagerPanel::saveRouteGroup);
  connect(open_route_button, &QPushButton::clicked, this, &WaypointManagerPanel::openRouteGroup);
  connect(navigate_button, &QPushButton::clicked, this, &WaypointManagerPanel::navigateSelected);
  connect(refresh_charger_button, &QPushButton::clicked, this, &WaypointManagerPanel::refreshChargers);
  connect(charge_button, &QPushButton::clicked, this, &WaypointManagerPanel::navigateToCharger);
  connect(
    add_keepout_button, &QPushButton::clicked,
    this, &WaypointManagerPanel::activateKeepoutTool);
  connect(
    keepout_combo_, &QComboBox::currentTextChanged,
    keepout_rename_edit_, &QLineEdit::setText);
  connect(refresh_keepout_button, &QPushButton::clicked, this, &WaypointManagerPanel::refreshKeepouts);
  connect(
    rename_keepout_button, &QPushButton::clicked,
    this, &WaypointManagerPanel::renameSelectedKeepout);
  connect(delete_keepout_button, &QPushButton::clicked, this, &WaypointManagerPanel::deleteSelectedKeepout);
  connect(keepout_enable_button_, &QPushButton::clicked, this, &WaypointManagerPanel::toggleKeepouts);
  connect(save_keepout_button, &QPushButton::clicked, this, &WaypointManagerPanel::saveKeepoutGroup);
  connect(open_keepout_button, &QPushButton::clicked, this, &WaypointManagerPanel::openKeepoutGroup);
  connect(
    add_speed_zone_button, &QPushButton::clicked,
    this, &WaypointManagerPanel::activateSpeedZoneTool);
  connect(
    speed_zone_combo_, &QComboBox::currentTextChanged,
    speed_zone_rename_edit_, &QLineEdit::setText);
  connect(
    refresh_speed_zone_button, &QPushButton::clicked,
    this, &WaypointManagerPanel::refreshSpeedZones);
  connect(
    rename_speed_zone_button, &QPushButton::clicked,
    this, &WaypointManagerPanel::renameSelectedSpeedZone);
  connect(
    delete_speed_zone_button, &QPushButton::clicked,
    this, &WaypointManagerPanel::deleteSelectedSpeedZone);
  connect(
    speed_zone_enable_button_, &QPushButton::clicked,
    this, &WaypointManagerPanel::toggleSpeedZones);
  connect(
    save_speed_zone_button, &QPushButton::clicked,
    this, &WaypointManagerPanel::saveSpeedZoneGroup);
  connect(
    open_speed_zone_button, &QPushButton::clicked,
    this, &WaypointManagerPanel::openSpeedZoneGroup);
  connect(start_button_, &QPushButton::clicked, this, &WaypointManagerPanel::startRoute);
  connect(pause_button_, &QPushButton::clicked, this, &WaypointManagerPanel::pauseOrResumeRoute);
  connect(stop_button_, &QPushButton::clicked, this, &WaypointManagerPanel::stopRoute);
  connect(export_history_button, &QPushButton::clicked, this, &WaypointManagerPanel::exportTaskHistory);

  QSettings settings("robotcar_navigation", "waypoint_manager_panel");
  last_directory_ = settings.value("last_directory", QDir::homePath()).toString();
  history_file_ = QDir::homePath() + "/.ros/robotcar_navigation/task_history.jsonl";
  loadTaskHistory();
}

void WaypointManagerPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus(tr("无法取得 RViz ROS 节点"), true);
    return;
  }
  node_ = abstraction->get_raw_node();
  names_client_ = node_->create_client<srv::GetWaypointNames>(
    "/waterplus/get_waypoint_names");
  charger_names_client_ = node_->create_client<srv::GetWaypointNames>(
    "/waterplus/get_charger_names");
  rename_client_ = node_->create_client<srv::RenameWaypoint>(
    "/waterplus/rename_waypoint");
  save_group_client_ = node_->create_client<srv::WaypointFile>(
    "/waterplus/save_waypoint_group");
  load_group_client_ = node_->create_client<srv::WaypointFile>(
    "/waterplus/load_waypoint_group");
  load_map_client_ = node_->create_client<nav2_msgs::srv::LoadMap>(
    "/map_server/load_map");
  cancel_client_ = node_->create_client<action_msgs::srv::CancelGoal>(
    "/navigate_to_pose/_action/cancel_goal");
  keepout_names_client_ = node_->create_client<srv::GetWaypointNames>(
    "/waterplus/get_keepout_names");
  rename_keepout_client_ = node_->create_client<srv::RenameWaypoint>(
    "/waterplus/rename_keepout");
  save_keepout_client_ = node_->create_client<srv::WaypointFile>(
    "/waterplus/save_keepout_group");
  load_keepout_client_ = node_->create_client<srv::WaypointFile>(
    "/waterplus/load_keepout_group");
  enable_keepout_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/waterplus/enable_keepouts");
  speed_zone_names_client_ = node_->create_client<srv::GetWaypointNames>(
    "/waterplus/get_speed_zone_names");
  rename_speed_zone_client_ = node_->create_client<srv::RenameWaypoint>(
    "/waterplus/rename_speed_zone");
  save_speed_zone_client_ = node_->create_client<srv::WaypointFile>(
    "/waterplus/save_speed_zone_group");
  load_speed_zone_client_ = node_->create_client<srv::WaypointFile>(
    "/waterplus/load_speed_zone_group");
  enable_speed_zone_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/waterplus/enable_speed_zones");
  navigation_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/waterplus/navi_waypoint", 10);
  charger_navigation_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/waterplus/navi_charger", 10);
  delete_keepout_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/waterplus/delete_keepout", 10);
  delete_speed_zone_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/waterplus/delete_speed_zone", 10);
  route_control_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/waterplus/route_control", 10);
  navigation_result_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/waterplus/navi_result", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      handleNavigationResult(message);
    });
  route_progress_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/waterplus/route_progress", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      handleExternalRouteProgress(message);
    });
  charge_result_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/waterplus/charge_result", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      handleChargeResult(message);
    });
  keepout_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/waterplus/keepout_status", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      handleKeepoutStatus(message);
    });
  speed_zone_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/waterplus/speed_zone_status", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      handleSpeedZoneStatus(message);
    });
  setStatus(tr("已连接航点编辑服务"));
  QTimer::singleShot(300, this, &WaypointManagerPanel::refreshWaypoints);
  QTimer::singleShot(400, this, &WaypointManagerPanel::refreshChargers);
  QTimer::singleShot(500, this, &WaypointManagerPanel::refreshKeepouts);
  QTimer::singleShot(600, this, &WaypointManagerPanel::refreshSpeedZones);
}

void WaypointManagerPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "color: #e35d6a;" : "");
}

void WaypointManagerPanel::activateZoneTool(
  const QString & class_id, const QString & display_name, const QString & shape)
{
  auto * tool_manager = getDisplayContext()->getToolManager();
  if (!tool_manager) {
    setStatus(tr("无法取得 RViz 工具管理器"), true);
    return;
  }

  for (int index = 0; index < tool_manager->numTools(); ++index) {
    auto * tool = tool_manager->getTool(index);
    if (tool && tool->getClassId() == class_id) {
      tool->getPropertyContainer()->subProp("形状")->setValue(shape);
      tool_manager->setCurrentTool(tool);
      setStatus(
        tr("已启用%1（%2）；从一个角拖到对角，松开鼠标完成。")
        .arg(display_name, shape));
      return;
    }
  }

  setStatus(
    tr("找不到%1工具；请重新构建 robotcar_navigation 后重启 RViz").arg(display_name),
    true);
}

void WaypointManagerPanel::activateKeepoutTool()
{
  activateZoneTool(
    "robotcar_navigation/AddKeepoutZone", tr("禁区绘制"),
    keepout_shape_combo_->currentText());
}

void WaypointManagerPanel::activateSpeedZoneTool()
{
  activateZoneTool(
    "robotcar_navigation/AddSpeedZone", tr("限速区绘制"),
    speed_zone_shape_combo_->currentText());
}

QString WaypointManagerPanel::suggestedDirectory() const
{
  if (!active_group_file_.isEmpty()) {
    return QFileInfo(active_group_file_).absolutePath();
  }
  return last_directory_.isEmpty() ? QDir::homePath() : last_directory_;
}

void WaypointManagerPanel::refreshWaypoints()
{
  if (!names_client_ || !names_client_->service_is_ready()) {
    setStatus(tr("航点服务尚未就绪，请稍后点“刷新”"), true);
    return;
  }
  auto request = std::make_shared<srv::GetWaypointNames::Request>();
  names_client_->async_send_request(
    request, [this](rclcpp::Client<srv::GetWaypointNames>::SharedFuture future) {
      const auto response = future.get();
      QStringList names;
      for (const auto & name : response->names) {
        names.append(QString::fromStdString(name));
      }
      const QString active_file = QString::fromStdString(response->active_file);
      QMetaObject::invokeMethod(this, [this, names, active_file]() {
        const QString previous = waypoint_combo_->currentText();
        waypoint_combo_->clear();
        waypoint_combo_->addItems(names);
        const int previous_index = waypoint_combo_->findText(previous);
        if (previous_index >= 0) {
          waypoint_combo_->setCurrentIndex(previous_index);
        }
        active_group_file_ = active_file;
        active_group_label_->setText(tr("当前航点组：%1").arg(active_file));
        setStatus(tr("已读取 %1 个航点").arg(names.size()));
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::refreshChargers()
{
  if (!charger_names_client_ || !charger_names_client_->service_is_ready()) {
    return;
  }
  charger_names_client_->async_send_request(
    std::make_shared<srv::GetWaypointNames::Request>(),
    [this](rclcpp::Client<srv::GetWaypointNames>::SharedFuture future) {
      QStringList names;
      for (const auto & name : future.get()->names) {
        names.append(QString::fromStdString(name));
      }
      QMetaObject::invokeMethod(this, [this, names]() {
        const QString previous = charger_combo_->currentText();
        charger_combo_->clear();
        charger_combo_->addItems(names);
        const int index = charger_combo_->findText(previous);
        if (index >= 0) {
          charger_combo_->setCurrentIndex(index);
        }
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::refreshKeepouts()
{
  if (!keepout_names_client_ || !keepout_names_client_->service_is_ready()) {
    active_keepout_label_->setText(tr("当前禁区组：管理节点尚未就绪"));
    return;
  }
  keepout_names_client_->async_send_request(
    std::make_shared<srv::GetWaypointNames::Request>(),
    [this](rclcpp::Client<srv::GetWaypointNames>::SharedFuture future) {
      const auto response = future.get();
      QStringList names;
      for (const auto & name : response->names) {
        names.append(QString::fromStdString(name));
      }
      const QString active_file = QString::fromStdString(response->active_file);
      QMetaObject::invokeMethod(this, [this, names, active_file]() {
        const QString previous = keepout_combo_->currentText();
        keepout_combo_->clear();
        keepout_combo_->addItems(names);
        const int index = keepout_combo_->findText(previous);
        if (index >= 0) {
          keepout_combo_->setCurrentIndex(index);
        }
        active_keepout_label_->setText(
          tr("当前禁区组：%1（%2 个区域）")
          .arg(active_file.isEmpty() ? tr("尚未保存") : active_file).arg(names.size()));
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::saveKeepoutGroup()
{
  QString filename = QFileDialog::getSaveFileName(
    this, tr("命名保存临时禁区组"), suggestedDirectory() + "/keepouts.keepout.json",
    tr("临时禁区组 (*.keepout.json *.json)"));
  if (filename.isEmpty()) {
    return;
  }
  if (!filename.endsWith(".json", Qt::CaseInsensitive)) {
    filename += ".keepout.json";
  }
  if (!save_keepout_client_ || !save_keepout_client_->service_is_ready()) {
    setStatus(tr("禁区保存服务尚未就绪"), true);
    return;
  }
  auto request = std::make_shared<srv::WaypointFile::Request>();
  request->filename = filename.toStdString();
  save_keepout_client_->async_send_request(
    request, [this, filename](rclcpp::Client<srv::WaypointFile>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString detail = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, success, detail, filename]() {
        setStatus(detail, !success);
        appendTaskHistory(tr("保存禁区组"), success ? tr("成功") : tr("失败"), detail);
        if (success) {
          last_directory_ = QFileInfo(filename).absolutePath();
          refreshKeepouts();
        }
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::openKeepoutGroup()
{
  const QString filename = QFileDialog::getOpenFileName(
    this, tr("打开临时禁区组"), suggestedDirectory(),
    tr("临时禁区组 (*.keepout.json *.json)"));
  if (filename.isEmpty()) {
    return;
  }
  if (!load_keepout_client_ || !load_keepout_client_->service_is_ready()) {
    setStatus(tr("禁区打开服务尚未就绪"), true);
    return;
  }
  auto request = std::make_shared<srv::WaypointFile::Request>();
  request->filename = filename.toStdString();
  load_keepout_client_->async_send_request(
    request, [this, filename](rclcpp::Client<srv::WaypointFile>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString detail = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, success, detail, filename]() {
        setStatus(detail, !success);
        appendTaskHistory(tr("打开禁区组"), success ? tr("成功") : tr("失败"), detail);
        if (success) {
          last_directory_ = QFileInfo(filename).absolutePath();
          refreshKeepouts();
        }
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::renameSelectedKeepout()
{
  const QString old_name = keepout_combo_->currentText().trimmed();
  const QString new_name = keepout_rename_edit_->text().trimmed();
  if (old_name.isEmpty() || new_name.isEmpty()) {
    setStatus(tr("请先选择临时禁区并输入新名称"), true);
    return;
  }
  if (!rename_keepout_client_ || !rename_keepout_client_->service_is_ready()) {
    setStatus(tr("禁区重命名服务尚未就绪"), true);
    return;
  }
  auto request = std::make_shared<srv::RenameWaypoint::Request>();
  request->old_name = old_name.toStdString();
  request->new_name = new_name.toStdString();
  rename_keepout_client_->async_send_request(
    request, [this, new_name](rclcpp::Client<srv::RenameWaypoint>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString detail = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, success, detail, new_name]() {
        setStatus(detail, !success);
        appendTaskHistory(tr("重命名禁区"), success ? tr("成功") : tr("失败"), detail);
        if (success) {
          keepout_rename_edit_->clear();
          keepout_combo_->setItemText(keepout_combo_->currentIndex(), new_name);
          refreshKeepouts();
        }
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::deleteSelectedKeepout()
{
  const QString name = keepout_combo_->currentText();
  if (name.isEmpty() || !delete_keepout_pub_) {
    setStatus(tr("没有选中临时禁区"), true);
    return;
  }
  std_msgs::msg::String message;
  message.data = name.toStdString();
  delete_keepout_pub_->publish(message);
}

void WaypointManagerPanel::toggleKeepouts()
{
  const bool enabled = keepout_enable_button_->isChecked();
  keepout_enable_button_->setText(enabled ? tr("禁区已启用") : tr("禁区已关闭"));
  if (!enable_keepout_client_ || !enable_keepout_client_->service_is_ready()) {
    setStatus(tr("禁区启停服务尚未就绪"), true);
    keepout_enable_button_->setChecked(!enabled);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  enable_keepout_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString detail = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, success, detail]() {
        setStatus(detail, !success);
        appendTaskHistory(tr("临时禁区"), success ? tr("成功") : tr("失败"), detail);
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::refreshSpeedZones()
{
  if (!speed_zone_names_client_ || !speed_zone_names_client_->service_is_ready()) {
    active_speed_zone_label_->setText(tr("当前限速区组：管理节点尚未就绪"));
    return;
  }
  speed_zone_names_client_->async_send_request(
    std::make_shared<srv::GetWaypointNames::Request>(),
    [this](rclcpp::Client<srv::GetWaypointNames>::SharedFuture future) {
      const auto response = future.get();
      QStringList names;
      for (const auto & name : response->names) {
        names.append(QString::fromStdString(name));
      }
      const QString active_file = QString::fromStdString(response->active_file);
      QMetaObject::invokeMethod(this, [this, names, active_file]() {
        const QString previous = speed_zone_combo_->currentText();
        speed_zone_combo_->clear();
        speed_zone_combo_->addItems(names);
        const int index = speed_zone_combo_->findText(previous);
        if (index >= 0) {
          speed_zone_combo_->setCurrentIndex(index);
        }
        active_speed_zone_label_->setText(
          tr("当前限速区组：%1（%2 个区域）")
          .arg(active_file.isEmpty() ? tr("尚未保存") : active_file).arg(names.size()));
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::saveSpeedZoneGroup()
{
  QString filename = QFileDialog::getSaveFileName(
    this, tr("命名保存限速区组"), suggestedDirectory() + "/speeds.speed.json",
    tr("限速区组 (*.speed.json *.json)"));
  if (filename.isEmpty()) {
    return;
  }
  if (!filename.endsWith(".json", Qt::CaseInsensitive)) {
    filename += ".speed.json";
  }
  if (!save_speed_zone_client_ || !save_speed_zone_client_->service_is_ready()) {
    setStatus(tr("限速区保存服务尚未就绪"), true);
    return;
  }
  auto request = std::make_shared<srv::WaypointFile::Request>();
  request->filename = filename.toStdString();
  save_speed_zone_client_->async_send_request(
    request, [this, filename](rclcpp::Client<srv::WaypointFile>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString detail = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, success, detail, filename]() {
        setStatus(detail, !success);
        appendTaskHistory(tr("保存限速区组"), success ? tr("成功") : tr("失败"), detail);
        if (success) {
          last_directory_ = QFileInfo(filename).absolutePath();
          refreshSpeedZones();
        }
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::openSpeedZoneGroup()
{
  const QString filename = QFileDialog::getOpenFileName(
    this, tr("打开限速区组"), suggestedDirectory(),
    tr("限速区组 (*.speed.json *.json)"));
  if (filename.isEmpty()) {
    return;
  }
  if (!load_speed_zone_client_ || !load_speed_zone_client_->service_is_ready()) {
    setStatus(tr("限速区打开服务尚未就绪"), true);
    return;
  }
  auto request = std::make_shared<srv::WaypointFile::Request>();
  request->filename = filename.toStdString();
  load_speed_zone_client_->async_send_request(
    request, [this, filename](rclcpp::Client<srv::WaypointFile>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString detail = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, success, detail, filename]() {
        setStatus(detail, !success);
        appendTaskHistory(tr("打开限速区组"), success ? tr("成功") : tr("失败"), detail);
        if (success) {
          last_directory_ = QFileInfo(filename).absolutePath();
          refreshSpeedZones();
        }
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::renameSelectedSpeedZone()
{
  const QString old_name = speed_zone_combo_->currentText().trimmed();
  const QString new_name = speed_zone_rename_edit_->text().trimmed();
  if (old_name.isEmpty() || new_name.isEmpty()) {
    setStatus(tr("请先选择限速区并输入新名称"), true);
    return;
  }
  if (!rename_speed_zone_client_ || !rename_speed_zone_client_->service_is_ready()) {
    setStatus(tr("限速区重命名服务尚未就绪"), true);
    return;
  }
  auto request = std::make_shared<srv::RenameWaypoint::Request>();
  request->old_name = old_name.toStdString();
  request->new_name = new_name.toStdString();
  rename_speed_zone_client_->async_send_request(
    request, [this, new_name](rclcpp::Client<srv::RenameWaypoint>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString detail = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, success, detail, new_name]() {
        setStatus(detail, !success);
        appendTaskHistory(tr("重命名限速区"), success ? tr("成功") : tr("失败"), detail);
        if (success) {
          speed_zone_rename_edit_->clear();
          speed_zone_combo_->setItemText(speed_zone_combo_->currentIndex(), new_name);
          refreshSpeedZones();
        }
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::deleteSelectedSpeedZone()
{
  const QString name = speed_zone_combo_->currentText();
  if (name.isEmpty() || !delete_speed_zone_pub_) {
    setStatus(tr("没有选中限速区"), true);
    return;
  }
  std_msgs::msg::String message;
  message.data = name.toStdString();
  delete_speed_zone_pub_->publish(message);
}

void WaypointManagerPanel::toggleSpeedZones()
{
  const bool enabled = speed_zone_enable_button_->isChecked();
  speed_zone_enable_button_->setText(enabled ? tr("限速区已启用") : tr("限速区已关闭"));
  if (!enable_speed_zone_client_ || !enable_speed_zone_client_->service_is_ready()) {
    setStatus(tr("限速区启停服务尚未就绪"), true);
    speed_zone_enable_button_->setChecked(!enabled);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  enable_speed_zone_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString detail = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, success, detail]() {
        setStatus(detail, !success);
        appendTaskHistory(tr("区域限速"), success ? tr("成功") : tr("失败"), detail);
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::renameWaypoint()
{
  const QString old_name = waypoint_combo_->currentText().trimmed();
  const QString new_name = rename_edit_->text().trimmed();
  if (old_name.isEmpty() || new_name.isEmpty()) {
    setStatus(tr("请先选择航点并输入新名称"), true);
    return;
  }
  if (!rename_client_ || !rename_client_->service_is_ready()) {
    setStatus(tr("重命名服务尚未就绪"), true);
    return;
  }
  auto request = std::make_shared<srv::RenameWaypoint::Request>();
  request->old_name = old_name.toStdString();
  request->new_name = new_name.toStdString();
  rename_client_->async_send_request(
    request, [this, old_name, new_name](
      rclcpp::Client<srv::RenameWaypoint>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString message = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, success, message, old_name, new_name]() {
        setStatus(message, !success);
        if (success) {
          for (int i = 0; i < route_list_->count(); ++i) {
            if (route_list_->item(i)->text() == old_name) {
              route_list_->item(i)->setText(new_name);
            }
          }
          rename_edit_->clear();
          refreshWaypoints();
        }
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::requestWaypointFile(const QString & filename, bool save)
{
  auto client = save ? save_group_client_ : load_group_client_;
  if (!client || !client->service_is_ready()) {
    setStatus(save ? tr("保存航点组服务尚未就绪") : tr("打开航点组服务尚未就绪"), true);
    return;
  }
  auto request = std::make_shared<srv::WaypointFile::Request>();
  request->filename = filename.toStdString();
  client->async_send_request(
    request, [this](rclcpp::Client<srv::WaypointFile>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->success;
      const QString message = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, success, message]() {
        setStatus(message, !success);
        if (success) {
          refreshWaypoints();
        }
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::saveWaypointGroup()
{
  QString filename = QFileDialog::getSaveFileName(
    this, tr("命名保存航点组"), suggestedDirectory() + "/waypoint_group.xml",
    tr("航点 XML (*.xml)"));
  if (filename.isEmpty()) {
    return;
  }
  if (!filename.endsWith(".xml", Qt::CaseInsensitive)) {
    filename += ".xml";
  }
  last_directory_ = QFileInfo(filename).absolutePath();
  QSettings("robotcar_navigation", "waypoint_manager_panel").setValue(
    "last_directory", last_directory_);
  requestWaypointFile(filename, true);
}

void WaypointManagerPanel::openWaypointGroup()
{
  const QString filename = QFileDialog::getOpenFileName(
    this, tr("选择航点组"), suggestedDirectory(), tr("航点 XML (*.xml)"));
  if (filename.isEmpty()) {
    return;
  }
  last_directory_ = QFileInfo(filename).absolutePath();
  QSettings("robotcar_navigation", "waypoint_manager_panel").setValue(
    "last_directory", last_directory_);
  requestWaypointFile(filename, false);
}

void WaypointManagerPanel::openMap()
{
  const QString filename = QFileDialog::getOpenFileName(
    this, tr("选择 Nav2 二维地图"), suggestedDirectory(), tr("Nav2 地图 YAML (*.yaml *.yml)"));
  if (filename.isEmpty()) {
    return;
  }
  if (!load_map_client_ || !load_map_client_->service_is_ready()) {
    setStatus(tr("map_server/load_map 服务尚未就绪"), true);
    return;
  }
  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = filename.toStdString();
  load_map_client_->async_send_request(
    request, [this, filename](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response->result == 0;
      const int result_code = static_cast<int>(response->result);
      QMetaObject::invokeMethod(this, [this, success, filename, result_code]() {
        if (success) {
          active_map_file_ = filename;
          active_map_label_->setText(tr("当前二维地图：%1").arg(filename));
          setStatus(tr("二维地图已切换；若地图工程不同，请重启 Step2 以同步三维点云"));
        } else {
          setStatus(tr("地图加载失败，Nav2 错误码：%1").arg(result_code), true);
        }
      }, Qt::QueuedConnection);
    });
}

void WaypointManagerPanel::addSelectedToRoute()
{
  const QString name = waypoint_combo_->currentText();
  if (!name.isEmpty()) {
    route_list_->addItem(name);
    resetRouteProgress();
  }
}

void WaypointManagerPanel::useAllWaypoints()
{
  route_list_->clear();
  for (int i = 0; i < waypoint_combo_->count(); ++i) {
    route_list_->addItem(waypoint_combo_->itemText(i));
  }
  resetRouteProgress();
}

void WaypointManagerPanel::removeRouteItem()
{
  delete route_list_->takeItem(route_list_->currentRow());
  resetRouteProgress();
}

void WaypointManagerPanel::moveRouteItemUp()
{
  const int row = route_list_->currentRow();
  if (row > 0) {
    auto * item = route_list_->takeItem(row);
    route_list_->insertItem(row - 1, item);
    route_list_->setCurrentRow(row - 1);
    resetRouteProgress();
  }
}

void WaypointManagerPanel::moveRouteItemDown()
{
  const int row = route_list_->currentRow();
  if (row >= 0 && row + 1 < route_list_->count()) {
    auto * item = route_list_->takeItem(row);
    route_list_->insertItem(row + 1, item);
    route_list_->setCurrentRow(row + 1);
    resetRouteProgress();
  }
}

QStringList WaypointManagerPanel::routeNames() const
{
  QStringList result;
  for (int i = 0; i < route_list_->count(); ++i) {
    result.append(route_list_->item(i)->text());
  }
  return result;
}

void WaypointManagerPanel::saveRouteGroup()
{
  const QStringList names = routeNames();
  if (names.isEmpty()) {
    setStatus(tr("巡航顺序为空"), true);
    return;
  }
  QString filename = QFileDialog::getSaveFileName(
    this, tr("命名保存巡航顺序组"), suggestedDirectory() + "/route_group.route.json",
    tr("巡航顺序组 (*.route.json *.json)"));
  if (filename.isEmpty()) {
    return;
  }
  if (!filename.endsWith(".json", Qt::CaseInsensitive)) {
    filename += ".route.json";
  }
  QJsonArray array;
  for (const QString & name : names) {
    array.append(name);
  }
  QJsonObject root;
  root["version"] = 1;
  QString route_name = QFileInfo(filename).fileName();
  if (route_name.endsWith(".route.json", Qt::CaseInsensitive)) {
    route_name.chop(QString(".route.json").size());
  } else if (route_name.endsWith(".json", Qt::CaseInsensitive)) {
    route_name.chop(QString(".json").size());
  }
  root["name"] = route_name;
  root["waypoint_group"] = active_group_file_;
  root["waypoints"] = array;
  QFile output(filename);
  if (!output.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    setStatus(tr("无法写入巡航顺序组：%1").arg(output.errorString()), true);
    return;
  }
  output.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
  output.close();
  last_directory_ = QFileInfo(filename).absolutePath();
  QSettings("robotcar_navigation", "waypoint_manager_panel").setValue(
    "last_directory", last_directory_);
  setStatus(tr("巡航顺序组已保存：%1").arg(filename));
}

void WaypointManagerPanel::openRouteGroup()
{
  const QString filename = QFileDialog::getOpenFileName(
    this, tr("打开巡航顺序组"), suggestedDirectory(),
    tr("巡航顺序组 (*.route.json *.json)"));
  if (filename.isEmpty()) {
    return;
  }
  QFile input(filename);
  if (!input.open(QIODevice::ReadOnly)) {
    setStatus(tr("无法读取巡航顺序组：%1").arg(input.errorString()), true);
    return;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(input.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject() ||
    !document.object().value("waypoints").isArray())
  {
    setStatus(tr("巡航顺序组格式无效：%1").arg(parse_error.errorString()), true);
    return;
  }
  route_list_->clear();
  for (const auto value : document.object().value("waypoints").toArray()) {
    if (value.isString()) {
      route_list_->addItem(value.toString());
    }
  }
  resetRouteProgress();
  last_directory_ = QFileInfo(filename).absolutePath();
  QSettings("robotcar_navigation", "waypoint_manager_panel").setValue(
    "last_directory", last_directory_);
  const QString expected_group = document.object().value("waypoint_group").toString();
  if (!expected_group.isEmpty() && expected_group != active_group_file_) {
    setStatus(tr("顺序组已打开，正在打开它关联的航点组：%1").arg(expected_group));
    requestWaypointFile(expected_group, false);
  } else {
    setStatus(tr("已打开巡航顺序组：%1").arg(filename));
  }
}

void WaypointManagerPanel::navigateSelected()
{
  const QString name = waypoint_combo_->currentText();
  if (name.isEmpty()) {
    setStatus(tr("没有选中航点"), true);
    return;
  }
  if (!navigation_pub_ || navigation_pub_->get_subscription_count() == 0) {
    setStatus(tr("导航服务未启动"), true);
    return;
  }
  std_msgs::msg::String message;
  message.data = name.toStdString();
  discard_next_navigation_result_ = true;
  navigation_pub_->publish(message);
  setStatus(tr("已发送目标：%1").arg(name));
  appendTaskHistory(tr("单点导航"), tr("已派发"), name);
}

void WaypointManagerPanel::navigateToCharger()
{
  const QString name = charger_combo_->currentText();
  if (name.isEmpty()) {
    setStatus(tr("没有可用充电点；请先用 Add Charger 添加并保存"), true);
    return;
  }
  if (!charger_navigation_pub_ || charger_navigation_pub_->get_subscription_count() == 0) {
    setStatus(tr("充电导航执行器未启动"), true);
    return;
  }
  std_msgs::msg::String message;
  message.data = name.toStdString();
  charger_navigation_pub_->publish(message);
  setStatus(tr("正在前往充电点：%1").arg(name));
  appendTaskHistory(tr("一键充电"), tr("已派发"), name);
}

void WaypointManagerPanel::startRoute()
{
  if (route_running_) {
    return;
  }
  active_route_ = routeNames();
  if (active_route_.isEmpty()) {
    setStatus(tr("巡航顺序为空"), true);
    return;
  }
  if (!navigation_pub_ || navigation_pub_->get_subscription_count() == 0) {
    setStatus(tr("导航服务未启动；当前只能编辑和保存顺序"), true);
    return;
  }
  route_running_ = true;
  route_paused_ = false;
  active_route_index_ = 0;
  setRouteEditingEnabled(false);
  start_button_->setEnabled(false);
  pause_button_->setEnabled(true);
  pause_button_->setText(tr("暂停"));
  stop_button_->setEnabled(true);
  appendTaskHistory(
    tr("巡航"), tr("开始"), tr("共 %1 个航点").arg(active_route_.size()));
  publishCurrentRouteTarget();
}

void WaypointManagerPanel::pauseOrResumeRoute()
{
  if (!route_running_ && external_route_active_) {
    std_msgs::msg::String control;
    external_route_paused_ = !external_route_paused_;
    control.data = external_route_paused_ ? "pause" : "resume";
    route_control_pub_->publish(control);
    pause_button_->setText(external_route_paused_ ? tr("继续") : tr("暂停"));
    appendTaskHistory(
      tr("外部巡航"), external_route_paused_ ? tr("请求暂停") : tr("请求继续"), QString());
    return;
  }
  if (!route_running_) {
    return;
  }
  if (!route_paused_) {
    route_paused_ = true;
    discard_next_navigation_result_ = true;
    if (cancel_client_ && cancel_client_->service_is_ready()) {
      cancel_client_->async_send_request(
        std::make_shared<action_msgs::srv::CancelGoal::Request>());
    }
    pause_button_->setText(tr("继续"));
    showRouteProgress(
      std::max(0, active_route_index_), active_route_.size(), active_route_index_,
      active_route_.value(active_route_index_), tr("巡航已暂停"));
    setStatus(tr("巡航已暂停；点击“继续”将重新派发当前航点"));
    appendTaskHistory(
      tr("巡航"), tr("暂停"), active_route_.value(active_route_index_));
  } else {
    route_paused_ = false;
    pause_button_->setText(tr("暂停"));
    appendTaskHistory(
      tr("巡航"), tr("继续"), active_route_.value(active_route_index_));
    publishCurrentRouteTarget();
  }
}

void WaypointManagerPanel::publishCurrentRouteTarget()
{
  if (!route_running_ || active_route_index_ < 0 || active_route_index_ >= active_route_.size()) {
    const int total = active_route_.size();
    route_running_ = false;
    route_paused_ = false;
    active_route_index_ = -1;
    setRouteEditingEnabled(true);
    start_button_->setEnabled(true);
    pause_button_->setEnabled(false);
    stop_button_->setEnabled(false);
    showRouteProgress(total, total, -1, QString(), tr("巡航完成"));
    setStatus(tr("巡航顺序已完成"));
    appendTaskHistory(tr("巡航"), tr("完成"), tr("共 %1 个航点").arg(total));
    return;
  }
  if (route_paused_) {
    return;
  }
  std_msgs::msg::String message;
  message.data = active_route_.at(active_route_index_).toStdString();
  navigation_pub_->publish(message);
  showRouteProgress(
    active_route_index_, active_route_.size(), active_route_index_,
    active_route_.at(active_route_index_), tr("正在巡航"));
  setStatus(tr("巡航 %1/%2：%3").arg(active_route_index_ + 1).arg(
      active_route_.size()).arg(active_route_.at(active_route_index_)));
}

void WaypointManagerPanel::handleNavigationResult(
  const std_msgs::msg::String::SharedPtr message)
{
  if (discard_next_navigation_result_) {
    discard_next_navigation_result_ = false;
    return;
  }
  if (!route_running_) {
    return;
  }
  const bool success = message->data == "done";
  QMetaObject::invokeMethod(this, [this, success]() {
    if (!route_running_) {
      return;
    }
    if (route_paused_) {
      return;
    }
    if (!success) {
      const int failed_index = active_route_index_;
      const QString failed_name = active_route_.value(failed_index);
      route_running_ = false;
      active_route_index_ = -1;
      setRouteEditingEnabled(true);
      start_button_->setEnabled(true);
      pause_button_->setEnabled(false);
      stop_button_->setEnabled(false);
      showRouteProgress(
        failed_index, active_route_.size(), failed_index, failed_name,
        tr("巡航失败"), true);
      setStatus(tr("巡航在当前航点失败并停止"), true);
      appendTaskHistory(tr("巡航"), tr("失败"), failed_name);
      return;
    }
    appendTaskHistory(
      tr("巡航航点"), tr("到达"), active_route_.value(active_route_index_));
    ++active_route_index_;
    publishCurrentRouteTarget();
  }, Qt::QueuedConnection);
}

void WaypointManagerPanel::stopRoute()
{
  if (!route_running_ && external_route_active_) {
    std_msgs::msg::String control;
    control.data = "stop";
    route_control_pub_->publish(control);
    if (cancel_client_ && cancel_client_->service_is_ready()) {
      cancel_client_->async_send_request(
        std::make_shared<action_msgs::srv::CancelGoal::Request>());
    }
    external_route_active_ = false;
    external_route_paused_ = false;
    pause_button_->setEnabled(false);
    pause_button_->setText(tr("暂停"));
    stop_button_->setEnabled(false);
    appendTaskHistory(tr("外部巡航"), tr("请求停止"), QString());
    return;
  }
  const int stopped_index = active_route_index_;
  const QString stopped_name = active_route_.value(stopped_index);
  if (route_running_) {
    discard_next_navigation_result_ = true;
  }
  route_running_ = false;
  route_paused_ = false;
  active_route_index_ = -1;
  setRouteEditingEnabled(true);
  start_button_->setEnabled(true);
  pause_button_->setEnabled(false);
  pause_button_->setText(tr("暂停"));
  stop_button_->setEnabled(false);
  if (cancel_client_ && cancel_client_->service_is_ready()) {
    cancel_client_->async_send_request(std::make_shared<action_msgs::srv::CancelGoal::Request>());
  }
  showRouteProgress(
    std::max(0, stopped_index), active_route_.size(), stopped_index, stopped_name,
    tr("巡航已停止"), true);
  setStatus(tr("已停止顺序派发并请求取消当前 Nav2 目标"));
  appendTaskHistory(tr("巡航"), tr("停止"), stopped_name);
}

void WaypointManagerPanel::setRouteEditingEnabled(bool enabled)
{
  route_list_->setEnabled(enabled);
  for (auto * widget : route_edit_widgets_) {
    widget->setEnabled(enabled);
  }
}

void WaypointManagerPanel::resetRouteProgress()
{
  route_progress_label_->setText(tr("未开始"));
  route_progress_label_->setStyleSheet("");
  route_progress_bar_->setRange(0, 1);
  route_progress_bar_->setValue(0);
  route_progress_bar_->setFormat(tr("0 / 0（0%）"));
  route_progress_bar_->setStyleSheet("");
  for (int i = 0; i < route_list_->count(); ++i) {
    route_list_->item(i)->setBackground(QBrush());
    QFont font = route_list_->item(i)->font();
    font.setBold(false);
    route_list_->item(i)->setFont(font);
  }
}

void WaypointManagerPanel::showRouteProgress(
  int completed, int total, int current_index, const QString & current_name,
  const QString & state, bool error)
{
  completed = std::clamp(completed, 0, std::max(0, total));
  route_progress_bar_->setRange(0, std::max(1, total));
  route_progress_bar_->setValue(completed);
  route_progress_bar_->setFormat(
    total > 0 ? tr("%v / %m（%p%）") : tr("0 / 0（0%）"));
  route_progress_bar_->setStyleSheet(
    error ? "QProgressBar::chunk { background-color: #d9534f; }" : "");
  route_progress_label_->setStyleSheet(error ? "color: #e35d6a;" : "");

  if (total <= 0) {
    route_progress_label_->setText(state);
  } else if (current_index >= 0 && current_index < total && !current_name.isEmpty()) {
    route_progress_label_->setText(
      tr("%1：%2 / %3，当前航点“%4”，已完成 %5")
      .arg(state).arg(current_index + 1).arg(total).arg(current_name).arg(completed));
  } else {
    route_progress_label_->setText(
      tr("%1：已完成 %2 / %3").arg(state).arg(completed).arg(total));
  }

  if (route_list_->count() == total) {
    for (int i = 0; i < total; ++i) {
      auto * item = route_list_->item(i);
      if (i < completed) {
        item->setBackground(QColor(40, 167, 69, 90));
      } else {
        item->setBackground(QBrush());
      }
      QFont font = item->font();
      font.setBold(i == current_index);
      item->setFont(font);
    }
    if (current_index >= 0 && current_index < total) {
      route_list_->item(current_index)->setBackground(
        error ? QColor(220, 53, 69, 110) : QColor(255, 193, 7, 110));
      route_list_->scrollToItem(route_list_->item(current_index));
    }
  }
}

void WaypointManagerPanel::handleExternalRouteProgress(
  const std_msgs::msg::String::SharedPtr message)
{
  if (route_running_) {
    return;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(
    QByteArray::fromStdString(message->data), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    return;
  }
  const QJsonObject progress = document.object();
  const int completed = progress.value("completed").toInt();
  const int total = progress.value("total").toInt();
  const int current_index = progress.value("current_index").toInt() - 1;
  const QString current_name = progress.value("current_name").toString();
  const QString state = progress.value("state").toString();
  QString state_text = tr("外部巡航");
  bool error = false;
  if (state == "waiting") {
    state_text = tr("等待开始");
  } else if (state == "navigating") {
    state_text = tr("正在巡航");
  } else if (state == "reached") {
    state_text = tr("已到达");
  } else if (state == "completed") {
    state_text = tr("巡航完成");
  } else if (state == "failed") {
    state_text = tr("巡航失败");
    error = true;
  } else if (state == "stopped") {
    state_text = tr("巡航已停止");
    error = true;
  } else if (state == "paused") {
    state_text = tr("巡航已暂停");
  }
  const bool active = state == "waiting" || state == "navigating" || state == "reached" ||
    state == "paused";
  const QString progress_key = QString("%1|%2|%3|%4")
    .arg(state).arg(completed).arg(current_index).arg(current_name);
  QMetaObject::invokeMethod(this, [
    this, completed, total, current_index, current_name, state, state_text, error,
    active, progress_key]() {
      showRouteProgress(
        completed, total, current_index, current_name, state_text, error);
      external_route_active_ = active;
      external_route_paused_ = state == "paused";
      pause_button_->setEnabled(active);
      stop_button_->setEnabled(active);
      pause_button_->setText(external_route_paused_ ? tr("继续") : tr("暂停"));
      if (progress_key != last_external_progress_key_) {
        appendTaskHistory(
          tr("外部巡航"), state_text,
          current_name.isEmpty() ? tr("已完成 %1/%2").arg(completed).arg(total) : current_name);
        last_external_progress_key_ = progress_key;
      }
    }, Qt::QueuedConnection);
}

void WaypointManagerPanel::handleChargeResult(
  const std_msgs::msg::String::SharedPtr message)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(
    QByteArray::fromStdString(message->data), &error);
  bool success = message->data == "done";
  QString charger;
  QString detail = success ? tr("已到达充电点并请求开始停靠") : tr("充电导航失败");
  if (error.error == QJsonParseError::NoError && document.isObject()) {
    success = document.object().value("success").toBool();
    charger = document.object().value("charger").toString();
    detail = document.object().value("message").toString(detail);
  }
  QMetaObject::invokeMethod(this, [this, success, charger, detail]() {
    setStatus(detail, !success);
    appendTaskHistory(
      tr("一键充电"), success ? tr("已到达") : tr("失败"),
      charger.isEmpty() ? detail : tr("%1：%2").arg(charger, detail));
  }, Qt::QueuedConnection);
}

void WaypointManagerPanel::handleKeepoutStatus(
  const std_msgs::msg::String::SharedPtr message)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(
    QByteArray::fromStdString(message->data), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    return;
  }
  const bool success = document.object().value("success").toBool();
  const QString detail = document.object().value("message").toString();
  QMetaObject::invokeMethod(this, [this, success, detail]() {
    setStatus(detail, !success);
    appendTaskHistory(tr("临时禁区"), success ? tr("成功") : tr("失败"), detail);
    refreshKeepouts();
  }, Qt::QueuedConnection);
}

void WaypointManagerPanel::handleSpeedZoneStatus(
  const std_msgs::msg::String::SharedPtr message)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(
    QByteArray::fromStdString(message->data), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    return;
  }
  const bool success = document.object().value("success").toBool();
  const QString detail = document.object().value("message").toString();
  QMetaObject::invokeMethod(this, [this, success, detail]() {
    setStatus(detail, !success);
    appendTaskHistory(tr("区域限速"), success ? tr("成功") : tr("失败"), detail);
    refreshSpeedZones();
  }, Qt::QueuedConnection);
}

void WaypointManagerPanel::appendTaskHistory(
  const QString & task, const QString & result, const QString & detail)
{
  const QString timestamp = QDateTime::currentDateTime().toString(Qt::ISODate);
  auto * item = new QTreeWidgetItem();
  item->setText(0, timestamp);
  item->setText(1, task);
  item->setText(2, result);
  item->setText(3, detail);
  history_tree_->insertTopLevelItem(0, item);
  while (history_tree_->topLevelItemCount() > 200) {
    delete history_tree_->takeTopLevelItem(history_tree_->topLevelItemCount() - 1);
  }

  QDir().mkpath(QFileInfo(history_file_).absolutePath());
  QFile output(history_file_);
  if (output.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
    QJsonObject record;
    record["timestamp"] = timestamp;
    record["task"] = task;
    record["result"] = result;
    record["detail"] = detail;
    output.write(QJsonDocument(record).toJson(QJsonDocument::Compact));
    output.write("\n");
  }
}

void WaypointManagerPanel::loadTaskHistory()
{
  QFile input(history_file_);
  if (!input.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return;
  }
  const QList<QByteArray> lines = input.readAll().split('\n');
  const int line_count = static_cast<int>(lines.size());
  const int first = std::max(0, line_count - 201);
  for (int index = line_count - 1; index >= first; --index) {
    if (lines.at(index).trimmed().isEmpty()) {
      continue;
    }
    const auto document = QJsonDocument::fromJson(lines.at(index));
    if (!document.isObject()) {
      continue;
    }
    const auto record = document.object();
    auto * item = new QTreeWidgetItem(history_tree_);
    item->setText(0, record.value("timestamp").toString());
    item->setText(1, record.value("task").toString());
    item->setText(2, record.value("result").toString());
    item->setText(3, record.value("detail").toString());
  }
}

void WaypointManagerPanel::exportTaskHistory()
{
  QString filename = QFileDialog::getSaveFileName(
    this, tr("导出巡航日志与任务历史"), suggestedDirectory() + "/task_history.json",
    tr("JSON 文件 (*.json)"));
  if (filename.isEmpty()) {
    return;
  }
  if (!filename.endsWith(".json", Qt::CaseInsensitive)) {
    filename += ".json";
  }
  QJsonArray records;
  for (int index = history_tree_->topLevelItemCount() - 1; index >= 0; --index) {
    const auto * item = history_tree_->topLevelItem(index);
    QJsonObject record;
    record["timestamp"] = item->text(0);
    record["task"] = item->text(1);
    record["result"] = item->text(2);
    record["detail"] = item->text(3);
    records.append(record);
  }
  QJsonObject root;
  root["version"] = 1;
  root["records"] = records;
  QFile output(filename);
  if (!output.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    setStatus(tr("无法导出任务历史：%1").arg(output.errorString()), true);
    return;
  }
  output.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
  setStatus(tr("任务历史已导出：%1").arg(filename));
}

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

PLUGINLIB_EXPORT_CLASS(
  robotcar_navigation::rviz_plugins::WaypointManagerPanel,
  rviz_common::Panel)
