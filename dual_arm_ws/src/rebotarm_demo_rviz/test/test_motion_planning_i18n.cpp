#include <cstdlib>
#include <iostream>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QDir>
#include <QDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>

#include "rebotarm_demo_rviz/motion_planning_i18n.hpp"
#include "rebotarm_demo_rviz/demo_panel.hpp"
#include "rebotarm_demo_rviz/dual_arm_selector_panel.hpp"
#include "pluginlib/class_loader.hpp"
#include "rviz_common/display.hpp"

namespace
{

bool expectEqual(const QString & actual, const QString & expected, const char * label)
{
  if (actual == expected) {
    return true;
  }
  std::cerr << label << ": expected '" << expected.toStdString() << "', got '"
            << actual.toStdString() << "'\n";
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  QApplication application(argc, argv);
  bool ok = true;
  try {
    pluginlib::ClassLoader<rviz_common::Display> loader(
      "rviz_common", "rviz_common::Display");
    if (!loader.isClassAvailable("moveit_rviz_plugin/HandEyeCalibration")) {
      std::cerr << "MoveIt Calibration RViz display is not registered\n";
      ok = false;
    }
  } catch (const pluginlib::PluginlibException & error) {
    std::cerr << "MoveIt Calibration RViz display load failed: " << error.what() << "\n";
    ok = false;
  }
  QDockWidget dock(QStringLiteral("MotionPlanning"));
  auto * root = new QWidget();
  auto * layout = new QVBoxLayout(root);
  auto * tabs = new QTabWidget(root);
  tabs->setObjectName(QStringLiteral("tabWidget"));
  auto * context_page = new QWidget(tabs);
  auto * planning_page = new QWidget(tabs);
  auto * joints_page = new QWidget(tabs);
  auto * scene_objects_page = new QWidget(tabs);
  auto * stored_scenes_page = new QWidget(tabs);
  auto * stored_states_page = new QWidget(tabs);
  auto * native_status_page = new QWidget(tabs);
  auto * manipulation_page = new QWidget(tabs);
  auto * planning_layout = new QVBoxLayout(planning_page);
  auto * commands = new QGroupBox(QStringLiteral("Commands"), planning_page);
  auto * command_layout = new QVBoxLayout(commands);
  auto * plan = new QPushButton(QStringLiteral("&Plan"), commands);
  plan->setObjectName(QStringLiteral("plan_button"));
  auto * status = new QLabel(QStringLiteral("Planning..."), commands);
  auto * states = new QComboBox(commands);
  states->addItem(QStringLiteral("<current>"));
  command_layout->addWidget(plan);
  command_layout->addWidget(status);
  command_layout->addWidget(states);
  planning_layout->addWidget(commands);
  tabs->addTab(context_page, QStringLiteral("Context"));
  tabs->addTab(planning_page, QStringLiteral("Planning"));
  tabs->addTab(joints_page, QStringLiteral("Joints"));
  tabs->addTab(scene_objects_page, QStringLiteral("Scene Objects"));
  tabs->addTab(stored_scenes_page, QStringLiteral("Stored Scenes"));
  tabs->addTab(stored_states_page, QStringLiteral("Stored States"));
  tabs->addTab(native_status_page, QStringLiteral("Status"));
  tabs->addTab(manipulation_page, QStringLiteral("Manipulation"));
  layout->addWidget(tabs);
  dock.setWidget(root);
  dock.show();
  application.processEvents();

  ok = (rebotarm_demo_rviz::motionPlanningDockWidget() == &dock) && ok;
  ok = rebotarm_demo_rviz::translateMotionPlanningUi(true) && ok;
  ok = expectEqual(dock.windowTitle(), QStringLiteral("运动规划"), "dock title") && ok;
  ok = expectEqual(tabs->tabText(0), QStringLiteral("上下文"), "context tab") && ok;
  ok = expectEqual(tabs->tabText(5), QStringLiteral("已存姿态"), "stored poses tab") && ok;
  ok = rebotarm_demo_rviz::setMotionPlanningIntegratedMode(true) && ok;
  if (!tabs->isTabVisible(0) || !tabs->isTabVisible(1) || !tabs->isTabVisible(2)) {
    std::cerr << "unique MoveIt planning pages were hidden\n";
    ok = false;
  }
  for (int index = 3; index < tabs->count(); ++index) {
    if (tabs->isTabVisible(index)) {
      std::cerr << "duplicate MoveIt page stayed visible: " << index << "\n";
      ok = false;
    }
  }
  ok = expectEqual(commands->title(), QStringLiteral("命令"), "commands group") && ok;
  ok = expectEqual(plan->text(), QStringLiteral("规划"), "plan button") && ok;
  ok = expectEqual(status->text(), QStringLiteral("正在规划…"), "status") && ok;
  ok = expectEqual(states->itemText(0), QStringLiteral("<current>"), "semantic combo item") && ok;

  status->setText(QStringLiteral("Executed"));
  ok = rebotarm_demo_rviz::translateMotionPlanningUi(true) && ok;
  ok = expectEqual(status->text(), QStringLiteral("已执行"), "dynamic status") && ok;

  ok = rebotarm_demo_rviz::translateMotionPlanningUi(false) && ok;
  ok = expectEqual(dock.windowTitle(), QStringLiteral("MotionPlanning"), "English dock title") && ok;
  ok = expectEqual(tabs->tabText(0), QStringLiteral("Context"), "English context tab") && ok;
  ok = expectEqual(tabs->tabText(5), QStringLiteral("Stored States"), "English stored states tab") && ok;
  ok = expectEqual(commands->title(), QStringLiteral("Commands"), "English commands group") && ok;
  ok = expectEqual(plan->text(), QStringLiteral("&Plan"), "English plan button") && ok;
  ok = expectEqual(status->text(), QStringLiteral("Executed"), "English dynamic status") && ok;
  ok = rebotarm_demo_rviz::setMotionPlanningIntegratedMode(false) && ok;
  for (int index = 0; index < tabs->count(); ++index) {
    if (!tabs->isTabVisible(index)) {
      std::cerr << "MoveIt page was not restored: " << index << "\n";
      ok = false;
    }
  }

  rebotarm_demo_rviz::DemoPanel panel;
  panel.show();
  application.processEvents();
  bool found_main_tabs = false;
  bool found_teach_tabs = false;
  bool found_action_mode_tabs = false;
  for (auto * panel_tabs : panel.findChildren<QTabWidget *>()) {
    if (panel_tabs->count() == 5 &&
      panel_tabs->tabText(0) == QStringLiteral("真机与安全") &&
      panel_tabs->tabText(1) == QStringLiteral("演示与动作") &&
      panel_tabs->tabText(2) == QStringLiteral("摄像头") &&
      panel_tabs->tabText(3) == QStringLiteral("手部视觉") &&
      panel_tabs->tabText(4) == QStringLiteral("手眼标定"))
    {
      found_main_tabs = true;
    }
    if (panel_tabs->objectName() == QStringLiteral("teach_tabs") &&
      panel_tabs->count() == 4 &&
      panel_tabs->tabText(0) == QStringLiteral("单动作与形状") &&
      panel_tabs->tabText(1) == QStringLiteral("真机演示操作") &&
      panel_tabs->tabText(2) == QStringLiteral("动作组编排") &&
      panel_tabs->tabText(3) == QStringLiteral("自由运动规划"))
    {
      found_teach_tabs = true;
    }
    if (panel_tabs->objectName() == QStringLiteral("teach_action_mode_tabs") &&
      panel_tabs->count() == 2 &&
      panel_tabs->tabText(0) == QStringLiteral("新建 / 编辑参数") &&
      panel_tabs->tabText(1) == QStringLiteral("RViz / 真机播放"))
    {
      found_action_mode_tabs = true;
    }
  }
  if (!found_main_tabs || !found_teach_tabs || !found_action_mode_tabs) {
    std::cerr << "integrated control tabs are missing\n";
  }
  ok = found_main_tabs && found_teach_tabs && found_action_mode_tabs && ok;

  auto * action_content = panel.findChild<QWidget *>(
    QStringLiteral("teach_action_content"));
  auto * action_editor_box = panel.findChild<QGroupBox *>(
    QStringLiteral("teach_action_editor_box"));
  auto * action_layout = action_content ? qobject_cast<QVBoxLayout *>(
    action_content->layout()) : nullptr;
  if (!action_layout || action_layout->count() != 2 ||
    action_layout->itemAt(1)->widget() != action_editor_box ||
    action_layout->stretch(1) != 1 || !action_editor_box ||
    action_editor_box->sizePolicy().verticalPolicy() != QSizePolicy::Expanding)
  {
    std::cerr << "action editor does not fill the available dock height\n";
    ok = false;
  }

  auto * hand_vision_large_button = panel.findChild<QPushButton *>(
    QStringLiteral("hand_vision_large_button"));
  if (hand_vision_large_button) {hand_vision_large_button->click();}
  application.processEvents();
  auto * hand_vision_large_dialog = panel.findChild<QDialog *>(
    QStringLiteral("hand_vision_large_dialog"));
  auto * hand_vision_large_label = panel.findChild<QLabel *>(
    QStringLiteral("hand_vision_large_label"));
  if (!hand_vision_large_button ||
    hand_vision_large_button->text() != QStringLiteral("独立大屏显示") ||
    !hand_vision_large_dialog || !hand_vision_large_dialog->isVisible() ||
    !hand_vision_large_label)
  {
    std::cerr << "standalone hand-vision large view is unavailable\n";
    ok = false;
  }
  if (hand_vision_large_dialog) {hand_vision_large_dialog->close();}

  auto * shape_editor_scroll = panel.findChild<QScrollArea *>(
    QStringLiteral("teach_shape_editor_scroll"));
  auto * shape_editor_content = panel.findChild<QWidget *>(
    QStringLiteral("teach_shape_editor_content"));
  if (!shape_editor_scroll || !shape_editor_scroll->widgetResizable() ||
    shape_editor_scroll->widget() != shape_editor_content || !shape_editor_content ||
    !shape_editor_content->layout() ||
    shape_editor_content->layout()->sizeConstraint() != QLayout::SetMinAndMaxSize)
  {
    std::cerr << "shape editor is not protected from layout compression\n";
    ok = false;
  }
  auto * compact_main_tabs = panel.findChild<QTabWidget *>(QStringLiteral("main_tabs"));
  auto * compact_teach_tabs = panel.findChild<QTabWidget *>(QStringLiteral("teach_tabs"));
  auto * compact_action_tabs = panel.findChild<QTabWidget *>(
    QStringLiteral("teach_action_mode_tabs"));
  panel.resize(720, 540);
  if (compact_main_tabs) {compact_main_tabs->setCurrentIndex(1);}
  if (compact_teach_tabs) {compact_teach_tabs->setCurrentIndex(0);}
  if (compact_action_tabs) {compact_action_tabs->setCurrentIndex(0);}
  application.processEvents();
  if (shape_editor_content) {
    for (auto * label : shape_editor_content->findChildren<QLabel *>()) {
      if (label->isVisible() && label->height() < label->fontMetrics().height()) {
        std::cerr << "shape editor label was compressed below one text line: "
                  << label->text().toStdString() << "\n";
        ok = false;
      }
    }
  }
  if (compact_main_tabs) {compact_main_tabs->setCurrentIndex(0);}
  application.processEvents();

  auto * new_shape_button = panel.findChild<QPushButton *>(
    QStringLiteral("teach_shape_new_button"));
  auto * shape_width = panel.findChild<QDoubleSpinBox *>(
    QStringLiteral("teach_shape_width_spin"));
  auto * shape_height = panel.findChild<QDoubleSpinBox *>(
    QStringLiteral("teach_shape_height_spin"));
  auto * pen_lift = panel.findChild<QDoubleSpinBox *>(
    QStringLiteral("teach_shape_pen_lift_spin"));
  if (new_shape_button) {new_shape_button->click();}
  application.processEvents();
  if (!new_shape_button || !shape_width || !shape_height || !pen_lift ||
    shape_width->value() < 0.01 || shape_height->value() < 0.01 ||
    pen_lift->value() < 1.0)
  {
    std::cerr << "new shape action starts with safety-invalid geometry\n";
    ok = false;
  }

  panel.setActiveRobot(QStringLiteral("piperh"));
  auto * handeye_name = panel.findChild<QLineEdit *>(QStringLiteral("handeye_name_edit"));
  auto * handeye_base = panel.findChild<QLineEdit *>(
    QStringLiteral("handeye_robot_base_edit"));
  auto * handeye_effector = panel.findChild<QLineEdit *>(
    QStringLiteral("handeye_robot_effector_edit"));
  auto * piper_pulse_approach = panel.findChild<QPushButton *>(
    QStringLiteral("pulse_approach_button"));
  const bool piper_eye_in_hand_saved = QFileInfo::exists(
    QDir::homePath() +
    QStringLiteral("/.ros2/easy_handeye2/calibrations/piperh_camera_eye_in_hand.calib"));
  const QString expected_piper_calibration = piper_eye_in_hand_saved ?
    QStringLiteral("piperh_camera_eye_in_hand") : QStringLiteral("piperh_camera");
  if (!handeye_name || handeye_name->text() != expected_piper_calibration ||
    !handeye_base || handeye_base->text() != QStringLiteral("piperh/base_link") ||
    !handeye_effector || handeye_effector->text() != QStringLiteral("piperh/Link5") ||
    !piper_pulse_approach || !piper_pulse_approach->isEnabled() ||
    piper_pulse_approach->text() !=
      QStringLiteral("假体测试：前往视觉目标（压力停止）"))
  {
    std::cerr << "Piper-H shared vision/calibration pages are not safely configured\n";
    ok = false;
  }
  panel.setActiveRobot(QStringLiteral("rebotarm"));

  auto * main_tabs = panel.findChild<QTabWidget *>(QStringLiteral("main_tabs"));
  if (!main_tabs || main_tabs->tabPosition() != QTabWidget::North ||
    !main_tabs->usesScrollButtons())
  {
    std::cerr << "main control pages do not use the compact top-aligned tab bar\n";
    ok = false;
  }
  auto * camera_view = panel.findChild<QLabel *>(QStringLiteral("camera_view_label"));
  auto * camera_color = panel.findChild<QCheckBox *>(QStringLiteral("camera_color_checkbox"));
  auto * camera_depth = panel.findChild<QCheckBox *>(QStringLiteral("camera_depth_checkbox"));
  auto * camera_ir = panel.findChild<QCheckBox *>(QStringLiteral("camera_ir_checkbox"));
  auto * camera_info = panel.findChild<QLabel *>(QStringLiteral("camera_stream_info_label"));
  auto * hand_vision_view = panel.findChild<QLabel *>(QStringLiteral("hand_vision_view_label"));
  if (!camera_view || !hand_vision_view || camera_view->minimumHeight() != 0 ||
    hand_vision_view->minimumHeight() != 0)
  {
    std::cerr << "image pages still impose a fixed minimum height\n";
    ok = false;
  }
  if (!camera_color || !camera_depth || !camera_ir || !camera_info ||
    !camera_color->isChecked() || camera_depth->isChecked() || camera_ir->isChecked() ||
    camera_color->text() != QStringLiteral("彩色图传输") ||
    camera_depth->text() != QStringLiteral("深度图传输") ||
    camera_ir->text() != QStringLiteral("红外图传输") ||
    !camera_info->text().contains(QStringLiteral("分辨率")))
  {
    std::cerr << "camera stream controls or image information are missing\n";
    ok = false;
  }

  rebotarm_demo_rviz::DualArmSelectorPanel dual_panel;
  auto * selection_scroll = dual_panel.findChild<QScrollArea *>(
    QStringLiteral("dual_arm_selection_scroll"));
  auto * comprehensive_scroll = dual_panel.findChild<QScrollArea *>(
    QStringLiteral("dual_arm_comprehensive_scroll"));
  auto * chassis_scroll = dual_panel.findChild<QScrollArea *>(
    QStringLiteral("dual_arm_chassis_scroll"));
  auto * feature_tabs = dual_panel.findChild<QTabWidget *>(
    QStringLiteral("dual_arm_feature_tabs"));
  auto * piper_control_tabs = dual_panel.findChild<QTabWidget *>(
    QStringLiteral("piper_control_tabs"));
  auto * piper_motor_button = dual_panel.findChild<QPushButton *>(
    QStringLiteral("piper_motor_enable_button"));
  auto * piper_motor_disable_button = dual_panel.findChild<QPushButton *>(
    QStringLiteral("piper_motor_disable_button"));
  auto * piper_xbox_button = dual_panel.findChild<QPushButton *>(
    QStringLiteral("piper_xbox_arm_button"));
  auto * piper_control_status = dual_panel.findChild<QLabel *>(
    QStringLiteral("piper_control_status_label"));
  if (!selection_scroll || !selection_scroll->widgetResizable() ||
    !comprehensive_scroll || !comprehensive_scroll->widgetResizable() ||
    !chassis_scroll || !chassis_scroll->widgetResizable() ||
    !feature_tabs || feature_tabs->count() != 3 ||
    feature_tabs->tabText(0) != QStringLiteral("机械臂选择") ||
    feature_tabs->tabText(1) != QStringLiteral("机械臂控制") ||
    feature_tabs->tabText(2) != QStringLiteral("底盘控制") ||
    feature_tabs->tabPosition() != QTabWidget::North || !feature_tabs->usesScrollButtons() ||
    !piper_control_tabs || piper_control_tabs->count() != 2 ||
    piper_control_tabs->tabText(0) != QStringLiteral("真机与安全") ||
    piper_control_tabs->tabText(1) != QStringLiteral("演示与动作") ||
    !piper_motor_button || !piper_motor_disable_button || !piper_xbox_button ||
    !piper_control_status)
  {
    std::cerr << "dual-arm feature pages are not peers or are not shrinkable\n";
    ok = false;
  }
  QMetaObject::invokeMethod(&dual_panel, "togglePiperArmed", Qt::DirectConnection);
  if (!piper_control_status || !piper_xbox_button ||
    !piper_control_status->styleSheet().contains(QStringLiteral("#c22")) ||
    !piper_xbox_button->styleSheet().contains(QStringLiteral("#c62828")))
  {
    std::cerr << "Piper control failures do not produce red visual feedback\n";
    ok = false;
  }

  QMainWindow integration_window;
  auto * selector_dock = new QDockWidget(QStringLiteral("双臂统一工作台"));
  auto * integrated_selector = new rebotarm_demo_rviz::DualArmSelectorPanel();
  selector_dock->setWidget(integrated_selector);
  integration_window.addDockWidget(Qt::LeftDockWidgetArea, selector_dock);
  auto * rebot_dock = new QDockWidget(QStringLiteral("reBot 综合控制"));
  auto * integrated_demo = new rebotarm_demo_rviz::DemoPanel();
  rebot_dock->setWidget(integrated_demo);
  integration_window.addDockWidget(Qt::LeftDockWidgetArea, rebot_dock);
  auto * preset_dock = new QDockWidget(QStringLiteral("双臂预设动作"));
  auto * preset_widget = new QWidget();
  preset_widget->setObjectName(QStringLiteral("test_dual_arm_preset"));
  preset_dock->setWidget(preset_widget);
  integration_window.addDockWidget(Qt::LeftDockWidgetArea, preset_dock);
  auto * vehicle_dock = new QDockWidget(QStringLiteral("车辆控制中心"));
  auto * vehicle_widget = new QWidget();
  vehicle_widget->setObjectName(QStringLiteral("test_vehicle_control"));
  vehicle_dock->setWidget(vehicle_widget);
  integration_window.addDockWidget(Qt::LeftDockWidgetArea, vehicle_dock);
  auto * waypoint_dock = new QDockWidget(QStringLiteral("航点与巡航组"));
  auto * waypoint_widget = new QWidget();
  waypoint_widget->setObjectName(QStringLiteral("test_waypoint_control"));
  waypoint_dock->setWidget(waypoint_widget);
  integration_window.addDockWidget(Qt::LeftDockWidgetArea, waypoint_dock);
  integration_window.show();
  application.processEvents();
  QMetaObject::invokeMethod(integrated_selector, "organizeDockPages");
  application.processEvents();
  if (integrated_selector->isAncestorOf(root)) {
    std::cerr << "selector stole MotionPlanning from the nested free-planning page\n";
    ok = false;
  }
  auto * switched_tabs = integrated_demo->findChild<QTabWidget *>(QStringLiteral("main_tabs"));
  QWidget * shared_safety_page = switched_tabs ? switched_tabs->widget(0) : nullptr;
  QWidget * shared_actions_page = switched_tabs ? switched_tabs->widget(1) : nullptr;
  QMetaObject::invokeMethod(
    integrated_selector, "updateSelected", Q_ARG(QString, QStringLiteral("piperh")));
  application.processEvents();
  auto * shared_zones = integrated_demo->findChild<QGroupBox *>(
    QStringLiteral("forbidden_zone_box"));
  auto * piper_safety_controls = integrated_selector->findChild<QWidget *>(
    QStringLiteral("piper_specific_safety_box"));
  auto * piper_teach_tabs = integrated_demo->findChild<QTabWidget *>(
    QStringLiteral("teach_tabs"));
  auto * action_mode_tabs = integrated_demo->findChild<QTabWidget *>(
    QStringLiteral("teach_action_mode_tabs"));
  if (!switched_tabs || switched_tabs->count() != 5 ||
    switched_tabs->widget(0) != shared_safety_page ||
    switched_tabs->widget(1) != shared_actions_page ||
    switched_tabs->tabText(2) != QStringLiteral("摄像头") ||
    switched_tabs->tabText(3) != QStringLiteral("手部视觉") ||
    switched_tabs->tabText(4) != QStringLiteral("手眼标定") ||
    !piper_teach_tabs || piper_teach_tabs->count() != 4 ||
    piper_teach_tabs->tabText(0) != QStringLiteral("单动作与形状") ||
    piper_teach_tabs->tabText(1) != QStringLiteral("真机演示操作") ||
    piper_teach_tabs->tabText(2) != QStringLiteral("动作组编排") ||
    piper_teach_tabs->tabText(3) != QStringLiteral("自由运动规划") ||
    !action_mode_tabs || action_mode_tabs->count() != 3 ||
    !shared_zones || !shared_safety_page->isAncestorOf(shared_zones) ||
    !piper_safety_controls || !shared_safety_page->isAncestorOf(piper_safety_controls) ||
    !shared_actions_page->isAncestorOf(preset_widget))
  {
    std::cerr << "Piper-H UI is not module-identical to the reBot workspace\n";
    ok = false;
  }

  auto * integrated_feature_tabs = integrated_selector->findChild<QTabWidget *>(
    QStringLiteral("dual_arm_feature_tabs"));
  auto * chassis_tabs = integrated_selector->findChild<QTabWidget *>(
    QStringLiteral("chassis_control_tabs"));
  application.processEvents();
  if (!chassis_tabs || chassis_tabs->count() != 2 ||
    chassis_tabs->widget(0) != vehicle_widget || chassis_tabs->widget(1) != waypoint_widget ||
    !integrated_selector->isAncestorOf(vehicle_widget) ||
    !integrated_selector->isAncestorOf(waypoint_widget))
  {
    std::cerr << "complete chassis panels were not embedded in chassis control\n";
    ok = false;
  }

  QMetaObject::invokeMethod(&panel, "integrateMotionPlanningPanel");
  application.processEvents();
  if (!panel.isAncestorOf(root) || root->isHidden() || dock.isVisible()) {
    std::cerr <<
      "MotionPlanning panel was not visibly embedded or duplicate dock stayed visible\n";
    ok = false;
  }
  auto * integrated_teach_tabs = panel.findChild<QTabWidget *>(QStringLiteral("teach_tabs"));
  if (!integrated_teach_tabs || integrated_teach_tabs->count() != 4 ||
    !integrated_teach_tabs->widget(3)->isAncestorOf(root))
  {
    std::cerr << "MotionPlanning panel is not inside the teaching workspace\n";
    ok = false;
  }
  for (int index = 3; index < tabs->count(); ++index) {
    if (tabs->isTabVisible(index)) {
      std::cerr << "integrated duplicate MoveIt page stayed visible: " << index << "\n";
      ok = false;
    }
  }
  const auto sliders = panel.findChildren<QSlider *>();
  ok = (!sliders.isEmpty()) && ok;
  if (sliders.isEmpty()) {
    std::cerr << "teaching timeline slider is missing\n";
  }
  auto * pulse_approach = panel.findChild<QPushButton *>(
    QStringLiteral("pulse_approach_button"));
  auto * pulse_region = panel.findChild<QLabel *>(
    QStringLiteral("pulse_region_label"));
  auto * pulse_standoff = panel.findChild<QDoubleSpinBox *>(
    QStringLiteral("pulse_standoff_spin"));
  auto * handeye_tag_family = panel.findChild<QComboBox *>(
    QStringLiteral("handeye_tag_family_combo"));
  auto * handeye_mode = panel.findChild<QComboBox *>(
    QStringLiteral("handeye_mode_combo"));
  auto * handeye_backend = panel.findChild<QComboBox *>(
    QStringLiteral("handeye_backend_combo"));
  auto * handeye_target = panel.findChild<QComboBox *>(
    QStringLiteral("handeye_target_combo"));
  auto * handeye_tag_id = panel.findChild<QSpinBox *>(
    QStringLiteral("handeye_tag_id_spin"));
  if (!pulse_approach || !pulse_region || !pulse_standoff ||
    pulse_approach->text() != QStringLiteral("识别把脉区并前往预接触位") ||
    pulse_standoff->value() != 60.0)
  {
    std::cerr << "pulse-region pre-contact controls are missing\n";
    ok = false;
  }
  if (!handeye_mode || handeye_mode->count() != 2 ||
    handeye_mode->itemData(0).toString() != QStringLiteral("eye_on_base") ||
    handeye_mode->itemData(1).toString() != QStringLiteral("eye_in_hand"))
  {
    std::cerr << "hand-eye calibration mode selector is missing or invalid\n";
    ok = false;
  }
  if (!handeye_backend || handeye_backend->count() != 2 ||
    handeye_backend->itemData(0).toString() != QStringLiteral("easy_handeye2") ||
    handeye_backend->itemData(1).toString() != QStringLiteral("moveit_calibration"))
  {
    std::cerr << "hand-eye calibration package selector is missing or invalid\n";
    ok = false;
  } else if (handeye_target) {
    handeye_backend->setCurrentIndex(1);
    if (handeye_target->isEnabled() ||
      handeye_target->currentData().toString() != QStringLiteral("charuco_a4_5x7"))
    {
      std::cerr << "MoveIt Calibration did not select and lock the ChArUco target\n";
      ok = false;
    }
    handeye_backend->setCurrentIndex(0);
  }
  if (!handeye_target || handeye_target->count() != 3 ||
    handeye_target->itemData(0).toString() != QStringLiteral("single_tag") ||
    handeye_target->itemData(1).toString() != QStringLiteral("a4_4tag_board") ||
    handeye_target->itemData(2).toString() != QStringLiteral("charuco_a4_5x7"))
  {
    std::cerr << "hand-eye calibration target selector is missing or invalid\n";
    ok = false;
  }
  if (!handeye_tag_family || !handeye_tag_id || handeye_tag_family->count() != 8 ||
    handeye_tag_family->currentText() != QStringLiteral("36h11") ||
    handeye_tag_family->findText(QStringLiteral("Standard52h13")) < 0 ||
    handeye_tag_id->maximum() != 586)
  {
    std::cerr << "AprilTag family selector is missing or invalid\n";
    ok = false;
  } else {
    handeye_tag_family->setCurrentText(QStringLiteral("16h5"));
    if (handeye_tag_id->maximum() != 29) {
      std::cerr << "AprilTag family did not update the tag ID range\n";
      ok = false;
    }
    handeye_tag_family->setCurrentText(QStringLiteral("36h11"));
    if (handeye_target) {
      handeye_target->setCurrentIndex(1);
      if (handeye_tag_family->isEnabled() || handeye_tag_id->isEnabled() ||
        handeye_tag_family->currentText() != QStringLiteral("36h11") ||
        handeye_tag_id->value() != 0)
      {
        std::cerr << "A4 board selection did not lock the fixed family and IDs\n";
        ok = false;
      }
      handeye_target->setCurrentIndex(0);
      handeye_target->setCurrentIndex(2);
      if (handeye_tag_family->isEnabled() || handeye_tag_id->isEnabled()) {
        std::cerr << "ChArUco selection did not lock irrelevant AprilTag controls\n";
        ok = false;
      }
      handeye_target->setCurrentIndex(0);
    }
  }
  auto * singularity_escape = panel.findChild<QPushButton *>(
    QStringLiteral("singularity_escape_button"));
  auto * singularity_reset = panel.findChild<QPushButton *>(
    QStringLiteral("singularity_escape_reset_button"));
  auto * singularity_joint_3 = panel.findChild<QDoubleSpinBox *>(
    QStringLiteral("singularity_escape_joint_3_spin"));
  auto * singularity_joint_5 = panel.findChild<QDoubleSpinBox *>(
    QStringLiteral("singularity_escape_joint_5_spin"));
  if (!singularity_escape || !singularity_reset || !singularity_joint_3 ||
    !singularity_joint_5 ||
    singularity_escape->text() != QStringLiteral("规划并脱离奇异位形") ||
    singularity_reset->text() != QStringLiteral("恢复默认角度") ||
    singularity_joint_3->value() != -0.90 || singularity_joint_5->value() != 0.50)
  {
    std::cerr << "singularity-escape joint controls are missing or invalid\n";
    ok = false;
  }
  bool found_teach_group = false;
  auto * shape_reachability = panel.findChild<QPushButton *>(
    QStringLiteral("teach_shape_reachability_button"));
  auto * shape_reachability_label = panel.findChild<QLabel *>(
    QStringLiteral("teach_shape_reachability_label"));
  if (!shape_reachability || !shape_reachability_label ||
    shape_reachability->text() != QStringLiteral("预检当前笔尖轨迹（不动真机）") ||
    !shape_reachability_label->text().contains(QStringLiteral("尚未预检")))
  {
    std::cerr << "shape reachability preflight controls are missing\n";
    ok = false;
  }
  auto * sequence_overlay = panel.findChild<QPushButton *>(
    QStringLiteral("teach_sequence_preview_button"));
  auto * teach_tabs = panel.findChild<QTabWidget *>(QStringLiteral("teach_tabs"));
  auto * teach_action_combo = panel.findChild<QComboBox *>(
    QStringLiteral("teach_action_combo"));
  auto * robot_demo_controls = panel.findChild<QGroupBox *>(
    QStringLiteral("demo_operation_box"));
  auto * teach_delete = panel.findChild<QPushButton *>(
    QStringLiteral("teach_delete_button"));
  auto * teach_copy = panel.findChild<QPushButton *>(
    QStringLiteral("teach_copy_button"));
  auto * sequence_play = panel.findChild<QPushButton *>(
    QStringLiteral("teach_sequence_play_button"));
  auto * sequence_pause = panel.findChild<QPushButton *>(
    QStringLiteral("teach_sequence_pause_button"));
  auto * sequence_cancel = panel.findChild<QPushButton *>(
    QStringLiteral("teach_sequence_cancel_button"));
  auto * sequence_speed = panel.findChild<QDoubleSpinBox *>(
    QStringLiteral("teach_sequence_replay_speed_spin"));
  auto * sequence_slider = panel.findChild<QSlider *>(
    QStringLiteral("teach_sequence_slider"));
  auto * sequence_list = panel.findChild<QListWidget *>(
    QStringLiteral("teach_sequence_list"));
  if (!teach_delete || teach_delete->text() != QStringLiteral("删除")) {
    std::cerr << "teach action-pack delete control is missing\n";
    ok = false;
  }
  if (!teach_copy || teach_copy->text() != QStringLiteral("复制") ||
    !sequence_play || sequence_play->text() != QStringLiteral("播放勾选动作的 RViz 动画") ||
    !sequence_pause || sequence_pause->text() != QStringLiteral("暂停 RViz 动画") ||
    !sequence_cancel || sequence_cancel->text() != QStringLiteral("取消 RViz 动画") ||
    !sequence_speed || sequence_speed->value() != 1.0 ||
    !sequence_slider || !sequence_list)
  {
    std::cerr << "copy or sequence playback controls are missing\n";
    ok = false;
  }
  if (sequence_overlay) {
    std::cerr << "removed sequence overlay control is still present\n";
    ok = false;
  }
  if (!teach_tabs || !teach_action_combo || !robot_demo_controls || teach_tabs->count() < 2 ||
    !teach_tabs->widget(0)->isAncestorOf(teach_action_combo) ||
    teach_tabs->widget(1)->isAncestorOf(teach_action_combo) ||
    teach_tabs->widget(0)->isAncestorOf(robot_demo_controls) ||
    !teach_tabs->widget(1)->isAncestorOf(robot_demo_controls))
  {
    std::cerr << "single-action and robot-demo controls are not in separate peer tabs\n";
    ok = false;
  }
  auto * forbidden_zone_box = panel.findChild<QGroupBox *>(
    QStringLiteral("forbidden_zone_box"));
  auto * forbidden_zone_reload = panel.findChild<QPushButton *>(
    QStringLiteral("forbidden_zone_reload_button"));
  auto * forbidden_zone_state = panel.findChild<QLabel *>(
    QStringLiteral("forbidden_zone_state_label"));
  auto * forbidden_zone_shape = panel.findChild<QComboBox *>(
    QStringLiteral("forbidden_zone_shape_combo"));
  auto * forbidden_zone_apply = panel.findChild<QPushButton *>(
    QStringLiteral("forbidden_zone_apply_button"));
  auto * forbidden_zone_group_members = panel.findChild<QListWidget *>(
    QStringLiteral("forbidden_zone_group_members_list"));
  auto * forbidden_zone_group_save = panel.findChild<QPushButton *>(
    QStringLiteral("forbidden_zone_group_save_button"));
  if (!forbidden_zone_box || !forbidden_zone_reload || !forbidden_zone_state ||
    !forbidden_zone_shape || !forbidden_zone_apply || !forbidden_zone_group_members ||
    !forbidden_zone_group_save || forbidden_zone_shape->count() != 5 ||
    forbidden_zone_shape->findData(QStringLiteral("mesh")) < 0 ||
    forbidden_zone_box->title() != QStringLiteral("三维禁止通行区域") ||
    forbidden_zone_reload->text() != QStringLiteral("重新加载禁区") ||
    forbidden_zone_apply->text() != QStringLiteral("导入并加入 MoveIt 禁区") ||
    forbidden_zone_group_save->text() != QStringLiteral("保存或替换命名禁区组") ||
    !forbidden_zone_state->text().contains(QStringLiteral("禁区")))
  {
    std::cerr << "forbidden-zone safety panel is missing or untranslated\n";
    ok = false;
  }
  bool found_xbox_mode_button = false;
  bool found_camera_button = false;
  QPushButton * xbox_help_button = nullptr;
  for (auto * button : panel.findChildren<QPushButton *>()) {
    found_xbox_mode_button = found_xbox_mode_button ||
      button->text().contains(QStringLiteral("Xbox 控制"));
    found_camera_button = found_camera_button ||
      button->text() == QStringLiteral("打开摄像头");
    if (button->text().contains(QStringLiteral("Xbox 按键说明"))) {
      xbox_help_button = button;
    }
  }
  if (!found_xbox_mode_button || !found_camera_button || !xbox_help_button) {
    std::cerr << "Xbox mode, camera, or controller-help button is missing\n";
    ok = false;
  }
  auto * xbox_help_table = panel.findChild<QTableWidget *>();
  if (!xbox_help_table || xbox_help_table->rowCount() != 12 ||
    xbox_help_table->isVisible())
  {
    std::cerr << "collapsed Xbox controller mapping table is invalid\n";
    ok = false;
  } else if (xbox_help_button) {
    xbox_help_button->setChecked(true);
    application.processEvents();
    if (!xbox_help_table->isVisible() ||
      xbox_help_table->horizontalHeaderItem(0)->text() != QStringLiteral("按键/控制"))
    {
      std::cerr << "Xbox controller mapping table did not expand\n";
      ok = false;
    }
  }
  for (auto * group : panel.findChildren<QGroupBox *>()) {
    if (group->title().contains(QStringLiteral("动作编辑"))) {
      found_teach_group = true;
      break;
    }
  }
  if (!found_teach_group) {
    std::cerr << "drag teaching controls are missing\n";
  }
  ok = found_teach_group && ok;

  QComboBox * shape_combo = nullptr;
  for (auto * combo : panel.findChildren<QComboBox *>()) {
    if (combo->findData(QStringLiteral("rectangle")) >= 0 &&
      combo->findData(QStringLiteral("star")) >= 0 &&
      combo->findData(QStringLiteral("text")) >= 0 &&
      combo->findData(QStringLiteral("image")) >= 0 &&
      combo->findData(QStringLiteral("freehand")) >= 0)
    {
      shape_combo = combo;
      break;
    }
  }
  if (!shape_combo) {
    std::cerr << "shape selector is missing\n";
    ok = false;
  } else {
    shape_combo->setCurrentIndex(shape_combo->findData(QStringLiteral("freehand")));
    application.processEvents();
    bool offers_show = false;
    for (auto * button : panel.findChildren<QPushButton *>()) {
      offers_show = offers_show || button->text() == QStringLiteral("显示空间定位标记");
    }
    if (!offers_show) {
      std::cerr << "freehand selection did not hide the shape marker\n";
      ok = false;
    }

    shape_combo->setCurrentIndex(shape_combo->findData(QStringLiteral("star")));
    application.processEvents();
    bool offers_hide = false;
    for (auto * button : panel.findChildren<QPushButton *>()) {
      offers_hide = offers_hide || button->text() == QStringLiteral("隐藏空间定位标记");
    }
    if (!offers_hide) {
      std::cerr << "built-in shape selection did not restore the shape marker\n";
      ok = false;
    }
  }
  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
