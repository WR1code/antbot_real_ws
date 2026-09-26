#include "rebotarm_demo_rviz/motion_planning_i18n.hpp"

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAction>
#include <QApplication>
#include <QByteArray>
#include <QDockWidget>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QStringList>
#include <QTableWidget>
#include <QTabWidget>
#include <QTreeWidget>
#include <QVariant>
#include <QWidget>

namespace rebotarm_demo_rviz
{
namespace
{

struct Translation
{
  const char * english;
  const char * chinese;
};

// Text comes from MotionPlanningUI and MotionPlanningFrame in MoveIt 2 Jazzy.
constexpr Translation kTranslations[]{
  {"MotionPlanning", "运动规划"},
  {"MoveIt Planning Frame", "MoveIt 运动规划"},
  {"Context", "上下文"},
  {"Planning", "规划"},
  {"Joints", "关节"},
  {"Scene Objects", "场景物体"},
  {"Scene Geometry", "场景几何"},
  {"Stored Scenes", "已存场景"},
  {"Stored States", "已存姿态"},
  {"Status", "状态"},
  {"Manipulation", "操作"},
  {"Planning Library", "规划库"},
  {"Planning Library Name", "规划库名称"},
  {"Planner Parameters", "规划器参数"},
  {"Warehouse", "数据库"},
  {"Host:", "主机："},
  {"Port:", "端口："},
  {"Reset database ...", "重置数据库…"},
  {"Connect", "连接"},
  {"Center (XYZ):", "中心 (XYZ)："},
  {"Size (XYZ):", "尺寸 (XYZ)："},
  {"Commands", "命令"},
  {"&Plan", "规划"},
  {"&Execute", "执行"},
  {"Plan && E&xecute", "规划并执行"},
  {"Plan & Execute", "规划并执行"},
  {"&Stop", "停止"},
  {"Clear octomap", "清除八叉树地图"},
  {"Planning Group:", "规划组："},
  {"Start State:", "起始状态："},
  {"Goal State:", "目标状态："},
  {"Path Constraints", "路径约束"},
  {"Options", "选项"},
  {"Planning Time (s):", "规划时间（秒）："},
  {"Planning Attempts:", "规划尝试次数："},
  {"Velocity Scaling:", "速度缩放："},
  {"Accel. Scaling:", "加速度缩放："},
  {"Use Cartesian Path", "使用笛卡尔路径"},
  {"Collision-aware IK", "带碰撞检查的逆解"},
  {"Approx IK Solutions", "允许近似逆解"},
  {"External Comm.", "允许外部控制"},
  {"Sensor Positioning", "传感器定位"},
  {"Current Scene Objects", "当前场景物体"},
  {"Add/Remove scene object(s)", "添加/移除场景物体"},
  {"Change object pose/scale", "修改物体位姿/缩放"},
  {"Position: ", "位置："},
  {"Rotation:", "旋转："},
  {"Scale:", "缩放："},
  {"Object status", "物体状态"},
  {"&Publish", "发布"},
  {"&Export", "导出"},
  {"&Import", "导入"},
  {"Box", "长方体"},
  {"Sphere", "球体"},
  {"Cylinder", "圆柱体"},
  {"Cone", "圆锥体"},
  {"Mesh from file", "从文件加载网格"},
  {"Mesh from URL", "从 URL 加载网格"},
  {"Planning Scenes and Queries", "规划场景和查询"},
  {"Saved Scenes", "已保存场景"},
  {"Load &Scene", "加载场景"},
  {"Load &Query", "加载查询"},
  {"Delete Scene", "删除场景"},
  {"Delete Query", "删除查询"},
  {"Current Scene", "当前场景"},
  {"Save Scene", "保存场景"},
  {"Save Query", "保存查询"},
  {"Stored Robot States", "已存机械臂状态"},
  {"Load a robot state", "加载机械臂状态"},
  {"Load", "加载"},
  {"Clear", "清空"},
  {"Selected Robot State", "选中的机械臂状态"},
  {"Set as Start", "设为起点"},
  {"Set as Goal", "设为目标"},
  {"Remove", "移除"},
  {"Current Robot State", "当前机械臂状态"},
  {"Save Start", "保存起点"},
  {"Save Goal", "保存目标"},
  {"Detected Objects", "检测到的物体"},
  {"&Detect", "检测"},
  {"Support Surfaces", "支撑面"},
  {"P&lace", "放置"},
  {"&Pick", "抓取"},
  {"Size (m): ", "尺寸（米）："},
  {"Center (m): ", "中心（米）："},
  {"<random valid>", "<随机有效状态>"},
  {"<random>", "<随机状态>"},
  {"<current>", "<当前状态>"},
  {"<same as goal>", "<与目标相同>"},
  {"<previous>", "<上一状态>"},
  {"<same as start>", "<与起点相同>"},
  {"Joint Name", "关节名称"},
  {"Value", "数值"},
  {"Group joints:", "规划组关节："},
  {"Nullspace exploration:", "零空间探索："},
  {"Planning...", "正在规划…"},
  {"Failed", "失败"},
  {"Stopped", "已停止"},
  {"Executed", "已执行"},
  {"Not connected to a database.", "未连接数据库。"},
};

QString translatedFromEnglish(const QString & english, bool chinese)
{
  if (!chinese) {
    return english;
  }
  for (const auto & item : kTranslations) {
    if (english == QString::fromUtf8(item.english)) {
      return QString::fromUtf8(item.chinese);
    }
  }

  const QString english_time = QStringLiteral("Time: ");
  if (english.startsWith(english_time)) {
    return QStringLiteral("耗时：") + english.mid(english_time.size());
  }
  return english;
}

QString translatedProperty(
  QObject * object, const char * property_name, const QString & current, bool chinese)
{
  const QVariant stored = object->property(property_name);
  QString original = stored.toString();
  if (!stored.isValid() ||
    (current != original && current != translatedFromEnglish(original, true)))
  {
    original = current;
    object->setProperty(property_name, original);
  }
  return translatedFromEnglish(original, chinese);
}

void translateWidget(QWidget * widget, bool chinese)
{
  if (auto * label = qobject_cast<QLabel *>(widget)) {
    label->setText(translatedProperty(
      label, "rebot_i18n_text", label->text(), chinese));
  }
  if (auto * button = qobject_cast<QAbstractButton *>(widget)) {
    button->setText(translatedProperty(
      button, "rebot_i18n_text", button->text(), chinese));
  }
  if (auto * group = qobject_cast<QGroupBox *>(widget)) {
    group->setTitle(translatedProperty(
      group, "rebot_i18n_title", group->title(), chinese));
  }
  if (auto * tabs = qobject_cast<QTabWidget *>(widget)) {
    for (int index = 0; index < tabs->count(); ++index) {
      const QByteArray text_property = QByteArray("rebot_i18n_tab_text_") + QByteArray::number(index);
      const QByteArray tip_property = QByteArray("rebot_i18n_tab_tip_") + QByteArray::number(index);
      const QString translated_text = translatedProperty(
        tabs, text_property.constData(), tabs->tabText(index), chinese);
      tabs->setTabText(index, translated_text);
      tabs->setTabToolTip(index, translatedProperty(
        tabs, tip_property.constData(), tabs->tabToolTip(index), chinese));
    }
  }
  // Do not rewrite QComboBox items: MoveIt compares special values such as
  // <current> and shape names by currentText(), so changing them would alter behavior.
  if (auto * tree = qobject_cast<QTreeWidget *>(widget)) {
    for (int column = 0; column < tree->columnCount(); ++column) {
      const QByteArray property = QByteArray("rebot_i18n_tree_header_") +
        QByteArray::number(column);
      tree->headerItem()->setText(
        column, translatedProperty(
          tree, property.constData(), tree->headerItem()->text(column), chinese));
    }
  }
  if (auto * table = qobject_cast<QTableWidget *>(widget)) {
    for (int column = 0; column < table->columnCount(); ++column) {
      if (auto * item = table->horizontalHeaderItem(column)) {
        const QByteArray property = QByteArray("rebot_i18n_table_header_") +
          QByteArray::number(column);
        item->setText(translatedProperty(
          table, property.constData(), item->text(), chinese));
      }
    }
  }
  if (auto * view = qobject_cast<QAbstractItemView *>(widget)) {
    if (auto * model = view->model()) {
      for (int column = 0; column < model->columnCount(); ++column) {
        const QString current = model->headerData(column, Qt::Horizontal).toString();
        const QByteArray property = QByteArray("rebot_i18n_view_header_") +
          QByteArray::number(column);
        model->setHeaderData(
          column, Qt::Horizontal,
          translatedProperty(view, property.constData(), current, chinese));
      }
    }
  }
  widget->setToolTip(translatedProperty(
    widget, "rebot_i18n_tooltip", widget->toolTip(), chinese));
  widget->setWhatsThis(translatedProperty(
    widget, "rebot_i18n_whats_this", widget->whatsThis(), chinese));
}

QWidget * motionPlanningRoot()
{
  for (auto * widget : QApplication::allWidgets()) {
    if (widget->objectName() != QStringLiteral("plan_button")) {
      continue;
    }
    auto * plan_button = qobject_cast<QPushButton *>(widget);
    if (!plan_button) {
      continue;
    }
    for (QWidget * ancestor = plan_button->parentWidget(); ancestor;
      ancestor = ancestor->parentWidget())
    {
      if (ancestor->findChild<QTabWidget *>(
          QStringLiteral("tabWidget"), Qt::FindDirectChildrenOnly))
      {
        return ancestor;
      }
      if (qobject_cast<QDockWidget *>(ancestor)) {
        break;
      }
    }
  }
  return nullptr;
}

}  // namespace

bool translateMotionPlanningUi(bool chinese)
{
  auto * root = motionPlanningRoot();
  if (!root) {
    return false;
  }

  translateWidget(root, chinese);
  for (auto * widget : root->findChildren<QWidget *>()) {
    translateWidget(widget, chinese);
  }
  for (auto * action : root->findChildren<QAction *>()) {
    action->setText(translatedProperty(
      action, "rebot_i18n_text", action->text(), chinese));
    action->setToolTip(translatedProperty(
      action, "rebot_i18n_tooltip", action->toolTip(), chinese));
  }

  for (QWidget * ancestor = root->parentWidget(); ancestor;
    ancestor = ancestor->parentWidget())
  {
    if (auto * dock = qobject_cast<QDockWidget *>(ancestor)) {
      if (!dock->property("rebot_preserve_dock_title").toBool()) {
        dock->setWindowTitle(
          chinese ? QStringLiteral("运动规划") : QStringLiteral("MotionPlanning"));
      }
      break;
    }
  }
  return true;
}

bool setMotionPlanningIntegratedMode(bool integrated)
{
  auto * root = motionPlanningRoot();
  if (!root) {
    return false;
  }
  auto * tabs = root->findChild<QTabWidget *>(
    QStringLiteral("tabWidget"), Qt::FindDirectChildrenOnly);
  if (!tabs) {
    return false;
  }

  // Context, Planning and Joints are the unique low-level planning tools.
  // Scene editing, stored data, status and manipulation are already covered by
  // reBot's forbidden-zone, action-library, readiness and pick/place controls.
  const QStringList redundant_pages{
    QStringLiteral("Scene Objects"),
    QStringLiteral("Stored Scenes"),
    QStringLiteral("Stored States"),
    QStringLiteral("Status"),
    QStringLiteral("Manipulation"),
  };
  for (int index = 0; index < tabs->count(); ++index) {
    const QByteArray text_property = QByteArray("rebot_i18n_tab_text_") +
      QByteArray::number(index);
    const QVariant stored = tabs->property(text_property.constData());
    const QString original_text = stored.isValid() ? stored.toString() : tabs->tabText(index);
    tabs->setTabVisible(index, !integrated || !redundant_pages.contains(original_text));
  }
  return true;
}

QDockWidget * motionPlanningDockWidget()
{
  auto * root = motionPlanningRoot();
  if (!root) {
    return nullptr;
  }
  for (QWidget * ancestor = root; ancestor; ancestor = ancestor->parentWidget()) {
    if (auto * dock = qobject_cast<QDockWidget *>(ancestor)) {
      return dock;
    }
  }
  return nullptr;
}

}  // namespace rebotarm_demo_rviz
