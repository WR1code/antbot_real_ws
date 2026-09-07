#ifndef ROBOTCAR_NAVIGATION__WAYPOINT_MANAGER_PANEL_H_
#define ROBOTCAR_NAVIGATION__WAYPOINT_MANAGER_PANEL_H_

#include <memory>
#include <string>
#include <vector>

#include <QPointer>
#include <QStringList>

#include "action_msgs/srv/cancel_goal.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotcar_navigation/srv/get_waypoint_names.hpp"
#include "robotcar_navigation/srv/rename_waypoint.hpp"
#include "robotcar_navigation/srv/waypoint_file.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QProgressBar;
class QPushButton;
class QTreeWidget;

namespace robotcar_navigation
{
namespace rviz_plugins
{

class WaypointManagerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointManagerPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void refreshWaypoints();
  void refreshChargers();
  void renameWaypoint();
  void saveWaypointGroup();
  void openWaypointGroup();
  void openMap();
  void addSelectedToRoute();
  void useAllWaypoints();
  void removeRouteItem();
  void moveRouteItemUp();
  void moveRouteItemDown();
  void saveRouteGroup();
  void openRouteGroup();
  void navigateSelected();
  void navigateToCharger();
  void startRoute();
  void pauseOrResumeRoute();
  void stopRoute();
  void refreshKeepouts();
  void activateKeepoutTool();
  void saveKeepoutGroup();
  void openKeepoutGroup();
  void renameSelectedKeepout();
  void deleteSelectedKeepout();
  void toggleKeepouts();
  void refreshSpeedZones();
  void activateSpeedZoneTool();
  void saveSpeedZoneGroup();
  void openSpeedZoneGroup();
  void renameSelectedSpeedZone();
  void deleteSelectedSpeedZone();
  void toggleSpeedZones();
  void exportTaskHistory();

private:
  void activateZoneTool(
    const QString & class_id, const QString & display_name, const QString & shape);
  void setStatus(const QString & text, bool error = false);
  void requestWaypointFile(const QString & filename, bool save);
  void publishCurrentRouteTarget();
  void handleNavigationResult(const std_msgs::msg::String::SharedPtr message);
  void handleExternalRouteProgress(const std_msgs::msg::String::SharedPtr message);
  void handleChargeResult(const std_msgs::msg::String::SharedPtr message);
  void handleKeepoutStatus(const std_msgs::msg::String::SharedPtr message);
  void handleSpeedZoneStatus(const std_msgs::msg::String::SharedPtr message);
  void appendTaskHistory(
    const QString & task, const QString & result, const QString & detail);
  void loadTaskHistory();
  void resetRouteProgress();
  void showRouteProgress(
    int completed, int total, int current_index, const QString & current_name,
    const QString & state, bool error = false);
  void setRouteEditingEnabled(bool enabled);
  QString suggestedDirectory() const;
  QStringList routeNames() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<srv::GetWaypointNames>::SharedPtr names_client_;
  rclcpp::Client<srv::RenameWaypoint>::SharedPtr rename_client_;
  rclcpp::Client<srv::GetWaypointNames>::SharedPtr charger_names_client_;
  rclcpp::Client<srv::WaypointFile>::SharedPtr save_group_client_;
  rclcpp::Client<srv::WaypointFile>::SharedPtr load_group_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_client_;
  rclcpp::Client<srv::GetWaypointNames>::SharedPtr keepout_names_client_;
  rclcpp::Client<srv::RenameWaypoint>::SharedPtr rename_keepout_client_;
  rclcpp::Client<srv::WaypointFile>::SharedPtr save_keepout_client_;
  rclcpp::Client<srv::WaypointFile>::SharedPtr load_keepout_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_keepout_client_;
  rclcpp::Client<srv::GetWaypointNames>::SharedPtr speed_zone_names_client_;
  rclcpp::Client<srv::RenameWaypoint>::SharedPtr rename_speed_zone_client_;
  rclcpp::Client<srv::WaypointFile>::SharedPtr save_speed_zone_client_;
  rclcpp::Client<srv::WaypointFile>::SharedPtr load_speed_zone_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_speed_zone_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr charger_navigation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr delete_keepout_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr delete_speed_zone_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_control_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_result_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr route_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr charge_result_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keepout_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr speed_zone_status_sub_;

  QComboBox * waypoint_combo_;
  QComboBox * charger_combo_;
  QComboBox * keepout_combo_;
  QComboBox * speed_zone_combo_;
  QComboBox * keepout_shape_combo_;
  QComboBox * speed_zone_shape_combo_;
  QLineEdit * rename_edit_;
  QLineEdit * keepout_rename_edit_;
  QLineEdit * speed_zone_rename_edit_;
  QLabel * active_group_label_;
  QLabel * active_map_label_;
  QLabel * active_keepout_label_;
  QLabel * active_speed_zone_label_;
  QListWidget * route_list_;
  QLabel * route_progress_label_;
  QProgressBar * route_progress_bar_;
  QLabel * status_label_;
  QPushButton * start_button_;
  QPushButton * pause_button_;
  QPushButton * stop_button_;
  QPushButton * keepout_enable_button_;
  QPushButton * speed_zone_enable_button_;
  QTreeWidget * history_tree_;
  std::vector<QWidget *> route_edit_widgets_;

  QString active_group_file_;
  QString active_map_file_;
  QString last_directory_;
  QStringList active_route_;
  int active_route_index_ = -1;
  bool route_running_ = false;
  bool route_paused_ = false;
  bool external_route_active_ = false;
  bool external_route_paused_ = false;
  bool discard_next_navigation_result_ = false;
  QString last_external_progress_key_;
  QString history_file_;
};

}  // namespace rviz_plugins
}  // namespace robotcar_navigation

#endif  // ROBOTCAR_NAVIGATION__WAYPOINT_MANAGER_PANEL_H_
