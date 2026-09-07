#ifndef ROBOTCAR_NAVIGATION__WAYPOINT_MANAGER_CORE_HPP_
#define ROBOTCAR_NAVIGATION__WAYPOINT_MANAGER_CORE_HPP_

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robotcar_navigation/msg/waypoint.hpp"
#include "robotcar_navigation/srv/get_charger_by_name.hpp"
#include "robotcar_navigation/srv/get_num_of_waypoints.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_index.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_name.hpp"
#include "robotcar_navigation/srv/save_waypoints.hpp"
#include "robotcar_navigation/waypoint_xml.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace robotcar_navigation
{

class WaypointManagerNode : public rclcpp::Node
{
public:
  explicit WaypointManagerNode(const std::string & node_name)
  : Node(node_name)
  {
    const char * home = std::getenv("HOME");
    const std::string default_file = std::string(home ? home : "/tmp") + "/waypoints.xml";
    load_file_ = declare_parameter<std::string>("load", default_file);

    if (!load_file_.empty()) {
      RCLCPP_INFO(get_logger(), "Loading waypoints from: %s", load_file_.c_str());
      if (!load_waypoints(load_file_, waypoints_, chargers_)) {
        RCLCPP_WARN(get_logger(), "Failed to load waypoint file: %s", load_file_.c_str());
      }
    }

    waypoints_pub_ = create_publisher<visualization_msgs::msg::Marker>("waypoints_marker", 100);
    chargers_pub_ = create_publisher<visualization_msgs::msg::Marker>("chargers_marker", 100);

    add_waypoint_sub_ = create_subscription<msg::Waypoint>(
      "waterplus/add_waypoint", 10,
      [this](const msg::Waypoint::SharedPtr msg) { add_waypoint(*msg); });
    add_charger_sub_ = create_subscription<msg::Waypoint>(
      "waterplus/add_charger", 10,
      [this](const msg::Waypoint::SharedPtr msg) { add_charger(*msg); });

    srv_get_num_wp_ = create_service<srv::GetNumOfWaypoints>(
      "waterplus/get_num_waypoint",
      [this](const std::shared_ptr<srv::GetNumOfWaypoints::Request> request,
      std::shared_ptr<srv::GetNumOfWaypoints::Response> response) {
        (void)request;
        response->num = static_cast<int32_t>(waypoints_.size());
      });
    srv_get_wp_index_ = create_service<srv::GetWaypointByIndex>(
      "waterplus/get_waypoint_index",
      [this](const std::shared_ptr<srv::GetWaypointByIndex::Request> request,
      std::shared_ptr<srv::GetWaypointByIndex::Response> response) {
        fill_by_index(waypoints_, request->index, response->name, response->pose);
      });
    srv_get_wp_name_ = create_service<srv::GetWaypointByName>(
      "waterplus/get_waypoint_name",
      [this](const std::shared_ptr<srv::GetWaypointByName::Request> request,
      std::shared_ptr<srv::GetWaypointByName::Response> response) {
        fill_by_name(waypoints_, request->name, response->name, response->pose);
      });
    srv_save_wp_ = create_service<srv::SaveWaypoints>(
      "waterplus/save_waypoints",
      [this](const std::shared_ptr<srv::SaveWaypoints::Request> request,
      std::shared_ptr<srv::SaveWaypoints::Response> response) {
        (void)response;
        save_waypoints(request->filename, waypoints_, chargers_);
      });

    srv_get_num_charger_ = create_service<srv::GetNumOfWaypoints>(
      "waterplus/get_num_charger",
      [this](const std::shared_ptr<srv::GetNumOfWaypoints::Request> request,
      std::shared_ptr<srv::GetNumOfWaypoints::Response> response) {
        (void)request;
        response->num = static_cast<int32_t>(chargers_.size());
      });
    srv_get_charger_index_ = create_service<srv::GetWaypointByIndex>(
      "waterplus/get_charger_index",
      [this](const std::shared_ptr<srv::GetWaypointByIndex::Request> request,
      std::shared_ptr<srv::GetWaypointByIndex::Response> response) {
        fill_by_index(chargers_, request->index, response->name, response->pose);
      });
    srv_get_charger_name_ = create_service<srv::GetChargerByName>(
      "waterplus/get_charger_name",
      [this](const std::shared_ptr<srv::GetChargerByName::Request> request,
      std::shared_ptr<srv::GetChargerByName::Response> response) {
        fill_by_name(chargers_, request->name, response->name, response->pose);
      });

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publish_markers(); });
  }

private:
  static bool fill_by_index(
    const std::vector<msg::Waypoint> & items,
    int32_t index,
    std::string & name,
    geometry_msgs::msg::Pose & pose)
  {
    if (index < 0 || static_cast<size_t>(index) >= items.size()) {
      return false;
    }
    name = items[static_cast<size_t>(index)].name;
    pose = items[static_cast<size_t>(index)].pose;
    return true;
  }

  static bool fill_by_name(
    const std::vector<msg::Waypoint> & items,
    const std::string & query,
    std::string & name,
    geometry_msgs::msg::Pose & pose)
  {
    auto found = std::find_if(items.begin(), items.end(), [&query](const auto & item) {
      return item.name.find(query) != std::string::npos;
    });
    if (found == items.end()) {
      return false;
    }
    name = found->name;
    pose = found->pose;
    return true;
  }

  void add_waypoint(const msg::Waypoint & waypoint)
  {
    waypoints_.push_back(waypoint);
    RCLCPP_INFO(get_logger(), "Added waypoint: %s", waypoint.name.c_str());
  }

  void add_charger(const msg::Waypoint & charger)
  {
    chargers_.push_back(charger);
    RCLCPP_INFO(get_logger(), "Added charger: %s", charger.name.c_str());
  }

  visualization_msgs::msg::Marker make_mesh_marker(
    const std::string & ns,
    int32_t id,
    const msg::Waypoint & waypoint,
    const std::string & mesh_resource) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.mesh_resource = mesh_resource;
    marker.mesh_use_embedded_materials = true;
    marker.pose = waypoint.pose;
    marker.pose.position.z = -0.01;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker;
  }

  visualization_msgs::msg::Marker make_text_marker(
    int32_t id,
    const std::string & text,
    double x,
    double y,
    double z) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "text";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    marker.pose.orientation = tf2::toMsg(q);
    marker.text = text;
    return marker;
  }

  void publish_markers()
  {
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto & waypoint = waypoints_[i];
      waypoints_pub_->publish(make_mesh_marker(
        "marker_waypoints", static_cast<int32_t>(i), waypoint,
        "package://robotcar_navigation/meshes/waypoint.dae"));
      waypoints_pub_->publish(make_text_marker(
        static_cast<int32_t>(i), waypoint.name,
        waypoint.pose.position.x, waypoint.pose.position.y, 0.55));
    }

    for (size_t i = 0; i < chargers_.size(); ++i) {
      const auto & charger = chargers_[i];
      chargers_pub_->publish(make_mesh_marker(
        "marker_chargers", static_cast<int32_t>(i), charger,
        "package://robotcar_navigation/meshes/charger.dae"));
      chargers_pub_->publish(make_text_marker(
        static_cast<int32_t>(waypoints_.size() + i), charger.name,
        charger.pose.position.x, charger.pose.position.y, 0.35));
    }
  }

  std::string load_file_;
  std::vector<msg::Waypoint> waypoints_;
  std::vector<msg::Waypoint> chargers_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr chargers_pub_;
  rclcpp::Subscription<msg::Waypoint>::SharedPtr add_waypoint_sub_;
  rclcpp::Subscription<msg::Waypoint>::SharedPtr add_charger_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Service<srv::GetNumOfWaypoints>::SharedPtr srv_get_num_wp_;
  rclcpp::Service<srv::GetWaypointByIndex>::SharedPtr srv_get_wp_index_;
  rclcpp::Service<srv::GetWaypointByName>::SharedPtr srv_get_wp_name_;
  rclcpp::Service<srv::SaveWaypoints>::SharedPtr srv_save_wp_;
  rclcpp::Service<srv::GetNumOfWaypoints>::SharedPtr srv_get_num_charger_;
  rclcpp::Service<srv::GetWaypointByIndex>::SharedPtr srv_get_charger_index_;
  rclcpp::Service<srv::GetChargerByName>::SharedPtr srv_get_charger_name_;
};

}  // namespace robotcar_navigation

#endif  // ROBOTCAR_NAVIGATION__WAYPOINT_MANAGER_CORE_HPP_
