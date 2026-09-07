#include <algorithm>
#include <cctype>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotcar_navigation/msg/waypoint.hpp"
#include "robotcar_navigation/srv/get_charger_by_name.hpp"
#include "robotcar_navigation/srv/get_num_of_waypoints.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_index.hpp"
#include "robotcar_navigation/srv/get_waypoint_by_name.hpp"
#include "robotcar_navigation/srv/get_waypoint_names.hpp"
#include "robotcar_navigation/srv/rename_waypoint.hpp"
#include "robotcar_navigation/srv/save_waypoints.hpp"
#include "robotcar_navigation/srv/waypoint_file.hpp"
#include "robotcar_navigation/waypoint_xml.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace robotcar_navigation
{

class WaypointEditNode : public rclcpp::Node
{
public:
  WaypointEditNode()
  : Node("wp_edit_node")
  {
    const auto default_file =
      ament_index_cpp::get_package_share_directory("robotcar_navigation") + "/config/waypoints.xml";
    load_file_ = declare_parameter<std::string>("load", default_file);

    const auto text_qos = rclcpp::QoS(rclcpp::KeepLast(100)).transient_local().reliable();
    text_pub_ = create_publisher<visualization_msgs::msg::Marker>("text_marker", text_qos);
    add_waypoint_sub_ = create_subscription<msg::Waypoint>(
      "waterplus/add_waypoint", 10,
      [this](const msg::Waypoint::SharedPtr msg) { add_waypoint(*msg); });
    add_charger_sub_ = create_subscription<msg::Waypoint>(
      "waterplus/add_charger", 10,
      [this](const msg::Waypoint::SharedPtr msg) { add_charger(*msg); });
    reached_sub_ = create_subscription<std_msgs::msg::String>(
      "/waypoint_reached", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { waypoint_reached(msg->data); });

    create_services();
  }

  void initialize()
  {
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "waypoints_move", shared_from_this());

    waypoint_menu_.insert(
      "Delete",
      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback) {
        delete_waypoint_name_ = feedback->marker_name;
        delete_waypoint_pending_ = true;
      });
    charger_menu_.insert(
      "Delete",
      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback) {
        delete_charger_name_ = feedback->marker_name;
        delete_charger_pending_ = true;
      });

    reload_from_file();
    update_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { on_timer(); });
  }

private:
  void create_services()
  {
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
    srv_save_ = create_service<srv::SaveWaypoints>(
      "waterplus/save_waypoints",
      [this](const std::shared_ptr<srv::SaveWaypoints::Request> request,
      std::shared_ptr<srv::SaveWaypoints::Response> response) {
        (void)response;
        save_waypoints(request->filename, waypoints_, chargers_, &waypoint_actions_);
      });
    srv_reload_ = create_service<std_srvs::srv::Empty>(
      "waterplus/reload_waypoints",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        (void)request;
        (void)response;
        reload_from_file();
      });
    srv_get_names_ = create_service<srv::GetWaypointNames>(
      "waterplus/get_waypoint_names",
      [this](const std::shared_ptr<srv::GetWaypointNames::Request> request,
      std::shared_ptr<srv::GetWaypointNames::Response> response) {
        (void)request;
        response->active_file = load_file_;
        for (const auto & waypoint : waypoints_) {
          response->names.push_back(waypoint.name);
        }
      });
    srv_rename_ = create_service<srv::RenameWaypoint>(
      "waterplus/rename_waypoint",
      [this](const std::shared_ptr<srv::RenameWaypoint::Request> request,
      std::shared_ptr<srv::RenameWaypoint::Response> response) {
        rename_waypoint(request->old_name, request->new_name, *response);
      });
    srv_save_group_ = create_service<srv::WaypointFile>(
      "waterplus/save_waypoint_group",
      [this](const std::shared_ptr<srv::WaypointFile::Request> request,
      std::shared_ptr<srv::WaypointFile::Response> response) {
        if (request->filename.empty()) {
          response->success = false;
          response->message = "航点组文件名不能为空";
          return;
        }
        response->success = save_waypoints(
          request->filename, waypoints_, chargers_, &waypoint_actions_);
        if (response->success) {
          load_file_ = request->filename;
          set_parameter(rclcpp::Parameter("load", load_file_));
          response->message = "航点组已保存并设为当前组：" + load_file_;
        } else {
          response->message = "无法保存航点组：" + request->filename;
        }
      });
    srv_save_active_group_ = create_service<std_srvs::srv::Trigger>(
      "waterplus/save_active_waypoint_group",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        response->success = save_waypoints(
          load_file_, waypoints_, chargers_, &waypoint_actions_);
        response->message = response->success ?
          "当前航点组已保存：" + load_file_ : "无法保存当前航点组：" + load_file_;
      });
    srv_load_group_ = create_service<srv::WaypointFile>(
      "waterplus/load_waypoint_group",
      [this](const std::shared_ptr<srv::WaypointFile::Request> request,
      std::shared_ptr<srv::WaypointFile::Response> response) {
        std::string error;
        response->success = reload_from_file(request->filename, &error);
        response->message = response->success ?
          "已打开航点组：" + load_file_ : error;
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
    srv_get_charger_names_ = create_service<srv::GetWaypointNames>(
      "waterplus/get_charger_names",
      [this](const std::shared_ptr<srv::GetWaypointNames::Request> request,
      std::shared_ptr<srv::GetWaypointNames::Response> response) {
        (void)request;
        response->active_file = load_file_;
        for (const auto & charger : chargers_) {
          response->names.push_back(charger.name);
        }
      });
  }

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

  int next_id(const std::vector<msg::Waypoint> & items) const
  {
    int max_id = 0;
    for (const auto & item : items) {
      std::string digits;
      for (auto it = item.name.rbegin(); it != item.name.rend(); ++it) {
        if (std::isdigit(static_cast<unsigned char>(*it))) {
          digits.insert(digits.begin(), *it);
        } else if (!digits.empty()) {
          break;
        }
      }
      if (!digits.empty()) {
        max_id = std::max(max_id, std::stoi(digits));
      }
    }
    return max_id + 1;
  }

  static std::string trim_name(const std::string & name)
  {
    const auto first = std::find_if_not(name.begin(), name.end(), [](unsigned char character) {
      return std::isspace(character);
    });
    if (first == name.end()) {
      return "";
    }
    const auto last = std::find_if_not(name.rbegin(), name.rend(), [](unsigned char character) {
      return std::isspace(character);
    }).base();
    return std::string(first, last);
  }

  bool name_exists(const std::string & name) const
  {
    const auto has_name = [&name](const auto & item) {return item.name == name;};
    return std::any_of(waypoints_.begin(), waypoints_.end(), has_name) ||
           std::any_of(chargers_.begin(), chargers_.end(), has_name);
  }

  bool reload_from_file(const std::string & requested_file = "", std::string * error = nullptr)
  {
    if (!server_) {
      if (error) {
        *error = "航点交互标记服务器尚未就绪";
      }
      return false;
    }
    const std::string filename = requested_file.empty() ? load_file_ : requested_file;
    std::vector<msg::Waypoint> loaded_waypoints;
    std::vector<msg::Waypoint> loaded_chargers;
    std::map<std::string, std::string> loaded_actions;
    if (!load_waypoints(filename, loaded_waypoints, loaded_chargers, &loaded_actions)) {
      const std::string message = "无法打开航点组：" + filename;
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
      if (error) {
        *error = message;
      }
      return false;
    }
    std::vector<std::string> loaded_names;
    for (const auto & waypoint : loaded_waypoints) {
      const std::string name = trim_name(waypoint.name);
      if (name.empty() || std::find(loaded_names.begin(), loaded_names.end(), name) !=
        loaded_names.end())
      {
        const std::string message = "航点组包含空名称或重复名称：" + filename;
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
        if (error) {
          *error = message;
        }
        return false;
      }
      loaded_names.push_back(name);
    }

    server_->clear();
    waypoints_ = std::move(loaded_waypoints);
    chargers_ = std::move(loaded_chargers);
    waypoint_actions_ = std::move(loaded_actions);
    finished_.clear();
    load_file_ = filename;
    set_parameter(rclcpp::Parameter("load", load_file_));
    clear_text_markers();

    for (const auto & waypoint : waypoints_) {
      new_waypoint_marker(waypoint.name, waypoint.pose);
      waypoint_menu_.apply(*server_, waypoint.name);
    }
    for (const auto & charger : chargers_) {
      new_charger_marker(charger.name, charger.pose);
      charger_menu_.apply(*server_, charger.name);
    }
    server_->applyChanges();
    RCLCPP_INFO(get_logger(), "Loaded waypoint group: %s", load_file_.c_str());
    return true;
  }

  void rename_waypoint(
    const std::string & old_name_input,
    const std::string & new_name_input,
    srv::RenameWaypoint::Response & response)
  {
    const std::string old_name = trim_name(old_name_input);
    const std::string new_name = trim_name(new_name_input);
    if (old_name.empty() || new_name.empty()) {
      response.success = false;
      response.message = "原名称和新名称都不能为空";
      return;
    }
    if (old_name != new_name && name_exists(new_name)) {
      response.success = false;
      response.message = "名称已存在：" + new_name;
      return;
    }
    auto found = std::find_if(waypoints_.begin(), waypoints_.end(), [&old_name](const auto & item) {
      return item.name == old_name;
    });
    if (found == waypoints_.end()) {
      response.success = false;
      response.message = "找不到航点：" + old_name;
      return;
    }
    if (old_name == new_name) {
      response.success = true;
      response.message = "航点名称未改变";
      return;
    }

    const auto pose = found->pose;
    found->name = new_name;
    waypoint_actions_[new_name] = waypoint_actions_[old_name];
    waypoint_actions_.erase(old_name);
    if (finished_.erase(old_name) > 0) {
      finished_[new_name] = true;
    }
    server_->erase(old_name);
    new_waypoint_marker(new_name, pose);
    waypoint_menu_.apply(*server_, new_name);
    clear_text_markers();
    server_->applyChanges();
    response.success = true;
    response.message = "航点已重命名：" + old_name + " -> " + new_name;
    RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
  }

  void add_waypoint(const msg::Waypoint & input)
  {
    msg::Waypoint waypoint = input;
    waypoint.name = trim_name(input.name);
    if (waypoint.name.empty()) {
      std::ostringstream generated_name;
      generated_name << std::setw(2) << std::setfill('0') << next_id(waypoints_);
      waypoint.name = generated_name.str();
    }
    if (name_exists(waypoint.name)) {
      RCLCPP_WARN(
        get_logger(), "Waypoint name '%s' already exists; waypoint was not added",
        waypoint.name.c_str());
      return;
    }
    waypoint_actions_[waypoint.name] = "none";
    waypoints_.push_back(waypoint);

    new_waypoint_marker(waypoint.name, waypoint.pose);
    waypoint_menu_.apply(*server_, waypoint.name);
    server_->applyChanges();
    RCLCPP_INFO(get_logger(), "Added waypoint '%s'", waypoint.name.c_str());
  }

  void add_charger(const msg::Waypoint & input)
  {
    msg::Waypoint charger = input;
    charger.name = trim_name(input.name);
    if (charger.name.empty()) {
      charger.name = "charger_" + std::to_string(next_id(chargers_));
    }
    if (name_exists(charger.name)) {
      RCLCPP_WARN(
        get_logger(), "Charger name '%s' already exists; charger was not added",
        charger.name.c_str());
      return;
    }
    chargers_.push_back(charger);

    new_charger_marker(charger.name, charger.pose);
    charger_menu_.apply(*server_, charger.name);
    server_->applyChanges();
  }

  void waypoint_reached(const std::string & name)
  {
    finished_[name] = true;
    visualization_msgs::msg::InteractiveMarker marker;
    if (server_->get(name, marker)) {
      new_waypoint_marker(name, marker.pose);
      waypoint_menu_.apply(*server_, name);
      server_->applyChanges();
    }
  }

  visualization_msgs::msg::Marker make_mesh(const std::string & mesh) const
  {
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = mesh;
    marker.mesh_use_embedded_materials = true;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker;
  }

  void new_waypoint_marker(const std::string & name, const geometry_msgs::msg::Pose & pose)
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.name = name;
    marker.pose = pose;
    marker.scale = 0.45;

    std::string mesh = "package://robotcar_navigation/meshes/waypoint.dae";
    if (finished_[name]) {
      mesh = "package://robotcar_navigation/meshes/waypoint_finish.dae";
    } else if (waypoint_actions_[name] == "detect") {
      mesh = "package://robotcar_navigation/meshes/waypoint_detect.dae";
    }

    visualization_msgs::msg::InteractiveMarkerControl display_control;
    display_control.always_visible = true;
    display_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MENU;
    display_control.markers.push_back(make_mesh(mesh));
    marker.controls.push_back(display_control);

    visualization_msgs::msg::InteractiveMarkerControl move_control;
    move_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    move_control.orientation.w = 1.0;
    move_control.orientation.x = 1.0;
    marker.controls.push_back(move_control);

    move_control.orientation.x = 0.0;
    move_control.orientation.z = 1.0;
    marker.controls.push_back(move_control);

    move_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    move_control.orientation.y = 1.0;
    move_control.orientation.z = 0.0;
    marker.controls.push_back(move_control);

    server_->insert(
      marker,
      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
        for (auto & waypoint : waypoints_) {
          if (waypoint.name == feedback->marker_name) {
            waypoint.pose = feedback->pose;
            break;
          }
        }
      });
  }

  void new_charger_marker(const std::string & name, const geometry_msgs::msg::Pose & pose)
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.name = name;
    marker.pose = pose;
    marker.scale = 0.45;

    visualization_msgs::msg::InteractiveMarkerControl display_control;
    display_control.always_visible = true;
    display_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MENU;
    display_control.markers.push_back(make_mesh("package://robotcar_navigation/meshes/charger.dae"));
    marker.controls.push_back(display_control);

    server_->insert(
      marker,
      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
        for (auto & charger : chargers_) {
          if (charger.name == feedback->marker_name) {
            charger.pose = feedback->pose;
            break;
          }
        }
      });
  }

  void publish_text_marker(
    int32_t id,
    const std::string & text,
    double x,
    double y,
    double z,
    float r,
    float g,
    float b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "text";
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    marker.pose.orientation = tf2::toMsg(q);
    marker.text = text;
    text_pub_->publish(marker);
  }

  void clear_text_markers()
  {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    text_pub_->publish(marker);
  }

  void publish_text()
  {
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto & waypoint = waypoints_[i];
      float r = 1.0;
      float g = 1.0;
      float b = 1.0;
      if (finished_[waypoint.name]) {
        r = 0.0;
        g = 1.0;
        b = 0.0;
      } else if (waypoint_actions_[waypoint.name] == "detect") {
        r = 1.0;
        g = 1.0;
        b = 0.0;
      }
      publish_text_marker(
        static_cast<int32_t>(i), waypoint.name,
        waypoint.pose.position.x, waypoint.pose.position.y, 0.55, r, g, b);
    }

    for (size_t i = 0; i < chargers_.size(); ++i) {
      const auto & charger = chargers_[i];
      publish_text_marker(
        static_cast<int32_t>(waypoints_.size() + i), charger.name,
        charger.pose.position.x, charger.pose.position.y, 0.35, 0.0, 1.0, 1.0);
    }
  }

  void on_timer()
  {
    if (delete_waypoint_pending_) {
      server_->erase(delete_waypoint_name_);
      waypoints_.erase(
        std::remove_if(waypoints_.begin(), waypoints_.end(), [this](const auto & waypoint) {
          return waypoint.name == delete_waypoint_name_;
        }),
        waypoints_.end());
      waypoint_actions_.erase(delete_waypoint_name_);
      finished_.erase(delete_waypoint_name_);
      delete_waypoint_pending_ = false;
      clear_text_markers();
      server_->applyChanges();
    }

    if (delete_charger_pending_) {
      server_->erase(delete_charger_name_);
      chargers_.erase(
        std::remove_if(chargers_.begin(), chargers_.end(), [this](const auto & charger) {
          return charger.name == delete_charger_name_;
        }),
        chargers_.end());
      delete_charger_pending_ = false;
      clear_text_markers();
      server_->applyChanges();
    }

    publish_text();
  }

  std::string load_file_;
  std::vector<msg::Waypoint> waypoints_;
  std::vector<msg::Waypoint> chargers_;
  std::map<std::string, std::string> waypoint_actions_;
  std::map<std::string, bool> finished_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler waypoint_menu_;
  interactive_markers::MenuHandler charger_menu_;

  bool delete_waypoint_pending_ = false;
  bool delete_charger_pending_ = false;
  std::string delete_waypoint_name_;
  std::string delete_charger_name_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_pub_;
  rclcpp::Subscription<msg::Waypoint>::SharedPtr add_waypoint_sub_;
  rclcpp::Subscription<msg::Waypoint>::SharedPtr add_charger_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reached_sub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  rclcpp::Service<srv::GetNumOfWaypoints>::SharedPtr srv_get_num_wp_;
  rclcpp::Service<srv::GetWaypointByIndex>::SharedPtr srv_get_wp_index_;
  rclcpp::Service<srv::GetWaypointByName>::SharedPtr srv_get_wp_name_;
  rclcpp::Service<srv::SaveWaypoints>::SharedPtr srv_save_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reload_;
  rclcpp::Service<srv::GetWaypointNames>::SharedPtr srv_get_names_;
  rclcpp::Service<srv::RenameWaypoint>::SharedPtr srv_rename_;
  rclcpp::Service<srv::WaypointFile>::SharedPtr srv_save_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_active_group_;
  rclcpp::Service<srv::WaypointFile>::SharedPtr srv_load_group_;
  rclcpp::Service<srv::GetNumOfWaypoints>::SharedPtr srv_get_num_charger_;
  rclcpp::Service<srv::GetWaypointByIndex>::SharedPtr srv_get_charger_index_;
  rclcpp::Service<srv::GetChargerByName>::SharedPtr srv_get_charger_name_;
  rclcpp::Service<srv::GetWaypointNames>::SharedPtr srv_get_charger_names_;
};

}  // namespace robotcar_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotcar_navigation::WaypointEditNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
