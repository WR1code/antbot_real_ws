#ifndef ROBOTCAR_NAVIGATION__WAYPOINT_XML_HPP_
#define ROBOTCAR_NAVIGATION__WAYPOINT_XML_HPP_

#include <cmath>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "robotcar_navigation/msg/waypoint.hpp"
#include "tinyxml2.h"

namespace robotcar_navigation
{

inline std::string to_string(double value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

inline void normalize_orientation(geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm > 0.0) {
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  } else {
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
  }
}

inline const char * child_text(tinyxml2::XMLElement * parent, const char * name)
{
  auto * child = parent ? parent->FirstChildElement(name) : nullptr;
  return child ? child->GetText() : nullptr;
}

inline void read_legacy_pose(tinyxml2::XMLElement * elem, msg::Waypoint & waypoint)
{
  if (const char * text = child_text(elem, "Name")) {
    waypoint.name = text;
  }
  if (const char * text = child_text(elem, "Pos_x")) {
    waypoint.pose.position.x = std::atof(text);
  }
  if (const char * text = child_text(elem, "Pos_y")) {
    waypoint.pose.position.y = std::atof(text);
  }
  if (const char * text = child_text(elem, "Pos_z")) {
    waypoint.pose.position.z = std::atof(text);
  }
  if (const char * text = child_text(elem, "Ori_x")) {
    waypoint.pose.orientation.x = std::atof(text);
  }
  if (const char * text = child_text(elem, "Ori_y")) {
    waypoint.pose.orientation.y = std::atof(text);
  }
  if (const char * text = child_text(elem, "Ori_z")) {
    waypoint.pose.orientation.z = std::atof(text);
  }
  if (const char * text = child_text(elem, "Ori_w")) {
    waypoint.pose.orientation.w = std::atof(text);
  }
  normalize_orientation(waypoint.pose.orientation);
}

inline void read_modern_pose(
  tinyxml2::XMLElement * elem,
  msg::Waypoint & waypoint,
  std::map<std::string, std::string> * actions = nullptr)
{
  const char * name = elem->Attribute("name");
  waypoint.name = name ? name : "waypoint";

  if (auto * pose = elem->FirstChildElement("pose")) {
    if (auto * position = pose->FirstChildElement("position")) {
      position->QueryDoubleAttribute("x", &waypoint.pose.position.x);
      position->QueryDoubleAttribute("y", &waypoint.pose.position.y);
      position->QueryDoubleAttribute("z", &waypoint.pose.position.z);
    }
    if (auto * orientation = pose->FirstChildElement("orientation")) {
      orientation->QueryDoubleAttribute("x", &waypoint.pose.orientation.x);
      orientation->QueryDoubleAttribute("y", &waypoint.pose.orientation.y);
      orientation->QueryDoubleAttribute("z", &waypoint.pose.orientation.z);
      orientation->QueryDoubleAttribute("w", &waypoint.pose.orientation.w);
      normalize_orientation(waypoint.pose.orientation);
    }
  }

  if (actions) {
    auto * action = elem->FirstChildElement("action");
    const char * action_text = action ? action->GetText() : nullptr;
    (*actions)[waypoint.name] = action_text ? action_text : "none";
  }
}

inline bool load_waypoints(
  const std::string & filename,
  std::vector<msg::Waypoint> & waypoints,
  std::vector<msg::Waypoint> & chargers,
  std::map<std::string, std::string> * actions = nullptr)
{
  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    return false;
  }

  auto * root = document.RootElement();
  if (!root) {
    return false;
  }

  int generated_id = 1;
  for (auto * item = root->FirstChildElement("waypoint"); item;
    item = item->NextSiblingElement("waypoint"))
  {
    msg::Waypoint waypoint;
    read_modern_pose(item, waypoint, actions);
    if (waypoint.name.empty() || waypoint.name == "waypoint") {
      waypoint.name = "wp_" + std::to_string(generated_id++);
    }
    waypoints.push_back(waypoint);
  }

  for (auto * item = root->FirstChildElement("charger"); item;
    item = item->NextSiblingElement("charger"))
  {
    msg::Waypoint charger;
    read_modern_pose(item, charger, nullptr);
    chargers.push_back(charger);
  }

  for (auto * item = root->FirstChildElement("Waypoint"); item;
    item = item->NextSiblingElement("Waypoint"))
  {
    msg::Waypoint waypoint;
    read_legacy_pose(item, waypoint);
    waypoints.push_back(waypoint);
  }

  for (auto * item = root->FirstChildElement("Charger"); item;
    item = item->NextSiblingElement("Charger"))
  {
    msg::Waypoint charger;
    read_legacy_pose(item, charger);
    chargers.push_back(charger);
  }

  return true;
}

inline void write_pose(tinyxml2::XMLDocument & document, tinyxml2::XMLElement * parent,
  const geometry_msgs::msg::Pose & pose)
{
  auto * pose_elem = document.NewElement("pose");
  auto * position = document.NewElement("position");
  position->SetAttribute("x", to_string(pose.position.x).c_str());
  position->SetAttribute("y", to_string(pose.position.y).c_str());
  position->SetAttribute("z", to_string(pose.position.z).c_str());

  auto * orientation = document.NewElement("orientation");
  orientation->SetAttribute("x", to_string(pose.orientation.x).c_str());
  orientation->SetAttribute("y", to_string(pose.orientation.y).c_str());
  orientation->SetAttribute("z", to_string(pose.orientation.z).c_str());
  orientation->SetAttribute("w", to_string(pose.orientation.w).c_str());

  pose_elem->InsertEndChild(position);
  pose_elem->InsertEndChild(orientation);
  parent->InsertEndChild(pose_elem);
}

inline bool save_waypoints(
  const std::string & filename,
  const std::vector<msg::Waypoint> & waypoints,
  const std::vector<msg::Waypoint> & chargers,
  const std::map<std::string, std::string> * actions = nullptr)
{
  tinyxml2::XMLDocument document;
  document.InsertEndChild(document.NewDeclaration());
  auto * root = document.NewElement("Waterplus");
  document.InsertEndChild(root);

  for (const auto & waypoint : waypoints) {
    auto * elem = document.NewElement("waypoint");
    elem->SetAttribute("name", waypoint.name.c_str());
    write_pose(document, elem, waypoint.pose);

    auto * action = document.NewElement("action");
    std::string action_text = "none";
    if (actions) {
      auto found = actions->find(waypoint.name);
      if (found != actions->end()) {
        action_text = found->second;
      }
    }
    action->SetText(action_text.c_str());
    elem->InsertEndChild(action);
    root->InsertEndChild(elem);
  }

  for (const auto & charger : chargers) {
    auto * elem = document.NewElement("charger");
    elem->SetAttribute("name", charger.name.c_str());
    write_pose(document, elem, charger.pose);
    root->InsertEndChild(elem);
  }

  return document.SaveFile(filename.c_str()) == tinyxml2::XML_SUCCESS;
}

}  // namespace robotcar_navigation

#endif  // ROBOTCAR_NAVIGATION__WAYPOINT_XML_HPP_
