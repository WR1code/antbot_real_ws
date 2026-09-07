cmake_minimum_required(VERSION 3.16)
project(robotcar_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/KeepoutZone.msg
  msg/SpeedZone.msg
  msg/Waypoint.msg
  srv/AddNewWaypoint.srv
  srv/GetChargerByName.srv
  srv/GetNumOfWaypoints.srv
  srv/GetWaypointByIndex.srv
  srv/GetWaypointByName.srv
  srv/GetWaypointNames.srv
  srv/RenameWaypoint.srv
  srv/SaveWaypoints.srv
  srv/WaypointFile.srv
  DEPENDENCIES geometry_msgs std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)

set(common_deps
  ament_index_cpp
  geometry_msgs
  interactive_markers
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

set(rviz_plugin_target ${PROJECT_NAME}_rviz_plugins)

# RViz Jazzy's MovableText iterates UTF-8 strings byte-by-byte and its default
# Liberation Sans atlas has no CJK glyphs.  This small, RViz-only preload
# library fixes both issues without replacing the system ROS installation.
add_library(robotcar_rviz_unicode SHARED
  src/rviz_unicode_preload.cpp
)
target_link_libraries(robotcar_rviz_unicode
  ${CMAKE_DL_LIBS}
)
ament_target_dependencies(robotcar_rviz_unicode
  rviz_rendering
)

add_library(${rviz_plugin_target} SHARED
  include/add_charger_tool.h
  include/add_keepout_zone_tool.h
  include/add_speed_zone_tool.h
  include/add_waypoint_tool.h
  include/vehicle_status_panel.h
  include/waypoint_manager_panel.h
  src/add_charger_tool.cpp
  src/add_keepout_zone_tool.cpp
  src/add_speed_zone_tool.cpp
  src/add_waypoint_tool.cpp
  src/vehicle_status_panel.cpp
  src/waypoint_manager_panel.cpp
)
target_link_libraries(${rviz_plugin_target}
  Qt5::Widgets
  ${cpp_typesupport_target}
)
ament_target_dependencies(${rviz_plugin_target}
  action_msgs
  geometry_msgs
  nav2_msgs
  pluginlib
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)
pluginlib_export_plugin_description_file(rviz_common robotcar_navigation_plugin.xml)

function(add_robotcar_node target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target}
    tinyxml2::tinyxml2
    Threads::Threads
    ${cpp_typesupport_target}
  )
  ament_target_dependencies(${target} ${common_deps})
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_robotcar_node(charger_get_position src/charger_get_position.cpp)
add_robotcar_node(demo_map_tool src/demo_map_tool.cpp)
add_robotcar_node(floor_texture_publisher src/floor_texture_publisher.cpp)
add_robotcar_node(pose_navi_server src/pose_navi_server.cpp)
add_robotcar_node(set_pose_from_waypoint src/set_pose_from_waypoint.cpp)
add_robotcar_node(wp_edit_node src/wp_edit_node.cpp)
add_robotcar_node(wp_manager src/wp_manager.cpp)
add_robotcar_node(wp_nav_odom_report src/wp_nav_odom_report.cpp src/network/UDPClient.c)
add_robotcar_node(wp_nav_remote src/wp_nav_remote.cpp src/network/UDPServer.c)
add_robotcar_node(wp_nav_test src/wp_nav_test.cpp)
add_robotcar_node(wp_navi_server src/wp_navi_server.cpp)
add_robotcar_node(wp_saver src/wp_saver.cpp)
add_robotcar_node(wp_set_pose src/wp_set_pose.cpp)
add_robotcar_node(wr_manager src/wr_manager.cpp)

install(TARGETS ${rviz_plugin_target} robotcar_rviz_unicode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/keepout_zone_manager.py
  scripts/manage_detect_action.py
  scripts/pure_pursuit_planner.py
  scripts/robot_intent_monitor.py
  scripts/robot_awareness_monitor.py
  scripts/speed_zone_manager.py
  scripts/wr_navigate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  icons
  launch
  maps
  meshes
  rviz
  textures
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  robotcar_navigation_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  geometry_msgs
  pluginlib
  rclcpp
  rosidl_default_runtime
  rviz_common
  rviz_default_plugins
)
ament_export_libraries(${rviz_plugin_target})

ament_package()
