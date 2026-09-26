cmake_minimum_required(VERSION 3.16)
project(piperh_motion_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rebotarm_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME}_plugin SHARED
  include/piperh_motion_rviz/motion_preset_panel.hpp
  src/motion_preset_panel.cpp
)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_plugin Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME}_plugin
  moveit_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rebotarm_msgs
  rviz_common
  sensor_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(rviz_common piperh_motion_rviz_plugin.xml)

install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES piperh_motion_rviz_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(pluginlib rclcpp rviz_common)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_package()
