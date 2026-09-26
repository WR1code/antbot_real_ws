cmake_minimum_required(VERSION 3.16)
project(rebotarm_demo_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rebotarm_msgs REQUIRED)
find_package(rebot_teach_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME}_plugin SHARED
  include/rebotarm_demo_rviz/demo_panel.hpp
  include/rebotarm_demo_rviz/dual_arm_selector_panel.hpp
  src/demo_panel.cpp
  src/dual_arm_selector_panel.cpp
  src/motion_planning_i18n.cpp
)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_plugin Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME}_plugin
  action_msgs
  ament_index_cpp
  geometry_msgs
  moveit_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rebotarm_msgs
  rebot_teach_msgs
  rviz_common
  sensor_msgs
  std_msgs
  std_srvs
)

pluginlib_export_plugin_description_file(rviz_common rebotarm_demo_rviz_plugin.xml)

install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES rebotarm_demo_rviz_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(action_msgs geometry_msgs pluginlib rclcpp rviz_common std_msgs)
ament_export_libraries(${PROJECT_NAME}_plugin)

if(BUILD_TESTING)
  add_executable(test_motion_planning_i18n test/test_motion_planning_i18n.cpp)
  target_link_libraries(test_motion_planning_i18n
    ${PROJECT_NAME}_plugin
    Qt5::Widgets
  )
  add_test(NAME test_motion_planning_i18n COMMAND test_motion_planning_i18n)
  set_tests_properties(test_motion_planning_i18n PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
  )
endif()

ament_package()
