cmake_minimum_required(VERSION 3.16)
project(antbot_mid360_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(pointcloud_utils src/pointcloud_utils.cpp)
target_include_directories(pointcloud_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pointcloud_utils geometry_msgs sensor_msgs tf2)

add_executable(dual_mid360_fusion_node src/dual_mid360_fusion_node.cpp)
target_link_libraries(dual_mid360_fusion_node pointcloud_utils)
ament_target_dependencies(dual_mid360_fusion_node
  geometry_msgs rclcpp sensor_msgs tf2 tf2_ros)

install(TARGETS pointcloud_utils dual_mid360_fusion_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(PROGRAMS scripts/benchmark_dual_mid360.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pointcloud_utils test/test_pointcloud_utils.cpp)
  target_link_libraries(test_pointcloud_utils pointcloud_utils)
  ament_target_dependencies(test_pointcloud_utils geometry_msgs sensor_msgs tf2)
endif()

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs sensor_msgs tf2)
ament_package()
