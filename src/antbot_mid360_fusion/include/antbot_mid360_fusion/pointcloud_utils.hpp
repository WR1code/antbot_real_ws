#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace antbot_mid360_fusion
{

struct CropResult
{
  sensor_msgs::msg::PointCloud2 kept;
  sensor_msgs::msg::PointCloud2 rejected;
  std::size_t input_points{0};
  std::size_t invalid_points{0};
  std::size_t range_rejected_points{0};
  std::size_t blind_rejected_points{0};
};

double normalize_angle(double angle);
bool angle_in_sector(double angle, double center, double width);

double blind_center_from_transform(
  const geometry_msgs::msg::Transform & source_to_base,
  double center_x = 0.0,
  double center_y = 0.0,
  double center_z = 0.0);

CropResult crop_cloud(
  const sensor_msgs::msg::PointCloud2 & input,
  double blind_center_rad,
  double blind_width_rad,
  double min_range,
  double max_range,
  bool collect_rejected = true);

void transform_xyz_in_place(
  sensor_msgs::msg::PointCloud2 & cloud,
  const geometry_msgs::msg::Transform & transform);

bool layouts_match(
  const sensor_msgs::msg::PointCloud2 & first,
  const sensor_msgs::msg::PointCloud2 & second,
  std::string * reason = nullptr);

sensor_msgs::msg::PointCloud2 merge_clouds(
  const sensor_msgs::msg::PointCloud2 & first,
  const sensor_msgs::msg::PointCloud2 & second,
  const std::string & target_frame);

}  // namespace antbot_mid360_fusion
