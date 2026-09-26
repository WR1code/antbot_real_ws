#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "antbot_mid360_fusion/pointcloud_utils.hpp"

namespace
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

PointCloud2 make_cloud()
{
  PointCloud2 cloud;
  cloud.header.frame_id = "lidar_test";
  cloud.height = 1;
  cloud.width = 5;
  cloud.fields = {
    PointField().set__name("x").set__offset(0).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("y").set__offset(4).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("z").set__offset(8).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("intensity").set__offset(12).set__datatype(PointField::FLOAT32).set__count(1),
    PointField().set__name("offset_time").set__offset(16).set__datatype(PointField::UINT32).set__count(1),
    PointField().set__name("tag").set__offset(20).set__datatype(PointField::UINT8).set__count(1),
    PointField().set__name("line").set__offset(21).set__datatype(PointField::UINT8).set__count(1),
  };
  cloud.point_step = 24;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  cloud.is_dense = false;
  const float xyz[5][3] = {
    {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {-1.0F, 0.0F, 0.0F},
    {0.0F, -1.0F, 0.0F}, {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F}};
  for (std::size_t index = 0; index < 5; ++index) {
    auto * point = cloud.data.data() + index * cloud.point_step;
    std::memcpy(point, xyz[index], sizeof(xyz[index]));
    const float intensity = static_cast<float>(10 + index);
    const std::uint32_t time = static_cast<std::uint32_t>(1000 + index);
    std::memcpy(point + 12, &intensity, sizeof(intensity));
    std::memcpy(point + 16, &time, sizeof(time));
    point[20] = static_cast<std::uint8_t>(20 + index);
    point[21] = static_cast<std::uint8_t>(30 + index);
  }
  return cloud;
}

float read_float(const PointCloud2 & cloud, std::size_t point, std::size_t offset)
{
  float value;
  std::memcpy(&value, cloud.data.data() + point * cloud.point_step + offset, sizeof(value));
  return value;
}

TEST(Angles, HandlesWrapAround)
{
  constexpr double pi = 3.14159265358979323846;
  EXPECT_TRUE(antbot_mid360_fusion::angle_in_sector(-179.0 * pi / 180.0, 179.0 * pi / 180.0, 10.0 * pi / 180.0));
  EXPECT_FALSE(antbot_mid360_fusion::angle_in_sector(0.0, pi, pi / 2.0));
}

TEST(BlindDirection, UsesTranslationAndInverseSensorYaw)
{
  constexpr double pi = 3.14159265358979323846;
  geometry_msgs::msg::Transform front;
  front.translation.x = 1.0;
  front.rotation.w = 1.0;
  EXPECT_NEAR(antbot_mid360_fusion::blind_center_from_transform(front), pi, 1.0e-9);

  geometry_msgs::msg::Transform rear;
  rear.translation.x = -1.0;
  rear.rotation.z = 1.0;
  rear.rotation.w = 0.0;
  EXPECT_NEAR(std::abs(antbot_mid360_fusion::blind_center_from_transform(rear)), pi, 1.0e-9);
}

TEST(Crop, PreservesAllFieldsAndOriginalBytes)
{
  auto cloud = make_cloud();
  const auto result = antbot_mid360_fusion::crop_cloud(
    cloud, 0.0, 3.14159265358979323846 / 2.0, 0.1, 10.0);
  ASSERT_EQ(result.input_points, 5U);
  ASSERT_EQ(result.invalid_points, 1U);
  ASSERT_EQ(result.blind_rejected_points, 1U);
  ASSERT_EQ(result.rejected.width, 1U);
  ASSERT_EQ(result.kept.width, 3U);
  EXPECT_EQ(result.kept.fields, cloud.fields);
  EXPECT_EQ(result.rejected.fields, cloud.fields);
  EXPECT_EQ(result.rejected.data[20], 20U);
  EXPECT_FLOAT_EQ(read_float(result.rejected, 0, 12), 10.0F);
}

TEST(Transform, ChangesOnlyXyz)
{
  auto result = antbot_mid360_fusion::crop_cloud(
    make_cloud(), 0.0, 0.1, 0.1, 10.0);
  ASSERT_GT(result.kept.width, 0U);
  const auto before = result.kept.data;
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 2.0;
  transform.rotation.w = 1.0;
  antbot_mid360_fusion::transform_xyz_in_place(result.kept, transform);
  EXPECT_FLOAT_EQ(read_float(result.kept, 0, 0), 2.0F);
  for (std::size_t point = 0; point < result.kept.width; ++point) {
    EXPECT_EQ(result.kept.data[point * result.kept.point_step + 20],
      before[point * result.kept.point_step + 20]);
    EXPECT_EQ(result.kept.data[point * result.kept.point_step + 21],
      before[point * result.kept.point_step + 21]);
  }
}

TEST(Merge, RejectsDifferentSchemas)
{
  auto first = make_cloud();
  auto second = make_cloud();
  second.fields.back().name = "ring";
  EXPECT_THROW(
    antbot_mid360_fusion::merge_clouds(first, second, "base_link"), std::invalid_argument);
}

}  // namespace
