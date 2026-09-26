#include "antbot_mid360_fusion/pointcloud_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace antbot_mid360_fusion
{
namespace
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

const PointField & find_field(const PointCloud2 & cloud, const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const PointField & field) {return field.name == name;});
  if (it == cloud.fields.end()) {
    throw std::invalid_argument("PointCloud2 is missing required field '" + name + "'");
  }
  if (it->count != 1 ||
    (it->datatype != PointField::FLOAT32 && it->datatype != PointField::FLOAT64))
  {
    throw std::invalid_argument("field '" + name + "' must be scalar FLOAT32 or FLOAT64");
  }
  const std::size_t size = it->datatype == PointField::FLOAT32 ? sizeof(float) : sizeof(double);
  if (static_cast<std::size_t>(it->offset) + size > cloud.point_step) {
    throw std::invalid_argument("field '" + name + "' exceeds point_step");
  }
  return *it;
}

template<typename T>
T byte_swap(T value)
{
  static_assert(std::is_trivially_copyable<T>::value, "byte_swap requires POD data");
  T output{};
  const auto * source = reinterpret_cast<const std::uint8_t *>(&value);
  auto * destination = reinterpret_cast<std::uint8_t *>(&output);
  std::reverse_copy(source, source + sizeof(T), destination);
  return output;
}

bool host_is_big_endian()
{
  const std::uint16_t value = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01;
}

template<typename T>
T read_value(const std::uint8_t * data, bool data_big_endian)
{
  T value;
  std::memcpy(&value, data, sizeof(T));
  if (data_big_endian != host_is_big_endian()) {
    value = byte_swap(value);
  }
  return value;
}

template<typename T>
void write_value(std::uint8_t * data, T value, bool data_big_endian)
{
  if (data_big_endian != host_is_big_endian()) {
    value = byte_swap(value);
  }
  std::memcpy(data, &value, sizeof(T));
}

double read_coordinate(
  const std::uint8_t * point, const PointField & field, bool data_big_endian)
{
  if (field.datatype == PointField::FLOAT32) {
    return static_cast<double>(read_value<float>(point + field.offset, data_big_endian));
  }
  return read_value<double>(point + field.offset, data_big_endian);
}

void write_coordinate(
  std::uint8_t * point, const PointField & field, bool data_big_endian, double value)
{
  if (field.datatype == PointField::FLOAT32) {
    write_value<float>(point + field.offset, static_cast<float>(value), data_big_endian);
  } else {
    write_value<double>(point + field.offset, value, data_big_endian);
  }
}

void prepare_output(const PointCloud2 & input, PointCloud2 & output)
{
  output.header = input.header;
  output.height = 1;
  output.width = 0;
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.row_step = 0;
  output.is_dense = true;
  output.data.clear();
}

void append_point(PointCloud2 & output, const std::uint8_t * point)
{
  const auto old_size = output.data.size();
  output.data.resize(old_size + output.point_step);
  std::memcpy(output.data.data() + old_size, point, output.point_step);
  ++output.width;
}

bool fields_equal(const PointField & a, const PointField & b)
{
  return a.name == b.name && a.offset == b.offset && a.datatype == b.datatype &&
         a.count == b.count;
}

}  // namespace

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool angle_in_sector(double angle, double center, double width)
{
  if (!std::isfinite(angle) || !std::isfinite(center) || !std::isfinite(width) || width < 0.0) {
    return false;
  }
  if (width >= 2.0 * M_PI) {
    return true;
  }
  return std::abs(normalize_angle(angle - center)) <= width * 0.5;
}

double blind_center_from_transform(
  const geometry_msgs::msg::Transform & source_to_base,
  double center_x,
  double center_y,
  double center_z)
{
  const auto & t = source_to_base.translation;
  const tf2::Vector3 inward_base(center_x - t.x, center_y - t.y, center_z - t.z);
  tf2::Quaternion rotation(
    source_to_base.rotation.x, source_to_base.rotation.y,
    source_to_base.rotation.z, source_to_base.rotation.w);
  if (rotation.length2() < std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("TF rotation quaternion has zero length");
  }
  rotation.normalize();
  const tf2::Vector3 inward_local = tf2::quatRotate(rotation.inverse(), inward_base);
  if (std::hypot(inward_local.x(), inward_local.y()) < 1.0e-6) {
    throw std::invalid_argument("lidar XY position coincides with robot center");
  }
  return std::atan2(inward_local.y(), inward_local.x());
}

CropResult crop_cloud(
  const PointCloud2 & input,
  double blind_center_rad,
  double blind_width_rad,
  double min_range,
  double max_range,
  bool collect_rejected)
{
  if (input.point_step == 0 || input.row_step < input.width * input.point_step) {
    throw std::invalid_argument("invalid PointCloud2 point_step/row_step");
  }
  if (min_range < 0.0 || max_range <= min_range) {
    throw std::invalid_argument("range limits must satisfy 0 <= min_range < max_range");
  }
  const auto & x_field = find_field(input, "x");
  const auto & y_field = find_field(input, "y");
  const auto & z_field = find_field(input, "z");

  CropResult result;
  prepare_output(input, result.kept);
  prepare_output(input, result.rejected);
  result.input_points = static_cast<std::size_t>(input.width) * input.height;
  result.kept.data.reserve(result.input_points * input.point_step * 3 / 4);
  if (collect_rejected) {
    result.rejected.data.reserve(result.input_points * input.point_step / 4);
  }
  const double minimum_squared = min_range * min_range;
  const double maximum_squared = max_range * max_range;

  for (std::uint32_t row = 0; row < input.height; ++row) {
    for (std::uint32_t column = 0; column < input.width; ++column) {
      const std::size_t offset = static_cast<std::size_t>(row) * input.row_step +
        static_cast<std::size_t>(column) * input.point_step;
      if (offset + input.point_step > input.data.size()) {
        throw std::invalid_argument("PointCloud2 data is shorter than its dimensions declare");
      }
      const auto * point = input.data.data() + offset;
      const double x = read_coordinate(point, x_field, input.is_bigendian);
      const double y = read_coordinate(point, y_field, input.is_bigendian);
      const double z = read_coordinate(point, z_field, input.is_bigendian);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++result.invalid_points;
        continue;
      }
      const double range_squared = x * x + y * y + z * z;
      if (range_squared < minimum_squared || range_squared > maximum_squared) {
        ++result.range_rejected_points;
        continue;
      }
      if (angle_in_sector(std::atan2(y, x), blind_center_rad, blind_width_rad)) {
        ++result.blind_rejected_points;
        if (collect_rejected) {append_point(result.rejected, point);}
      } else {
        append_point(result.kept, point);
      }
    }
  }
  result.kept.row_step = result.kept.width * result.kept.point_step;
  result.rejected.row_step = result.rejected.width * result.rejected.point_step;
  return result;
}

void transform_xyz_in_place(PointCloud2 & cloud, const geometry_msgs::msg::Transform & transform)
{
  const auto & x_field = find_field(cloud, "x");
  const auto & y_field = find_field(cloud, "y");
  const auto & z_field = find_field(cloud, "z");
  tf2::Quaternion rotation(
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  if (rotation.length2() < std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("TF rotation quaternion has zero length");
  }
  rotation.normalize();
  const tf2::Vector3 translation(
    transform.translation.x, transform.translation.y, transform.translation.z);

  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t column = 0; column < cloud.width; ++column) {
      const std::size_t offset = static_cast<std::size_t>(row) * cloud.row_step +
        static_cast<std::size_t>(column) * cloud.point_step;
      if (offset + cloud.point_step > cloud.data.size()) {
        throw std::invalid_argument("PointCloud2 data is shorter than its dimensions declare");
      }
      auto * point = cloud.data.data() + offset;
      const tf2::Vector3 source(
        read_coordinate(point, x_field, cloud.is_bigendian),
        read_coordinate(point, y_field, cloud.is_bigendian),
        read_coordinate(point, z_field, cloud.is_bigendian));
      const tf2::Vector3 destination = tf2::quatRotate(rotation, source) + translation;
      write_coordinate(point, x_field, cloud.is_bigendian, destination.x());
      write_coordinate(point, y_field, cloud.is_bigendian, destination.y());
      write_coordinate(point, z_field, cloud.is_bigendian, destination.z());
    }
  }
}

bool layouts_match(const PointCloud2 & first, const PointCloud2 & second, std::string * reason)
{
  auto fail = [reason](const std::string & text) {
      if (reason) {*reason = text;}
      return false;
    };
  if (first.point_step != second.point_step) {return fail("point_step differs");}
  if (first.is_bigendian != second.is_bigendian) {return fail("endianness differs");}
  if (first.fields.size() != second.fields.size()) {return fail("field count differs");}
  for (std::size_t index = 0; index < first.fields.size(); ++index) {
    if (!fields_equal(first.fields[index], second.fields[index])) {
      return fail("field layout differs at index " + std::to_string(index));
    }
  }
  return true;
}

PointCloud2 merge_clouds(
  const PointCloud2 & first, const PointCloud2 & second, const std::string & target_frame)
{
  std::string reason;
  if (!layouts_match(first, second, &reason)) {
    throw std::invalid_argument("cannot merge clouds: " + reason);
  }
  if (first.data.size() != static_cast<std::size_t>(first.row_step) * first.height ||
    second.data.size() != static_cast<std::size_t>(second.row_step) * second.height)
  {
    throw std::invalid_argument("cannot merge clouds containing row padding or malformed data");
  }
  PointCloud2 output = first;
  output.header.frame_id = target_frame;
  const auto first_stamp = static_cast<std::int64_t>(first.header.stamp.sec) * 1000000000LL +
    first.header.stamp.nanosec;
  const auto second_stamp = static_cast<std::int64_t>(second.header.stamp.sec) * 1000000000LL +
    second.header.stamp.nanosec;
  if (second_stamp > first_stamp) {
    output.header.stamp = second.header.stamp;
  }
  output.height = 1;
  output.width = first.width * first.height + second.width * second.height;
  output.row_step = output.width * output.point_step;
  output.is_dense = first.is_dense && second.is_dense;
  output.data.clear();
  output.data.reserve(first.data.size() + second.data.size());
  output.data.insert(output.data.end(), first.data.begin(), first.data.end());
  output.data.insert(output.data.end(), second.data.begin(), second.data.end());
  return output;
}

}  // namespace antbot_mid360_fusion
