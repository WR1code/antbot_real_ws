#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "antbot_mid360_fusion/pointcloud_utils.hpp"

namespace antbot_mid360_fusion
{
namespace
{

using Cloud = sensor_msgs::msg::PointCloud2;
using SteadyClock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;

double elapsed_ms(const SteadyClock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

struct Distribution
{
  double average{0.0};
  double p50{0.0};
  double p95{0.0};
  double p99{0.0};
  double maximum{0.0};
};

Distribution summarize(const std::vector<double> & values)
{
  if (values.empty()) {return {};}
  std::vector<double> sorted(values);
  std::sort(sorted.begin(), sorted.end());
  const auto percentile = [&sorted](double fraction) {
      const auto index = static_cast<std::size_t>(
        std::ceil(fraction * static_cast<double>(sorted.size())) - 1.0);
      return sorted[std::min(index, sorted.size() - 1)];
    };
  Distribution result;
  result.average = std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size();
  result.p50 = percentile(0.50);
  result.p95 = percentile(0.95);
  result.p99 = percentile(0.99);
  result.maximum = sorted.back();
  return result;
}

std::string describe_fields(const Cloud & cloud)
{
  std::ostringstream output;
  for (std::size_t index = 0; index < cloud.fields.size(); ++index) {
    if (index != 0) {output << ", ";}
    const auto & field = cloud.fields[index];
    output << field.name << "(offset=" << field.offset << ",type=" <<
      static_cast<int>(field.datatype) << ",count=" << field.count << ")";
  }
  return output.str();
}

}  // namespace

class DualMid360FusionNode : public rclcpp::Node
{
public:
  DualMid360FusionNode()
  : Node("dual_mid360_fusion"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    processing_stage_ = declare_parameter<std::string>("processing_stage", "merge");
    auto_blind_direction_ = declare_parameter<bool>("auto_blind_direction", true);
    blind_width_rad_ = declare_parameter<double>("blind_width_deg", 90.0) * kPi / 180.0;
    center_x_ = declare_parameter<double>("robot_center.x", 0.0);
    center_y_ = declare_parameter<double>("robot_center.y", 0.0);
    center_z_ = declare_parameter<double>("robot_center.z", 0.0);
    min_range_ = declare_parameter<double>("min_range", 0.1);
    max_range_ = declare_parameter<double>("max_range", 100.0);
    sync_tolerance_ns_ = static_cast<std::int64_t>(
      declare_parameter<double>("sync_tolerance_ms", 30.0) * 1.0e6);
    queue_size_ = static_cast<std::size_t>(declare_parameter<int>("sync_queue_size", 10));
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    report_period_sec_ = declare_parameter<double>("report_period_sec", 5.0);
    publish_rejected_ = declare_parameter<bool>("publish_rejected_cloud", true);
    metrics_csv_path_ = declare_parameter<std::string>("metrics_csv_path", "");

    front_.name = "front";
    rear_.name = "rear";
    configure_stream(front_, "/antbot/lidar/front_left/points_raw_native",
      "/mid360/front/filtered", "/mid360/front/rejected", 0.0);
    configure_stream(rear_, "/antbot/lidar/rear_right/points_raw_native",
      "/mid360/rear/filtered", "/mid360/rear/rejected", 180.0);
    const auto merged_topic = declare_parameter<std::string>("merged_topic", "/mid360/merged");

    if (processing_stage_ != "crop" && processing_stage_ != "transform" &&
      processing_stage_ != "merge")
    {
      throw std::invalid_argument("processing_stage must be crop, transform, or merge");
    }
    if (blind_width_rad_ < 0.0 || blind_width_rad_ > 2.0 * kPi ||
      min_range_ < 0.0 || max_range_ <= min_range_ || sync_tolerance_ns_ < 0 ||
      queue_size_ == 0 || report_period_sec_ <= 0.0)
    {
      throw std::invalid_argument("invalid blind/range/sync/report parameter");
    }

    const auto qos = rclcpp::SensorDataQoS();
    front_.filtered_publisher = create_publisher<Cloud>(front_.filtered_topic, qos);
    rear_.filtered_publisher = create_publisher<Cloud>(rear_.filtered_topic, qos);
    if (publish_rejected_) {
      front_.rejected_publisher = create_publisher<Cloud>(front_.rejected_topic, qos);
      rear_.rejected_publisher = create_publisher<Cloud>(rear_.rejected_topic, qos);
    }
    merged_publisher_ = create_publisher<Cloud>(merged_topic, qos);
    front_.subscription = create_subscription<Cloud>(
      front_.input_topic, qos,
      [this](Cloud::ConstSharedPtr message) {handle_cloud(front_, message);});
    rear_.subscription = create_subscription<Cloud>(
      rear_.input_topic, qos,
      [this](Cloud::ConstSharedPtr message) {handle_cloud(rear_, message);});

    if (!metrics_csv_path_.empty()) {
      metrics_csv_.open(metrics_csv_path_, std::ios::out | std::ios::trunc);
      if (!metrics_csv_) {
        throw std::runtime_error("cannot open metrics_csv_path: " + metrics_csv_path_);
      }
      metrics_csv_ << "stamp_ns,stage,front_crop_ms,rear_crop_ms,front_tf_ms,rear_tf_ms,"
        "merge_ms,total_ms,front_input_points,rear_input_points,front_kept_points,"
        "rear_kept_points,sync_success,sync_miss,dropped_clouds\n";
      metrics_csv_.flush();
    }

    window_started_ = SteadyClock::now();
    report_timer_ = create_wall_timer(
      std::chrono::duration<double>(report_period_sec_), [this]() {report_and_reset();});

    RCLCPP_INFO(get_logger(),
      "stage=%s target=%s auto_blind=%s blind_width=%.1f deg sync=%.1f ms",
      processing_stage_.c_str(), target_frame_.c_str(), auto_blind_direction_ ? "true" : "false",
      blind_width_rad_ * 180.0 / kPi, sync_tolerance_ns_ / 1.0e6);
    RCLCPP_INFO(get_logger(), "front input: %s; rear input: %s",
      front_.input_topic.c_str(), rear_.input_topic.c_str());
  }

private:
  struct PreparedCloud
  {
    Cloud filtered;
    std::int64_t stamp{0};
    double crop_ms{0.0};
    double preprocess_ms{0.0};
    std::size_t input_points{0};
  };

  struct Stream
  {
    std::string name;
    std::string input_topic;
    std::string filtered_topic;
    std::string rejected_topic;
    double manual_blind_center_rad{0.0};
    bool described{false};
    rclcpp::Subscription<Cloud>::SharedPtr subscription;
    rclcpp::Publisher<Cloud>::SharedPtr filtered_publisher;
    rclcpp::Publisher<Cloud>::SharedPtr rejected_publisher;
    std::deque<PreparedCloud> queue;
    std::uint64_t input_messages{0};
    std::uint64_t input_points{0};
    std::uint64_t kept_points{0};
    std::uint64_t rejected_points{0};
    std::vector<double> crop_times;
    std::vector<double> tf_times;
  };

  void configure_stream(
    Stream & stream, const std::string & input_default, const std::string & filtered_default,
    const std::string & rejected_default, double blind_default_deg)
  {
    stream.input_topic = declare_parameter<std::string>(stream.name + ".input_topic", input_default);
    stream.filtered_topic = declare_parameter<std::string>(
      stream.name + ".filtered_topic", filtered_default);
    stream.rejected_topic = declare_parameter<std::string>(
      stream.name + ".rejected_topic", rejected_default);
    stream.manual_blind_center_rad = declare_parameter<double>(
      stream.name + ".blind_center_deg", blind_default_deg) * kPi / 180.0;
  }

  geometry_msgs::msg::TransformStamped lookup_transform(const Cloud & cloud)
  {
    if (cloud.header.frame_id.empty()) {
      throw tf2::TransformException("input cloud has an empty frame_id");
    }
    return tf_buffer_.lookupTransform(
      target_frame_, cloud.header.frame_id, rclcpp::Time(cloud.header.stamp),
      rclcpp::Duration::from_seconds(tf_timeout_sec_));
  }

  double blind_center_for(const Stream & stream, const Cloud & cloud)
  {
    if (!auto_blind_direction_) {return stream.manual_blind_center_rad;}
    const auto transform = lookup_transform(cloud);
    try {
      return blind_center_from_transform(transform.transform, center_x_, center_y_, center_z_);
    } catch (const std::invalid_argument & error) {
      throw tf2::TransformException(error.what());
    }
  }

  void handle_cloud(Stream & stream, const Cloud::ConstSharedPtr & message)
  {
    const auto whole_start = SteadyClock::now();
    ++stream.input_messages;
    const std::size_t input_count = static_cast<std::size_t>(message->width) * message->height;
    stream.input_points += input_count;
    if (!stream.described) {
      RCLCPP_INFO(get_logger(),
        "%s first cloud: topic=%s frame_id=%s points=%zu point_step=%u fields=[%s]",
        stream.name.c_str(), stream.input_topic.c_str(), message->header.frame_id.c_str(), input_count,
        message->point_step, describe_fields(*message).c_str());
      stream.described = true;
    }

    try {
      const double center = blind_center_for(stream, *message);
      const auto crop_start = SteadyClock::now();
      auto result = crop_cloud(
        *message, center, blind_width_rad_, min_range_, max_range_, publish_rejected_);
      const double crop_ms = elapsed_ms(crop_start);
      stream.crop_times.push_back(crop_ms);
      stream.kept_points += result.kept.width;
      stream.rejected_points += result.blind_rejected_points;
      stream.filtered_publisher->publish(result.kept);
      if (publish_rejected_) {stream.rejected_publisher->publish(result.rejected);}

      if (processing_stage_ == "crop") {
        write_single_csv(stream, crop_ms, 0.0, elapsed_ms(whole_start), input_count, result.kept.width);
        return;
      }
      if (processing_stage_ == "transform") {
        auto transformed = std::move(result.kept);
        const double tf_ms = transform_cloud(transformed);
        stream.tf_times.push_back(tf_ms);
        write_single_csv(stream, crop_ms, tf_ms, elapsed_ms(whole_start), input_count, transformed.width);
        return;
      }

      PreparedCloud prepared;
      prepared.filtered = std::move(result.kept);
      prepared.stamp = stamp_ns(message->header.stamp);
      prepared.crop_ms = crop_ms;
      prepared.preprocess_ms = elapsed_ms(whole_start);
      prepared.input_points = input_count;
      stream.queue.push_back(std::move(prepared));
      enforce_queue_limit(stream);
      try_pair();
    } catch (const tf2::TransformException & error) {
      ++dropped_clouds_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%s cloud dropped: TF unavailable (%s)", stream.name.c_str(), error.what());
    } catch (const std::exception & error) {
      ++dropped_clouds_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "%s cloud dropped: %s", stream.name.c_str(), error.what());
    }
  }

  double transform_cloud(Cloud & cloud)
  {
    const auto start = SteadyClock::now();
    const auto transform = lookup_transform(cloud);
    transform_xyz_in_place(cloud, transform.transform);
    cloud.header.frame_id = target_frame_;
    return elapsed_ms(start);
  }

  void enforce_queue_limit(Stream & stream)
  {
    while (stream.queue.size() > queue_size_) {
      stream.queue.pop_front();
      ++sync_misses_;
      ++dropped_clouds_;
    }
  }

  void try_pair()
  {
    while (!front_.queue.empty() && !rear_.queue.empty()) {
      std::size_t best_front = 0;
      std::size_t best_rear = 0;
      std::int64_t best_delta = std::numeric_limits<std::int64_t>::max();
      for (std::size_t i = 0; i < front_.queue.size(); ++i) {
        for (std::size_t j = 0; j < rear_.queue.size(); ++j) {
          const auto delta = std::llabs(front_.queue[i].stamp - rear_.queue[j].stamp);
          if (delta < best_delta) {
            best_delta = delta;
            best_front = i;
            best_rear = j;
          }
        }
      }
      if (best_delta <= sync_tolerance_ns_) {
        auto front = std::move(front_.queue[best_front]);
        auto rear = std::move(rear_.queue[best_rear]);
        front_.queue.erase(front_.queue.begin() + static_cast<std::ptrdiff_t>(best_front));
        rear_.queue.erase(rear_.queue.begin() + static_cast<std::ptrdiff_t>(best_rear));
        process_pair(std::move(front), std::move(rear));
        continue;
      }
      if (front_.queue.front().stamp < rear_.queue.front().stamp - sync_tolerance_ns_) {
        front_.queue.pop_front();
      } else if (rear_.queue.front().stamp < front_.queue.front().stamp - sync_tolerance_ns_) {
        rear_.queue.pop_front();
      } else {
        break;
      }
      ++sync_misses_;
      ++dropped_clouds_;
    }
  }

  void process_pair(PreparedCloud front, PreparedCloud rear)
  {
    const auto total_start = SteadyClock::now();
    try {
      const double front_tf_ms = transform_cloud(front.filtered);
      const double rear_tf_ms = transform_cloud(rear.filtered);
      front_.tf_times.push_back(front_tf_ms);
      rear_.tf_times.push_back(rear_tf_ms);
      const auto merge_start = SteadyClock::now();
      auto merged = merge_clouds(front.filtered, rear.filtered, target_frame_);
      const double merge_ms = elapsed_ms(merge_start);
      const double total_ms = front.preprocess_ms + rear.preprocess_ms + elapsed_ms(total_start);
      merge_times_.push_back(merge_ms);
      total_times_.push_back(total_ms);
      merged_points_ += merged.width;
      ++merged_messages_;
      ++sync_success_;
      merged_publisher_->publish(merged);
      write_pair_csv(front, rear, front_tf_ms, rear_tf_ms, merge_ms, total_ms);
    } catch (const tf2::TransformException & error) {
      dropped_clouds_ += 2;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "synchronized pair dropped: TF unavailable (%s)", error.what());
    } catch (const std::exception & error) {
      dropped_clouds_ += 2;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "synchronized pair dropped: %s", error.what());
    }
  }

  void write_single_csv(
    const Stream & stream, double crop_ms, double tf_ms, double total_ms,
    std::size_t input_points, std::size_t kept_points)
  {
    if (!metrics_csv_) {return;}
    const bool is_front = stream.name == "front";
    metrics_csv_ << 0 << ',' << processing_stage_ << ',' <<
      (is_front ? crop_ms : 0.0) << ',' << (is_front ? 0.0 : crop_ms) << ',' <<
      (is_front ? tf_ms : 0.0) << ',' << (is_front ? 0.0 : tf_ms) << ",0," << total_ms << ',' <<
      (is_front ? input_points : 0) << ',' << (is_front ? 0 : input_points) << ',' <<
      (is_front ? kept_points : 0) << ',' << (is_front ? 0 : kept_points) << ',' <<
      sync_success_ << ',' << sync_misses_ << ',' << dropped_clouds_ << '\n';
  }

  void write_pair_csv(
    const PreparedCloud & front, const PreparedCloud & rear, double front_tf_ms,
    double rear_tf_ms, double merge_ms, double total_ms)
  {
    if (!metrics_csv_) {return;}
    metrics_csv_ << std::max(front.stamp, rear.stamp) << ",merge," << front.crop_ms << ',' <<
      rear.crop_ms << ',' << front_tf_ms << ',' << rear_tf_ms << ',' << merge_ms << ',' <<
      total_ms << ',' << front.input_points << ',' << rear.input_points << ',' <<
      front.filtered.width << ',' << rear.filtered.width << ',' << sync_success_ << ',' <<
      sync_misses_ << ',' << dropped_clouds_ << '\n';
  }

  void report_and_reset()
  {
    const double seconds = std::chrono::duration<double>(SteadyClock::now() - window_started_).count();
    if (seconds <= 0.0) {return;}
    const auto front_crop = summarize(front_.crop_times);
    const auto rear_crop = summarize(rear_.crop_times);
    const auto front_tf = summarize(front_.tf_times);
    const auto rear_tf = summarize(rear_.tf_times);
    const auto merge = summarize(merge_times_);
    const auto total = summarize(total_times_);
    const auto ratio = [](std::uint64_t kept, std::uint64_t input) {
        return input == 0 ? 0.0 : 100.0 * static_cast<double>(kept) / input;
      };

    std::ostringstream output;
    output << std::fixed << std::setprecision(2)
      << "\n========== Dual MID360 Benchmark ==========\n"
      << "Front:\n"
      << "input        " << front_.input_points / seconds << " pts/s\n"
      << "filtered     " << front_.kept_points / seconds << " pts/s\n"
      << "rejected     " << front_.rejected_points / seconds << " pts/s\n"
      << "keep ratio   " << ratio(front_.kept_points, front_.input_points) << " %\n"
      << "input Hz     " << front_.input_messages / seconds << " Hz\n\n"
      << "Rear:\n"
      << "input        " << rear_.input_points / seconds << " pts/s\n"
      << "filtered     " << rear_.kept_points / seconds << " pts/s\n"
      << "rejected     " << rear_.rejected_points / seconds << " pts/s\n"
      << "keep ratio   " << ratio(rear_.kept_points, rear_.input_points) << " %\n"
      << "input Hz     " << rear_.input_messages / seconds << " Hz\n\n"
      << "Merged:\n"
      << "points       " << merged_points_ / seconds << " pts/s\n"
      << "output Hz    " << merged_messages_ / seconds << " Hz\n\n"
      << "Timing:\n"
      << "crop front avg " << front_crop.average << " ms\n"
      << "crop rear avg  " << rear_crop.average << " ms\n"
      << "TF front avg   " << front_tf.average << " ms\n"
      << "TF rear avg    " << rear_tf.average << " ms\n"
      << "merge avg      " << merge.average << " ms\n"
      << "total avg      " << total.average << " ms\n"
      << "P50            " << total.p50 << " ms\n"
      << "P95            " << total.p95 << " ms\n"
      << "P99            " << total.p99 << " ms\n"
      << "max            " << total.maximum << " ms\n\n"
      << "sync successes " << sync_success_ << "\n"
      << "sync misses    " << sync_misses_ << "\n"
      << "dropped clouds " << dropped_clouds_ << "\n"
      << "===========================================\n";
    RCLCPP_INFO(get_logger(), "%s", output.str().c_str());
    if (metrics_csv_) {metrics_csv_.flush();}

    front_.input_messages = front_.input_points = front_.kept_points = front_.rejected_points = 0;
    rear_.input_messages = rear_.input_points = rear_.kept_points = rear_.rejected_points = 0;
    merged_messages_ = merged_points_ = 0;
    front_.crop_times.clear();
    rear_.crop_times.clear();
    front_.tf_times.clear();
    rear_.tf_times.clear();
    merge_times_.clear();
    total_times_.clear();
    window_started_ = SteadyClock::now();
  }

  std::string target_frame_;
  std::string processing_stage_;
  bool auto_blind_direction_{true};
  double blind_width_rad_{kPi / 2.0};
  double center_x_{0.0};
  double center_y_{0.0};
  double center_z_{0.0};
  double min_range_{0.1};
  double max_range_{100.0};
  std::int64_t sync_tolerance_ns_{30000000};
  std::size_t queue_size_{10};
  double tf_timeout_sec_{0.05};
  double report_period_sec_{5.0};
  bool publish_rejected_{true};
  std::string metrics_csv_path_;
  std::ofstream metrics_csv_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Stream front_;
  Stream rear_;
  rclcpp::Publisher<Cloud>::SharedPtr merged_publisher_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  SteadyClock::time_point window_started_;
  std::vector<double> merge_times_;
  std::vector<double> total_times_;
  std::uint64_t merged_messages_{0};
  std::uint64_t merged_points_{0};
  std::uint64_t sync_success_{0};
  std::uint64_t sync_misses_{0};
  std::uint64_t dropped_clouds_{0};
};

}  // namespace antbot_mid360_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<antbot_mid360_fusion::DualMid360FusionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("dual_mid360_fusion"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
