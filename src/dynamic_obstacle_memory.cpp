#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace
{
struct VoxelKey
{
  int32_t x;
  int32_t y;
  int32_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash
{
  size_t operator()(const VoxelKey & key) const
  {
    size_t seed = std::hash<int32_t>{}(key.x);
    seed ^= std::hash<int32_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int32_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct Point3f
{
  float x;
  float y;
  float z;
};

struct VoxelEntry
{
  Point3f point{};
  size_t hits{0U};
  bool confirmed{false};
  std::chrono::steady_clock::time_point first_seen{};
  std::chrono::steady_clock::time_point last_seen{};
};

Point3f transform_point(
  const Point3f & point, const geometry_msgs::msg::Transform & transform)
{
  const double qx = transform.rotation.x;
  const double qy = transform.rotation.y;
  const double qz = transform.rotation.z;
  const double qw = transform.rotation.w;
  const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double r01 = 2.0 * (qx * qy - qz * qw);
  const double r02 = 2.0 * (qx * qz + qy * qw);
  const double r10 = 2.0 * (qx * qy + qz * qw);
  const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
  const double r12 = 2.0 * (qy * qz - qx * qw);
  const double r20 = 2.0 * (qx * qz - qy * qw);
  const double r21 = 2.0 * (qy * qz + qx * qw);
  const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
  return {
    static_cast<float>(r00 * point.x + r01 * point.y + r02 * point.z +
      transform.translation.x),
    static_cast<float>(r10 * point.x + r11 * point.y + r12 * point.z +
      transform.translation.y),
    static_cast<float>(r20 * point.x + r21 * point.y + r22 * point.z +
      transform.translation.z)};
}
}  // namespace

class DynamicObstacleMemory final : public rclcpp::Node
{
public:
  DynamicObstacleMemory()
  : Node("r680_dynamic_obstacle_memory"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/r680_nav/d455/points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/r680_nav/d455/points_confirmed");
    target_frame_ = declare_parameter<std::string>("target_frame", "d455_floor_odom");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.10);
    min_hits_ = static_cast<size_t>(
      std::max<int64_t>(1, declare_parameter<int64_t>("min_hits", 3)));
    confirmation_window_ = declare_parameter<double>("confirmation_window", 0.60);
    max_observation_gap_ = declare_parameter<double>("max_observation_gap", 0.30);
    persistence_ = declare_parameter<double>("persistence", 4.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 5.0);
    max_voxels_ = static_cast<size_t>(
      std::max<int64_t>(100, declare_parameter<int64_t>("max_voxels", 30000)));

    if (voxel_size_ <= 0.0 || confirmation_window_ <= 0.0 ||
      max_observation_gap_ <= 0.0 || persistence_ <= 0.0 || publish_rate_ <= 0.0)
    {
      throw std::invalid_argument("invalid dynamic obstacle memory parameters");
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&DynamicObstacleMemory::cloud_callback, this, std::placeholders::_1));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_)),
      std::bind(&DynamicObstacleMemory::publish_confirmed, this));
  }

private:
  VoxelKey voxel_key(const Point3f & point) const
  {
    return {
      static_cast<int32_t>(std::floor(point.x / voxel_size_)),
      static_cast<int32_t>(std::floor(point.y / voxel_size_)),
      static_cast<int32_t>(std::floor(point.z / voxel_size_))};
  }

  void cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    geometry_msgs::msg::Transform transform;
    transform.rotation.w = 1.0;
    if (cloud->header.frame_id != target_frame_) {
      try {
        transform = tf_buffer_.lookupTransform(
          target_frame_, cloud->header.frame_id, tf2::TimePointZero).transform;
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "waiting for dynamic-obstacle transform %s <- %s: %s",
          target_frame_.c_str(), cloud->header.frame_id.c_str(), error.what());
        return;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    std::unordered_set<VoxelKey, VoxelHash> observed;
    observed.reserve(cloud->width * cloud->height / 2U + 1U);
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
      const Point3f point = transform_point({*x, *y, *z}, transform);
      const auto key = voxel_key(point);
      if (!observed.insert(key).second) {continue;}
      auto found = voxels_.find(key);
      if (found == voxels_.end()) {
        if (voxels_.size() >= max_voxels_) {continue;}
        VoxelEntry entry;
        entry.point = point;
        entry.hits = 1U;
        entry.first_seen = now;
        entry.last_seen = now;
        voxels_.emplace(key, entry);
        continue;
      }

      auto & entry = found->second;
      const double gap = std::chrono::duration<double>(now - entry.last_seen).count();
      const double age = std::chrono::duration<double>(now - entry.first_seen).count();
      if (!entry.confirmed &&
        (gap > max_observation_gap_ || age > confirmation_window_))
      {
        entry.hits = 1U;
        entry.first_seen = now;
      } else {
        ++entry.hits;
      }
      entry.point = point;
      entry.last_seen = now;
      if (entry.hits >= min_hits_) {entry.confirmed = true;}
    }
  }

  void publish_confirmed()
  {
    const auto now = std::chrono::steady_clock::now();
    std::vector<Point3f> points;
    points.reserve(voxels_.size());
    for (auto it = voxels_.begin(); it != voxels_.end();) {
      const double age = std::chrono::duration<double>(now - it->second.last_seen).count();
      if (age > persistence_) {
        it = voxels_.erase(it);
        continue;
      }
      if (it->second.confirmed) {points.push_back(it->second.point);}
      ++it;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = get_clock()->now();
    cloud.header.frame_id = target_frame_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points) {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      ++x;
      ++y;
      ++z;
    }
    cloud.is_dense = true;
    cloud_pub_->publish(cloud);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double voxel_size_{};
  size_t min_hits_{};
  double confirmation_window_{};
  double max_observation_gap_{};
  double persistence_{};
  double publish_rate_{};
  size_t max_voxels_{};
  std::unordered_map<VoxelKey, VoxelEntry, VoxelHash> voxels_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicObstacleMemory>());
  rclcpp::shutdown();
  return 0;
}
