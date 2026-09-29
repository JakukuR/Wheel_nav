#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "wla_r680_navigation/isolated_clusters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

using namespace std::chrono_literals;

namespace
{
struct Point3f
{
  float x;
  float y;
  float z;
};

int64_t cell_key(int32_t x, int32_t y)
{
  const auto ux = static_cast<uint64_t>(static_cast<uint32_t>(x));
  const auto uy = static_cast<uint64_t>(static_cast<uint32_t>(y));
  return static_cast<int64_t>((ux << 32U) | uy);
}
}  // namespace

class DepthToPoints final : public rclcpp::Node
{
public:
  DepthToPoints()
  : Node("r680_d455_depth_to_points"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    stride_ = static_cast<int>(std::max<int64_t>(1, declare_parameter<int64_t>("stride", 4)));
    min_depth_ = declare_parameter<double>("min_depth", 0.15);
    max_depth_ = declare_parameter<double>("max_depth", 5.0);
    max_rate_ = std::max(0.1, declare_parameter<double>("max_rate", 10.0));
    target_frame_ = declare_parameter<std::string>("target_frame", "r680_mapping_floor");
    ground_filter_enabled_ = declare_parameter<bool>("ground_filter_enabled", true);
    ground_cell_size_ = std::max(0.05, declare_parameter<double>("ground_cell_size", 0.20));
    ground_quantile_ = std::clamp(declare_parameter<double>("ground_quantile", 0.20), 0.0, 1.0);
    ground_min_points_ = static_cast<size_t>(
      std::max<int64_t>(1, declare_parameter<int64_t>("ground_min_points", 5)));
    ground_candidate_min_height_ = declare_parameter<double>(
      "ground_candidate_min_height", -0.08);
    ground_candidate_max_height_ = declare_parameter<double>(
      "ground_candidate_max_height", 0.12);
    ground_clearance_ = declare_parameter<double>("ground_clearance", 0.05);
    fallback_obstacle_min_height_ = declare_parameter<double>(
      "fallback_obstacle_min_height", 0.08);
    obstacle_max_height_ = declare_parameter<double>("obstacle_max_height", 0.50);
    clearing_min_height_ = declare_parameter<double>("clearing_min_height", -0.08);
    clearing_max_height_ = declare_parameter<double>("clearing_max_height", 0.50);
    isolated_cluster_cell_size_ = std::max(
      0.01, declare_parameter<double>("isolated_cluster_cell_size", 0.05));
    isolated_cluster_min_cells_ = static_cast<size_t>(
      std::max<int64_t>(1, declare_parameter<int64_t>("isolated_cluster_min_cells", 3)));

    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/r680/d455/aligned_depth_to_color/image_raw");
    const auto info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/r680/d455/color/camera_info");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/r680_nav/d455/points");
    const auto safety_topic = declare_parameter<std::string>(
      "safety_output_topic", "/r680_nav/d455/points_safety");
    const auto clearing_topic = declare_parameter<std::string>(
      "clearing_output_topic", "/r680_nav/d455/clearing_points");
    const auto visualization_topic = declare_parameter<std::string>(
      "visualization_output_topic", "/r680_nav/d455/points_viz");

    const auto qos = rclcpp::SensorDataQoS();
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (msg->k[0] > 0.0 && msg->k[4] > 0.0) {camera_info_ = msg;}
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, qos, std::bind(&DepthToPoints::depth_callback, this, std::placeholders::_1));

    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, output_qos);
    safety_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(safety_topic, output_qos);
    clearing_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(clearing_topic, output_qos);
    visualization_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      visualization_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
  }

private:
  float depth_at(const sensor_msgs::msg::Image & image, uint32_t u, uint32_t v) const
  {
    const auto * row = image.data.data() + static_cast<size_t>(v) * image.step;
    if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      image.encoding == sensor_msgs::image_encodings::MONO16)
    {
      uint16_t value;
      std::memcpy(&value, row + static_cast<size_t>(u) * sizeof(value), sizeof(value));
      return static_cast<float>(value) * 0.001F;
    }
    if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      float value;
      std::memcpy(&value, row + static_cast<size_t>(u) * sizeof(value), sizeof(value));
      return value;
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  static Point3f transform_point(
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

  std::vector<Point3f> filter_obstacles(const std::vector<Point3f> & points) const
  {
    if (!ground_filter_enabled_) {
      std::vector<Point3f> result;
      result.reserve(points.size());
      for (const auto & point : points) {
        if (point.z >= fallback_obstacle_min_height_ && point.z <= obstacle_max_height_) {
          result.push_back(point);
        }
      }
      return result;
    }

    std::unordered_map<int64_t, std::vector<float>> ground_samples;
    ground_samples.reserve(points.size() / 8U + 1U);
    for (const auto & point : points) {
      if (point.z < ground_candidate_min_height_ || point.z > ground_candidate_max_height_) {
        continue;
      }
      const auto ix = static_cast<int32_t>(std::floor(point.x / ground_cell_size_));
      const auto iy = static_cast<int32_t>(std::floor(point.y / ground_cell_size_));
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          ground_samples[cell_key(ix + dx, iy + dy)].push_back(point.z);
        }
      }
    }

    std::unordered_map<int64_t, float> ground_height;
    ground_height.reserve(ground_samples.size());
    for (auto & [key, samples] : ground_samples) {
      if (samples.size() < ground_min_points_) {continue;}
      const size_t index = static_cast<size_t>(
        ground_quantile_ * static_cast<double>(samples.size() - 1U));
      std::nth_element(samples.begin(), samples.begin() + index, samples.end());
      ground_height.emplace(key, samples[index]);
    }

    std::vector<Point3f> result;
    result.reserve(points.size() / 4U + 1U);
    for (const auto & point : points) {
      if (point.z > obstacle_max_height_) {continue;}
      const auto ix = static_cast<int32_t>(std::floor(point.x / ground_cell_size_));
      const auto iy = static_cast<int32_t>(std::floor(point.y / ground_cell_size_));
      const auto estimate = ground_height.find(cell_key(ix, iy));
      const bool obstacle = estimate != ground_height.end() ?
        point.z - estimate->second >= ground_clearance_ :
        point.z >= fallback_obstacle_min_height_;
      if (obstacle) {result.push_back(point);}
    }
    return result;
  }

  static sensor_msgs::msg::PointCloud2 make_cloud(
    const std_msgs::msg::Header & header, const std::string & frame,
    const std::vector<Point3f> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = frame;
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
    return cloud;
  }

  void depth_callback(sensor_msgs::msg::Image::ConstSharedPtr image)
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_publish_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - last_publish_).count() < 1.0 / max_rate_)
    {
      return;
    }
    const auto info = camera_info_;
    if (!info || info->width != image->width || info->height != image->height) {return;}
    if (image->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      image->encoding != sensor_msgs::image_encodings::MONO16 &&
      image->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "unsupported depth encoding: %s", image->encoding.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped optical_to_base;
    try {
      optical_to_base = tf_buffer_.lookupTransform(
        target_frame_, image->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "waiting for static D455 transform %s <- %s: %s",
        target_frame_.c_str(), image->header.frame_id.c_str(), error.what());
      return;
    }

    const float fx = static_cast<float>(info->k[0]);
    const float fy = static_cast<float>(info->k[4]);
    const float cx = static_cast<float>(info->k[2]);
    const float cy = static_cast<float>(info->k[5]);
    std::vector<Point3f> points;
    points.reserve((image->width / stride_ + 1) * (image->height / stride_ + 1));
    for (uint32_t v = 0; v < image->height; v += stride_) {
      for (uint32_t u = 0; u < image->width; u += stride_) {
        const float depth = depth_at(*image, u, v);
        if (!std::isfinite(depth) || depth < min_depth_ || depth > max_depth_) {continue;}
        const Point3f optical{
          (static_cast<float>(u) - cx) * depth / fx,
          (static_cast<float>(v) - cy) * depth / fy,
          depth};
        points.push_back(transform_point(optical, optical_to_base.transform));
      }
    }

    std::vector<Point3f> clearing_points;
    clearing_points.reserve(points.size());
    for (const auto & point : points) {
      if (point.z >= clearing_min_height_ && point.z <= clearing_max_height_) {
        clearing_points.push_back(point);
      }
    }
    const auto ground_filtered = filter_obstacles(clearing_points);
    safety_pub_->publish(make_cloud(image->header, target_frame_, ground_filtered));
    const auto obstacle_points = wla_r680_navigation::remove_isolated_clusters(
      ground_filtered, isolated_cluster_cell_size_, isolated_cluster_min_cells_);
    auto obstacle_cloud = make_cloud(image->header, target_frame_, obstacle_points);
    points_pub_->publish(obstacle_cloud);
    clearing_pub_->publish(make_cloud(image->header, target_frame_, clearing_points));

    // RViz is diagnostic only. A zero stamp asks tf2 for the latest complete
    // map->base chain and avoids rejecting valid clouds while map correction lags.
    obstacle_cloud.header.stamp.sec = 0;
    obstacle_cloud.header.stamp.nanosec = 0;
    visualization_pub_->publish(obstacle_cloud);
    last_publish_ = now;
  }

  int stride_{4};
  double min_depth_{0.15};
  double max_depth_{5.0};
  double max_rate_{10.0};
  std::string target_frame_{"r680_mapping_floor"};
  bool ground_filter_enabled_{true};
  double ground_cell_size_{0.20};
  double ground_quantile_{0.20};
  size_t ground_min_points_{5};
  double ground_candidate_min_height_{-0.08};
  double ground_candidate_max_height_{0.12};
  double ground_clearance_{0.05};
  double fallback_obstacle_min_height_{0.08};
  double obstacle_max_height_{0.50};
  double clearing_min_height_{-0.08};
  double clearing_max_height_{0.50};
  double isolated_cluster_cell_size_{0.05};
  size_t isolated_cluster_min_cells_{3};
  std::chrono::steady_clock::time_point last_publish_{};
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr safety_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visualization_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthToPoints>());
  rclcpp::shutdown();
  return 0;
}
