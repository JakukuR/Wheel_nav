#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

class DepthToPoints final : public rclcpp::Node
{
public:
  DepthToPoints()
  : Node("r680_d455_depth_to_points")
  {
    stride_ = static_cast<int>(std::max<int64_t>(1, declare_parameter<int64_t>("stride", 4)));
    min_depth_ = declare_parameter<double>("min_depth", 0.15);
    max_depth_ = declare_parameter<double>("max_depth", 5.0);
    max_rate_ = std::max(0.1, declare_parameter<double>("max_rate", 10.0));
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/r680/d455/aligned_depth_to_color/image_raw");
    const auto info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/r680/d455/color/camera_info");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/r680_nav/d455/points");

    const auto qos = rclcpp::SensorDataQoS();
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (msg->k[0] > 0.0 && msg->k[4] > 0.0) {camera_info_ = msg;}
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, qos, std::bind(&DepthToPoints::depth_callback, this, std::placeholders::_1));
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);
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

    const float fx = static_cast<float>(info->k[0]);
    const float fy = static_cast<float>(info->k[4]);
    const float cx = static_cast<float>(info->k[2]);
    const float cy = static_cast<float>(info->k[5]);
    std::vector<float> xyz;
    xyz.reserve((image->width / stride_ + 1) * (image->height / stride_ + 1) * 3);
    for (uint32_t v = 0; v < image->height; v += stride_) {
      for (uint32_t u = 0; u < image->width; u += stride_) {
        const float z = depth_at(*image, u, v);
        if (!std::isfinite(z) || z < min_depth_ || z > max_depth_) {continue;}
        xyz.push_back((static_cast<float>(u) - cx) * z / fx);
        xyz.push_back((static_cast<float>(v) - cy) * z / fy);
        xyz.push_back(z);
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = image->header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(xyz.size() / 3);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (size_t i = 0; i < xyz.size(); i += 3, ++x, ++y, ++z) {
      *x = xyz[i]; *y = xyz[i + 1]; *z = xyz[i + 2];
    }
    cloud.is_dense = true;
    points_pub_->publish(cloud);
    last_publish_ = now;
  }

  int stride_{4};
  double min_depth_{0.15};
  double max_depth_{5.0};
  double max_rate_{10.0};
  std::chrono::steady_clock::time_point last_publish_{};
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthToPoints>());
  rclcpp::shutdown();
  return 0;
}
