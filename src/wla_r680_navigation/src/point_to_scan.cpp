#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "wla_r680_navigation/scan_projection.hpp"

namespace
{
struct Point3f {float x; float y; float z;};
}

class PointToScan final : public rclcpp::Node
{
public:
  PointToScan()
  : Node("r680_d455_point_to_scan")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/r680_nav/d455/points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/r680_nav/d455/scan");
    expected_frame_ = declare_parameter<std::string>(
      "expected_frame", "r680_mapping_floor");
    config_ = {
      declare_parameter<double>("angle_min", -M_PI_2),
      declare_parameter<double>("angle_max", M_PI_2),
      declare_parameter<double>("angle_increment", M_PI / 360.0),
      declare_parameter<double>("range_min", 0.15),
      declare_parameter<double>("range_max", 4.5),
      declare_parameter<double>("min_height", -0.08),
      declare_parameter<double>("max_height", 0.50)};
    scan_time_ = declare_parameter<double>("scan_time", 0.10);
    if (!std::isfinite(config_.angle_min) || !std::isfinite(config_.angle_max) ||
      !std::isfinite(config_.angle_increment) ||
      config_.angle_max <= config_.angle_min || config_.angle_increment <= 0.0 ||
      wla_r680_navigation::scan_ray_count(config_) > 1441 ||
      config_.range_min < 0.0 || config_.range_max <= config_.range_min ||
      config_.max_height < config_.min_height || scan_time_ <= 0.0)
    {
      throw std::invalid_argument("invalid point-to-scan projection parameters");
    }
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(2);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&PointToScan::cloud_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Projecting %s into %s in frame %s",
      input_topic_.c_str(), output_topic_.c_str(), expected_frame_.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != expected_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping cloud in unexpected frame %s",
        msg->header.frame_id.c_str());
      return;
    }
    sensor_msgs::msg::LaserScan scan;
    scan.header = msg->header;
    scan.angle_min = static_cast<float>(config_.angle_min);
    scan.angle_increment = static_cast<float>(config_.angle_increment);
    scan.ranges.assign(
      wla_r680_navigation::scan_ray_count(config_),
      std::numeric_limits<float>::infinity());
    scan.angle_max = scan.angle_min +
      static_cast<float>(scan.ranges.size() - 1) * scan.angle_increment;
    scan.range_min = static_cast<float>(config_.range_min);
    scan.range_max = static_cast<float>(config_.range_max);
    scan.scan_time = static_cast<float>(scan_time_);
    scan.time_increment = 0.0F;  // The depth image is one camera exposure.
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        wla_r680_navigation::add_scan_point(Point3f{*x, *y, *z}, config_, scan.ranges);
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping malformed obstacle cloud: %s",
        error.what());
      return;
    }
    scan_pub_->publish(std::move(scan));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string expected_frame_;
  double scan_time_;
  wla_r680_navigation::ScanProjectionConfig config_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointToScan>());
  rclcpp::shutdown();
  return 0;
}
