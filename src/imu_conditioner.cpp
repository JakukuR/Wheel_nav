#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuConditioner final : public rclcpp::Node
{
public:
  ImuConditioner() : Node("r680_chassis_imu_conditioner")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/wheel/imu/data_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/r680_nav/chassis/imu_calibrated_raw");
    calibration_samples_ = declare_parameter<int>("calibration_samples", 100);
    max_stationary_gyro_ = declare_parameter<double>("max_stationary_gyro", 0.15);
    min_gravity_ = declare_parameter<double>("min_gravity", 8.0);
    max_gravity_ = declare_parameter<double>("max_gravity", 11.5);
    gyro_variance_ = declare_parameter<double>("gyro_variance", 4.0e-4);
    accel_variance_ = declare_parameter<double>("accel_variance", 4.0e-2);
    if (calibration_samples_ < 20 || max_stationary_gyro_ <= 0.0 || min_gravity_ <= 0.0 ||
        max_gravity_ <= min_gravity_ || gyro_variance_ <= 0.0 || accel_variance_ <= 0.0) {
      throw std::invalid_argument("invalid IMU conditioner parameters");
    }

    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuConditioner::on_imu, this, std::placeholders::_1));
    RCLCPP_WARN(get_logger(), "keep chassis stationary while collecting %d gyro-bias samples",
      calibration_samples_);
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    const auto & w = msg->angular_velocity;
    const auto & a = msg->linear_acceleration;
    const double wn = std::sqrt(w.x*w.x + w.y*w.y + w.z*w.z);
    const double an = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
    if (!ready_) {
      if (!std::isfinite(wn) || !std::isfinite(an) || wn > max_stationary_gyro_ ||
          an < min_gravity_ || an > max_gravity_) {
        count_ = 0;
        sum_x_ = sum_y_ = sum_z_ = 0.0;
        return;
      }
      sum_x_ += w.x; sum_y_ += w.y; sum_z_ += w.z;
      if (++count_ < calibration_samples_) return;
      bias_x_ = sum_x_ / count_; bias_y_ = sum_y_ / count_; bias_z_ = sum_z_ / count_;
      ready_ = true;
      RCLCPP_INFO(get_logger(), "gyro bias ready: [%.6f, %.6f, %.6f] rad/s",
        bias_x_, bias_y_, bias_z_);
    }

    auto out = *msg;
    out.orientation.x = out.orientation.y = out.orientation.z = 0.0;
    out.orientation.w = 1.0;
    out.orientation_covariance.fill(0.0);
    out.orientation_covariance[0] = -1.0;  // raw message has no trusted orientation
    out.angular_velocity.x -= bias_x_;
    out.angular_velocity.y -= bias_y_;
    out.angular_velocity.z -= bias_z_;
    out.angular_velocity_covariance.fill(0.0);
    out.linear_acceleration_covariance.fill(0.0);
    for (const std::size_t i : {0U, 4U, 8U}) {
      out.angular_velocity_covariance[i] = gyro_variance_;
      out.linear_acceleration_covariance[i] = accel_variance_;
    }
    pub_->publish(out);
  }

  std::string input_topic_, output_topic_;
  int calibration_samples_{};
  int count_{};
  double max_stationary_gyro_{}, min_gravity_{}, max_gravity_{};
  double gyro_variance_{}, accel_variance_{};
  double sum_x_{}, sum_y_{}, sum_z_{};
  double bias_x_{}, bias_y_{}, bias_z_{};
  bool ready_{false};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuConditioner>());
  rclcpp::shutdown();
  return 0;
}
