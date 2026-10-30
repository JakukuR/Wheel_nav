// Gravity-aligned 6-DoF odometry for terrain analysis on slopes.
// Planar wheel odometry has no pitch/roll or height. Roll/pitch come from the
// chassis IMU (complementary filter: gyro integration corrected towards the
// accelerometer's gravity direction while not accelerating hard); the wheel
// speed along the body x axis is integrated in 3-D, so climbing a ramp raises z.
#include <cmath>
#include <deque>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class TerrainOdometry : public rclcpp::Node {
 public:
  TerrainOdometry() : Node("terrain_odometry") {
    const auto wheel = declare_parameter<std::string>("wheel_topic", "/wheel/odom_50hz");
    const auto imu = declare_parameter<std::string>("imu_topic", "/wheel/imu/data_raw");
    frame_ = declare_parameter<std::string>("odom_frame", "terrain_odom");
    base_ = declare_parameter<std::string>("base_frame", "r680_mapping_floor");
    gain_ = declare_parameter<double>("accel_gain", .02);           // per IMU sample
    accel_window_ = declare_parameter<double>("accel_window", .5);  // |a|-g gate, m/s^2
    bias_time_ = declare_parameter<double>("bias_calibration_s", 2.);
    pub_ = create_publisher<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("output_topic", "/r680_nav/terrain_odom"), 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { on_imu(*m); });
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(wheel, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_wheel(*m); });
  }

 private:
  static double stamp(const builtin_interfaces::msg::Time &t) { return t.sec + t.nanosec * 1e-9; }

  void on_imu(const sensor_msgs::msg::Imu &m) {
    const double t = stamp(m.header.stamp);
    const auto &g = m.angular_velocity;
    const auto &a = m.linear_acceleration;
    if (!std::isfinite(g.x + g.y + g.z + a.x + a.y + a.z)) return;
    if (!started_) { started_ = true; t0_ = t; last_imu_ = t; }
    // Gyro bias from the first seconds while the wheels are still.
    if (t - t0_ < bias_time_) {
      if (std::abs(speed_) < .01) { bx_ += g.x; by_ += g.y; ++bias_n_; }
      roll_ = std::atan2(a.y, a.z);
      pitch_ = std::atan2(-a.x, std::hypot(a.y, a.z));
      last_imu_ = t;
      return;
    }
    const double dt = t - last_imu_;
    last_imu_ = t;
    if (dt <= 0 || dt > .1) return;
    const double gx = g.x - (bias_n_ ? bx_ / bias_n_ : 0), gy = g.y - (bias_n_ ? by_ / bias_n_ : 0);
    // Euler-rate kinematics for roll/pitch (yaw comes from the wheels).
    const double cr = std::cos(roll_), sr = std::sin(roll_), tp = std::tan(pitch_);
    roll_ += (gx + (sr * tp) * gy) * dt;
    pitch_ += (cr * gy) * dt;
    const double norm = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (std::abs(norm - 9.81) < accel_window_) {
      roll_ += gain_ * (std::atan2(a.y, a.z) - roll_);
      pitch_ += gain_ * (std::atan2(-a.x, std::hypot(a.y, a.z)) - pitch_);
    }
    imu_ok_ = true;
  }

  void on_wheel(const nav_msgs::msg::Odometry &m) {
    const double t = stamp(m.header.stamp);
    const double v = m.twist.twist.linear.x, w = m.twist.twist.angular.z;
    if (!std::isfinite(v) || !std::isfinite(w)) return;
    if (last_wheel_ > 0) {
      const double dt = t - last_wheel_;
      if (dt > 0 && dt < .2) {
        const double yaw_mid = yaw_ + .5 * w * dt;
        // Body x axis in the odom frame for ZYX (yaw, pitch, roll): its z part is -sin(pitch).
        x_ += v * dt * std::cos(pitch_) * std::cos(yaw_mid);
        y_ += v * dt * std::cos(pitch_) * std::sin(yaw_mid);
        z_ += -v * dt * std::sin(pitch_);
        yaw_ += w * dt;
      }
    }
    last_wheel_ = t; speed_ = v;
    nav_msgs::msg::Odometry out;
    out.header.stamp = m.header.stamp;
    out.header.frame_id = frame_;
    out.child_frame_id = base_;
    out.pose.pose.position.x = x_; out.pose.pose.position.y = y_; out.pose.pose.position.z = z_;
    const double cy = std::cos(yaw_ * .5), sy = std::sin(yaw_ * .5), cp = std::cos(pitch_ * .5),
                 sp = std::sin(pitch_ * .5), cr = std::cos(roll_ * .5), sr = std::sin(roll_ * .5);
    out.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    out.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    out.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    out.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    // Covariance marks attitude unusable until the IMU filter is running.
    out.pose.covariance[21] = out.pose.covariance[28] = imu_ok_ ? 1e-3 : 1e6;
    out.twist.twist = m.twist.twist;
    pub_->publish(out);
  }

  std::string frame_, base_;
  double gain_, accel_window_, bias_time_;
  bool started_ = false, imu_ok_ = false;
  double t0_ = 0, last_imu_ = 0, last_wheel_ = 0, bx_ = 0, by_ = 0, speed_ = 0;
  int bias_n_ = 0;
  double roll_ = 0, pitch_ = 0, yaw_ = 0, x_ = 0, y_ = 0, z_ = 0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainOdometry>());
  rclcpp::shutdown();
  return 0;
}
