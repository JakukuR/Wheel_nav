// Sliding-window odometry node: replaces degraded_odometry in the cuVSLAM
// continuity slot. cuVSLAM publishes /r680_nav/vio_raw_odom without TF; this
// node owns /d455_slam/odom and odom->base, keeps them continuous through
// visual dropouts, cuVSLAM jumps and restarts, and reports
// GOOD/BRIDGING/LOST plus an epoch that increments when continuity was lost.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "wla_r680_navigation/odom_fusion.hpp"

using namespace std::chrono_literals;
namespace fusion = wla_r680_navigation::fusion;

class OdomFusion final : public rclcpp::Node
{
  using Clock = std::chrono::steady_clock;
  using Odom = nav_msgs::msg::Odometry;

public:
  OdomFusion()
  : Node("r680_odom_fusion"), est_(config())
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "d455_floor_odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "r680_mapping_floor");
    imu_frame_ = declare_parameter<std::string>("expected_imu_frame", "gyro_link");
    wheel_timeout_ = positive("wheel_timeout_s", 0.15);
    imu_timeout_ = positive("imu_timeout_s", 0.10);
    visual_timeout_ = positive("visual_timeout_s", 0.15);
    input_timeout_ = positive("input_timeout_s", 0.20);
    lag_ = positive("keyframe_lag_s", 0.15);
    max_v_ = positive("max_measured_linear_mps", 1.5);
    max_w_ = positive("max_measured_angular_radps", 2.5);
    auto q = declare_parameter<std::vector<double>>("base_from_imu_rotation", {0, 0, 0, 1});
    if (q.size() != 4 || !std::all_of(q.begin(), q.end(), [](double x) {return std::isfinite(x);})) {
      throw std::invalid_argument("invalid IMU rotation");
    }
    tf2::Quaternion quat(q[0], q[1], q[2], q[3]);
    if (std::abs(quat.length() - 1) > 0.01) {throw std::invalid_argument("IMU rotation not normalized");}
    imu_rotation_ = tf2::Matrix3x3(quat.normalized());

    pub_ = create_publisher<Odom>(declare_parameter<std::string>("output_topic", "/d455_slam/odom"), 10);
    // Same health topics as degraded_odometry, so watchdog, interface monitor
    // and guard (bounded speed while BRIDGING) need no change.
    health_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/continuous_odom_healthy", 10);
    degraded_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/odom_degraded", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/r680_nav/continuous_odom_status", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/r680_nav/odom_state", 10);
    epoch_pub_ = create_publisher<std_msgs::msg::UInt32>("/r680_nav/odom_epoch",
      rclcpp::QoS(1).reliable().transient_local());
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto qos = rclcpp::SensorDataQoS().keep_last(5);
    raw_sub_ = create_subscription<Odom>(
      declare_parameter<std::string>("raw_odom_topic", "/r680_nav/vio_raw_odom"), qos,
      [this](Odom::ConstSharedPtr m) {onRaw(*m);});
    quality_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      declare_parameter<std::string>("quality_topic", "/r680_nav/vio_visual_quality"), 10,
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) {
        if (m->data.size() >= 2) {
          observations_ = static_cast<int>(m->data[0]); coverage_ = m->data[1]; quality_seen_ = Clock::now();
        }
      });
    visual_sub_ = create_subscription<std_msgs::msg::Bool>("/r680_nav/vio_visual_observed", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {visual_observed_ = m->data; visual_seen_ = Clock::now();});
    health_sub_ = create_subscription<std_msgs::msg::Bool>("/r680_nav/vio_tracking_healthy", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {tracking_ = m->data; tracking_seen_ = Clock::now();});
    wheel_sub_ = create_subscription<Odom>(
      declare_parameter<std::string>("wheel_topic", "/wheel/odom"), qos,
      [this](Odom::ConstSharedPtr m) {
        const auto & t = m->twist.twist;
        if (fresh(m->header.stamp, wheel_timeout_) && std::isfinite(t.linear.x + t.angular.z) &&
          std::abs(t.linear.x) <= max_v_ && std::abs(t.angular.z) <= max_w_)
        {
          v_ = t.linear.x; wheel_w_ = t.angular.z; wheel_seen_ = Clock::now();
        }
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/wheel/imu/data_raw"), qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {onImu(*m);});
    timer_ = create_wall_timer(10ms, [this] {tick();});
  }

private:
  fusion::Config config()
  {
    fusion::Config c;
    auto d = [this](const char * n, double & v) {v = declare_parameter<double>(n, v);};
    auto i = [this](const char * n, int & v) {v = static_cast<int>(declare_parameter<int64_t>(n, v));};
    d("keyframe_s", c.keyframe_s); d("window_s", c.window_s);
    d("wheel_sigma_min", c.wheel_sigma_min); d("wheel_sigma_per_m", c.wheel_sigma_per_m);
    d("gyro_noise", c.gyro_noise); d("bias_walk", c.bias_walk); d("bias_prior", c.bias_prior);
    d("vo_sigma_xy", c.vo_sigma_xy); d("vo_sigma_yaw", c.vo_sigma_yaw); d("zupt_sigma", c.zupt_sigma);
    d("stationary_v", c.stationary_v); d("stationary_w", c.stationary_w);
    i("min_observations", c.min_observations); i("ref_observations", c.ref_observations);
    d("ref_coverage", c.ref_coverage); d("visual_off", c.visual_off); d("visual_on", c.visual_on);
    i("visual_on_frames", c.visual_on_frames); d("visual_max_gap_s", c.visual_max_gap_s);
    d("max_sample_gap_s", c.max_sample_gap_s);
    d("vo_jump_m", c.vo_jump_m); d("vo_jump_rad", c.vo_jump_rad);
    d("slip_w", c.slip_w); d("slip_a", c.slip_a); d("slip_vo_m", c.slip_vo_m);
    d("slip_on", c.slip_on); d("slip_off", c.slip_off);
    i("slip_on_frames", c.slip_on_frames); i("slip_off_frames", c.slip_off_frames);
    d("slip_inflation", c.slip_inflation);
    d("wheel_scale_sigma", c.wheel_scale_sigma); d("bias_sigma", c.bias_sigma);
    d("max_sigma_xy", c.max_sigma_xy); d("max_sigma_yaw", c.max_sigma_yaw);
    return c;
  }
  double positive(const char * name, double value)
  {
    const double v = declare_parameter<double>(name, value);
    if (!std::isfinite(v) || v <= 0) {throw std::invalid_argument(name);}
    return v;
  }
  static double age(Clock::time_point t)
  {
    return t.time_since_epoch().count() == 0 ? 1e9 : std::chrono::duration<double>(Clock::now() - t).count();
  }
  bool fresh(const builtin_interfaces::msg::Time & stamp, double limit) const
  {
    const double a = (get_clock()->now() - rclcpp::Time(stamp)).seconds();
    return a >= -0.02 && a <= limit;
  }
  static double seconds(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}
  static double yaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }

  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const auto & g = m.angular_velocity;
    const auto & a = m.linear_acceleration;
    const double t = seconds(m.header.stamp);
    if (m.header.frame_id != imu_frame_ || !fresh(m.header.stamp, imu_timeout_) || t <= last_imu_ ||
      !std::isfinite(g.x + g.y + g.z + a.x + a.y + a.z) || std::hypot(g.x, g.y, g.z) > max_w_) {return;}
    last_imu_ = t;
    // Wheel speed is held between wheel messages; a stale wheel leaves a gap,
    // which the estimator treats as lost continuity.
    if (age(wheel_seen_) > wheel_timeout_) {return;}
    const auto gyro = imu_rotation_ * tf2::Vector3(g.x, g.y, g.z);
    const auto accel = imu_rotation_ * tf2::Vector3(a.x, a.y, a.z);
    est_.addSample({t, v_, wheel_w_, gyro.z(), accel.x()});
    last_sample_ = t;
    sample_seen_ = Clock::now();
  }

  void onRaw(const Odom & m)
  {
    const auto & p = m.pose.pose;
    const auto & q = p.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (m.header.frame_id != odom_frame_ || m.child_frame_id != base_frame_ ||
      !std::isfinite(p.position.x + p.position.y + p.position.z + norm) || std::abs(norm - 1) > 0.02 ||
      !fresh(m.header.stamp, visual_timeout_)) {return;}
    raw_ = m;
    raw_seen_ = Clock::now();
    const bool visual = visual_observed_ && age(visual_seen_) <= visual_timeout_ &&
      tracking_ && age(tracking_seen_) <= visual_timeout_;
    if (!visual) {return;}
    const bool quality = age(quality_seen_) <= 0.2;
    est_.addVisual({seconds(m.header.stamp), gtsam::Pose2(p.position.x, p.position.y, yaw(q)),
      quality ? observations_ : -1, quality ? coverage_ : -1.0});
  }

  void tick()
  {
    if (last_sample_ > 0 && age(sample_seen_) > input_timeout_) {est_.markStale("wheel/IMU input stale");}
    if (last_sample_ > 0) {
      const double ready = last_sample_ - lag_;
      if (auto k = est_.nextKeyframe()) {
        while (*k <= ready) {est_.step(*k); k = est_.nextKeyframe();}
      } else {
        est_.step(ready);
      }
    }
    const auto state = est_.state();
    if (state == fusion::State::Waiting) {
      // Before the first anchor keep the raw TF, so costmaps and the
      // restricted initialization chain do not wait on each other.
      if (age(raw_seen_) <= visual_timeout_ && rclcpp::Time(raw_.header.stamp).nanoseconds() > published_) {
        publish(raw_);
      }
    } else if (const auto out = est_.predict()) {
      publish(compose(*out));
    }
    std_msgs::msg::Bool healthy, degraded;
    healthy.data = state == fusion::State::Good || state == fusion::State::Bridging;
    degraded.data = state == fusion::State::Bridging;
    health_pub_->publish(healthy);
    degraded_pub_->publish(degraded);
    if (!epoch_sent_ || est_.epoch() != epoch_) {
      epoch_ = est_.epoch(); epoch_sent_ = true;
      std_msgs::msg::UInt32 e; e.data = epoch_; epoch_pub_->publish(e);
      if (epoch_ > 0) {RCLCPP_WARN(get_logger(), "odometry epoch %u: continuity was lost and re-anchored", epoch_);}
    }
    if (state != last_state_) {
      RCLCPP_WARN(get_logger(), "odometry state %s -> %s %s", fusion::name(last_state_),
        fusion::name(state), est_.reason().c_str());
      last_state_ = state;
    }
    if (++ticks_ % 10 == 0) {
      std_msgs::msg::String s;
      s.data = fusion::name(state);
      state_pub_->publish(s);
      std::ostringstream o;
      o << fusion::name(state) << " epoch=" << est_.epoch() << " bias=" << est_.bias()
        << " slip=" << est_.slipScore() << " slipping=" << est_.slipping()
        << " visual_score=" << est_.visualQuality() << " visual_active=" << est_.visualActive()
        << " sigma_xy=" << est_.sigmaXy() << " sigma_yaw=" << est_.sigmaYaw()
        << " visual_factors=" << est_.visualFactors() << " visual_jumps=" << est_.visualJumps()
        << " reason=" << est_.reason();
      s.data = o.str();
      status_pub_->publish(s);
    }
  }

  Odom compose(const fusion::Output & out) const
  {
    Odom m;
    m.header.frame_id = odom_frame_;
    m.child_frame_id = base_frame_;
    m.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(out.t * 1e9)), RCL_ROS_TIME);
    m.pose.pose.position.x = out.pose.x();
    m.pose.pose.position.y = out.pose.y();
    // Height, roll and pitch stay cuVSLAM's (last value while blind); only the
    // planar part is fused.
    m.pose.pose.position.z = raw_.pose.pose.position.z;
    const auto & r = raw_.pose.pose.orientation;
    tf2::Quaternion raw(r.x, r.y, r.z, r.w), heading, own;
    if (raw.length2() < 0.5) {raw = tf2::Quaternion(0, 0, 0, 1);}
    heading.setRPY(0, 0, yaw(raw_.pose.pose.orientation));
    own.setRPY(0, 0, out.pose.theta());
    const auto q = (own * heading.inverse() * raw).normalized();
    m.pose.pose.orientation.x = q.x(); m.pose.pose.orientation.y = q.y();
    m.pose.pose.orientation.z = q.z(); m.pose.pose.orientation.w = q.w();
    // Nominal front-end covariance (see odometry_covariance.hpp) plus the
    // bridging uncertainty accumulated without vision.
    for (int i : {0, 7}) {m.pose.covariance[i] = 0.001 + out.sigma_xy * out.sigma_xy;}
    for (int i : {14, 21, 28}) {m.pose.covariance[i] = 0.001;}
    m.pose.covariance[35] = 0.001 + out.sigma_yaw * out.sigma_yaw;
    m.twist.twist.linear.x = out.v;
    m.twist.twist.angular.z = out.w;
    for (int i : {0, 7, 14, 21, 28, 35}) {m.twist.covariance[i] = 0.01;}
    return m;
  }

  void publish(const Odom & m)
  {
    const int64_t stamp = rclcpp::Time(m.header.stamp).nanoseconds();
    if (stamp <= published_) {return;}
    published_ = stamp;
    pub_->publish(m);
    geometry_msgs::msg::TransformStamped t;
    t.header = m.header;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = m.pose.pose.position.x;
    t.transform.translation.y = m.pose.pose.position.y;
    t.transform.translation.z = m.pose.pose.position.z;
    t.transform.rotation = m.pose.pose.orientation;
    tf_->sendTransform(t);
  }

  fusion::Estimator est_;
  std::string odom_frame_, base_frame_, imu_frame_;
  double wheel_timeout_{}, imu_timeout_{}, visual_timeout_{}, input_timeout_{}, lag_{}, max_v_{}, max_w_{};
  tf2::Matrix3x3 imu_rotation_;
  double v_{0}, wheel_w_{0}, coverage_{-1}, last_imu_{0}, last_sample_{0};
  int observations_{-1};
  bool visual_observed_{false}, tracking_{false}, epoch_sent_{false};
  uint32_t epoch_{0};
  uint64_t ticks_{0};
  int64_t published_{0};
  fusion::State last_state_{fusion::State::Waiting};
  Clock::time_point wheel_seen_{}, sample_seen_{}, raw_seen_{}, visual_seen_{}, tracking_seen_{}, quality_seen_{};
  Odom raw_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<Odom>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_, degraded_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr epoch_pub_;
  rclcpp::Subscription<Odom>::SharedPtr raw_sub_, wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr quality_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visual_sub_, health_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFusion>());
  rclcpp::shutdown();
  return 0;
}
