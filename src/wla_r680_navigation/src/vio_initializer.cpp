#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "wla_r680_navigation/vio_init_policy.hpp"

using namespace std::chrono_literals;
namespace wla = wla_r680_navigation;
class VioInitializer final : public rclcpp::Node {
public:
  VioInitializer() : Node("r680_vio_initializer"), boot_(Clock::now()) {
    config_.startup_timeout = parameter("startup_timeout_s", 45);
    config_.init_timeout = parameter("initialization_timeout_s", 20);
    config_.stationary_time = parameter("stationary_time_s", 0.8);
    config_.stop_time = parameter("stopping_time_s", 0.5);
    config_.motion_duration = parameter("motion_duration_s", 8);
    config_.target_travel = parameter("target_travel_m", 0.16);
    config_.max_travel = parameter("max_travel_m", 0.25);
    config_.max_radius = parameter("max_radius_m", 0.25);
    config_.max_yaw = parameter("max_yaw_rad", 0.35);
    config_.linear = parameter("linear_speed", 0.06);
    config_.angular = parameter("angular_speed", 0.20);
    config_.max_measured_linear = parameter("max_measured_linear", 0.12);
    config_.max_measured_angular = parameter("max_measured_angular", 0.35);
    if (config_.linear > 0.08 || config_.angular > 0.20 ||
      config_.target_travel >= config_.max_travel || config_.max_radius > 0.25 ||
      config_.max_travel > 0.25 || config_.max_yaw > 0.35 || config_.init_timeout > 20 ||
      config_.motion_duration > config_.init_timeout) throw std::invalid_argument("initialization envelope exceeds trial limits");
    policy_ = std::make_unique<wla::InitPolicy>(config_);
    freshness_ = parameter("sensor_timeout_s", 0.35);
    min_rate_ = parameter("minimum_imu_rate", 100);
    min_valid_ = parameter("minimum_depth_valid_ratio", 0.35);
    obstacle_forward_ = parameter("obstacle_forward_distance", 0.65);
    obstacle_width_ = parameter("obstacle_half_width", 0.40);
    stopped_linear_ = parameter("stopped_linear", 0.015);
    stopped_angular_ = parameter("stopped_angular", 0.04);
    if (freshness_ > 0.5 || min_valid_ > 1) throw std::invalid_argument("invalid input gate");
    base_ = declare_parameter<std::string>("base_frame", "r680_mapping_floor");
    result_ = declare_parameter<std::string>("result_path", "/tmp/r680_vio_init.json");
    auto qos = rclcpp::SensorDataQoS();
    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("odom_topic", "/d455_slam/odom"), qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onPose(*m, true);});
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom"), qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onPose(*m, false);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/wheel/imu/data_raw"), qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        const auto & a = m->linear_acceleration; const auto & w = m->angular_velocity;
        const double t = rclcpp::Time(m->header.stamp).seconds();
        const double norm = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
        if (!stampFresh(m->header.stamp) || m->header.frame_id != "gyro_link" ||
          !std::isfinite(norm + w.x + w.y + w.z) || norm < 7 || norm > 13 ||
          (!imu_stamps_.empty() && t <= imu_stamps_.back())) {imu_seen_ = {}; reject("invalid_imu"); return;}
        imu_stamps_.push_back(t);
        while (imu_stamps_.size() > 2 && t - imu_stamps_.front() > 0.5) imu_stamps_.pop_front();
        imu_seen_ = Clock::now();
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/r680/d455/aligned_depth_to_color/image_raw", qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onDepth(*m);});
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/r680_nav/d455/points_safety", qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        bool xyz_valid = true;
        for (const auto * name : {"x", "y", "z"}) {
          const auto field = std::find_if(m->fields.begin(), m->fields.end(),
            [name](const auto & f) {return f.name == name;});
          xyz_valid &= field != m->fields.end() && field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
            field->count == 1 && field->offset + sizeof(float) <= m->point_step;
        }
        if (!stampFresh(m->header.stamp) || m->header.frame_id != base_ || m->point_step == 0 ||
          !xyz_valid || m->is_bigendian || m->height != 1 ||
          m->row_step != static_cast<size_t>(m->width)*m->point_step || m->data.size() != m->row_step) {
          points_seen_ = {}; reject("invalid_safety_cloud"); return;
        }
        bool blocked = false;
        size_t blocking_points = 0;
        std::vector<double> first_blocking_point;
        try {
          sensor_msgs::PointCloud2ConstIterator<float> x(*m, "x"), y(*m, "y"), z(*m, "z");
          for (; x != x.end(); ++x, ++y, ++z) {
            if (!std::isfinite(*x + *y + *z)) {points_seen_ = {}; reject("invalid_safety_point"); return;}
            if (*x > 0.20 && *x < obstacle_forward_ && std::abs(*y) < obstacle_width_ &&
              *z >= 0.04 && *z <= 0.50) {
              blocked = true; ++blocking_points;
              if (first_blocking_point.empty()) first_blocking_point = {*x, *y, *z};
            }
          }
        } catch (const std::exception &) {points_seen_ = {}; reject("malformed_safety_cloud"); return;}
        blocked_ = blocked; blocking_points_ = blocking_points;
        first_blocking_point_ = first_blocking_point; points_seen_ = Clock::now();
      });
    vio_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/r680_nav/vio_status", 1, [this](std_msgs::msg::String::ConstSharedPtr m) {
        const bool tracking = m->data == "tracking_inertial_ready" ||
          m->data == "waiting_for_stable_inertial_initialization";
        if (tracking) sdk_seen_ = Clock::now();
        else {sdk_seen_ = {}; reject("frontend_reset_or_tracking_fault");}
      });
    vio_health_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/r680_nav/vio_tracking_healthy", 1, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        inertial_ = m->data; inertial_seen_ = Clock::now();
      });
    localization_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/r680_nav/localization_ready", 1, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        localized_ = m->data; localized_seen_ = Clock::now();
      });
    nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/r680_nav/nav_command_input", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        if (policy_->state() == wla::InitPolicy::State::Succeeded) {nav_ = *m; nav_seen_ = Clock::now();}
      });
    cancel_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/r680_nav/vio_init_cancel", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data) {policy_->fail("operator_cancelled"); nav_seen_ = {};}
      });
    mux_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 1);
    request_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/r680_nav/vio_init_request", 1);
    permit_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/vio_init_permit", 1);
    state_pub_ = create_publisher<std_msgs::msg::String>("/r680_nav/vio_init_state", 1);
    for (const auto & name : {"velocity_smoother", "collision_monitor"}) {
      clients_.push_back(create_client<lifecycle_msgs::srv::GetState>(std::string("/") + name + "/get_state"));
    }
    service_seen_.resize(2); service_active_.resize(2, false); pending_.resize(2, false);
    poll_ = create_wall_timer(250ms, [this]() {pollServices();});
    tick_ = create_wall_timer(50ms, [this]() {tick();});
    RCLCPP_WARN(get_logger(), "Automatic VIO initialization trial: requests <=0.08m/s, <=0.20rad/s, no reverse; only final guard owns chassis output");
  }
private:
  using Clock = std::chrono::steady_clock;
  struct Pose {double x{0}, y{0}, yaw{0}, stamp{-1};};
  double parameter(const char * name, double value) {
    const auto p = declare_parameter<double>(name, value);
    if (!std::isfinite(p) || p <= 0) throw std::invalid_argument(std::string("invalid ") + name);
    return p;
  }
  bool fresh(Clock::time_point p, double limit = -1) const {
    return p.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(Clock::now() - p).count() <= (limit < 0 ? freshness_ : limit);
  }
  bool stampFresh(const builtin_interfaces::msg::Time & stamp) const {
    const double age = (now() - rclcpp::Time(stamp)).seconds();
    return age >= -0.1 && age <= freshness_;
  }
  static double angle(double a) {return std::atan2(std::sin(a), std::cos(a));}
  void reject(const std::string & why) {
    const auto s = policy_->state();
    if (s == wla::InitPolicy::State::Moving || s == wla::InitPolicy::State::Stopping ||
      s == wla::InitPolicy::State::Localization) policy_->fail(why);
  }
  void onPose(const nav_msgs::msg::Odometry & m, bool vo) {
    const auto & p = m.pose.pose.position; const auto & q = m.pose.pose.orientation;
    const double qnorm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    const double t = rclcpp::Time(m.header.stamp).seconds();
    const double v = m.twist.twist.linear.x, w = m.twist.twist.angular.z;
    auto & seen = vo ? vo_seen_ : wheel_seen_;
    auto & previous = vo ? vo_ : wheel_;
    if (!stampFresh(m.header.stamp) || !std::isfinite(p.x+p.y+p.z+qnorm+v+w) ||
      qnorm < 0.98 || qnorm > 1.02 || m.header.frame_id.empty() ||
      (vo && (m.header.frame_id != "d455_floor_odom" || m.child_frame_id != base_)) || t <= previous.stamp) {
      seen = {}; reject("invalid_or_reordered_odometry"); return;
    }
    Pose next{p.x, p.y, std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), t};
    if (previous.stamp >= 0) {
      const double dt = t - previous.stamp;
      const double distance = std::hypot(next.x-previous.x, next.y-previous.y);
      if (dt < 0.5 && (distance > 0.05 + 2*dt || std::abs(angle(next.yaw-previous.yaw)) > 0.10 + 3*dt)) {
        seen = {}; reject("odometry_jump"); return;
      }
      if (!vo && anchored_ && policy_->state() != wla::InitPolicy::State::Succeeded) travel_ += distance;
    }
    previous = next; seen = Clock::now();
    if (!vo) {wheel_v_ = v; wheel_w_ = w;}
  }
  void onDepth(const sensor_msgs::msg::Image & m) {
    const bool u16 = m.encoding == "16UC1" || m.encoding == "mono16";
    const bool f32 = m.encoding == "32FC1";
    const size_t bytes = u16 ? 2 : 4;
    if (!stampFresh(m.header.stamp) || (!u16 && !f32) || m.width < 16 || m.height < 16 ||
      m.is_bigendian || m.step < m.width * bytes || m.data.size() < m.height * static_cast<size_t>(m.step)) {
      depth_seen_ = {}; reject("invalid_depth_image"); return;
    }
    size_t total = 0, valid = 0;
    for (size_t v = m.height * 3 / 10; v < m.height * 9 / 10; v += 8) {
      for (size_t u = m.width * 15 / 100; u < m.width * 85 / 100; u += 8) {
        const auto * pixel = m.data.data() + v*m.step + u*bytes;
        float d;
        if (u16) {uint16_t raw; std::memcpy(&raw, pixel, 2); d = raw * 0.001F;}
        else std::memcpy(&d, pixel, 4);
        ++total; if (std::isfinite(d) && d >= 0.15F && d <= 4.5F) ++valid;
      }
    }
    depth_ratio_ = total ? static_cast<double>(valid)/total : 0;
    depth_seen_ = Clock::now();
  }
  void pollServices() {
    for (size_t i = 0; i < clients_.size(); ++i) {
      if (pending_[i] || !clients_[i]->service_is_ready()) continue;
      pending_[i] = true;
      clients_[i]->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
        [this, i](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture f) {
          pending_[i] = false;
          try {service_active_[i] = f.get()->current_state.id == 3; service_seen_[i] = Clock::now();}
          catch (const std::exception &) {service_active_[i] = false;}
        });
    }
  }
  std::string inputBlocker() const {
    if (!last_inputs_.sensors) return "sensor_missing_stale_or_invalid";
    if (!last_inputs_.services) return "safety_lifecycle_not_active";
    if (blocked_) return "near_field_obstacle";
    if (depth_ratio_ < min_valid_) return "insufficient_valid_depth";
    if (!last_inputs_.stopped) return "wheel_not_stationary";
    return "";
  }
  void writeResult() {
    try {
      const auto path = std::filesystem::path(result_);
      if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
      const auto temporary = result_ + ".tmp";
      std::ofstream out(temporary);
      out << std::setprecision(9) << "{\n  \"state\": " << std::quoted(policy_->name())
        << ",\n  \"reason\": " << std::quoted(policy_->reason())
        << ",\n  \"input_blocker\": " << std::quoted(inputBlocker())
        << ",\n  \"blocking_point_count\": " << blocking_points_
        << ",\n  \"blocking_cloud_fresh\": " << (fresh(points_seen_) ? "true" : "false")
        << ",\n  \"first_blocking_point_xyz\": [";
      for (size_t i = 0; i < first_blocking_point_.size(); ++i) {
        if (i) {out << ", ";}
        out << first_blocking_point_[i];
      }
      out << "]"
        << ",\n  \"requested_motion\": " << (ever_moved_ ? "true" : "false")
        << ",\n  \"wheel_travel_m\": " << travel_ << ",\n  \"radius_m\": " << radius_
        << ",\n  \"depth_valid_ratio\": " << depth_ratio_ << ",\n  \"imu_rate_hz\": " << imu_rate_
        << ",\n  \"inertial_ready\": " << (last_inputs_.inertial ? "true" : "false")
        << ",\n  \"mapping_or_localization_ready\": " << (last_inputs_.localized ? "true" : "false")
        << std::boolalpha << ",\n  \"gates\": {\n"
        << "    \"sensors_ready\": " << last_inputs_.sensors
        << ",\n    \"safety_lifecycle_active\": " << last_inputs_.services
        << ",\n    \"near_field_clear\": " << last_inputs_.clear
        << ",\n    \"cloud_blocked\": " << blocked_
        << ",\n    \"wheel_stopped\": " << last_inputs_.stopped
        << ",\n    \"vo_fresh\": " << fresh(vo_seen_)
        << ",\n    \"wheel_fresh\": " << fresh(wheel_seen_)
        << ",\n    \"imu_fresh\": " << fresh(imu_seen_)
        << ",\n    \"sdk_tracking_fresh\": " << fresh(sdk_seen_)
        << ",\n    \"depth_fresh\": " << fresh(depth_seen_)
        << ",\n    \"safety_cloud_fresh\": " << fresh(points_seen_)
        << ",\n    \"inertial_health_fresh\": " << fresh(inertial_seen_)
        << "\n  }\n}\n";
      out.close(); std::filesystem::rename(temporary, path);
    } catch (const std::exception & ex) {RCLCPP_ERROR(get_logger(), "cannot save init result: %s", ex.what());}
  }
  void tick() {
    imu_rate_ = imu_stamps_.size() > 2 ? (imu_stamps_.size()-1)/(imu_stamps_.back()-imu_stamps_.front()) : 0;
    wla::InitInput input;
    input.sensors = fresh(vo_seen_) && fresh(wheel_seen_) && fresh(imu_seen_) && fresh(sdk_seen_) &&
      fresh(depth_seen_) && fresh(points_seen_) && fresh(inertial_seen_) && imu_rate_ >= min_rate_;
    input.services = true;
    for (size_t i = 0; i < service_seen_.size(); ++i) input.services &= service_active_[i] && fresh(service_seen_[i], 1.0);
    input.clear = !blocked_ && depth_ratio_ >= min_valid_;
    input.stopped = std::abs(wheel_v_) < stopped_linear_ && std::abs(wheel_w_) < stopped_angular_;
    input.inertial = inertial_ && fresh(inertial_seen_);
    input.localized = localized_ && fresh(localized_seen_);
    input.speed = wheel_v_; input.angular_speed = wheel_w_;
    last_inputs_ = input;
    if (anchored_) {
      radius_ = std::max(std::hypot(vo_.x-anchor_vo_.x, vo_.y-anchor_vo_.y),
        std::hypot(wheel_.x-anchor_wheel_.x, wheel_.y-anchor_wheel_.y));
      input.radius = radius_; input.travel = travel_;
      input.yaw = std::max(std::abs(angle(vo_.yaw-anchor_vo_.yaw)), std::abs(angle(wheel_.yaw-anchor_wheel_.yaw)));
    }
    const auto previous = policy_->state();
    const auto command = policy_->tick(std::chrono::duration<double>(Clock::now()-boot_).count(), input);
    if (policy_->state() == wla::InitPolicy::State::Moving && !anchored_) {
      anchor_vo_ = vo_; anchor_wheel_ = wheel_; travel_ = 0; anchored_ = true;
    }
    if (policy_->state() != previous) {
      nav_seen_ = {}; // Never replay a command buffered during initialization.
      RCLCPP_WARN(get_logger(), "VIO init state: %s %s; input blocker=%s, near-field points=%zu",
        policy_->name(), policy_->reason().c_str(), inputBlocker().c_str(), blocking_points_);
    }
    geometry_msgs::msg::Twist output;
    output.linear.x = command.linear; output.angular.z = command.angular;
    geometry_msgs::msg::TwistStamped request;
    request.header.stamp = now(); request.header.frame_id = base_; request.twist = output;
    request_pub_->publish(request);
    std_msgs::msg::Bool permit;
    permit.data = policy_->state() == wla::InitPolicy::State::Moving && input.sensors && input.services && input.clear;
    permit_pub_->publish(permit);
    std_msgs::msg::String state;
    state.data = policy_->name(); state_pub_->publish(state);
    ever_moved_ |= command.linear != 0 || command.angular != 0;
    if (policy_->state() == wla::InitPolicy::State::Succeeded && fresh(nav_seen_, 0.20)) output = nav_;
    mux_pub_->publish(output);
    if (policy_->state() != previous || ++ticks_ % 20 == 0) writeResult();
  }
  Clock::time_point boot_, vo_seen_{}, wheel_seen_{}, imu_seen_{}, sdk_seen_{}, depth_seen_{}, points_seen_{};
  Clock::time_point inertial_seen_{}, localized_seen_{}, nav_seen_{};
  wla::InitConfig config_; std::unique_ptr<wla::InitPolicy> policy_;
  wla::InitInput last_inputs_;
  std::string base_, result_;
  double freshness_{}, min_rate_{}, min_valid_{}, obstacle_forward_{}, obstacle_width_{}, stopped_linear_{}, stopped_angular_{};
  double imu_rate_{}, depth_ratio_{}, radius_{}, travel_{}, wheel_v_{}, wheel_w_{};
  bool blocked_{true}, inertial_{false}, localized_{false}, anchored_{false}, ever_moved_{false};
  size_t ticks_{}, blocking_points_{}; Pose vo_, wheel_, anchor_vo_, anchor_wheel_;
  std::vector<double> first_blocking_point_;
  std::deque<double> imu_stamps_;
  geometry_msgs::msg::Twist nav_;
  std::vector<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> clients_;
  std::vector<Clock::time_point> service_seen_;
  std::vector<bool> service_active_, pending_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_, wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vio_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vio_health_sub_, localization_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mux_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr request_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr permit_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr poll_, tick_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<VioInitializer>());}
  catch (const std::exception & ex) {RCLCPP_FATAL(rclcpp::get_logger("r680_vio_initializer"), "%s", ex.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
