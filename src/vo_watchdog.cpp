#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <unistd.h>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtabmap_msgs/msg/info.hpp"
#include "rtabmap_msgs/msg/odom_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wla_r680_navigation/relocalization_prior.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class VoWatchdog final : public rclcpp::Node
{
public:
  VoWatchdog() : Node("r680_vo_watchdog"), started_(Clock::now())
  {
    frontend_ = declare_parameter<std::string>("frontend", "rgbd");
    if (frontend_ != "rgbd" && frontend_ != "cuvslam") throw std::invalid_argument("unknown frontend");
    startup_grace_s_ = declare_parameter<double>("startup_grace_s", 12.0);
    stale_s_ = declare_parameter<double>("stale_s", 1.5);
    lost_limit_ = declare_parameter<int>("lost_limit", 3);
    jump_linear_mps_ = declare_parameter<double>("jump_linear_mps", 2.5);
    jump_angular_rps_ = declare_parameter<double>("jump_angular_rps", 3.0);
    wheel_window_s_ = declare_parameter<double>("wheel_window_s", 0.5);
    wheel_translation_error_m_ = declare_parameter<double>("wheel_translation_error_m", 0.45);
    wheel_yaw_error_rad_ = declare_parameter<double>("wheel_yaw_error_rad", 0.65);
    wheel_mismatch_limit_ = declare_parameter<int>("wheel_mismatch_limit", 3);
    compare_wheel_yaw_ = declare_parameter<bool>("compare_wheel_yaw", false);
    restart_enabled_ = declare_parameter<bool>("restart_enabled", true);
    recovery_stable_s_ = declare_parameter<double>("recovery_stable_s", 2.0);
    localization_stale_s_ = declare_parameter<double>("localization_stale_s", 3.0);
    localization_translation_error_m_ =
      declare_parameter<double>("localization_translation_error_m", 0.40);
    localization_yaw_change_rad_ =
      declare_parameter<double>("localization_yaw_change_rad", 0.80);
    prior_enabled_ = declare_parameter<bool>("relocalization_prior_enabled", true);
    prior_max_travel_ = declare_parameter<double>("prior_max_translation_m", 0.50);
    prior_max_yaw_ = declare_parameter<double>("prior_max_yaw_rad", 0.80);
    prior_stop_time_ = declare_parameter<double>("prior_stop_stable_s", 0.50);
    prior_wait_time_ = declare_parameter<double>("prior_tracking_stable_s", 1.0);
    prior_max_wait_ = declare_parameter<double>("prior_max_wait_s", 20.0);
    const auto initial_pose_topic = declare_parameter<std::string>(
      "initial_pose_topic", "/d455_slam/initialpose");
    if (!std::isfinite(prior_max_travel_+prior_max_yaw_+prior_stop_time_+
        prior_wait_time_+prior_max_wait_) || prior_max_travel_ <= 0 ||
        prior_max_yaw_ <= 0 || prior_stop_time_ <= 0 || prior_wait_time_ <= 0 ||
        prior_max_wait_ <= prior_wait_time_) throw std::invalid_argument("invalid recovery prior limits");
    const auto vo_topic = declare_parameter<std::string>("vo_topic", "/r680_nav/vo_odom");
    const auto info_topic = declare_parameter<std::string>("info_topic", "/d455_slam/odom_info");
    const auto wheel_topic = declare_parameter<std::string>("wheel_topic", "/wheel/odom");
    const auto localization_topic = declare_parameter<std::string>(
      "localization_topic", "/d455_slam/localization_pose");
    const auto map_info_topic = declare_parameter<std::string>(
      "map_info_topic", "/d455_slam/info");

    if (startup_grace_s_ <= 0 || stale_s_ <= 0 || lost_limit_ < 1 ||
      jump_linear_mps_ <= 0 || jump_angular_rps_ <= 0 || wheel_window_s_ <= 0 ||
      wheel_translation_error_m_ <= 0 || wheel_yaw_error_rad_ <= 0 ||
      wheel_mismatch_limit_ < 1 || recovery_stable_s_ <= 0 ||
      localization_stale_s_ <= 0 || localization_translation_error_m_ <= 0 ||
      localization_yaw_change_rad_ <= 0) {
      throw std::invalid_argument("invalid VO watchdog thresholds");
    }

    health_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/vo_watchdog_healthy", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/r680_nav/vo_watchdog_status", 10);
    prior_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initial_pose_topic, 1);
    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vo_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onVo(*msg);});
    if (frontend_ == "rgbd") {
      info_sub_ = create_subscription<rtabmap_msgs::msg::OdomInfo>(
        info_topic, rclcpp::SensorDataQoS(),
        [this](rtabmap_msgs::msg::OdomInfo::ConstSharedPtr msg) {onInfo(*msg);});
    } else {
      frontend_health_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/r680_nav/vio_tracking_healthy", 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (fault_latched_ && respawn_seen_.time_since_epoch().count() == 0) return;
          info_seen_ = Clock::now();
          frontend_ready_ = msg->data;
          if (msg->data) frontend_was_ready_ = true;
          lost_count_ = msg->data ? 0 : lost_count_ + 1;
          // Initial SDK gravity estimation is a waiting state, not tracking loss.
          if (!fault_latched_ && frontend_was_ready_ && lost_count_ >= lost_limit_)
            fault("cuVSLAM inertial tracking unhealthy");
        });
    }
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const double stamp_age=(get_clock()->now()-rclcpp::Time(msg->header.stamp)).seconds();
        if (finitePose(*msg) && stamp_age>=-0.2 && stamp_age<0.30 &&
            std::isfinite(msg->twist.twist.linear.x+msg->twist.twist.angular.z)) {
          wheel_ = poseOf(*msg);
          wheel_seen_ = Clock::now();
          wheel_linear_ = msg->twist.twist.linear.x;
          wheel_angular_ = msg->twist.twist.angular.z;
        }
      });
    localization_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      localization_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        onLocalization(*msg);
      });
    map_info_sub_ = create_subscription<rtabmap_msgs::msg::Info>(
      map_info_topic, rclcpp::SensorDataQoS(),
      [this](rtabmap_msgs::msg::Info::ConstSharedPtr msg) {
        if (msg->loop_closure_id > 0 || msg->proximity_detection_id > 0 || msg->landmark_id > 0) {
          initial_map_matched_ = true;
        }
        if (fault_latched_ && respawn_seen_.time_since_epoch().count() != 0 &&
          rclcpp::Time(msg->header.stamp) > respawn_ros_stamp_ &&
          (msg->loop_closure_id > 0 || msg->proximity_detection_id > 0 ||
          msg->landmark_id > 0)) {
          map_matched_after_respawn_ = true;
        }
      });
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(100ms, [this]() {tick();});
    RCLCPP_INFO(get_logger(), "VO watchdog armed; restart=%s", restart_enabled_ ? "on" : "off");
  }

private:
  using Clock = std::chrono::steady_clock;
  struct Pose {double x, y, yaw;};

  static bool finitePose(const geometry_msgs::msg::Pose & pose)
  {
    const auto & p = pose.position;
    const auto & q = pose.orientation;
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w) &&
           (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) > 0.5;
  }

  static bool finitePose(const nav_msgs::msg::Odometry & msg)
  {
    return finitePose(msg.pose.pose);
  }

  static Pose poseOf(const geometry_msgs::msg::Pose & pose)
  {
    const auto & p = pose.position;
    const auto & q = pose.orientation;
    return {p.x, p.y, std::atan2(2.0*(q.w*q.z + q.x*q.y),
      1.0 - 2.0*(q.y*q.y + q.z*q.z))};
  }

  static Pose poseOf(const nav_msgs::msg::Odometry & msg)
  {
    return poseOf(msg.pose.pose);
  }

  static double angleDifference(double a, double b)
  {
    return std::atan2(std::sin(a-b), std::cos(a-b));
  }

  static double distance(const Pose & a, const Pose & b)
  {
    return std::hypot(a.x-b.x, a.y-b.y);
  }

  static double age(const Clock::time_point & t)
  {
    if (t.time_since_epoch().count() == 0) {return 1e9;}
    return std::chrono::duration<double>(Clock::now()-t).count();
  }

  std::optional<Pose> currentMapPose() const
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        "map", "r680_mapping_floor", tf2::TimePointZero);
      const double stamp_age =
        (get_clock()->now() - rclcpp::Time(transform.header.stamp)).seconds();
      if (stamp_age < -0.2 || stamp_age > localization_stale_s_) {return std::nullopt;}
      const auto & p = transform.transform.translation;
      const auto & q = transform.transform.rotation;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w)) {return std::nullopt;}
      return Pose{p.x, p.y, std::atan2(2.0*(q.w*q.z + q.x*q.y),
        1.0 - 2.0*(q.y*q.y + q.z*q.z))};
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void onLocalization(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    if (msg.header.frame_id != "map" || !finitePose(msg.pose.pose)) {return;}
    const auto & covariance = msg.pose.covariance;
    if (!std::isfinite(covariance[0]) || !std::isfinite(covariance[7]) ||
      !std::isfinite(covariance[35]) || covariance[0] < 0 ||
      covariance[7] < 0 || covariance[35] < 0 ||
      covariance[0] > 1.0 || covariance[7] > 1.0 || covariance[35] > 1.0) {return;}
    const double stamp_age = (get_clock()->now() - rclcpp::Time(msg.header.stamp)).seconds();
    if (stamp_age < -0.2 || stamp_age > localization_stale_s_) {return;}
    initial_localization_seen_ = Clock::now();
    if (fault_latched_) {
      if (respawn_seen_.time_since_epoch().count() != 0 &&
        rclcpp::Time(msg.header.stamp) > respawn_ros_stamp_) {
        recovery_map_pose_ = poseOf(msg.pose.pose);
        recovery_map_seen_ = Clock::now();
      }
    }
  }

  void onInfo(const rtabmap_msgs::msg::OdomInfo & msg)
  {
    if (fault_latched_ && respawn_seen_.time_since_epoch().count() == 0) {return;}
    info_seen_ = Clock::now();
    lost_count_ = msg.lost ? lost_count_ + 1 : 0;
    if (!fault_latched_ && lost_count_ >= lost_limit_) {fault("visual tracking lost");}
  }

  void onVo(const nav_msgs::msg::Odometry & msg)
  {
    if (fault_latched_ && respawn_seen_.time_since_epoch().count() == 0) {return;}
    if (!finitePose(msg)) {
      if (fault_latched_) {vo_seen_ = {}; recovery_stable_since_ = {};}
      else {fault("non-finite VO pose");}
      return;
    }
    const auto stamp = rclcpp::Time(msg.header.stamp);
    const auto pose = poseOf(msg);
    if (last_vo_) {
      const double dt = (stamp - last_stamp_).seconds();
      if (dt > 0.0 && dt < 0.5) {
        if (distance(pose, *last_vo_) > 0.15 + jump_linear_mps_*dt ||
          std::abs(angleDifference(pose.yaw, last_vo_->yaw)) > 0.20 + jump_angular_rps_*dt) {
          if (fault_latched_) {vo_seen_ = {}; recovery_stable_since_ = {};}
          else {fault("VO pose discontinuity");}
          last_vo_ = pose;
          last_stamp_ = stamp;
          return;
        }
      }
    }
    vo_seen_ = Clock::now();
    last_vo_ = pose;
    last_stamp_ = stamp;
    if (fault_latched_) {return;}

    // Compare movement over windows, not single frames; wheel odometry is an
    // independent diagnostic, not a replacement for camera localization.
    if (age(wheel_seen_) > 0.30) {window_vo_.reset(); mismatch_count_ = 0; return;}
    if (!window_vo_) {
      window_vo_ = pose;
      window_wheel_ = wheel_;
      window_started_ = Clock::now();
      return;
    }
    if (age(window_started_) < wheel_window_s_) {return;}
    const bool mismatch =
      std::abs(distance(pose, *window_vo_) - distance(wheel_, window_wheel_)) >
        wheel_translation_error_m_ ||
      (compare_wheel_yaw_ &&
       std::abs(angleDifference(
         angleDifference(pose.yaw, window_vo_->yaw),
         angleDifference(wheel_.yaw, window_wheel_.yaw))) > wheel_yaw_error_rad_);
    mismatch_count_ = mismatch ? mismatch_count_ + 1 : 0;
    window_vo_ = pose;
    window_wheel_ = wheel_;
    window_started_ = Clock::now();
    if (mismatch_count_ >= wheel_mismatch_limit_) {fault("VO/wheel motion disagreement");}
  }

  std::vector<pid_t> voPids() const
  {
    std::vector<pid_t> result;
    for (const auto & entry : fs::directory_iterator("/proc")) {
      const auto name = entry.path().filename().string();
      if (name.empty() || !std::all_of(name.begin(), name.end(), ::isdigit)) {continue;}
      std::error_code ec;
      const auto exe = fs::read_symlink(entry.path() / "exe", ec).string();
      const auto expected = frontend_ == "cuvslam" ?
        "/wla_cuvslam_navigation/cuvslam_odometry" : "/rtabmap_odom/rgbd_odometry";
      if (ec || exe.size() < std::string(expected).size() ||
        exe.compare(exe.size() - std::string(expected).size(), std::string(expected).size(), expected) != 0) {continue;}
      std::ifstream command(entry.path() / "cmdline", std::ios::binary);
      std::string args((std::istreambuf_iterator<char>(command)), std::istreambuf_iterator<char>());
      const auto ns = frontend_ == "cuvslam" ? "__ns:=/d455_vio" : "__ns:=/d455_vo";
      const auto node = frontend_ == "cuvslam" ? "__node:=cuvslam_odometry" : "__node:=rgbd_odometry";
      if (args.find(ns) != std::string::npos && args.find(node) != std::string::npos) {
        result.push_back(static_cast<pid_t>(std::stoi(name)));
      }
    }
    return result;
  }

  void fault(const std::string & why)
  {
    if (fault_latched_) {return;}
    fault_latched_ = true;
    reason_ = why;
    anchor_valid_at_fault_ = anchor_map_pose_.has_value() &&
      age(anchor_map_seen_) < localization_stale_s_ && age(wheel_seen_) < 0.30;
    respawn_seen_ = {};
    recovery_map_seen_ = {};
    map_matched_after_respawn_ = false;
    recovery_map_pose_.reset();
    recovery_stable_since_ = {};
    prior_sent_ = false;
    prior_stopped_since_ = {};
    prior_tracking_since_ = {};
    vo_seen_ = {};
    info_seen_ = {};
    last_vo_.reset();
    lost_count_ = 0;
    frontend_ready_ = false;
    frontend_was_ready_ = false;
    std_msgs::msg::Bool unhealthy;
    unhealthy.data = false;
    health_pub_->publish(unhealthy);
    RCLCPP_ERROR(get_logger(), "VO fault: %s; motion locked until VO and map relocalize",
      why.c_str());
    if (!restart_enabled_) {return;}
    const auto pids = voPids();
    if (pids.size() != 1) {
      RCLCPP_ERROR(get_logger(), "expected exactly one %s frontend process, found %zu",
        frontend_.c_str(), pids.size());
      return;
    }
    vo_pid_ = pids.front();
    if (!anchor_valid_at_fault_) {
      RCLCPP_ERROR(get_logger(), "no recent map/odometry anchor; automatic motion recovery disabled");
    }
    if (::kill(vo_pid_, SIGTERM) == 0) {
      RCLCPP_WARN(get_logger(), "sent SIGTERM to VO pid %d; launch will respawn it", vo_pid_);
    } else {
      RCLCPP_ERROR(get_logger(), "could not signal VO pid %d", vo_pid_);
    }
  }

  void tick()
  {
    if (!fault_latched_ && (frontend_ == "rgbd" || frontend_ready_) && age(vo_seen_) <= stale_s_ &&
      age(info_seen_) <= stale_s_ && age(wheel_seen_) < 0.30) {
      const auto map_pose = currentMapPose();
      if (map_pose) {
        anchor_map_pose_ = map_pose;
        anchor_wheel_pose_ = wheel_;
        anchor_map_seen_ = Clock::now();
      }
    }
    if (fault_latched_ && restart_enabled_ && vo_pid_ > 0) {
      if (respawn_seen_.time_since_epoch().count() == 0) {
        const auto pids = voPids();
        if (pids.size() == 1 && pids.front() != vo_pid_) {
          respawn_seen_ = Clock::now();
          respawn_ros_stamp_ = get_clock()->now();
          vo_seen_ = {};
          info_seen_ = {};
          map_matched_after_respawn_ = false;
          last_vo_.reset();
          recovery_stable_since_ = {};
          RCLCPP_WARN(get_logger(), "new VO process %d detected; waiting for relocalization",
            pids.front());
        }
      } else {
        const bool tracking = age(vo_seen_) <= stale_s_ &&
          age(info_seen_) <= stale_s_ && lost_count_ == 0 &&
          (frontend_ == "rgbd" || frontend_ready_);
        const bool stopped = age(wheel_seen_) < 0.30 &&
          std::isfinite(wheel_linear_+wheel_angular_) &&
          std::abs(wheel_linear_) < 0.02 && std::abs(wheel_angular_) < 0.04;
        if (!stopped) prior_stopped_since_ = {};
        else if (prior_stopped_since_.time_since_epoch().count() == 0) prior_stopped_since_ = Clock::now();
        if (!tracking) prior_tracking_since_ = {};
        else if (prior_tracking_since_.time_since_epoch().count() == 0) prior_tracking_since_ = Clock::now();
        // Send once per frontend generation, only before a genuine map match.
        // Publication does not clear any recovery proof or motion lock.
        if (prior_enabled_ && anchor_valid_at_fault_ && !prior_sent_ &&
            !map_matched_after_respawn_ && age(respawn_seen_) < prior_max_wait_ &&
            age(prior_stopped_since_) >= prior_stop_time_ && stopped &&
            age(prior_tracking_since_) >= prior_wait_time_ && tracking) {
          const auto prior = wla_r680_navigation::stoppedRelocalizationPrior(
            {anchor_map_pose_->x, anchor_map_pose_->y, anchor_map_pose_->yaw},
            {anchor_wheel_pose_.x, anchor_wheel_pose_.y, anchor_wheel_pose_.yaw},
            {wheel_.x, wheel_.y, wheel_.yaw}, prior_max_travel_, prior_max_yaw_);
          if (prior) {
            geometry_msgs::msg::PoseWithCovarianceStamped msg;
            msg.header.frame_id = "map"; msg.header.stamp = get_clock()->now();
            msg.pose.pose.position.x = prior->x; msg.pose.pose.position.y = prior->y;
            msg.pose.pose.orientation.z = std::sin(prior->yaw/2);
            msg.pose.pose.orientation.w = std::cos(prior->yaw/2);
            const double travel = distance(wheel_, anchor_wheel_pose_);
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.04+travel*travel;
            msg.pose.covariance[35] = 0.04;
            prior_pub_->publish(msg); prior_sent_ = true;
            RCLCPP_WARN(get_logger(), "relocalization prior sent: x=%.3f y=%.3f yaw=%.3f, wheel travel=%.3f; map match still required",
              prior->x, prior->y, prior->yaw, travel);
          } else {
            prior_sent_ = true;  // Bound exceeded: no retry with an untrusted prior.
            RCLCPP_WARN(get_logger(), "relocalization prior refused: wheel displacement exceeds short-stop bounds");
          }
        }
        const auto map_tf_pose = currentMapPose();
        const bool localized = anchor_valid_at_fault_ && recovery_map_pose_ &&
          map_tf_pose &&
          age(recovery_map_seen_) <= localization_stale_s_ &&
          map_matched_after_respawn_ &&
          distance(*map_tf_pose, *recovery_map_pose_) <= localization_translation_error_m_ &&
          std::abs(angleDifference(map_tf_pose->yaw, recovery_map_pose_->yaw)) <=
            localization_yaw_change_rad_ &&
          std::abs(distance(*recovery_map_pose_, *anchor_map_pose_) -
            distance(wheel_, anchor_wheel_pose_)) <= localization_translation_error_m_ &&
          std::abs(angleDifference(recovery_map_pose_->yaw, anchor_map_pose_->yaw)) <=
            localization_yaw_change_rad_ && age(wheel_seen_) < 0.30;
        if (tracking && localized) {
          if (recovery_stable_since_.time_since_epoch().count() == 0) {
            recovery_stable_since_ = Clock::now();
          }
          if (age(recovery_stable_since_) >= recovery_stable_s_) {
            fault_latched_ = false;
            reason_.clear();
            window_vo_.reset();
            mismatch_count_ = 0;
            RCLCPP_WARN(get_logger(), "VO and map localization stable; motion health restored");
          }
        } else {
          recovery_stable_since_ = {};
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "recovery remains locked: tracking=%d map_match=%d fresh_map_pose=%d map_tf=%d anchor=%d prior_sent=%d; use initialpose if visual matching fails",
            tracking, map_matched_after_respawn_,
            recovery_map_pose_.has_value() && age(recovery_map_seen_) <= localization_stale_s_,
            map_tf_pose.has_value(), anchor_valid_at_fault_, prior_sent_);
        }
      }
    }
    if (!fault_latched_ && age(started_) > startup_grace_s_ &&
      (age(vo_seen_) > stale_s_ || age(info_seen_) > stale_s_)) {
      fault("VO odometry or tracking status timed out");
    }
    const bool healthy = !fault_latched_ && age(vo_seen_) <= stale_s_ &&
      age(info_seen_) <= stale_s_ && lost_count_ == 0 &&
      (frontend_ == "rgbd" || (frontend_ready_ && initial_map_matched_ &&
      age(initial_localization_seen_) <= localization_stale_s_ && currentMapPose().has_value()));
    std_msgs::msg::Bool health;
    health.data = healthy;
    health_pub_->publish(health);
    std_msgs::msg::String status;
    status.data = fault_latched_ ?
      (!anchor_valid_at_fault_ ? "manual_relocalization_required: " :
      respawn_seen_.time_since_epoch().count() == 0 ? "restarting_vo: " :
      "waiting_for_map_relocalization: ") + reason_ :
      (healthy ? "healthy" : frontend_ == "cuvslam" ?
        (frontend_ready_ ? "waiting_for_map_relocalization" : "waiting_for_inertial_initialization") : "waiting_for_vo");
    status_pub_->publish(status);
  }

  double startup_grace_s_{}, stale_s_{}, jump_linear_mps_{}, jump_angular_rps_{};
  double wheel_window_s_{}, wheel_translation_error_m_{}, wheel_yaw_error_rad_{};
  double recovery_stable_s_{}, localization_stale_s_{};
  double localization_translation_error_m_{}, localization_yaw_change_rad_{};
  bool prior_enabled_{true}, prior_sent_{false};
  double prior_max_travel_{}, prior_max_yaw_{}, prior_stop_time_{}, prior_wait_time_{}, prior_max_wait_{};
  double wheel_linear_{0}, wheel_angular_{0};
  Clock::time_point prior_stopped_since_{}, prior_tracking_since_{};
  int lost_limit_{}, wheel_mismatch_limit_{}, lost_count_{0}, mismatch_count_{0};
  bool compare_wheel_yaw_{false}, restart_enabled_{true}, fault_latched_{false};
  bool anchor_valid_at_fault_{false}, map_matched_after_respawn_{false};
  std::string frontend_;
  bool frontend_ready_{false}, frontend_was_ready_{false}, initial_map_matched_{false};
  Clock::time_point initial_localization_seen_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr frontend_health_sub_;
  pid_t vo_pid_{-1};
  std::string reason_;
  Clock::time_point started_, vo_seen_{}, info_seen_{}, wheel_seen_{}, window_started_{};
  Clock::time_point anchor_map_seen_{}, recovery_map_seen_{}, respawn_seen_{};
  Clock::time_point recovery_stable_since_{};
  Pose wheel_{}, window_wheel_{}, anchor_wheel_pose_{};
  std::optional<Pose> last_vo_, window_vo_, anchor_map_pose_, recovery_map_pose_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time respawn_ros_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_, wheel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    localization_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr map_info_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::OdomInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr prior_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoWatchdog>());
  rclcpp::shutdown();
  return 0;
}
