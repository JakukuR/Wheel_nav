#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
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
#include "rclcpp/parameter_client.hpp"
#include "rtabmap_msgs/msg/info.hpp"
#include "rtabmap_msgs/msg/odom_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wla_r680_navigation/relocalization_prior.hpp"
#include "wla_r680_navigation/recovery_evidence.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class VoWatchdog final : public rclcpp::Node
{
public:
  VoWatchdog() : Node("r680_vo_watchdog"), started_(Clock::now())
  {
    frontend_ = declare_parameter<std::string>("frontend", "rgbd");
    bounded_degraded_ = declare_parameter<bool>("allow_bounded_degraded", false);
    degraded_map_grace_ = declare_parameter<double>("degraded_map_grace_s", 2.0);
    if(!std::isfinite(degraded_map_grace_) || degraded_map_grace_<0 || degraded_map_grace_>3)
      throw std::invalid_argument("invalid degraded map grace");
    if(bounded_degraded_) degraded_sub_=create_subscription<std_msgs::msg::Bool>(
      "/r680_nav/odom_degraded",10,[this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        degraded_active_=msg->data;degraded_seen_=Clock::now();
      });
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
    staged_search_ = declare_parameter<bool>("staged_relocalization_enabled", false);
    near_radius_ = declare_parameter<double>("recovery_near_radius_m", 1.0);
    expanded_radius_ = declare_parameter<double>("recovery_expanded_radius_m", 3.0);
    near_time_ = declare_parameter<double>("recovery_near_timeout_s", 8.0);
    expanded_time_ = declare_parameter<double>("recovery_expanded_timeout_s", 12.0);
    vpr_time_ = declare_parameter<double>("recovery_vpr_timeout_s", 30.0);
    if (!std::isfinite(near_radius_+expanded_radius_+near_time_+expanded_time_+vpr_time_) ||
        near_radius_<=0 || expanded_radius_<=near_radius_ ||
        near_time_<2 || expanded_time_<2 || vpr_time_<2)
      throw std::invalid_argument("invalid staged relocalization parameters");
    if (staged_search_) search_client_=std::make_shared<rclcpp::AsyncParametersClient>(
      this, declare_parameter<std::string>("rtabmap_node", "/d455_slam/rtabmap"));
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
    search_status_pub_=create_publisher<std_msgs::msg::String>("/r680_nav/relocalization_status",10);
    prior_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initial_pose_topic, 1);
    manual_prior_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic,10,[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        if(staged_search_ && fault_latched_ && search_stage_==4 && !search_pending_ &&
            !search_original_.empty() && search_client_->service_is_ready() &&
            msg->header.frame_id=="map" && finitePose(msg->pose.pose) &&
            (get_clock()->now()-rclcpp::Time(msg->header.stamp)).seconds()>=-0.2 &&
            (get_clock()->now()-rclcpp::Time(msg->header.stamp)).seconds()<1.0) {
          RCLCPP_WARN(get_logger(),"manual initial pose received; retrying geometric verification while motion remains locked");
          requestSearchStage(3);
        }
      });
    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vo_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onVo(*msg);});
    if (frontend_ == "rgbd") {
      info_sub_ = create_subscription<rtabmap_msgs::msg::OdomInfo>(
        info_topic, rclcpp::SensorDataQoS(),
        [this](rtabmap_msgs::msg::OdomInfo::ConstSharedPtr msg) {onInfo(*msg);});
    } else {
      frontend_health_topic_=declare_parameter<std::string>("frontend_health_topic", "/r680_nav/vio_tracking_healthy");
      frontend_health_sub_ = create_subscription<std_msgs::msg::Bool>(
        frontend_health_topic_, 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (fault_latched_ && respawn_seen_.time_since_epoch().count() == 0) return;
          info_seen_ = Clock::now();
          frontend_ready_ = msg->data;
          if (msg->data) frontend_was_ready_ = true;
          lost_count_ = msg->data ? 0 : lost_count_ + 1;
          // Initial SDK gravity estimation is a waiting state, not tracking loss.
          if (!fault_latched_ && frontend_was_ready_ && lost_count_ >= lost_limit_)
            fault(frontend_health_topic_=="/r680_nav/continuous_odom_healthy" ?
              "continuous odometry unhealthy" : "cuVSLAM inertial tracking unhealthy");
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
        if(fault_latched_ && search_stage_==3 && !search_pending_) {
          std::vector<std::pair<float,int>> candidates;
          for(size_t i=0;i<std::min(msg->posterior_keys.size(),msg->posterior_values.size());++i)
            if(msg->posterior_keys[i]>0) candidates.emplace_back(msg->posterior_values[i],msg->posterior_keys[i]);
          std::sort(candidates.rbegin(),candidates.rend());
          std::string summary;
          for(size_t i=0;i<std::min(size_t(3),candidates.size());++i)
            summary+=std::to_string(candidates[i].second)+":"+std::to_string(candidates[i].first)+" ";
          RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"VPR candidates (not verification): %s",summary.c_str());
        }
        if (fault_latched_ && search_stage_>=1 && search_stage_!=4 && !search_pending_ &&
            rclcpp::Time(msg->header.stamp)>search_ros_stamp_ &&
            (msg->loop_closure_id>0 || msg->proximity_detection_id>0)) {
          search_match_stamp_=rclcpp::Time(msg->header.stamp).nanoseconds();
          RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
            "relocalization geometric match: stage=%s node=%d frame=%d",
            searchName(),msg->loop_closure_id>0 ? msg->loop_closure_id : msg->proximity_detection_id,msg->ref_id);
        }
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
        search_map_stamp_=rclcpp::Time(msg.header.stamp).nanoseconds();
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
          else {
            RCLCPP_ERROR(get_logger(),"VO discontinuity details: dt=%.6f translation=%.6f/%.6f yaw=%.6f/%.6f previous=(%.6f,%.6f,%.6f) current=(%.6f,%.6f,%.6f)",
              dt,distance(pose,*last_vo_),0.15+jump_linear_mps_*dt,
              std::abs(angleDifference(pose.yaw,last_vo_->yaw)),0.20+jump_angular_rps_*dt,
              last_vo_->x,last_vo_->y,last_vo_->yaw,pose.x,pose.y,pose.yaw);
            fault("VO pose discontinuity");
          }
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
        // Never signal a frontend in another ROS domain (including isolated tests).
        std::ifstream environment(entry.path()/"environ",std::ios::binary);
          const std::string env((std::istreambuf_iterator<char>(environment)),std::istreambuf_iterator<char>());
        const auto domain=[](const std::string & values) {
          const auto pos=values.find("ROS_DOMAIN_ID=");
          if(pos==std::string::npos) return std::string("0");
          const auto begin=pos+14;return values.substr(begin,values.find('\0',begin)-begin);
        };
        const char * own=std::getenv("ROS_DOMAIN_ID");
        if(domain(env)!=(own ? own : "0")) continue;
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
    ++search_generation_;search_stage_=0;search_pending_=false;search_restored_=false;
    search_evidence_.reset();search_match_stamp_=0;search_map_stamp_=0;
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
    if (!fault_latched_ && !degradedActive() && (frontend_ == "rgbd" || frontend_ready_) && age(vo_seen_) <= stale_s_ &&
      age(info_seen_) <= stale_s_ && age(wheel_seen_) < 0.30 &&
      (frontend_=="rgbd" || (initial_map_matched_ && age(initial_localization_seen_)<=localization_stale_s_))) {
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
        const bool localized = staged_search_ ? stagedLocalized(tracking,stopped,map_tf_pose) :
          anchor_valid_at_fault_ && recovery_map_pose_ &&
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
      age(initial_localization_seen_) <= localization_stale_s_ +
        (degradedActive() ? degraded_map_grace_ : 0.0) && currentMapPose().has_value()));
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
    if (staged_search_ && fault_latched_) {
      std_msgs::msg::String search;
      search.data=std::string(searchName())+" map_match_verified="+(search_evidence_.verified() ? "true" : "false")+
        " parameter_pending="+(search_pending_ ? "true" : "false");
      search_status_pub_->publish(search);
    }
  }

  const char * searchName() const {
    switch(search_stage_) {case 1:return "near";case 2:return "expanded";case 3:return "vpr";
      case 4:return "manual_required";case 5:return "verified";default:return "waiting_tracking";}
  }

  std::optional<wla_r680_navigation::PlanarPose> searchPrior() const {
    if (!anchor_valid_at_fault_ || !anchor_map_pose_ || age(wheel_seen_)>=0.30) return {};
    return wla_r680_navigation::stoppedRelocalizationPrior(
      {anchor_map_pose_->x,anchor_map_pose_->y,anchor_map_pose_->yaw},
      {anchor_wheel_pose_.x,anchor_wheel_pose_.y,anchor_wheel_pose_.yaw},
      {wheel_.x,wheel_.y,wheel_.yaw},prior_max_travel_,prior_max_yaw_);
  }

  void sendSearchPrior() {
    const auto p=searchPrior();if(!p) return;
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id="map";msg.header.stamp=get_clock()->now();
    msg.pose.pose.position.x=p->x;msg.pose.pose.position.y=p->y;
    msg.pose.pose.orientation.z=std::sin(p->yaw/2);msg.pose.pose.orientation.w=std::cos(p->yaw/2);
    msg.pose.covariance[0]=msg.pose.covariance[7]=0.04;msg.pose.covariance[35]=0.04;
    prior_pub_->publish(msg);prior_sent_=true;
  }

  void restoreSearchParameters() {
    if(search_pending_ || search_original_.empty()) return;
    search_pending_=true;search_request_seen_=Clock::now();const auto generation=search_generation_;
    search_client_->set_parameters(search_original_,[this,generation](auto future) {
      if(generation!=search_generation_) return;
      search_pending_=false;
      try {
        const auto results=future.get();
        search_restored_=results.size()==search_original_.size() &&
          std::all_of(results.begin(),results.end(),[](const auto & r){return r.successful;});
      } catch(const std::exception & e) {RCLCPP_ERROR(get_logger(),"restore RTAB parameters: %s",e.what());}
      if(!search_restored_) {search_stage_=4;RCLCPP_ERROR(get_logger(),"RTAB parameter restoration failed; motion remains locked");}
      else RCLCPP_INFO(get_logger(),"RTAB original search/registration parameters restored");
    });
  }

  void requestSearchStage(int stage) {
    search_stage_=stage;search_pending_=true;search_restored_=false;
    search_request_seen_=Clock::now();search_evidence_.reset();search_match_stamp_=0;
    recovery_stable_since_={};map_matched_after_respawn_=false;recovery_map_seen_={};recovery_map_pose_.reset();
    auto parameters=search_original_;
    const auto set=[&](const std::string & key,const std::string & value) {
      for(auto & p:parameters) if(p.get_name()==key) {p=rclcpp::Parameter(key,value);return;}
    };
    set("RGBD/ProximityBySpace",stage<3 ? "true" : "false");
    if(stage<3) set("RGBD/LocalRadius",std::to_string(stage==1 ? near_radius_ : expanded_radius_));
    set("RGBD/ProximityAngle",stage==1 ? "90" : "180");
    set("RGBD/ProximityMaxPaths",stage==1 ? "3" : "5");
    // Keep BoW retrieval available but suppress global acceptance in spatial stages.
    if(stage<3) {set("Rtabmap/LoopThr","1");set("RGBD/AggressiveLoopThr","1");}
    set("Reg/Strategy","0");set("Vis/EstimationType","0"); // RGB-D 3D-to-3D RANSAC
    const auto generation=search_generation_;
    search_client_->set_parameters(parameters,[this,generation,stage](auto future) {
      if(generation!=search_generation_ || stage!=search_stage_) return;
      search_pending_=false;bool success=false;
      try {const auto r=future.get();success=r.size()==search_original_.size() &&
        std::all_of(r.begin(),r.end(),[](const auto & x){return x.successful;});}
      catch(const std::exception & e) {RCLCPP_ERROR(get_logger(),"RTAB stage parameters: %s",e.what());}
      if(!success) {search_stage_=4;RCLCPP_ERROR(get_logger(),"RTAB stage change failed; motion remains locked");restoreSearchParameters();return;}
      search_stage_seen_=Clock::now();search_ros_stamp_=get_clock()->now()+rclcpp::Duration::from_seconds(1.0);
      if(stage<3) sendSearchPrior();
      RCLCPP_WARN(get_logger(),"relocalization stage=%s spatial_radius=%.2fm; RGB-D geometry verification required",
        searchName(),stage==3 ? 0.0 : stage==1 ? near_radius_ : expanded_radius_);
    });
  }

  bool stagedLocalized(bool tracking,bool stopped,const std::optional<Pose> & map_tf) {
    if(search_pending_) {
      if(age(search_request_seen_)>5.0) {++search_generation_;search_pending_=false;search_stage_=4;
        RCLCPP_ERROR(get_logger(),"RTAB parameter service timed out; manual relocalization required");}
      return false;
    }
    if(search_stage_==4) return false;
    const auto prior=searchPrior();
    if(!tracking || !stopped || !prior) return false;
    if(search_stage_==0) {
      if(age(prior_stopped_since_)<prior_stop_time_ || age(prior_tracking_since_)<prior_wait_time_) return false;
      if(!search_client_->service_is_ready()) return false;
      if(search_original_.empty()) {
        search_pending_=true;search_request_seen_=Clock::now();const auto generation=search_generation_;
        const std::vector<std::string> keys={"RGBD/ProximityBySpace","RGBD/LocalRadius","RGBD/ProximityAngle",
          "RGBD/ProximityMaxPaths","Rtabmap/LoopThr","RGBD/AggressiveLoopThr","Reg/Strategy","Vis/EstimationType"};
        search_client_->get_parameters(keys,[this,generation,keys](auto future) {
          if(generation!=search_generation_) return;
          search_pending_=false;
          try {
            auto values=future.get();
            if(values.size()!=keys.size() || std::any_of(values.begin(),values.end(),[](const auto & p){
                return p.get_type()!=rclcpp::ParameterType::PARAMETER_STRING;}))
              throw std::runtime_error("RTAB recovery parameters must exist and be strings");
            search_original_=std::move(values);requestSearchStage(1);
          } catch(const std::exception & e) {search_stage_=4;RCLCPP_ERROR(get_logger(),"RTAB recovery unavailable: %s",e.what());}
        });
      } else requestSearchStage(1);
      return false;
    }
    const bool map_consistent=age(recovery_map_seen_)<=localization_stale_s_ && recovery_map_pose_ && map_tf &&
      distance(*map_tf,*recovery_map_pose_)<=localization_translation_error_m_ &&
      std::abs(angleDifference(map_tf->yaw,recovery_map_pose_->yaw))<=localization_yaw_change_rad_;
    const bool paired=map_consistent && search_match_stamp_>search_ros_stamp_.nanoseconds() &&
      std::abs(search_map_stamp_-search_match_stamp_)<250000000LL;
    // RTAB-Map can accept a single relocalization and then track at that location
    // without reporting new loop/proximity matches. Keep that verified result
    // for this search generation; current pose/TF and tracking remain checked.
    if(search_stage_==5) return search_restored_ && map_consistent && search_evidence_.verified();
    if(paired) {
      const auto & p=*recovery_map_pose_;
      const double from_prior=std::hypot(p.x-prior->x,p.y-prior->y);
      const bool in_region=search_stage_>=3 || from_prior<=(search_stage_==1 ? near_radius_ : expanded_radius_);
      if(in_region) {
        if(search_evidence_.accept(search_match_stamp_,{p.x,p.y,p.yaw}))
          RCLCPP_INFO(get_logger(),"single relocalization match verified: stage=%s map=(%.3f,%.3f,%.3f)",
            searchName(),p.x,p.y,p.yaw);
      }
      else search_evidence_.reset();
    }
    if(search_evidence_.verified()) {
      search_stage_=5;restoreSearchParameters();return false;
    }
    const double limit=search_stage_==1 ? near_time_ : search_stage_==2 ? expanded_time_ : vpr_time_;
    if(age(search_stage_seen_)>limit) {
      if(search_stage_<3) requestSearchStage(search_stage_+1);
      else {search_stage_=4;RCLCPP_ERROR(get_logger(),"VPR recovery timed out; manual initial pose required");restoreSearchParameters();}
    }
    return false;
  }

  double startup_grace_s_{}, stale_s_{}, jump_linear_mps_{}, jump_angular_rps_{};
  bool degradedActive() const {return bounded_degraded_ && degraded_active_ && age(degraded_seen_)<0.20;}
  bool bounded_degraded_{false},degraded_active_{false};double degraded_map_grace_{};
  Clock::time_point degraded_seen_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr degraded_sub_;
  double wheel_window_s_{}, wheel_translation_error_m_{}, wheel_yaw_error_rad_{};
  double recovery_stable_s_{}, localization_stale_s_{};
  double localization_translation_error_m_{}, localization_yaw_change_rad_{};
  bool prior_enabled_{true}, prior_sent_{false};
  double prior_max_travel_{}, prior_max_yaw_{}, prior_stop_time_{}, prior_wait_time_{}, prior_max_wait_{};
  double wheel_linear_{0}, wheel_angular_{0};
  bool staged_search_{false},search_pending_{false},search_restored_{false};
  int search_stage_{0},search_generation_{0};
  double near_radius_{},expanded_radius_{},near_time_{},expanded_time_{},vpr_time_{};
  int64_t search_match_stamp_{0},search_map_stamp_{0};
  rclcpp::Time search_ros_stamp_{0,0,RCL_ROS_TIME};
  Clock::time_point search_request_seen_{},search_stage_seen_{};
  wla_r680_navigation::RecoveryEvidence search_evidence_;
  std::vector<rclcpp::Parameter> search_original_;
  std::shared_ptr<rclcpp::AsyncParametersClient> search_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr search_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr manual_prior_sub_;
  Clock::time_point prior_stopped_since_{}, prior_tracking_since_{};
  int lost_limit_{}, wheel_mismatch_limit_{}, lost_count_{0}, mismatch_count_{0};
  bool compare_wheel_yaw_{false}, restart_enabled_{true}, fault_latched_{false};
  bool anchor_valid_at_fault_{false}, map_matched_after_respawn_{false};
  std::string frontend_;
  std::string frontend_health_topic_;
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
