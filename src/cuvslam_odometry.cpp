#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <unistd.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <Eigen/Geometry>
#include "cuvslam2.h"
#include "cuvslam2_internal.h"
#include "initialization_sampling.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "odometry_contract.hpp"
#include "health_evidence.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Imu = sensor_msgs::msg::Imu;

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

cuvslam::Pose sdk_pose(const Eigen::Isometry3d & t)
{
  Eigen::Quaterniond q(t.rotation());
  q.normalize();
  cuvslam::Pose pose;
  pose.translation = {static_cast<float>(t.translation().x()),
    static_cast<float>(t.translation().y()), static_cast<float>(t.translation().z())};
  pose.rotation = {static_cast<float>(q.x()), static_cast<float>(q.y()),
    static_cast<float>(q.z()), static_cast<float>(q.w())};
  return pose;
}

Eigen::Isometry3d from_array(const std::vector<double> & values)
{
  if (values.size() != 7 || !std::all_of(values.begin(), values.end(),
    [](double x) {return std::isfinite(x);})) {
    throw std::invalid_argument("extrinsic must be finite xyz + xyzw");
  }
  Eigen::Quaterniond q(values[6], values[3], values[4], values[5]);
  if (std::abs(q.norm() - 1.0) > 0.01) {
    throw std::invalid_argument("extrinsic quaternion must be normalized");
  }
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
  return t;
}

class Validation final : public rclcpp::Node
{
public:
  Validation() : Node("cuvslam_odometry"), buffer_(get_clock()), listener_(buffer_)
  {
    use_imu_ = declare_parameter<bool>("use_imu", true);
    async_sba_ = declare_parameter<bool>("async_sba", false);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    initialization_stable_s_ = declare_parameter<double>("initialization_stable_s", 2.0);
    initialization_diagnostics_ = declare_parameter<bool>("initialization_diagnostics", false);
    initialization_sampling_ = std::make_unique<wla_vio::InitializationSampling>(
      declare_parameter<double>("initialization_keyframe_period_s", 0.0));
    twist_filter_tau_s_ = declare_parameter<double>("twist_filter_tau_s", 0.08);
    max_gyro_bias_ = declare_parameter<double>("max_gyro_bias_radps", 0.25);
    max_accel_bias_ = declare_parameter<double>("max_accel_bias_mps2", 3.0);
    if (!std::isfinite(initialization_stable_s_) || !std::isfinite(twist_filter_tau_s_) ||
      !std::isfinite(max_gyro_bias_) || !std::isfinite(max_accel_bias_) ||
      !use_imu_ || initialization_stable_s_ <= 0 || twist_filter_tau_s_ < 0 ||
      max_gyro_bias_ <= 0 || max_accel_bias_ <= 0) {
      throw std::invalid_argument("navigation frontend requires IMU and valid health thresholds");
    }
    base_ = declare_parameter<std::string>("base_frame", "r680_mapping_floor");
    camera_link_ = declare_parameter<std::string>("camera_link_frame", "d455_link");
    camera_mount_ = from_array(declare_parameter<std::vector<double>>("base_from_camera_link",
      {0.184428484, 0.059803590, 0.501660007, 0.000096773, 0.077125800, -0.000560247, 0.997021207}));
    odom_ = declare_parameter<std::string>("odom_frame", "d455_floor_odom");
    expected_imu_frame_ = declare_parameter<std::string>("expected_imu_frame", "gyro_link");
    statistics_path_ = declare_parameter<std::string>("statistics_path", "/tmp/cuvslam_validation_statistics.json");
    diagnostics_path_ = declare_parameter<std::string>("health_diagnostics_path", "");
    if (diagnostics_path_.empty()) diagnostics_path_=(std::filesystem::path(statistics_path_).parent_path()/
      ("vio_health-"+std::to_string(::getpid())+".jsonl")).string();
    frequency_ = declare_parameter<double>("imu_frequency", 200.0);
    gyro_noise_ = declare_parameter<double>("gyro_noise_density", 0.000079);
    gyro_walk_ = declare_parameter<double>("gyro_random_walk", 0.00002);
    accel_noise_ = declare_parameter<double>("accel_noise_density", 0.0012);
    accel_walk_ = declare_parameter<double>("accel_random_walk", 0.003);
    imu_offset_ns_ = static_cast<int64_t>(declare_parameter<double>("imu_time_offset_s", 0.0) * 1e9);
    tolerance_ns_ = static_cast<int64_t>(declare_parameter<double>("stereo_tolerance_ms", 1.0) * 1e6);
    max_age_ns_ = static_cast<int64_t>(declare_parameter<double>("max_input_age_s", 0.5) * 1e9);
    max_imu_gap_ns_ = static_cast<int64_t>(declare_parameter<double>("max_imu_gap_s", 0.15) * 1e9);
    depth_ = declare_parameter<int>("queue_depth", 8);
    image_decimation_ = declare_parameter<int>("image_decimation", 1);
    imu_pose_ = from_array(declare_parameter<std::vector<double>>("base_from_imu", {0, 0, 0, 0, 0, 0, 1}));
    cuvslam::SetVerbosity(declare_parameter<int>("verbosity", 0));
    if (frequency_ <= 0 || depth_ < 2 || depth_ > 100 || tolerance_ns_ < 0 ||
      max_age_ns_ <= 0 || max_imu_gap_ns_ <= 0 || image_decimation_ < 1 || image_decimation_ > 6) {
      throw std::invalid_argument("invalid timing or queue parameters");
    }
    pub_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("output_topic", "/d455_slam/odom"), 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("status_topic", "/r680_nav/vio_status"), 10);
    health_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/vio_tracking_healthy", 10);
    // Current-frame evidence, separate from the two-second initialization gate.
    // The navigation continuity node may use this only after a trusted startup.
    inertial_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/vio_inertial_valid", 10);
    visual_pub_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/vio_visual_observed", 10);
    health_reason_pub_ = create_publisher<std_msgs::msg::String>("/r680_nav/vio_health_reason", 10);
    minimum_visual_observations_ = declare_parameter<int>("minimum_visual_observations", 20);
    if(minimum_visual_observations_<1) throw std::invalid_argument("invalid visual evidence threshold");
    if (publish_tf_) broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    for (size_t i = 0; i < 2; ++i) {
      const std::string side = i == 0 ? "left" : "right";
      image_subs_[i] = create_subscription<Image>(
        declare_parameter<std::string>(side + "_topic", "/r680/d455/infra" + std::to_string(i + 1) + "/image_rect_raw"),
        rclcpp::SensorDataQoS().keep_last(depth_),
        [this, i](Image::ConstSharedPtr msg) {
          if (stopped_) return;
          std::lock_guard<std::mutex> lock(mutex_);
          if (msg->encoding != "mono8" || msg->step != msg->width ||
            msg->data.size() != static_cast<size_t>(msg->width) * msg->height) {
            ++invalid_images_;
            return;
          }
          if (!images_[i].empty() && stamp_ns(msg->header.stamp) <= stamp_ns(images_[i].back()->header.stamp)) {
            ++out_of_order_images_;
            return;
          }
          images_[i].push_back(msg);
          if (images_[i].size() > static_cast<size_t>(depth_)) {
            images_[i].pop_front();
            ++queue_drops_;
          }
          cv_.notify_one();
        });
      info_subs_[i] = create_subscription<Info>(
        declare_parameter<std::string>(side + "_info_topic", "/r680/d455/infra" + std::to_string(i + 1) + "/camera_info"),
        rclcpp::SensorDataQoS(), [this, i](Info::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          infos_[i] = msg;
          cv_.notify_one();
        });
    }
    imu_sub_ = create_subscription<Imu>(
      declare_parameter<std::string>("imu_topic", "/wheel/imu/data_raw"),
      rclcpp::SensorDataQoS().keep_last(1000), [this](Imu::ConstSharedPtr msg) {
        if (stopped_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        const auto t = stamp_ns(msg->header.stamp) + imu_offset_ns_;
        const auto & a = msg->linear_acceleration;
        const auto & w = msg->angular_velocity;
        if (msg->header.frame_id != expected_imu_frame_ ||
          !std::isfinite(a.x + a.y + a.z + w.x + w.y + w.z)) {
          ++invalid_imu_;
          return;
        }
        if (t <= last_received_imu_) {
          ++out_of_order_imu_;
          return;
        }
        last_received_imu_ = t;
        imus_.push_back(msg);
        if (imus_.size() > 2000) {
          imus_.pop_front();
          ++imu_queue_drops_;
        }
        ++received_imu_;
        cv_.notify_one();
      });
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom"),
      rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Observation only: wheel data is NOT an estimator input.
        ++wheel_samples_;
        max_wheel_linear_ = std::max(max_wheel_linear_, std::abs(msg->twist.twist.linear.x));
        max_wheel_angular_ = std::max(max_wheel_angular_, std::abs(msg->twist.twist.angular.z));
      });
    RCLCPP_WARN(get_logger(), "cuVSLAM navigation frontend: TF=%s; motion health requires stable inertial initialization; calibration provisional", publish_tf_ ? "on" : "off");
    worker_ = std::thread([this]() { run(); });
  }

  ~Validation() override
  {
    stopped_ = true;
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    write_statistics();
  }

private:
  static std::string jsonString(const std::string & value)
  {
    std::string out="\"";
    for (unsigned char c:value) {
      if (c=='"' || c=='\\') {out+='\\';out+=static_cast<char>(c);}
      else if (c=='\n') out+="\\n";
      else if (c=='\r') out+="\\r";
      else if (c=='\t') out+="\\t";
      else if (c<0x20) out+='?';
      else out+=static_cast<char>(c);
    }
    return out+'"';
  }

  // Transitions are immediate; repeated evidence is capped at 1 Hz. A PID file
  // preserves each respawn's evidence rather than overwriting the last run.
  void diagnostic(const std::string & reason,const std::string & details="",bool force=false)
  {
    const auto now=std::chrono::steady_clock::now();
    if (!force && reason==last_diagnostic_reason_ &&
        std::chrono::duration<double>(now-last_diagnostic_time_).count()<1.0) return;
    const bool changed=reason!=last_diagnostic_reason_;
    last_diagnostic_reason_=reason;last_diagnostic_time_=now;
    ++diagnostic_events_;
    const std::string evidence=details.empty() ? "{}" : details;
    if (reason!="tracking_inertial_ready" || changed || force)
      RCLCPP_WARN(get_logger(),"VIO diagnostic reason=%s evidence=%s",reason.c_str(),evidence.c_str());
    try {
      const auto path=std::filesystem::path(diagnostics_path_);
      if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
      std::ofstream out(path,std::ios::app);
      if (!out) throw std::runtime_error("cannot append diagnostic file");
      out<<"{\"ros_stamp_ns\":"<<get_clock()->now().nanoseconds()<<",\"pid\":"<<::getpid()
        <<",\"frame_stamp_ns\":"<<last_call_stamp_<<",\"reason\":"<<jsonString(reason)
        <<",\"tracked_frames\":"<<tracked_frames_<<",\"published_frames\":"<<published_frames_
        <<",\"registered_imu\":"<<registered_imu_<<",\"observations\":"<<last_observations_
        <<",\"warming_up\":"<<(last_warming_up_ ? "true" : "false")
        <<",\"evidence\":"<<evidence<<"}\n";
      out.flush();if(!out)throw std::runtime_error("diagnostic write failed");
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"VIO diagnostic persistence failed: %s",error.what());
    }
  }

  void status(const std::string & state)
  {
    diagnostic(state);
    std_msgs::msg::Bool health;
    health.data = state == "tracking_inertial_ready";
    health_pub_->publish(health);
    std_msgs::msg::Bool invalid_inertial;
    invalid_inertial.data = false;
    inertial_pub_->publish(invalid_inertial);
    visual_pub_->publish(invalid_inertial);
    if (!health.data) {
      last_gate_stamp_s_=get_clock()->now().seconds();
      initialization_gate_.update(false,last_gate_stamp_s_,initialization_stable_s_);
    }
    std_msgs::msg::String msg;
    msg.data = state;
    status_pub_->publish(msg);
    health_reason_pub_->publish(msg);
  }

  bool initialize(const std::array<Info::ConstSharedPtr, 2> & infos)
  {
    const auto & left = *infos[0];
    const auto & right = *infos[1];
    if (left.width != right.width || left.height != right.height ||
      left.p[0] <= 0 || right.p[0] <= 0 || left.p[5] <= 0 || right.p[5] <= 0) {
      throw std::runtime_error("invalid rectified stereo CameraInfo");
    }
    const double baseline = -right.p[3] / right.p[0] + left.p[3] / left.p[0];
    if (baseline < 0.01 || baseline > 1.0) {
      throw std::runtime_error("invalid stereo baseline in projection matrices");
    }
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = buffer_.lookupTransform(camera_link_, left.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "waiting for driver TF %s <- %s: %s", camera_link_.c_str(), left.header.frame_id.c_str(), ex.what());
      return false;
    }
    const auto & t = transform.transform.translation;
    const auto & q = transform.transform.rotation;
    const auto base_from_left = camera_mount_ * from_array({t.x, t.y, t.z, q.x, q.y, q.z, q.w});
    auto left_from_right = Eigen::Isometry3d::Identity();
    left_from_right.translation().x() = baseline;
    cuvslam::Rig rig;
    for (size_t i = 0; i < 2; ++i) {
      cuvslam::Camera camera;
      camera.size = {static_cast<int32_t>(infos[i]->width), static_cast<int32_t>(infos[i]->height)};
      camera.focal = {static_cast<float>(infos[i]->p[0]), static_cast<float>(infos[i]->p[5])};
      camera.principal = {static_cast<float>(infos[i]->p[2]), static_cast<float>(infos[i]->p[6])};
      camera.rig_from_camera = sdk_pose(i == 0 ? base_from_left : base_from_left * left_from_right);
      rig.cameras.push_back(camera);
    }
    if (use_imu_) {
      cuvslam::ImuCalibration calibration;
      calibration.rig_from_imu = sdk_pose(imu_pose_);
      calibration.frequency = static_cast<float>(frequency_);
      calibration.gyroscope_noise_density = static_cast<float>(gyro_noise_);
      calibration.gyroscope_random_walk = static_cast<float>(gyro_walk_);
      calibration.accelerometer_noise_density = static_cast<float>(accel_noise_);
      calibration.accelerometer_random_walk = static_cast<float>(accel_walk_);
      rig.imus.push_back(calibration);
    }
    cuvslam::Odometry::Config config;
    config.odometry_mode = use_imu_ ? cuvslam::Odometry::OdometryMode::Inertial : cuvslam::Odometry::OdometryMode::Multicamera;
    config.use_gpu = true;
    config.rectified_stereo_camera = true;
    config.async_sba = async_sba_;
    config.enable_observations_export = true;
    cuvslam::WarmUpGPU();
    tracker_ = std::make_unique<cuvslam::Odometry>(rig, config);
    RCLCPP_INFO(get_logger(), "cuVSLAM %s initialized: mode=%s baseline=%.6fm IMU=%.2fHz; mount=[%.5f %.5f %.5f]",
      std::string(cuvslam::GetVersion(nullptr, nullptr, nullptr)).c_str(),
      use_imu_ ? "Inertial" : "Multicamera", baseline, frequency_,
      base_from_left.translation().x(), base_from_left.translation().y(), base_from_left.translation().z());
    status("initialized_provisional_calibration");
    return true;
  }

  void run()
  {
    try {
      while (!stopped_ && rclcpp::ok()) {
        std::array<Image::ConstSharedPtr, 2> pair;
        std::array<Info::ConstSharedPtr, 2> infos;
        std::vector<Imu::ConstSharedPtr> inputs;
        int64_t frame_stamp = 0;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          cv_.wait_for(lock, std::chrono::milliseconds(20));
          if (stopped_) { break; }
          if (!infos_[0] || !infos_[1] || images_[0].empty() || images_[1].empty()) { continue; }
          const auto lt = stamp_ns(images_[0].front()->header.stamp);
          const auto rt = stamp_ns(images_[1].front()->header.stamp);
          if (std::abs(lt - rt) > tolerance_ns_) {
            images_[lt < rt ? 0 : 1].pop_front();
            ++unmatched_images_;
            continue;
          }
          frame_stamp = lt;
          const auto age = get_clock()->now().nanoseconds() - lt;
          if (age < -100000000LL || age > max_age_ns_ || frame_stamp <= last_call_stamp_) {
            images_[0].pop_front(); images_[1].pop_front(); ++stale_frames_; continue;
          }
          if (use_imu_ && (imus_.empty() || last_received_imu_ < lt)) { continue; }
          if (use_imu_ && last_received_imu_ - lt > max_age_ns_) {
            images_[0].pop_front(); images_[1].pop_front(); ++stale_frames_; continue;
          }
          if (++paired_frames_ % static_cast<size_t>(image_decimation_) != 0) {
            // Do not drain/register IMUs here: retain them for the next selected image frame.
            images_[0].pop_front(); images_[1].pop_front(); ++decimated_frames_; continue;
          }
          infos = infos_;
          for (size_t i = 0; i < 2; ++i) { pair[i] = images_[i].front(); images_[i].pop_front(); }
          while (!imus_.empty() && stamp_ns(imus_.front()->header.stamp) + imu_offset_ns_ <= lt) {
            inputs.push_back(imus_.front()); imus_.pop_front();
          }
        }
        if (!tracker_ && !initialize(infos)) { continue; }
        bool imu_gap = false;
        if (use_imu_) {
          for (const auto & msg : inputs) {
            const auto timestamp = stamp_ns(msg->header.stamp) + imu_offset_ns_;
            if (timestamp <= last_call_stamp_) { ++late_imu_; continue; }
            if (last_registered_imu_ > 0 && timestamp - last_registered_imu_ > max_imu_gap_ns_) { imu_gap = true; }
            cuvslam::ImuMeasurement sample;
            sample.timestamp_ns = timestamp;
            sample.angular_velocities = {static_cast<float>(msg->angular_velocity.x),
              static_cast<float>(msg->angular_velocity.y), static_cast<float>(msg->angular_velocity.z)};
            sample.linear_accelerations = {static_cast<float>(msg->linear_acceleration.x),
              static_cast<float>(msg->linear_acceleration.y), static_cast<float>(msg->linear_acceleration.z)};
            tracker_->RegisterImuMeasurement(0, sample);
            last_call_stamp_ = timestamp;
            last_registered_imu_ = timestamp;
            ++registered_imu_;
          }
          if (last_registered_imu_ == 0 || frame_stamp - last_registered_imu_ > max_imu_gap_ns_) { imu_gap = true; }
        }
        if (imu_gap) {
          ++imu_gap_frames_;
          diagnostic("imu_gap_no_output","{\"frame_stamp_ns\":"+std::to_string(frame_stamp)+
            ",\"last_registered_imu_ns\":"+std::to_string(last_registered_imu_)+
            ",\"max_imu_gap_ns\":"+std::to_string(max_imu_gap_ns_)+"}");
          status("imu_gap_no_output"); continue;
        }
        cuvslam::Odometry::ImageSet images;
        for (size_t i = 0; i < 2; ++i) {
          if (pair[i]->width != infos[i]->width || pair[i]->height != infos[i]->height) {
            throw std::runtime_error("image dimensions changed; restart frontend explicitly");
          }
          cuvslam::Image image{};
          image.timestamp_ns = stamp_ns(pair[i]->header.stamp);
          image.camera_index = static_cast<uint32_t>(i);
          image.pixels = pair[i]->data.data();
          image.width = static_cast<int32_t>(pair[i]->width);
          image.height = static_cast<int32_t>(pair[i]->height);
          image.pitch = static_cast<int32_t>(pair[i]->step);
          image.encoding = cuvslam::ImageData::Encoding::MONO;
          image.data_type = cuvslam::ImageData::DataType::UINT8;
          image.is_gpu_mem = false;
          images.push_back(image);
        }
        const auto start = std::chrono::steady_clock::now();
        cuvslam::internal::Internals frame_options;
        const bool force_keyframe = initialization_sampling_->request(frame_stamp, gravity_frames_ > 0);
        if (force_keyframe) {frame_options.kf_override_frame_selection = true; ++requested_init_keyframes_;}
        const auto estimate = tracker_->Track(images, {}, {}, force_keyframe ? &frame_options : nullptr);
        const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        if (tracked_frames_ == 0) { first_track_time_ = start; }
        last_track_time_ = std::chrono::steady_clock::now();
        last_call_stamp_ = frame_stamp;
        ++tracked_frames_;
        cuvslam::Odometry::State sdk_state;
        tracker_->GetState(sdk_state);
        last_observations_=static_cast<int64_t>(sdk_state.observations.size());
        last_warming_up_=sdk_state.warming_up;
        if (initialization_diagnostics_) {
          if (sdk_state.keyframe && estimate.world_from_rig) {
            ++keyframes_; keyframe_stamps_.push_back(frame_stamp);
          }
          while (!keyframe_stamps_.empty() && frame_stamp - keyframe_stamps_.front() > 25000000000LL) {
            keyframe_stamps_.pop_front();
          }
        }
        track_ms_.push_back(elapsed);
        if (track_ms_.size() > 4096) track_ms_.erase(track_ms_.begin(), track_ms_.begin() + 2048);
        if (!estimate.world_from_rig) { ++lost_frames_; status("tracking_lost_no_output"); continue; }
        const auto & result = *estimate.world_from_rig;
        bool finite = std::all_of(result.pose.translation.begin(), result.pose.translation.end(), [](float v) {return std::isfinite(v);}) &&
          std::all_of(result.pose.rotation.begin(), result.pose.rotation.end(), [](float v) {return std::isfinite(v);});
        if (!finite) { ++invalid_poses_; status("invalid_pose_no_output"); continue; }
        const Eigen::Vector3d position(result.pose.translation[0], result.pose.translation[1], result.pose.translation[2]);
        Eigen::Quaterniond rotation(result.pose.rotation[3], result.pose.rotation[0],
          result.pose.rotation[1], result.pose.rotation[2]);
        if (std::abs(rotation.norm() - 1.0) > 0.01) throw std::runtime_error("invalid SDK quaternion");
        rotation.normalize();
        const bool finite_covariance = std::all_of(result.covariance_xyz_rpy.begin(),
          result.covariance_xyz_rpy.end(), [](float v) {return std::isfinite(v);});
        bool negative_diagonal = false;
        for (size_t i = 0; i < 6; ++i) {
          negative_diagonal = negative_diagonal || result.covariance_xyz_rpy[i * 7] < 0;
        }
        if (!finite_covariance || negative_diagonal) {
          // Preserve the rejected matrix before the watchdog respawns us. Keep
          // the existing rejection rules; non-finite JSON entries become null.
          std::ostringstream evidence;
          evidence << std::setprecision(12) << "{\"finite\":"
            << (finite_covariance ? "true" : "false")
            << ",\"negative_diagonal\":" << (negative_diagonal ? "true" : "false")
            << ",\"covariance_xyz_rpy\":[";
          for (size_t i = 0; i < result.covariance_xyz_rpy.size(); ++i) {
            if (i) evidence << ',';
            const auto value = result.covariance_xyz_rpy[i];
            if (std::isfinite(value)) evidence << value; else evidence << "null";
          }
          evidence << "],\"position\":[" << position.x() << ',' << position.y()
            << ',' << position.z() << "]}";
          diagnostic("invalid_sdk_covariance", evidence.str(), true);
          throw std::runtime_error(finite_covariance ? "negative SDK covariance" : "non-finite SDK covariance");
        }
        double velocity_dt = 0;
        bool velocity_valid = false;
        if (have_previous_pose_) {
          const double dt = (estimate.timestamp_ns - previous_pose_stamp_) * 1e-9;
          // Do not reject finite SDK poses by translation/rotation magnitude.
          // The external VO watchdog retains motion supervision and recovery.
          if (dt <= 0) {
            ++invalid_poses_;
            diagnostic("invalid_pose_timestamp",
              "{\"dt_s\":" + std::to_string(dt) + "}", true);
            throw std::runtime_error("non-increasing SDK pose timestamp");
          }
          if (dt <= 0.2) {
            const auto velocity = wla_vio::bodyVelocity(previous_position_, previous_rotation_, position, rotation, dt);
            const double alpha = dt / (twist_filter_tau_s_ + dt);
            filtered_linear_ += alpha * (velocity.linear - filtered_linear_);
            filtered_angular_ += alpha * (velocity.angular - filtered_angular_);
            velocity_dt = dt;
            velocity_valid = true;
          }
        }
        previous_position_ = position;
        previous_rotation_ = rotation;
        previous_pose_stamp_ = estimate.timestamp_ns;
        have_previous_pose_ = true;
        bool inertial_valid = false;
        wla_vio::HealthEvidence health_evidence;
        health_evidence.velocity_valid=velocity_valid;
        double gravity_norm=-1,gyro_norm=-1,accel_norm=-1;
        if (use_imu_) {
          bool gravity_valid = false, bias_valid = false;
          if (auto gravity = tracker_->GetLastGravity()) {
            health_evidence.gravity_available=true;
            ++gravity_frames_;
            last_gravity_ = *gravity;
            const Eigen::Vector3d g((*gravity)[0], (*gravity)[1], (*gravity)[2]);
            gravity_valid = g.allFinite() && g.norm() >= 8.0 && g.norm() <= 11.5;
            gravity_norm=g.allFinite() ? g.norm() : -1;
            health_evidence.gravity_valid=gravity_valid;
          }
          if (auto imu = tracker_->GetImuState()) {
            health_evidence.imu_available=true;
            ++imu_state_frames_;
            last_gyro_bias_ = imu->gyro_bias;
            last_accel_bias_ = imu->acc_bias;
            const Eigen::Vector3d gyro(imu->gyro_bias[0], imu->gyro_bias[1], imu->gyro_bias[2]);
            const Eigen::Vector3d accel(imu->acc_bias[0], imu->acc_bias[1], imu->acc_bias[2]);
            bias_valid = gyro.allFinite() && accel.allFinite() &&
              gyro.norm() <= max_gyro_bias_ && accel.norm() <= max_accel_bias_;
            gyro_norm=gyro.allFinite() ? gyro.norm() : -1;
            accel_norm=accel.allFinite() ? accel.norm() : -1;
            health_evidence.gyro_bias_valid=gyro.allFinite() && gyro_norm<=max_gyro_bias_;
            health_evidence.accel_bias_valid=accel.allFinite() && accel_norm<=max_accel_bias_;
          }
          inertial_valid = gravity_valid && bias_valid;
        }
        nav_msgs::msg::Odometry message;
        message.header.stamp = rclcpp::Time(estimate.timestamp_ns);
        message.header.frame_id = odom_;
        message.child_frame_id = base_;
        message.pose.pose.position.x = result.pose.translation[0];
        message.pose.pose.position.y = result.pose.translation[1];
        message.pose.pose.position.z = result.pose.translation[2];
        message.pose.pose.orientation.x = rotation.x();
        message.pose.pose.orientation.y = rotation.y();
        message.pose.pose.orientation.z = rotation.z();
        message.pose.pose.orientation.w = rotation.w();
        for (size_t i = 0; i < 36; ++i) { message.pose.covariance[i] = result.covariance_xyz_rpy[i]; }
        // SDK v17 exposes pose covariance but no public velocity covariance.
        // Estimate body-frame twist from consecutive poses; explicit uncertain covariance.
        for (size_t i = 0; i < 6; ++i) { message.twist.covariance[i * 7] = 1e6; }
        if (velocity_valid) {
          message.twist.twist.linear.x = filtered_linear_.x();
          message.twist.twist.linear.y = filtered_linear_.y();
          message.twist.twist.linear.z = filtered_linear_.z();
          message.twist.twist.angular.x = filtered_angular_.x();
          message.twist.twist.angular.y = filtered_angular_.y();
          message.twist.twist.angular.z = filtered_angular_.z();
          for (size_t i = 0; i < 6; ++i) message.twist.covariance[i * 7] =
            std::max(i < 3 ? 0.0025 : 0.01, 2.0 * message.pose.covariance[i * 7] / (velocity_dt * velocity_dt));
        }
        if ((get_clock()->now().nanoseconds() - frame_stamp) > max_age_ns_) {
          diagnostic("stale_estimate_no_output","{\"age_ns\":"+
            std::to_string(get_clock()->now().nanoseconds()-frame_stamp)+
            ",\"max_age_ns\":"+std::to_string(max_age_ns_)+"}");
          status("stale_estimate_no_output"); continue;
        }
        std_msgs::msg::Bool current_inertial;
        current_inertial.data = inertial_valid && velocity_valid;
        inertial_pub_->publish(current_inertial);
        std_msgs::msg::Bool current_visual;
        current_visual.data = !sdk_state.warming_up && sdk_state.timestamp_ns == estimate.timestamp_ns &&
          last_observations_ >= minimum_visual_observations_;
        visual_pub_->publish(current_visual);
        pub_->publish(message);
        if (broadcaster_) {
          geometry_msgs::msg::TransformStamped tf;
          tf.header = message.header;
          tf.child_frame_id = base_;
          tf.transform.translation.x = position.x();
          tf.transform.translation.y = position.y();
          tf.transform.translation.z = position.z();
          tf.transform.rotation = message.pose.pose.orientation;
          broadcaster_->sendTransform(tf);
        }
        const double gate_stamp_s=estimate.timestamp_ns * 1e-9;
        const double gate_dt_s=last_gate_stamp_s_<0 ? -1 : gate_stamp_s-last_gate_stamp_s_;
        const bool gate_stamp_gap=last_gate_stamp_s_>=0 && (gate_dt_s<=0 || gate_dt_s>0.2);
        const bool ready = initialization_gate_.update(inertial_valid && velocity_valid,
          gate_stamp_s, initialization_stable_s_);
        last_gate_stamp_s_=gate_stamp_s;
        const auto health_reason=health_evidence.reason(ready);
        std_msgs::msg::String reason_message;
        reason_message.data=health_reason;
        health_reason_pub_->publish(reason_message);
        std::ostringstream health_details;health_details<<std::setprecision(12)
          <<"{\"ready\":"<<(ready ? "true" : "false")
          <<",\"gravity_available\":"<<(health_evidence.gravity_available ? "true" : "false")
          <<",\"gravity_valid\":"<<(health_evidence.gravity_valid ? "true" : "false")
          <<",\"gravity_norm\":"<<gravity_norm
          <<",\"imu_state_available\":"<<(health_evidence.imu_available ? "true" : "false")
          <<",\"gyro_bias_valid\":"<<(health_evidence.gyro_bias_valid ? "true" : "false")
          <<",\"accel_bias_valid\":"<<(health_evidence.accel_bias_valid ? "true" : "false")
          <<",\"gyro_bias_norm\":"<<gyro_norm<<",\"accel_bias_norm\":"<<accel_norm
          <<",\"max_gyro_bias\":"<<max_gyro_bias_<<",\"max_accel_bias\":"<<max_accel_bias_
          <<",\"velocity_valid\":"<<(velocity_valid ? "true" : "false")
          <<",\"velocity_dt_s\":"<<velocity_dt
          <<",\"gate_stamp_dt_s\":"<<gate_dt_s
          <<",\"gate_stamp_gap\":"<<(gate_stamp_gap ? "true" : "false")
          <<",\"required_stable_s\":"<<initialization_stable_s_
          <<",\"estimate_age_ms\":"<<(get_clock()->now().nanoseconds()-frame_stamp)*1e-6<<"}";
        diagnostic(health_reason,health_details.str());
        // False during initialization, loss, invalid bias, gaps and stale frames.
        std_msgs::msg::Bool health;
        health.data = ready;
        if (ready) ++ready_frames_;
        health_pub_->publish(health);
        std_msgs::msg::String state;
        state.data = ready ? "tracking_inertial_ready" : "waiting_for_stable_inertial_initialization";
        status_pub_->publish(state);
        ++published_frames_;
        latency_ms_.push_back((get_clock()->now().nanoseconds() - frame_stamp) * 1e-6);
        if (latency_ms_.size() > 4096) latency_ms_.erase(latency_ms_.begin(), latency_ms_.begin() + 2048);
        if (!have_first_pose_) { first_position_ = position; have_first_pose_ = true; }
        max_displacement_ = std::max(max_displacement_, (position - first_position_).norm());
        final_position_ = position;
        if (published_frames_ % 30 == 0) {
          RCLCPP_INFO(get_logger(), "frames=%zu lost=%zu track=%.2fms age=%.2fms IMU=%zu gravity=%zu imu_state=%zu keyframes=%zu recent_keyframes=%zu displacement=%.4fm",
            published_frames_, lost_frames_, elapsed, latency_ms_.back(), registered_imu_, gravity_frames_, imu_state_frames_, keyframes_, keyframe_stamps_.size(), max_displacement_);
          write_statistics();
        }
      }
    } catch (const std::exception & ex) {
      fault_ = ex.what();
      stopped_ = true;
      RCLCPP_ERROR(get_logger(), "cuVSLAM frontend halted: %s", ex.what());
      status("fatal_frontend_fault_no_output");
    }
    write_statistics();
  }

  static double percentile(std::vector<double> values, double fraction)
  {
    if (values.empty()) { return 0; }
    std::sort(values.begin(), values.end());
    return values[static_cast<size_t>((values.size() - 1) * fraction)];
  }

  void write_statistics() noexcept
  {
    try {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto path = std::filesystem::path(statistics_path_);
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    const auto temporary = statistics_path_ + ".tmp";
    std::ofstream out(temporary);
    if (!out) {throw std::runtime_error("cannot open cuVSLAM statistics: " + temporary);}
    out << std::setprecision(9) << "{\n"
      << "  \"health_diagnostics_path\": "<<jsonString(diagnostics_path_)<<",\n"
      << "  \"last_health_reason\": "<<jsonString(last_diagnostic_reason_)<<",\n"
      << "  \"health_diagnostic_events\": "<<diagnostic_events_<<",\n"
      << "  \"validation_only\": false,\n  \"calibration_provisional\": true,\n"
      << "  \"mode\": \"" << (use_imu_ ? "Inertial" : "Multicamera") << "\",\n"
      << "  \"inertial_initialized\": " << (gravity_frames_ > 0 ? "true" : "false") << ",\n"
      << "  \"inertial_ready_frames\": " << ready_frames_ << ",\n"
      << "  \"keyframes\": " << keyframes_ << ",\n"
      << "  \"recent_keyframes\": " << keyframe_stamps_.size() << ",\n"
      << "  \"requested_initialization_keyframes\": " << requested_init_keyframes_ << ",\n"
      << "  \"image_decimation\": " << image_decimation_ << ",\n"
      << "  \"decimated_frames\": " << decimated_frames_ << ",\n"
      << "  \"faulted\": " << (fault_.empty() ? "false" : "true") << ",\n"
      << "  \"tracked_frames\": " << tracked_frames_ << ",\n  \"published_frames\": " << published_frames_ << ",\n"
      << "  \"lost_frames\": " << lost_frames_ << ",\n  \"invalid_poses\": " << invalid_poses_ << ",\n"
      << "  \"received_imu\": " << received_imu_ << ",\n  \"registered_imu\": " << registered_imu_ << ",\n"
      << "  \"gravity_frames\": " << gravity_frames_ << ",\n  \"imu_state_frames\": " << imu_state_frames_ << ",\n"
      << "  \"late_imu\": " << late_imu_ << ",\n  \"imu_gap_frames\": " << imu_gap_frames_ << ",\n"
      << "  \"wheel_samples\": " << wheel_samples_ << ",\n"
      << "  \"max_wheel_linear_mps\": " << max_wheel_linear_ << ",\n"
      << "  \"max_wheel_angular_radps\": " << max_wheel_angular_ << ",\n"
      << "  \"tracking_span_s\": " << (tracked_frames_ > 0 ? std::chrono::duration<double>(last_track_time_ - first_track_time_).count() : 0) << ",\n"
      << "  \"queue_drops\": " << queue_drops_ << ",\n  \"stale_frames\": " << stale_frames_ << ",\n"
      << "  \"unmatched_images\": " << unmatched_images_ << ",\n  \"invalid_images\": " << invalid_images_ << ",\n"
      << "  \"invalid_imu\": " << invalid_imu_ << ",\n  \"out_of_order_imu\": " << out_of_order_imu_ << ",\n"
      << "  \"out_of_order_images\": " << out_of_order_images_ << ",\n  \"imu_queue_drops\": " << imu_queue_drops_ << ",\n"
      << "  \"track_ms_p50\": " << percentile(track_ms_, .5) << ",\n  \"track_ms_p95\": " << percentile(track_ms_, .95) << ",\n"
      << "  \"output_age_ms_p50\": " << percentile(latency_ms_, .5) << ",\n  \"output_age_ms_p95\": " << percentile(latency_ms_, .95) << ",\n"
      << "  \"max_displacement_m\": " << max_displacement_ << ",\n"
      << "  \"last_gravity\": [" << last_gravity_[0] << ", " << last_gravity_[1] << ", " << last_gravity_[2] << "],\n"
      << "  \"last_gyro_bias\": [" << last_gyro_bias_[0] << ", " << last_gyro_bias_[1] << ", " << last_gyro_bias_[2] << "],\n"
      << "  \"last_accel_bias\": [" << last_accel_bias_[0] << ", " << last_accel_bias_[1] << ", " << last_accel_bias_[2] << "],\n"
      << "  \"final_position\": [" << final_position_.x() << ", " << final_position_.y() << ", " << final_position_.z() << "]\n}\n";
    out.close();
    if (!out) {throw std::runtime_error("cannot write cuVSLAM statistics: " + temporary);}
    std::filesystem::rename(temporary, path);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "cannot save frontend statistics: %s", ex.what());
    }
  }

  bool use_imu_{}, async_sba_{}, have_first_pose_{}, have_previous_pose_{};
  bool publish_tf_{};
  double initialization_stable_s_{}, twist_filter_tau_s_{}, max_gyro_bias_{}, max_accel_bias_{};
  Eigen::Vector3d filtered_linear_{Eigen::Vector3d::Zero()}, filtered_angular_{Eigen::Vector3d::Zero()};
  wla_vio::InitializationGate initialization_gate_;
  bool initialization_diagnostics_{false};
  std::unique_ptr<wla_vio::InitializationSampling> initialization_sampling_;
  size_t keyframes_{0}, requested_init_keyframes_{0};
  std::deque<int64_t> keyframe_stamps_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inertial_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_reason_pub_;
  int minimum_visual_observations_{};
  int64_t previous_pose_stamp_{};
  Eigen::Vector3d previous_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond previous_rotation_{Eigen::Quaterniond::Identity()};
  std::atomic<bool> stopped_{false};
  std::string base_, camera_link_, odom_, expected_imu_frame_, statistics_path_, fault_;
  std::string diagnostics_path_,last_diagnostic_reason_;
  std::chrono::steady_clock::time_point last_diagnostic_time_{};
  size_t diagnostic_events_{};
  double last_gate_stamp_s_{-1};
  int64_t last_observations_{-1};
  bool last_warming_up_{false};
  double frequency_{}, gyro_noise_{}, gyro_walk_{}, accel_noise_{}, accel_walk_{}, max_displacement_{};
  int depth_{}, image_decimation_{};
  size_t paired_frames_{}, decimated_frames_{};
  size_t ready_frames_{};
  int64_t imu_offset_ns_{}, tolerance_ns_{}, max_age_ns_{}, max_imu_gap_ns_{};
  int64_t last_received_imu_{}, last_registered_imu_{}, last_call_stamp_{};
  size_t received_imu_{}, registered_imu_{}, tracked_frames_{}, published_frames_{}, lost_frames_{}, invalid_poses_{};
  size_t late_imu_{}, imu_gap_frames_{}, gravity_frames_{}, imu_state_frames_{}, queue_drops_{}, stale_frames_{};
  size_t unmatched_images_{}, invalid_images_{}, invalid_imu_{}, out_of_order_imu_{}, out_of_order_images_{}, imu_queue_drops_{};
  Eigen::Isometry3d imu_pose_;
  Eigen::Isometry3d camera_mount_;
  Eigen::Vector3d first_position_{Eigen::Vector3d::Zero()}, final_position_{Eigen::Vector3d::Zero()};
  std::vector<double> track_ms_, latency_ms_;
  std::array<float, 3> last_gravity_{}, last_gyro_bias_{}, last_accel_bias_{};
  size_t wheel_samples_{};
  double max_wheel_linear_{}, max_wheel_angular_{};
  std::chrono::steady_clock::time_point first_track_time_{}, last_track_time_{};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  std::array<std::deque<Image::ConstSharedPtr>, 2> images_;
  std::deque<Imu::ConstSharedPtr> imus_;
  std::array<Info::ConstSharedPtr, 2> infos_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::unique_ptr<cuvslam::Odometry> tracker_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::array<rclcpp::Subscription<Image>::SharedPtr, 2> image_subs_;
  std::array<rclcpp::Subscription<Info>::SharedPtr, 2> info_subs_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Validation>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("cuvslam_validation"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  return 0;
}
