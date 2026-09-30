#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <Eigen/Geometry>
#include "cuvslam2.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
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
  Validation() : Node("cuvslam_validation"), buffer_(get_clock()), listener_(buffer_)
  {
    use_imu_ = declare_parameter<bool>("use_imu", true);
    async_sba_ = declare_parameter<bool>("async_sba", false);
    max_pose_speed_ = declare_parameter<double>("max_pose_speed_mps", 2.0);
    max_pose_angular_speed_ = declare_parameter<double>("max_pose_angular_speed_radps", 4.0);
    base_ = declare_parameter<std::string>("base_frame", "r680_mapping_floor");
    camera_link_ = declare_parameter<std::string>("camera_link_frame", "d455_link");
    camera_mount_ = from_array(declare_parameter<std::vector<double>>("base_from_camera_link",
      {0.184428484, 0.059803590, 0.501660007, 0.000096773, 0.077125800, -0.000560247, 0.997021207}));
    odom_ = declare_parameter<std::string>("odom_frame", "cuvslam_validation_odom");
    expected_imu_frame_ = declare_parameter<std::string>("expected_imu_frame", "gyro_link");
    statistics_path_ = declare_parameter<std::string>("statistics_path", "/tmp/cuvslam_validation_statistics.json");
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
      max_age_ns_ <= 0 || max_imu_gap_ns_ <= 0 || image_decimation_ < 1 || image_decimation_ > 6 ||
      !std::isfinite(max_pose_speed_) || max_pose_speed_ <= 0 ||
      !std::isfinite(max_pose_angular_speed_) || max_pose_angular_speed_ <= 0) {
      throw std::invalid_argument("invalid timing or queue parameters");
    }
    pub_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("output_topic", "/r680_vio_test/odom"), 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("status_topic", "/r680_vio_test/status"), 10);
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
      declare_parameter<std::string>("imu_topic", "/r680_vio_test/chassis/imu_raw"),
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
      declare_parameter<std::string>("wheel_odom_topic", "/r680_vio_test/wheel_odom"),
      rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Observation only: wheel data is NOT an estimator input.
        ++wheel_samples_;
        max_wheel_linear_ = std::max(max_wheel_linear_, std::abs(msg->twist.twist.linear.x));
        max_wheel_angular_ = std::max(max_wheel_angular_, std::abs(msg->twist.twist.angular.z));
      });
    RCLCPP_WARN(get_logger(), "VALIDATION ONLY: no command publisher, no TF broadcaster; chassis extrinsic/noise/time offset are provisional");
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
  void status(const std::string & state)
  {
    std_msgs::msg::String msg;
    msg.data = state;
    status_pub_->publish(msg);
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
        if (imu_gap) { ++imu_gap_frames_; status("imu_gap_no_output"); continue; }
        cuvslam::Odometry::ImageSet images;
        for (size_t i = 0; i < 2; ++i) {
          if (pair[i]->width != infos[i]->width || pair[i]->height != infos[i]->height) {
            throw std::runtime_error("image dimensions changed; restart validation explicitly");
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
        const auto estimate = tracker_->Track(images);
        const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        if (tracked_frames_ == 0) { first_track_time_ = start; }
        last_track_time_ = std::chrono::steady_clock::now();
        last_call_stamp_ = frame_stamp;
        ++tracked_frames_;
        track_ms_.push_back(elapsed);
        if (!estimate.world_from_rig) { ++lost_frames_; status("tracking_lost_no_output"); continue; }
        const auto & result = *estimate.world_from_rig;
        bool finite = std::all_of(result.pose.translation.begin(), result.pose.translation.end(), [](float v) {return std::isfinite(v);}) &&
          std::all_of(result.pose.rotation.begin(), result.pose.rotation.end(), [](float v) {return std::isfinite(v);});
        if (!finite) { ++invalid_poses_; status("invalid_pose_no_output"); continue; }
        const Eigen::Vector3d position(result.pose.translation[0], result.pose.translation[1], result.pose.translation[2]);
        const Eigen::Quaterniond rotation(result.pose.rotation[3], result.pose.rotation[0],
          result.pose.rotation[1], result.pose.rotation[2]);
        if (have_previous_pose_) {
          const double dt = (estimate.timestamp_ns - previous_pose_stamp_) * 1e-9;
          if (dt <= 0 || (position - previous_position_).norm() > max_pose_speed_ * dt ||
              previous_rotation_.angularDistance(rotation) > max_pose_angular_speed_ * dt) {
            ++invalid_poses_;
            status("implausible_pose_halted_no_output");
            throw std::runtime_error("estimated pose exceeds bounded sensor-test motion limits; capture retained, no navigation hookup");
          }
        }
        previous_position_ = position;
        previous_rotation_ = rotation;
        previous_pose_stamp_ = estimate.timestamp_ns;
        have_previous_pose_ = true;
        if (use_imu_) {
          if (auto gravity = tracker_->GetLastGravity()) {
            ++gravity_frames_;
            last_gravity_ = *gravity;
          }
          if (auto imu = tracker_->GetImuState()) {
            ++imu_state_frames_;
            last_gyro_bias_ = imu->gyro_bias;
            last_accel_bias_ = imu->acc_bias;
          }
        }
        nav_msgs::msg::Odometry message;
        message.header.stamp = rclcpp::Time(estimate.timestamp_ns);
        message.header.frame_id = odom_;
        message.child_frame_id = base_;
        message.pose.pose.position.x = result.pose.translation[0];
        message.pose.pose.position.y = result.pose.translation[1];
        message.pose.pose.position.z = result.pose.translation[2];
        message.pose.pose.orientation.x = result.pose.rotation[0];
        message.pose.pose.orientation.y = result.pose.rotation[1];
        message.pose.pose.orientation.z = result.pose.rotation[2];
        message.pose.pose.orientation.w = result.pose.rotation[3];
        for (size_t i = 0; i < 36; ++i) { message.pose.covariance[i] = result.covariance_xyz_rpy[i]; }
        // This isolated probe publishes pose only. Mark twist as unmeasured rather than inventing zero speed feedback.
        for (size_t i = 0; i < 6; ++i) { message.twist.covariance[i * 7] = 1e6; }
        pub_->publish(message);
        ++published_frames_;
        latency_ms_.push_back((get_clock()->now().nanoseconds() - frame_stamp) * 1e-6);
        if (!have_first_pose_) { first_position_ = position; have_first_pose_ = true; }
        max_displacement_ = std::max(max_displacement_, (position - first_position_).norm());
        final_position_ = position;
        if (published_frames_ % 30 == 0) {
          status(use_imu_ ? (gravity_frames_ > 0 ? "tracking_inertial_validation_only" :
            "tracking_visual_waiting_inertial_initialization") : "tracking_stereo_validation_only");
          RCLCPP_INFO(get_logger(), "frames=%zu lost=%zu track=%.2fms age=%.2fms IMU=%zu gravity=%zu imu_state=%zu displacement=%.4fm",
            published_frames_, lost_frames_, elapsed, latency_ms_.back(), registered_imu_, gravity_frames_, imu_state_frames_, max_displacement_);
        }
      }
    } catch (const std::exception & ex) {
      fault_ = ex.what();
      stopped_ = true;
      RCLCPP_ERROR(get_logger(), "validation halted: %s", ex.what());
      status("fatal_validation_fault_no_output");
    }
    write_statistics();
  }

  static double percentile(std::vector<double> values, double fraction)
  {
    if (values.empty()) { return 0; }
    std::sort(values.begin(), values.end());
    return values[static_cast<size_t>((values.size() - 1) * fraction)];
  }

  void write_statistics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ofstream out(statistics_path_);
    out << std::setprecision(9) << "{\n"
      << "  \"validation_only\": true,\n  \"calibration_provisional\": true,\n"
      << "  \"mode\": \"" << (use_imu_ ? "Inertial" : "Multicamera") << "\",\n"
      << "  \"inertial_initialized\": " << (gravity_frames_ > 0 ? "true" : "false") << ",\n"
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
  }

  bool use_imu_{}, async_sba_{}, have_first_pose_{}, have_previous_pose_{};
  double max_pose_speed_{}, max_pose_angular_speed_{};
  int64_t previous_pose_stamp_{};
  Eigen::Vector3d previous_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond previous_rotation_{Eigen::Quaterniond::Identity()};
  std::atomic<bool> stopped_{false};
  std::string base_, camera_link_, odom_, expected_imu_frame_, statistics_path_, fault_;
  double frequency_{}, gyro_noise_{}, gyro_walk_{}, accel_noise_{}, accel_walk_{}, max_displacement_{};
  int depth_{}, image_decimation_{};
  size_t paired_frames_{}, decimated_frames_{};
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
