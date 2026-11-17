#include "wla_uvc_depth/ground_filter.h"
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
namespace wla::uvc {
using Json = nlohmann::json;
namespace {
cv::Matx33d rotation(const std::vector<double> &q) {
  if (q.size() != 4)
    throw std::runtime_error("quaternion_size");
  double norm = 0;
  for (auto v : q)
    norm += v * v;
  if (!std::isfinite(norm) || std::abs(norm - 1) > .01)
    throw std::runtime_error("invalid_quaternion");
  const tf2::Matrix3x3 m(tf2::Quaternion(q[0], q[1], q[2], q[3]).normalized());
  cv::Matx33d out;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out(i, j) = m[i][j];
  return out;
}
cv::Vec3d vec(const Json &a) {
  if (!a.is_array() || a.size() != 3)
    throw std::runtime_error("invalid_vector");
  cv::Vec3d v(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
  if (!std::isfinite(cv::norm(v)))
    throw std::runtime_error("invalid_vector");
  return v;
}
double seconds(const builtin_interfaces::msg::Time &t) {
  return t.sec + t.nanosec * 1e-9;
}
struct Support {
  double joint, imu;
  cv::Vec3d n, t;
  double h;
  cv::Matx33d cameraR;
};
} // namespace
class GroundFilterNode final : public rclcpp::Node {
public:
  GroundFilterNode()
      : Node("stereo_ground_filter"), tf_(get_clock()), listener_(tf_) {
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    json_ = create_publisher<std_msgs::msg::String>(
        "/r680/debug/stereo/ground_filter", qos);
    cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/camera/stereo/points_ground_filtered", qos);
    depthSub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/stereo/depth/image_raw", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { depth_ = m; });
    infoSub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/stereo/depth/camera_info", qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { info_ = m; });
    supportSub_ = create_subscription<std_msgs::msg::String>(
        "/robot_state/geometry", qos, [this](const std_msgs::msg::String &m) {
          try {
            const auto j = Json::parse(m.data);
            supportValid_ = false;
            if (j.at("schema") != "robot-geometry/v1" ||
                j.at("frame") != "base_Link" ||
                !j.at("available").get<bool>() ||
                !j.at("ground").at("available").get<bool>())
              return;
            const auto &g = j.at("ground");
            if (g.at("reason") != "operator_confirmed_both_contact_flat_ground")
              return;
            const double joint = j.at("joint_stamp"), imu = j.at("imu_stamp");
            if (!fresh(joint, .2) || !fresh(imu, .2) ||
                std::abs(joint - imu) > .1)
              return;
            const auto heights =
                g.at("wheel_heights_m").get<std::vector<double>>();
            if (heights.size() != 2 || !std::isfinite(heights[0]) ||
                !std::isfinite(heights[1]) ||
                std::abs(heights[0] - heights[1]) > .05)
              return;
            for (const auto &f : j.at("frames"))
              if (f.at("name") == "bottom_camera_L") {
                Support s{joint,
                          imu,
                          vec(g.at("normal")),
                          vec(f.at("p")),
                          g.at("offset_m").get<double>(),
                          rotation(f.at("q").get<std::vector<double>>())};
                if (cv::norm(s.t) > .3)
                  throw std::runtime_error("mount_out_of_bounds");
                support_.push_back(s);
                while (support_.size() > 50)
                  support_.pop_front();
                supportValid_ = true;
                break;
              }
          } catch (const std::exception &) {
            supportValid_ = false;
          }
        });
    timer_ =
        create_wall_timer(std::chrono::milliseconds(200), [this] { tick(); });
  }

private:
  bool fresh(double t, double limit) const {
    const double age = now().seconds() - t;
    return std::isfinite(t) && t > 0 && age >= -.02 && age <= limit;
  }
  void publish(const GroundFilter &g, double stamp, Json extra,
               const cv::Matx33d &cameraR = cv::Matx33d::eye(),
               const cv::Vec3d &cameraT = cv::Vec3d()) {
    Json j = {{"schema", "stereo-ground-filter/v1"},
              {"frame", "sunplus_left_rect_optical_frame"},
              {"stamp", stamp},
              {"available", g.available},
              {"reason", g.reason},
              {"temporal_fusion", false},
              {"navigation_connected", false},
              {"geometry_calibrated", false},
              {"ground_removed", g.available},
              {"filter_band_m", .015},
              {"visual_coordinates", "fitted_plane_view_not_tf"}};
    j["plane"] = {{"normal_base", {g.normal[0], g.normal[1], g.normal[2]}},
                  {"offset_m", g.offset},
                  {"angle_to_support_deg", g.angleDeg},
                  {"offset_delta_m", g.offsetDelta},
                  {"prior_residual_m", g.priorResidual},
                  {"inlier_ratio", g.ratio},
                  {"rms_m", g.rms}};
    j["ground_points"] = g.ground.size();
    j["obstacle_points"] = g.obstacles.size();
    j["uncertain_points"] = g.uncertain.size();
    j.update(extra);
    Json points = Json::array();
    auto add = [&](const std::vector<cv::Vec3f> &p, int cls) {
      const size_t stride = std::max(size_t(1), (p.size() + 1599) / 1600);
      for (size_t i = 0; i < p.size(); i += stride) {
        auto q = g.bodyToPlane * cv::Vec3d(p[i]);
        q[2] += g.offset;
        points.push_back({int(std::lround(q[0] * 1000)),
                          int(std::lround(q[1] * 1000)),
                          int(std::lround(q[2] * 1000)), cls});
      }
    };
    if (g.available) {
      add(g.ground, 1);
      add(g.obstacles, 2);
      add(g.uncertain, 4);
    }
    j["points_mm"] = std::move(points);
    std_msgs::msg::String msg;
    msg.data = j.dump();
    json_->publish(msg);
    if (cloud_->get_subscription_count()) {
      sensor_msgs::msg::PointCloud2 pc;
      pc.header.frame_id = "sunplus_left_rect_optical_frame";
      pc.header.stamp = rclcpp::Time(int64_t(stamp * 1e9));
      if (g.available)
        pc.header.stamp = depth_->header.stamp;
      sensor_msgs::PointCloud2Modifier mod(pc);
      mod.setPointCloud2FieldsByString(1, "xyz");
      mod.resize(g.available ? g.obstacles.size() + g.uncertain.size() : 0);
      if (g.available) {
        sensor_msgs::PointCloud2Iterator<float> x(pc, "x"), y(pc, "y"),
            z(pc, "z");
        for (const auto *points : {&g.obstacles, &g.uncertain})
          for (const auto &p : *points) {
            const auto q = cameraR.t() * (cv::Vec3d(p) - cameraT);
            *x = q[0];
            *y = q[1];
            *z = q[2];
            ++x;
            ++y;
            ++z;
          }
      }
      cloud_->publish(pc);
    }
  }
  void tick() {
    if (!json_->get_subscription_count() && !cloud_->get_subscription_count())
      return;
    GroundFilter invalid;
    auto fail = [&](const char *s) {
      invalid.reason = s;
      publish(invalid, now().seconds(), Json::object());
    };
    if (!depth_ || !info_ || !supportValid_ || support_.empty()) {
      fail("waiting_for_depth_and_timestamped_support");
      return;
    }
    const auto &d = *depth_;
    const auto &k = *info_;
    const double stamp = seconds(d.header.stamp);
    if (!fresh(stamp, .30)) {
      fail("stale_depth");
      return;
    }
    if (d.header.frame_id != "sunplus_left_rect_optical_frame" ||
        k.header.frame_id != d.header.frame_id || k.width != d.width ||
        k.height != d.height ||
        std::abs(seconds(k.header.stamp) - stamp) > .1) {
      fail("depth_info_mismatch");
      return;
    }
    if (d.encoding != "16UC1" || d.is_bigendian || d.width < 2 ||
        d.height < 2 || d.width > 1920 || d.height > 1080 ||
        d.step < d.width * 2 || d.step % 2 ||
        d.data.size() != size_t(d.step) * d.height) {
      fail("depth_layout");
      return;
    }
    if (std::abs(k.k[2]) > 2 * d.width || std::abs(k.k[5]) > 2 * d.height ||
        k.distortion_model != "plumb_bob" || k.d.size() != 5 || k.k[0] < 1 ||
        k.k[4] < 1 || k.k[0] > 10000 || k.k[4] > 10000 || k.k[1] != 0 ||
        k.k[3] != 0 || k.k[6] != 0 || k.k[7] != 0 || k.k[8] != 1) {
      fail("intrinsics");
      return;
    }
    for (auto v : k.k)
      if (!std::isfinite(v)) {
        fail("intrinsics");
        return;
      }
    for (auto v : k.d)
      if (v != 0) {
        fail("non_rectified_input");
        return;
      }
    const auto s = std::min_element(
        support_.begin(), support_.end(),
        [stamp](const auto &a, const auto &b) {
          return std::max(std::abs(a.joint - stamp), std::abs(a.imu - stamp)) <
                 std::max(std::abs(b.joint - stamp), std::abs(b.imu - stamp));
        });
    const double skew =
        std::max(std::abs(s->joint - stamp), std::abs(s->imu - stamp));
    if (!fresh(s->joint, .2) || !fresh(s->imu, .2) || skew > .075) {
      fail("stale_or_skewed_support");
      return;
    }
    if (stamp == lastStamp_)
      return;
    lastStamp_ = stamp;
    const auto begin = std::chrono::steady_clock::now();
    try {
      const auto t =
          tf_.lookupTransform("sunplus_left_optical_frame", d.header.frame_id,
                              rclcpp::Time(d.header.stamp));
      const auto &q = t.transform.rotation;
      const auto rect = rotation({q.x, q.y, q.z, q.w});
      const auto &tr = t.transform.translation;
      if (std::abs(tr.x) + std::abs(tr.y) + std::abs(tr.z) > 1e-6)
        throw std::runtime_error("unexpected_rect_translation");
      const auto r =
          s->cameraR * cv::Matx33d(0, 0, 1, -1, 0, 0, 0, -1, 0) * rect;
      std::vector<GroundPoint> points;
      points.reserve((d.width / 3 + 1) * (d.height / 3 + 1));
      for (uint32_t v = 0; v < d.height; v += 3)
        for (uint32_t u = 0; u < d.width; u += 3) {
          uint16_t raw;
          std::memcpy(&raw, d.data.data() + v * d.step + u * 2, 2);
          const double z = raw * .001;
          if (z < .15 || z > 3)
            continue;
          const cv::Vec3d c((u - k.k[2]) * z / k.k[0],
                            (v - k.k[5]) * z / k.k[4], z);
          points.push_back({r * c + s->t, v > d.height * .45});
        }
      const auto g = MakeGroundFilter(points, s->n, s->h);
      if (!fresh(stamp, .30)) {
        fail("processing_exceeded_freshness");
        return;
      }
      publish(g, stamp,
              {{"compute_ms", std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - begin)
                                  .count()},
               {"support_offset_m", s->h},
               {"support_skew_ms", skew * 1000},
               {"source_age_ms", (now().seconds() - stamp) * 1000},
               {"camera_mount_source", "SW_mount_with_standard_optical_axis_"
                                       "convention_not_hand_eye_calibrated"}},
              r, s->t);
    } catch (const std::exception &e) {
      fail(e.what());
    }
  }
  double lastStamp_ = 0;
  bool supportValid_ = false;
  std::deque<Support> support_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  sensor_msgs::msg::Image::ConstSharedPtr depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr supportSub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace wla::uvc
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wla::uvc::GroundFilterNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("ground_filter"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
