// D455 depth -> slope/step terrain analysis (core: terrain_core.hpp).
// Publishes, in the robot base frame, the same cloud roles as
// wla_r680_navigation/depth_to_points so Nav2 needs no config change:
//   obstacle cloud  - z is the height above the LOCAL ground plane (not the
//                     base), so ramps are not obstacles; blocked steep/step/drop
//                     cells appear as points at z = 0.2 m;
//   clearing cloud  - analysed terrain points, z = height above ground (clamped)
// plus a CMU-style terrain map (odom frame, intensity = height above ground).
#include <algorithm>
#include <chrono>
#include <cstring>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "wla_terrain_analysis/terrain_core.hpp"

using wla_terrain::Point;
using wla_terrain::Pose;

class TerrainAnalysisNode : public rclcpp::Node {
 public:
  TerrainAnalysisNode() : Node("terrain_analysis") {
    base_ = declare_parameter<std::string>("base_frame", "r680_mapping_floor");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "terrain_odom");
    stride_ = declare_parameter<int>("stride", 4);
    min_depth_ = declare_parameter<double>("min_depth", .15);
    max_depth_ = declare_parameter<double>("max_depth", 5.);
    period_ = 1. / declare_parameter<double>("max_rate", 10.);
    max_pose_gap_ = declare_parameter<double>("max_pose_gap_s", .1);
    wla_terrain::Params p;
    p.planar_cell = declare_parameter<double>("planar_cell", p.planar_cell);
    p.ground_quantile = declare_parameter<double>("ground_quantile", p.ground_quantile);
    p.min_cell_points = declare_parameter<int>("min_cell_points", p.min_cell_points);
    p.fit_radius = declare_parameter<double>("fit_radius", p.fit_radius);
    p.fit_outlier = declare_parameter<double>("fit_outlier", p.fit_outlier);
    p.obstacle_height = declare_parameter<double>("obstacle_height", p.obstacle_height);
    p.vehicle_height = declare_parameter<double>("vehicle_height", p.vehicle_height);
    p.max_slope_deg = declare_parameter<double>("max_slope_deg", p.max_slope_deg);
    p.slope_margin_deg = declare_parameter<double>("slope_margin_deg", p.slope_margin_deg);
    p.max_step = declare_parameter<double>("max_step", p.max_step);
    p.step_span = declare_parameter<double>("step_span", p.step_span);
    p.drop_shadow = declare_parameter<double>("drop_shadow", p.drop_shadow);
    p.replace_age = declare_parameter<double>("replace_age", p.replace_age);
    p.decay_time = declare_parameter<double>("decay_time", p.decay_time);
    p.keep_radius = declare_parameter<double>("keep_radius", p.keep_radius);
    p.map_radius = declare_parameter<double>("map_radius", p.map_radius);
    p.terrain_range = declare_parameter<double>("terrain_range", p.terrain_range);
    p.min_confirm = declare_parameter<int>("min_confirm", p.min_confirm);
    analyzer_ = wla_terrain::TerrainAnalyzer(p);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(2));
    obstacles_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("output_topic", "/r680_nav/d455/points"), qos);
    safety_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("safety_output_topic", "/r680_nav/d455/points_safety"), qos);
    clearing_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("clearing_output_topic", "/r680_nav/d455/clearing_points"), qos);
    terrain_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("terrain_map_topic", "/r680_nav/terrain_map"), qos);
    cells_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("blocked_cells_topic", "/r680_nav/terrain_blocked_cells"), qos);

    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        declare_parameter<std::string>("camera_info_topic", "/r680/d455/color/camera_info"),
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { info_ = m; });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("odom_topic", "/r680_nav/terrain_odom"), 20,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          poses_.push_back(m);
          while (poses_.size() > 200) poses_.pop_front();
        });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("depth_topic", "/r680/d455/aligned_depth_to_color/image_raw"),
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_depth(*m); });
  }

 private:
  static double seconds(const builtin_interfaces::msg::Time &t) { return t.sec + t.nanosec * 1e-9; }

  bool pose_at(double t, Pose &out) const {
    const nav_msgs::msg::Odometry *best = nullptr;
    double gap = max_pose_gap_;
    for (const auto &m : poses_) {
      const double d = std::abs(seconds(m->header.stamp) - t);
      if (d <= gap) { gap = d; best = m.get(); }
    }
    if (!best || best->pose.covariance[21] > 1.) return false;  // attitude not yet valid
    const auto &q = best->pose.pose.orientation;
    double r, p, y;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(r, p, y);
    out = {best->pose.pose.position.x, best->pose.pose.position.y, best->pose.pose.position.z, r, p, y};
    return true;
  }

  void on_depth(const sensor_msgs::msg::Image &image) {
    const double t = seconds(image.header.stamp);
    if (t - last_ < period_ || !info_ || info_->width != image.width || info_->height != image.height) return;
    const bool mm = image.encoding == "16UC1" || image.encoding == "mono16";
    if (!mm && image.encoding != "32FC1") return;
    Pose pose;
    if (!pose_at(t, pose)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no attitude-valid terrain odometry near the depth stamp");
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_->lookupTransform(base_, image.header.frame_id, tf2::TimePointZero);  // rigid mount
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera mount TF: %s", e.what());
      return;
    }
    last_ = t;
    const auto started = std::chrono::steady_clock::now();
    const auto &tq = tf.transform.rotation;
    tf2::Matrix3x3 mount(tf2::Quaternion(tq.x, tq.y, tq.z, tq.w));
    const auto &tt = tf.transform.translation;
    const double fx = info_->k[0], fy = info_->k[4], cx = info_->k[2], cy = info_->k[5];
    const auto r = wla_terrain::rotation(pose);
    std::vector<Point> points;
    points.reserve((image.width / stride_ + 1) * (image.height / stride_ + 1));
    for (uint32_t v = 0; v < image.height; v += stride_) {
      const uint8_t *row = image.data.data() + v * image.step;
      for (uint32_t u = 0; u < image.width; u += stride_) {
        double z;
        if (mm) { uint16_t raw; std::memcpy(&raw, row + 2 * u, 2); z = raw * 1e-3; }
        else { float raw; std::memcpy(&raw, row + 4 * u, 4); z = raw; }
        if (!std::isfinite(z) || z < min_depth_ || z > max_depth_) continue;
        const tf2::Vector3 c((u - cx) * z / fx, (v - cy) * z / fy, z);
        const tf2::Vector3 b = mount * c;
        points.push_back(wla_terrain::to_odom(pose, r, {b.x() + tt.x, b.y() + tt.y, b.z() + tt.z}));
      }
    }
    analyzer_.add_frame(points, pose, t);
    const auto out = analyzer_.analyze(pose, t);
    // Base-frame (yaw-level) output: horizontal offsets rotated by -yaw, z = height above ground.
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    auto to_base = [&](const std::array<double, 4> &q, double z) {
      const double dx = q[0] - pose.x, dy = q[1] - pose.y;
      return std::array<float, 3>{float(c * dx + s * dy), float(-s * dx + c * dy), float(z)};
    };
    std::vector<std::array<float, 3>> obstacles, clearing;
    for (const auto &q : out.obstacles) obstacles.push_back(to_base(q, std::clamp(q[3], analyzer_.params().obstacle_height, analyzer_.params().vehicle_height - .01)));
    for (const auto &q : out.terrain) if (q[3] < analyzer_.params().vehicle_height) clearing.push_back(to_base(q, std::clamp(q[3], -.05, .45)));
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    sum_ms_ += ms; max_ms_ = std::max(max_ms_, ms); ++frames_;
    if (frames_ >= 50) {
      RCLCPP_INFO(get_logger(), "terrain: %.1f ms mean, %.1f ms max over %d frames; %zu obstacle pts, %zu stored pts",
                  sum_ms_ / frames_, max_ms_, frames_, obstacles.size(), analyzer_.stored_points());
      sum_ms_ = max_ms_ = 0; frames_ = 0;
    }
    obstacles_->publish(cloud(image.header.stamp, base_, obstacles));
    safety_->publish(cloud(image.header.stamp, base_, obstacles));
    clearing_->publish(cloud(image.header.stamp, base_, clearing));
    if (cells_->get_subscription_count() > 0) {
      // Diagnostics: confirmed blocked cells (odom frame), intensity = 10*class + reason.
      sensor_msgs::msg::PointCloud2 m;
      m.header.stamp = image.header.stamp; m.header.frame_id = odom_frame_;
      sensor_msgs::PointCloud2Modifier mod(m);
      mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                               "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
      std::vector<const wla_terrain::CellResult *> blocked;
      for (const auto &cr : out.cells) if (cr.cls != wla_terrain::CellClass::Ground && cr.cls != wla_terrain::CellClass::Unknown) blocked.push_back(&cr);
      mod.resize(blocked.size());
      sensor_msgs::PointCloud2Iterator<float> x(m, "x"), y(m, "y"), z(m, "z"), in(m, "intensity");
      const double cs = analyzer_.params().planar_cell;
      for (const auto *cr : blocked) {
        *x = (cr->ix + .5) * cs; *y = (cr->iy + .5) * cs; *z = cr->ground;
        *in = 10.f * static_cast<int>(cr->cls) + cr->reason; ++x; ++y; ++z; ++in;
      }
      cells_->publish(m);
    }
    if (terrain_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 map;
      map.header.stamp = image.header.stamp; map.header.frame_id = odom_frame_;
      sensor_msgs::PointCloud2Modifier mod(map);
      mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                               "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
      mod.resize(out.terrain.size());
      sensor_msgs::PointCloud2Iterator<float> x(map, "x"), y(map, "y"), z(map, "z"), in(map, "intensity");
      for (const auto &q : out.terrain) { *x = q[0]; *y = q[1]; *z = q[2]; *in = q[3]; ++x; ++y; ++z; ++in; }
      terrain_->publish(map);
    }
  }

  static sensor_msgs::msg::PointCloud2 cloud(const builtin_interfaces::msg::Time &stamp, const std::string &frame,
                                             const std::vector<std::array<float, 3>> &pts) {
    sensor_msgs::msg::PointCloud2 m;
    m.header.stamp = stamp; m.header.frame_id = frame;
    sensor_msgs::PointCloud2Modifier mod(m);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> x(m, "x"), y(m, "y"), z(m, "z");
    for (const auto &p : pts) { *x = p[0]; *y = p[1]; *z = p[2]; ++x; ++y; ++z; }
    return m;
  }

  std::string base_, odom_frame_;
  int stride_;
  double min_depth_, max_depth_, period_, max_pose_gap_, last_ = 0, sum_ms_ = 0, max_ms_ = 0;
  int frames_ = 0;
  wla_terrain::TerrainAnalyzer analyzer_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> poses_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_, safety_, clearing_, terrain_, cells_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainAnalysisNode>());
  rclcpp::shutdown();
  return 0;
}
