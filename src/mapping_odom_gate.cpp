#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

// Persistent mapping excludes provisional initialization poses; raw odometry
// remains available to the initialization supervisor and diagnostics.
class MappingOdomGate final : public rclcpp::Node {
public:
  MappingOdomGate() : Node("r680_mapping_odom_gate") {
    require_init_ = declare_parameter<bool>("require_initialization_supervisor", false);
    output_ = create_publisher<nav_msgs::msg::Odometry>("/r680_nav/mapping_odom", 1);
    ready_ = create_publisher<std_msgs::msg::Bool>("/r680_nav/mapping_odom_ready", 1);
    status_ = create_publisher<std_msgs::msg::String>("/r680_nav/mapping_odom_status", 1);
    health_sub_ = create_subscription<std_msgs::msg::Bool>("/r680_nav/vio_tracking_healthy", 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        healthy_ = m->data; health_seen_ = Clock::now();
        if (!healthy_ && started_) halt("inertial_tracking_lost");
      });
    init_sub_ = create_subscription<std_msgs::msg::String>("/r680_nav/vio_init_state", 1,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        init_ = m->data; init_seen_ = Clock::now();
        if (started_ && init_ == "failed") halt("initialization_cancelled_or_failed");
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/d455_slam/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const auto & p = m->pose.pose.position; const auto & q = m->pose.pose.orientation;
        const auto & v = m->twist.twist.linear; const auto & w = m->twist.twist.angular;
        const double t = rclcpp::Time(m->header.stamp).seconds();
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        const double age = (now() - rclcpp::Time(m->header.stamp)).seconds();
        if (!permitted() || m->header.frame_id != "d455_floor_odom" ||
          m->child_frame_id != "r680_mapping_floor" || age < -0.1 || age > 0.35 ||
          !std::isfinite(p.x+p.y+p.z+norm+v.x+v.y+v.z+w.x+w.y+w.z) ||
          norm < 0.98 || norm > 1.02 || t <= last_stamp_) return;
        last_stamp_ = t; output_->publish(*m); forwarded_ = Clock::now(); started_ = true;
      });
    timer_ = create_wall_timer(100ms, [this]() {
      if (started_ && !halted_ && (!permitted() || !fresh(forwarded_, 0.35))) halt("tracking_or_odometry_stale");
      std_msgs::msg::Bool m; m.data = permitted() && fresh(forwarded_, 0.35); ready_->publish(m);
      std_msgs::msg::String status; status.data = halted_ ? reason_ : (m.data ? "mapping" : "waiting_for_inertial_and_stop");
      status_->publish(status);
    });
  }
private:
  using Clock = std::chrono::steady_clock;
  bool fresh(Clock::time_point t, double limit) const {
    return t.time_since_epoch().count() != 0 && std::chrono::duration<double>(Clock::now()-t).count() <= limit;
  }
  bool permitted() const {
    return !halted_ && healthy_ && fresh(health_seen_, 0.35) && (!require_init_ ||
      (fresh(init_seen_, 0.20) && (init_ == "waiting_localization" || init_ == "succeeded")));
  }
  void halt(const std::string & reason) {
    if (halted_) return;
    halted_ = true; reason_ = reason;
    RCLCPP_ERROR(get_logger(), "Mapping odometry locked: %s; save this map and restart, no automatic session splice", reason.c_str());
  }
  bool require_init_{false}, healthy_{false}; std::string init_; double last_stamp_{-1};
  bool started_{false}, halted_{false}; std::string reason_;
  Clock::time_point health_seen_{}, init_seen_{}, forwarded_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr init_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<MappingOdomGate>()); rclcpp::shutdown();
}
