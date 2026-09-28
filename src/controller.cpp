#include "wla_diff_mpc/qp.hpp"
#include "wla_diff_mpc/solver.hpp"

#include <nav2_core/controller.hpp>
#if __has_include(<nav2_core/controller_exceptions.hpp>)
#include <nav2_core/controller_exceptions.hpp>
#else
#include <nav2_core/exceptions.hpp>
#endif
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace wla_diff_mpc {
namespace {

#if __has_include(<nav2_core/controller_exceptions.hpp>)
using ControlError = nav2_core::ControllerException;
using RetryableControlError = nav2_core::NoValidControl;
#else
using ControlError = nav2_core::PlannerException;
using RetryableControlError = nav2_core::PlannerException;
#endif

double unwrap(double angle, double previous) {
  return previous + std::remainder(angle - previous, 2.0 * M_PI);
}

Reference samplePath(const nav_msgs::msg::Path & path, const State & current,
                     int horizon, double dt, double v_max, double w_max) {
  if (path.poses.empty()) throw ControlError("MPC has no path");
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    const double d = std::hypot(p.x - current[0], p.y - current[1]);
    if (d < best) { best = d; nearest = i; }
  }
  if (best > 1.5) throw ControlError("MPC path is too far from robot");

  const double first_heading = nearest + 1 < path.poses.size() ?
    std::atan2(path.poses[nearest + 1].pose.position.y - path.poses[nearest].pose.position.y,
               path.poses[nearest + 1].pose.position.x - path.poses[nearest].pose.position.x) :
    tf2::getYaw(path.poses.back().pose.orientation);
  const double heading_error = std::remainder(first_heading - current[2], 2.0 * M_PI);
  if (std::abs(heading_error) > 0.7) {
    Reference rotation;
    rotation.states.reserve(horizon + 1);
    rotation.inputs.reserve(horizon);
    for (int k = 0; k <= horizon; ++k) {
      const double progress = std::min(std::abs(heading_error), k * w_max * dt);
      rotation.states.emplace_back(current[0], current[1],
        current[2] + std::copysign(progress, heading_error));
      if (k > 0) rotation.inputs.emplace_back(0.0,
        (rotation.states[k][2] - rotation.states[k - 1][2]) / dt);
    }
    return rotation;
  }

  Reference ref;
  ref.states.reserve(horizon + 1);
  ref.inputs.reserve(horizon);
  const double step_length = v_max * dt;
  size_t segment = nearest;
  double segment_offset = 0.0;
  for (int k = 0; k <= horizon; ++k) {
    double advance = k == 0 ? 0.0 : step_length;
    while (advance > 0.0 && segment + 1 < path.poses.size()) {
      const auto & a = path.poses[segment].pose.position;
      const auto & b = path.poses[segment + 1].pose.position;
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      if (length < 1e-6) { ++segment; segment_offset = 0.0; continue; }
      const double available = length - segment_offset;
      if (advance < available) { segment_offset += advance; advance = 0.0; }
      else { advance -= available; ++segment; segment_offset = 0.0; }
    }
    State x;
    if (segment + 1 < path.poses.size()) {
      const auto & a = path.poses[segment].pose.position;
      const auto & b = path.poses[segment + 1].pose.position;
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      const double t = length > 1e-6 ? segment_offset / length : 0.0;
      x << a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
        std::atan2(b.y - a.y, b.x - a.x);
    } else {
      const auto & goal = path.poses.back().pose;
      x << goal.position.x, goal.position.y, tf2::getYaw(goal.orientation);
    }
    x[2] = unwrap(x[2], k == 0 ? current[2] : ref.states.back()[2]);
    ref.states.push_back(x);
  }
  for (int k = 0; k < horizon; ++k) {
    const auto delta = ref.states[k + 1] - ref.states[k];
    Input u;
    u << std::min(v_max, std::hypot(delta[0], delta[1]) / dt),
      std::clamp(delta[2] / dt, -w_max, w_max);
    ref.inputs.push_back(u);
  }
  return ref;
}

}  // namespace

class DiffMpcController : public nav2_core::Controller {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_ = parent.lock();
    if (!node_) throw ControlError("MPC parent node expired");
    name_ = std::move(name);
    tf_ = std::move(tf);
    costmap_ros_ = std::move(costmap);
    costmap_ = costmap_ros_->getCostmap();
    auto integer = [&](const std::string & key, int default_value) {
      nav2_util::declare_parameter_if_not_declared(node_, name_ + "." + key,
        rclcpp::ParameterValue(default_value));
      return node_->get_parameter(name_ + "." + key).as_int();
    };
    auto number = [&](const std::string & key, double default_value) {
      nav2_util::declare_parameter_if_not_declared(node_, name_ + "." + key,
        rclcpp::ParameterValue(default_value));
      return node_->get_parameter(name_ + "." + key).as_double();
    };
    settings_.horizon = integer("horizon", settings_.horizon);
    settings_.dt = number("model_dt", settings_.dt);
    settings_.u_min[0] = number("vx_min", settings_.u_min[0]);
    settings_.u_max[0] = number("vx_max", settings_.u_max[0]);
    settings_.u_min[1] = -number("wz_max", settings_.u_max[1]);
    settings_.u_max[1] = -settings_.u_min[1];
    settings_.acceleration_max[0] = number("ax_max", settings_.acceleration_max[0]);
    settings_.acceleration_max[1] = number("az_max", settings_.acceleration_max[1]);
    settings_.q << number("q_x", settings_.q[0]), number("q_y", settings_.q[1]),
      number("q_yaw", settings_.q[2]);
    settings_.q_terminal << number("q_terminal_x", settings_.q_terminal[0]),
      number("q_terminal_y", settings_.q_terminal[1]),
      number("q_terminal_yaw", settings_.q_terminal[2]);
    settings_.r << number("r_v", settings_.r[0]), number("r_w", settings_.r[1]);
    settings_.r_rate << number("r_delta_v", settings_.r_rate[0]),
      number("r_delta_w", settings_.r_rate[1]);
    solve_limit_ = number("solve_time_limit", 0.025);
    cycle_limit_ = number("cycle_time_limit", 0.045);
    if (settings_.horizon < 2 || settings_.horizon > 80 || settings_.dt <= 0.0 ||
        settings_.dt > 0.5 || settings_.u_max[0] <= 0.0 || settings_.u_min[0] >= settings_.u_max[0] ||
        settings_.u_max[1] <= 0.0 || solve_limit_ <= 0.0 ||
        cycle_limit_ <= solve_limit_ || cycle_limit_ >= settings_.dt) {
      throw ControlError("invalid MPC configuration");
    }
    configured_v_max_ = settings_.u_max[0];
    predicted_pub_ = node_->create_publisher<nav_msgs::msg::Path>(name_ + "/predicted_path", 1);
  }

  void cleanup() override { predicted_pub_.reset(); node_.reset(); plan_.poses.clear(); }
  void activate() override { predicted_pub_->on_activate(); }
  void deactivate() override {
    predicted_pub_->on_deactivate();
    previous_command_.setZero();
  }
  void setPlan(const nav_msgs::msg::Path & path) override {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    plan_ = path;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override {
    if (!node_ || pose.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      throw ControlError("MPC pose and costmap frames differ");
    }
    nav_msgs::msg::Path plan;
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      plan = plan_;
    }
    if (plan.poses.empty()) throw ControlError("MPC plan is empty");
    const std::string source_frame = plan.header.frame_id.empty() ?
      plan.poses.front().header.frame_id : plan.header.frame_id;
    if (source_frame.empty()) throw ControlError("MPC plan has no frame");
    nav_msgs::msg::Path local_plan;
    local_plan.header = pose.header;
    local_plan.poses.reserve(plan.poses.size());
    if (source_frame == pose.header.frame_id) local_plan.poses = plan.poses;
    else {
      const auto transform = tf_->lookupTransform(pose.header.frame_id,
        source_frame, tf2::TimePointZero);
      for (auto point : plan.poses) {
        point.header.frame_id = source_frame;
        geometry_msgs::msg::PoseStamped transformed;
        tf2::doTransform(point, transformed, transform);
        local_plan.poses.push_back(std::move(transformed));
      }
    }
    const State current(pose.pose.position.x, pose.pose.position.y,
                        tf2::getYaw(pose.pose.orientation));
    const auto cycle_start = std::chrono::steady_clock::now();
    const auto reference = samplePath(local_plan, current, settings_.horizon,
      settings_.dt, settings_.u_max[0], settings_.u_max[1]);
    const auto problem = makeProblem(settings_, current, previous_command_, reference);
    const auto result = solve(problem, reference, solve_limit_);
    if (!result.valid || result.solve_seconds > solve_limit_) {
      previous_command_.setZero();  // Nav2 publishes zero for NoValidControl.
      throw RetryableControlError("MPC QP rejected: " +
        (result.valid ? std::string("solve time exceeded budget") : result.failure_reason) +
        ", solve_seconds=" + std::to_string(result.solve_seconds) +
        ", vx_max=" + std::to_string(settings_.u_max[0]) +
        ", previous_v=" + std::to_string(previous_command_[0]));
    }
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - cycle_start).count() >
        cycle_limit_) {
      previous_command_.setZero();
      throw RetryableControlError("MPC matrix build and solve exceeded cycle budget");
    }
    nav_msgs::msg::Path prediction;
    prediction.header = pose.header;
    for (int k = 0; k <= settings_.horizon; ++k) {
      const State predicted = reference.states[k] +
        result.decision.segment<3>(problem.stateIndex(k));
      if (!predicted.allFinite()) throw ControlError("MPC nonfinite prediction");
      geometry_msgs::msg::PoseStamped point;
      point.header = prediction.header;
      point.pose.position.x = predicted[0];
      point.pose.position.y = predicted[1];
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, predicted[2]);
      point.pose.orientation = tf2::toMsg(q);
      prediction.poses.push_back(point);
    }
    // This QP tracks a collision-free reference; it does not optimize obstacle
    // constraints. Reject any predicted footprint that enters lethal/unknown space.
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap_);
      const auto footprint = costmap_ros_->getRobotFootprint();
      for (size_t k = 1; k < prediction.poses.size(); ++k) {
        const auto & p = prediction.poses[k].pose;
        const double cost = checker.footprintCostAtPose(p.position.x, p.position.y,
          tf2::getYaw(p.orientation), footprint);
        if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          previous_command_.setZero();
          throw RetryableControlError("MPC predicted footprint is blocked: step=" +
            std::to_string(k) + ", x=" + std::to_string(p.position.x) +
            ", y=" + std::to_string(p.position.y) +
            ", cost=" + std::to_string(cost));
        }
      }
    }
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = node_->now();
    command.header.frame_id = costmap_ros_->getBaseFrameID();
    command.twist.linear.x = result.command[0];
    command.twist.angular.z = result.command[1];
    previous_command_ = result.command;
    predicted_pub_->publish(prediction);
    return command;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override {
    if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) settings_.u_max[0] = configured_v_max_;
    else if (percentage) settings_.u_max[0] = configured_v_max_ *
      std::clamp(speed_limit, 0.0, 100.0) / 100.0;
    else settings_.u_max[0] = std::clamp(speed_limit, 0.0, configured_v_max_);
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr predicted_pub_;
  std::mutex plan_mutex_;
  nav_msgs::msg::Path plan_;
  Settings settings_;
  Input previous_command_ = Input::Zero();
  double configured_v_max_ = 0.0;
  double solve_limit_ = 0.025;
  double cycle_limit_ = 0.045;
  std::string name_;
};

}  // namespace wla_diff_mpc

PLUGINLIB_EXPORT_CLASS(wla_diff_mpc::DiffMpcController, nav2_core::Controller)
