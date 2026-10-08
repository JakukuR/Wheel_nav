#include "wla_diff_mpc/qp.hpp"
#include "wla_diff_mpc/reference.hpp"
#include "wla_diff_mpc/solver.hpp"
#include "wla_diff_mpc/obstacle_braking.hpp"

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
    final_align_enter_ = number("final_align_enter_distance", 0.20);
    final_align_exit_ = number("final_align_exit_distance", 0.28);
    final_align_wz_max_ = number("final_align_wz_max", 0.22);
    turn_time_constant_ = number("turn_time_constant", 0.65);
    braking_deceleration_ = number("obstacle_braking_deceleration", 0.4);
    reaction_time_ = number("obstacle_reaction_time", 0.7);
    stop_margin_ = number("obstacle_stop_margin", 0.10);
    retry_scale_ = number("collision_retry_speed_scale", 0.5);
    if (!std::isfinite(braking_deceleration_+reaction_time_+stop_margin_+retry_scale_) ||
        braking_deceleration_<=0 || reaction_time_<0 || stop_margin_<0 || retry_scale_<=0 || retry_scale_>=1)
      throw ControlError("invalid obstacle braking configuration");
    if (settings_.horizon < 2 || settings_.horizon > 80 || settings_.dt <= 0.0 ||
        settings_.dt > 0.5 || settings_.u_max[0] <= 0.0 || settings_.u_min[0] >= settings_.u_max[0] ||
        settings_.u_max[1] <= 0.0 || solve_limit_ <= 0.0 ||
        cycle_limit_ <= solve_limit_ || cycle_limit_ >= settings_.dt ||
        final_align_enter_ <= 0.0 || final_align_exit_ <= final_align_enter_ ||
        final_align_wz_max_ <= 0.0 || turn_time_constant_ <= 0.0) {
      throw ControlError("invalid MPC configuration");
    }
    configured_v_max_ = settings_.u_max[0];
    predicted_pub_ = node_->create_publisher<nav_msgs::msg::Path>(name_ + "/predicted_path", 1);
  }

  void cleanup() override {
    predicted_pub_.reset(); node_.reset(); plan_.poses.clear(); final_alignment_ = false;
  }
  void activate() override { predicted_pub_->on_activate(); }
  void deactivate() override {
    predicted_pub_->on_deactivate();
    previous_command_.setZero();
    final_alignment_ = false;
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
    const auto & goal = local_plan.poses.back().pose;
    const double goal_distance = std::hypot(goal.position.x - current[0],
      goal.position.y - current[1]);
    final_alignment_ = finalAlignmentMode(final_alignment_, goal_distance,
      final_align_enter_, final_align_exit_);

    Settings active = settings_;
    Reference reference;
    if (final_alignment_) {
      active.u_min[0] = std::max(active.u_min[0], -0.03);
      active.u_max[0] = std::min(active.u_max[0], 0.03);
      active.u_max[1] = std::min(active.u_max[1], final_align_wz_max_);
      active.u_min[1] = -active.u_max[1];
      reference = makeRotationReference(current, tf2::getYaw(goal.orientation),
        active.horizon, active.dt, active.u_max[1], turn_time_constant_);
    } else {
      reference = samplePath(local_plan, current, active.horizon, active.dt,
        active.u_max[0], active.u_max[1], turn_time_constant_);
    }
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap_->getMutex());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap_);
    const auto footprint = costmap_ros_->getRobotFootprint();
    auto blocked=[&](const State & s) {
      const double cost=checker.footprintCostAtPose(s[0],s[1],s[2],footprint);
      if (cost<0 || cost>=nav2_costmap_2d::LETHAL_OBSTACLE) return true;
      // Also test interior cell centers: the Nav2 checker primarily samples edges.
      std::vector<std::pair<double,double>> polygon;
      double minx=1e9,miny=1e9,maxx=-1e9,maxy=-1e9;
      for (const auto & p:footprint) {
        const double x=s[0]+std::cos(s[2])*p.x-std::sin(s[2])*p.y;
        const double y=s[1]+std::sin(s[2])*p.x+std::cos(s[2])*p.y;
        polygon.emplace_back(x,y);minx=std::min(minx,x);maxx=std::max(maxx,x);
        miny=std::min(miny,y);maxy=std::max(maxy,y);
      }
      unsigned int x0,y0,x1,y1;
      if (polygon.size()<3 || !costmap_->worldToMap(minx,miny,x0,y0) ||
          !costmap_->worldToMap(maxx,maxy,x1,y1)) return true;
      for (unsigned int y=y0;y<=y1;++y) for (unsigned int x=x0;x<=x1;++x) {
        if (costmap_->getCost(x,y)<nav2_costmap_2d::LETHAL_OBSTACLE) continue;
        double wx,wy;costmap_->mapToWorld(x,y,wx,wy);bool inside=false;
        for (size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
          const auto & a=polygon[i];const auto & b=polygon[j];
          if ((a.second>wy)!=(b.second>wy) && wx<(b.first-a.first)*(wy-a.second)/(b.second-a.second)+a.first)
            inside=!inside;
        }
        if (inside) return true;
      }
      return false;
    };
    const double obstacle_distance=collisionDistance(reference.states,costmap_->getResolution(),blocked);
    if (std::isfinite(obstacle_distance) && !final_alignment_) {
      const double cap=brakingSpeed(obstacle_distance,braking_deceleration_,reaction_time_,stop_margin_);
      if (cap<0.01) {
        previous_command_.setZero();
        throw RetryableControlError("MPC reference blocked inside stopping margin; waiting for global replan");
      }
      if (cap<active.u_max[0]) {
        active.u_max[0]=cap; active.u_min[0]=std::max(active.u_min[0],-cap);
        reference=samplePath(local_plan,current,active.horizon,active.dt,cap,active.u_max[1],turn_time_constant_);
        RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
          "MPC obstacle braking: collision distance=%.3fm speed cap=%.3fm/s",obstacle_distance,cap);
      }
    }
    auto problem = makeProblem(active, current, previous_command_, reference);
    auto result = solve(problem, reference, solve_limit_);
    if (!result.valid || result.solve_seconds > solve_limit_) {
      const double previous_v = previous_command_[0];
      previous_command_.setZero();  // Nav2 publishes zero for NoValidControl.
      throw RetryableControlError("MPC QP rejected: " +
        (result.valid ? std::string("solve time exceeded budget") : result.failure_reason) +
        ", solve_seconds=" + std::to_string(result.solve_seconds) +
        ", vx_max=" + std::to_string(active.u_max[0]) +
        ", previous_v=" + std::to_string(previous_v));
    }
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - cycle_start).count() >
        cycle_limit_) {
      previous_command_.setZero();
      throw RetryableControlError("MPC matrix build and solve exceeded cycle budget");
    }
    auto predicted=nonlinearPrediction(current,reference,problem,result.decision,active.dt);
    auto safe=[&](){return !std::isfinite(collisionDistance(predicted,costmap_->getResolution(),blocked));};
    if (!safe() && !final_alignment_) {
      const double remaining=cycle_limit_-std::chrono::duration<double>(std::chrono::steady_clock::now()-cycle_start).count()-0.003;
      if (remaining>0.003) {
        active.u_max[0]*=retry_scale_; active.u_min[0]=std::max(active.u_min[0],-active.u_max[0]);
        reference=samplePath(local_plan,current,active.horizon,active.dt,active.u_max[0],active.u_max[1],turn_time_constant_);
        problem=makeProblem(active,current,previous_command_,reference);
        result=solve(problem,reference,std::min(solve_limit_,remaining));
        if (result.valid) predicted=nonlinearPrediction(current,reference,problem,result.decision,active.dt);
      } else result.valid=false;
    }
    if (!result.valid || !safe() ||
        std::chrono::duration<double>(std::chrono::steady_clock::now()-cycle_start).count()>cycle_limit_) {
      previous_command_.setZero();
      throw RetryableControlError("MPC collision/budget rejection after bounded braking retry; waiting for global replan");
    }
    map_lock.unlock();
    nav_msgs::msg::Path prediction; prediction.header=pose.header;
    for (const auto & p:predicted) {
      geometry_msgs::msg::PoseStamped point;point.header=pose.header;
      point.pose.position.x=p[0];point.pose.position.y=p[1];
      tf2::Quaternion q;q.setRPY(0,0,p[2]);point.pose.orientation=tf2::toMsg(q);
      prediction.poses.push_back(point);
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
  double final_align_enter_ = 0.20;
  double final_align_exit_ = 0.28;
  double final_align_wz_max_ = 0.22;
  double turn_time_constant_ = 0.65;
  double braking_deceleration_{0.4},reaction_time_{0.7},stop_margin_{0.1},retry_scale_{0.5};
  bool final_alignment_ = false;
  std::string name_;
};

}  // namespace wla_diff_mpc

PLUGINLIB_EXPORT_CLASS(wla_diff_mpc::DiffMpcController, nav2_core::Controller)
