#include "wla_diff_mpc/qp.hpp"
#include "wla_diff_mpc/reference.hpp"
#include "wla_diff_mpc/solver.hpp"
#include "wla_diff_mpc/obstacle_braking.hpp"
#include "wla_diff_mpc/terminal_alignment.hpp"

#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#if __has_include(<nav2_core/controller_exceptions.hpp>)
#include <nav2_core/controller_exceptions.hpp>
#else
#include <nav2_core/exceptions.hpp>
#endif
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
    TerminalSettings terminal_settings;
    terminal_settings.enter_distance=final_align_enter_;
    terminal_settings.exit_distance=final_align_exit_;
    terminal_settings.max_w=final_align_wz_max_;
    terminal_settings.stopped_v=number("final_align_stopped_linear_velocity",0.025);
    terminal_settings.stopped_w=number("final_align_stopped_angular_velocity",0.04);
    terminal_settings.stopped_duration=number("final_align_stop_stable_s",0.35);
    terminal_settings.angular_deceleration=number("final_align_angular_deceleration",0.4);
    terminal_settings.reaction_time=number("final_align_reaction_time",0.7);
    terminal_settings.yaw_gain=number("final_align_yaw_gain",1.0);
    terminal_=TerminalAlignment(terminal_settings);
    terminal_settings_=terminal_settings;
    velocity_timeout_=number("final_align_velocity_timeout_s",0.20);
    if (!std::isfinite(velocity_timeout_) || velocity_timeout_<=0)
      throw ControlError("invalid terminal velocity timeout");
    nav2_util::declare_parameter_if_not_declared(node_,name_+".final_align_velocity_topic",
      rclcpp::ParameterValue(std::string("/d455_slam/odom")));
    velocity_sub_=node_->create_subscription<nav_msgs::msg::Odometry>(
      node_->get_parameter(name_+".final_align_velocity_topic").as_string(),rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(velocity_mutex_); velocity_=*msg;
        velocity_seen_=std::chrono::steady_clock::now();
      });
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
    velocity_sub_.reset(); predicted_pub_.reset(); node_.reset(); plan_.poses.clear(); terminal_.reset();
  }
  void activate() override { predicted_pub_->on_activate(); }
  void deactivate() override {
    predicted_pub_->on_deactivate();
    previous_command_.setZero();
    terminal_.reset();
  }
  void setPlan(const nav_msgs::msg::Path & path) override {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    // Replans of the same target must not reset terminal stopping/hold states.
    if (!path.poses.empty() && (plan_.poses.empty() || path.header.frame_id!=plan_.header.frame_id ||
        std::hypot(path.poses.back().pose.position.x-plan_.poses.back().pose.position.x,
                   path.poses.back().pose.position.y-plan_.poses.back().pose.position.y)>0.05 ||
        std::abs(std::remainder(tf2::getYaw(path.poses.back().pose.orientation)-
          tf2::getYaw(plan_.poses.back().pose.orientation),2*M_PI))>0.05)) goal_changed_=true;
    plan_ = path;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker * goal_checker) override {
    if (!node_ || pose.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      throw ControlError("MPC pose and costmap frames differ");
    }
    nav_msgs::msg::Path plan;
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      plan = plan_;
      if (goal_changed_) {terminal_.reset();previous_command_.setZero();goal_changed_=false;}
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
    double xy_tolerance=final_align_enter_,yaw_tolerance=0.18;
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (goal_checker && goal_checker->getTolerances(pose_tolerance,velocity_tolerance)) {
      if (std::isfinite(pose_tolerance.position.x) && pose_tolerance.position.x>0)
        xy_tolerance=pose_tolerance.position.x;
      const auto & q=pose_tolerance.orientation;
      const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if (std::isfinite(norm) && std::abs(norm-1)<0.01) {
        const double yaw=std::abs(tf2::getYaw(q)); if (yaw>0) yaw_tolerance=yaw;
      }
    }
    double measured_v=0,measured_w=0;
    bool fresh_velocity=false;
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      const double age=(node_->now()-rclcpp::Time(velocity_.header.stamp)).seconds();
      const auto & v=velocity_.twist.twist;
      measured_v=std::hypot(v.linear.x,v.linear.y);measured_w=v.angular.z;
      fresh_velocity=velocity_seen_.time_since_epoch().count()!=0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now()-velocity_seen_).count()<velocity_timeout_ &&
        age>=-0.10 && age<velocity_timeout_ && std::isfinite(measured_v+measured_w) &&
        velocity_.child_frame_id==costmap_ros_->getBaseFrameID() &&
        std::isfinite(velocity_.twist.covariance[0]+velocity_.twist.covariance[35]) &&
        velocity_.twist.covariance[0]<1e6 && velocity_.twist.covariance[35]<1e6;
    }
    const auto old_phase=terminal_.phase(); double terminal_w=0;
    const double yaw_error=std::remainder(tf2::getYaw(goal.orientation)-current[2],2*M_PI);
    const bool terminal_engaged=terminal_.update(goal_distance,yaw_error,measured_v,measured_w,
      fresh_velocity,std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
      xy_tolerance,yaw_tolerance,terminal_w);
    if (terminal_engaged) {
      if (old_phase!=terminal_.phase()) RCLCPP_INFO(node_->get_logger(),
        "MPC terminal %s -> %s",TerminalAlignment::name(old_phase),TerminalAlignment::name(terminal_.phase()));
      RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "MPC terminal phase=%s distance=%.3f yaw_error=%.3f measured_v=%.3f measured_w=%.3f fresh=%d cmd_w=%.3f",
        TerminalAlignment::name(terminal_.phase()),goal_distance,yaw_error,measured_v,measured_w,fresh_velocity,terminal_w);
      if (terminal_w==0) return publishCommand(pose,{current},Input::Zero());
    }

    Settings active = settings_;
    Reference reference;
    if (!terminal_engaged) {
      // Slow the approach before the terminal stop, aiming inside XY tolerance.
      const double cap=brakingSpeed(goal_distance,braking_deceleration_,reaction_time_,0.5*xy_tolerance);
      active.u_max[0]=std::min(active.u_max[0],std::max(0.01,cap));
      active.u_min[0]=std::max(active.u_min[0],-active.u_max[0]);
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
    if (terminal_engaged) {
      // Independent yaw controller, with the existing swept-footprint guard.
      // Include residual measured rotation and a conservative command delay.
      std::vector<State> prediction{current};double rate=measured_w;
      for (int k=0;k<active.horizon;++k) {
        if ((k+1)*active.dt>terminal_settings_.reaction_time)
          rate+=std::clamp(terminal_w-rate,-terminal_settings_.angular_deceleration*active.dt,
            terminal_settings_.angular_deceleration*active.dt);
        State next=prediction.back();next[2]+=rate*active.dt;prediction.push_back(next);
      }
      if (std::isfinite(collisionDistance(prediction,costmap_->getResolution(),blocked))) {
        previous_command_.setZero();
        throw RetryableControlError("MPC terminal rotation footprint blocked; holding zero");
      }
      map_lock.unlock();return publishCommand(pose,prediction,Input(0,terminal_w));
    }
    const double obstacle_distance=collisionDistance(reference.states,costmap_->getResolution(),blocked);
    if (std::isfinite(obstacle_distance)) {
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
    if (!safe()) {
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
    return publishCommand(pose,predicted,result.command);
  }

  geometry_msgs::msg::TwistStamped publishCommand(const geometry_msgs::msg::PoseStamped & pose,
    const std::vector<State> & predicted,const Input & input) {
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
    command.twist.linear.x = input[0];
    command.twist.angular.z = input[1];
    previous_command_ = input;
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
  TerminalAlignment terminal_;
  TerminalSettings terminal_settings_;
  double velocity_timeout_{0.2};
  std::mutex velocity_mutex_;
  nav_msgs::msg::Odometry velocity_;
  std::chrono::steady_clock::time_point velocity_seen_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_sub_;
  bool goal_changed_{false};
  std::string name_;
};

}  // namespace wla_diff_mpc

PLUGINLIB_EXPORT_CLASS(wla_diff_mpc::DiffMpcController, nav2_core::Controller)
