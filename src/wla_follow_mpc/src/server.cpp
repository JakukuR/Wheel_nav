#include "wla_follow_mpc/srv/compute_mpc.hpp"
#include "wla_diff_mpc/qp.hpp"
#include "wla_diff_mpc/reference.hpp"
#include "wla_diff_mpc/solver.hpp"
#include "wla_diff_mpc/obstacle_braking.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

// Geometry-only service: no velocity publisher and no persistent command.
// The caller must verify voxel collision, freshness and target generation.
class FollowMpcServer : public rclcpp::Node {
 public:
  using Service = wla_follow_mpc::srv::ComputeMpc;
  FollowMpcServer() : Node("follow_mpc_server") {
    base_ = declare_parameter<std::string>("base_frame", "r680_mapping_floor");
    service_ = create_service<Service>("/r680/person_follow/compute_mpc",
      [this](Service::Request::SharedPtr req, Service::Response::SharedPtr out) {
        out->request_id=req->request_id;
        const auto started=std::chrono::steady_clock::now();
        try {
          const double age=(now()-rclcpp::Time(req->path.header.stamp)).seconds();
          if (req->path.header.frame_id!=base_ || age<-.05 || age>.20 ||
              req->path.poses.size()<2 || req->path.poses.size()>2000)
            throw std::runtime_error("invalid frame, age or path size");
          if (!std::isfinite(req->max_linear+req->max_angular+req->measured_linear+
                             req->measured_angular+req->control_period) ||
              req->max_linear<=0 || req->max_linear>.30 || req->max_angular<=0 ||
              req->max_angular>.50 || req->control_period<.02 || req->control_period>.10 ||
              std::abs(req->measured_linear)>.5 || std::abs(req->measured_angular)>1.)
            throw std::runtime_error("invalid limits or measured velocity");
          for (const auto & p:req->path.poses) {
            const auto & q=p.pose.orientation;
            const double norm2=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
            if ((!p.header.frame_id.empty() && p.header.frame_id!=base_) ||
                !std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) ||
                !std::isfinite(p.pose.position.z) || std::abs(p.pose.position.z)>.05 ||
                std::hypot(p.pose.position.x,p.pose.position.y)>30. ||
                !std::isfinite(norm2) || norm2<1e-12)
              throw std::runtime_error("invalid path point");
          }
          wla_diff_mpc::Settings settings;
          settings.horizon=20; settings.dt=.1;
          settings.u_min << 0.,-req->max_angular;
          settings.u_max << req->max_linear,req->max_angular;
          settings.acceleration_max << .4,.8;
          const wla_diff_mpc::State current=wla_diff_mpc::State::Zero();
          const wla_diff_mpc::Input measured(req->measured_linear,req->measured_angular);
          // The shared core deliberately gives newly lowered speed limits
          // precedence over slew bounds. This bounds the requested command,
          // not physical braking: the caller must check the stopping envelope
          // from the original measured velocity, including actuation delay.
          const wla_diff_mpc::Input rate_origin=measured.cwiseMax(settings.u_min).cwiseMin(settings.u_max);
          auto reference=wla_diff_mpc::samplePath(req->path,current,settings.horizon,
                            settings.dt,req->max_linear,req->max_angular,.65);
          auto problem=wla_diff_mpc::makeProblem(settings,current,measured,reference);
          auto result=wla_diff_mpc::solve(problem,reference,.020);
          out->solve_seconds=result.solve_seconds;
          if (!result.valid) throw std::runtime_error(result.failure_reason);
          if (std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>.04)
            throw std::runtime_error("MPC cycle exceeded 40 ms");
          // First input changes are constrained over the actual control period,
          // not just the model step. Modify QP bounds and re-solve if necessary.
          // makeProblem uses dt for rate bounds; reject rather than clip a
          // solved command and invalidate its nonlinear prediction.
          const auto delta=(result.command-rate_origin).cwiseAbs();
          if ((delta.array()>(settings.acceleration_max*req->control_period).array()+.002).any()) {
            auto tight=settings;
            tight.acceleration_max *= req->control_period/settings.dt;
            problem=wla_diff_mpc::makeProblem(tight,current,measured,reference);
            result=wla_diff_mpc::solve(problem,reference,.012);
            out->solve_seconds+=result.solve_seconds;
            if (!result.valid) throw std::runtime_error(result.failure_reason);
          }
          if (((result.command-rate_origin).cwiseAbs().array()>
               (settings.acceleration_max*req->control_period).array()+.002).any() ||
              (result.command.array()<settings.u_min.array()-.002).any() ||
              (result.command.array()>settings.u_max.array()+.002).any())
            throw std::runtime_error("MPC command violates limits or first-step slew");
          if (std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>.045)
            throw std::runtime_error("MPC cycle exceeded 45 ms");
          const auto predicted=wla_diff_mpc::nonlinearPrediction(current,reference,problem,
                                                                 result.decision,settings.dt);
          out->prediction.header=req->path.header;
          for (const auto & state:predicted) {
            if (!state.allFinite()) throw std::runtime_error("nonfinite nonlinear prediction");
            geometry_msgs::msg::PoseStamped p;p.header=out->prediction.header;
            p.pose.position.x=state[0];p.pose.position.y=state[1];
            p.pose.orientation.z=std::sin(state[2]/2.);p.pose.orientation.w=std::cos(state[2]/2.);
            out->prediction.poses.push_back(p);
          }
          const double response_age=(now()-rclcpp::Time(req->path.header.stamp)).seconds();
          if (response_age<-.05 || response_age>.20 ||
              std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>.045)
            throw std::runtime_error("MPC result expired before response");
          out->command.linear.x=result.command[0];out->command.angular.z=result.command[1];
          out->success=true;out->reason="solved";
        } catch (const std::exception & e) {
          out->success=false;out->reason=e.what();
          out->command=geometry_msgs::msg::Twist{};
          out->prediction=nav_msgs::msg::Path{};
        }
      });
  }
 private:
  std::string base_;
  rclcpp::Service<Service>::SharedPtr service_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<FollowMpcServer>());
  rclcpp::shutdown();return 0;
}
