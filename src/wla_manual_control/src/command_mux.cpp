// Velocity-priority arbiter, sole publisher of /r680_base/cmd_vel: 50 Hz output
// and 5 Hz control_state report. The policy is velocity_policy.cpp, checked
// against the reference scripts/velocity_policy.py by the equivalence test.
#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "wla_manual_control/velocity_policy.hpp"

namespace wla_manual_control
{

using geometry_msgs::msg::Twist;
using std_msgs::msg::String;
using std_srvs::srv::SetBool;

double monotonic_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string session_id()
{
  std::random_device rd;
  static const char * hex = "0123456789abcdef";
  std::string id(32, '0');
  for (auto & c : id) {c = hex[rd() & 15];}
  return id;
}

class CommandMux : public rclcpp::Node
{
public:
  CommandMux()
  : Node("r680_command_mux"),
    p_(declare_parameter("max_forward_mps", 1.0), declare_parameter("max_reverse_mps", .3),
      declare_parameter("max_turn_rps", 1.5)),
    control_session_(session_id())
  {
    out_ = create_publisher<Twist>("/r680_base/cmd_vel", 1);
    report_ = create_publisher<String>("/r680_base/control_state", 10);
    subs_.push_back(create_subscription<Twist>("/r680_manual/cmd_vel", 1,
      [this](Twist::ConstSharedPtr m) {manual(*m);}));
    subs_.push_back(create_subscription<Twist>("/r680_nav/chassis_cmd_vel", 1,
      [this](Twist::ConstSharedPtr m) {p_.automatic_input({m->linear.x, m->angular.z}, monotonic_s());}));
    subs_.push_back(create_subscription<String>("/r680/person_follow/command_candidate", 1,
      [this](String::ConstSharedPtr m) {p_.person_input(decode(m->data), monotonic_s(), now_ns());}));
    subs_.push_back(create_subscription<String>("/r680/person_follow/status", 1,
      [this](String::ConstSharedPtr m) {p_.person_status_input(decode(m->data), monotonic_s(), now_ns());}));
    subs_.push_back(create_subscription<String>("/gamepad_status", 10,
      [this](String::ConstSharedPtr m) {status(*m);}));
    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>("/wheel/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {wheel(*m);}));
    services_.push_back(create_service<SetBool>("/r680_base/set_automatic",
      [this](SetBool::Request::SharedPtr q, SetBool::Response::SharedPtr r) {mode(*q, *r);}));
    services_.push_back(create_service<SetBool>("/r680_base/set_person_source",
      [this](SetBool::Request::SharedPtr q, SetBool::Response::SharedPtr r) {source(*q, *r);}));
    services_.push_back(create_service<SetBool>("/r680_base/set_estop",
      [this](SetBool::Request::SharedPtr q, SetBool::Response::SharedPtr r) {estop(*q, *r);}));
    for (const char * action : {"navigate_to_pose", "navigate_through_poses"}) {
      cancel_.push_back(create_client<action_msgs::srv::CancelGoal>(
          std::string("/") + action + "/_action/cancel_goal"));
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {tick();});
  }

  void publish_zero()
  {
    out_->publish(Twist());
  }

private:
  int64_t now_ns()
  {
    return get_clock()->now().nanoseconds();
  }

  void cancel_navigation()
  {
    for (auto & client : cancel_) {
      if (client->service_is_ready()) {
        client->async_send_request(std::make_shared<action_msgs::srv::CancelGoal::Request>(),
          [](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture) {});
      }
      client->prune_requests_older_than(std::chrono::system_clock::now() - std::chrono::seconds(10));
    }
  }

  void manual(const Twist & msg)
  {
    const std::string previous = p_.mode;
    const bool stopped = p_.estop_latched;
    p_.manual_input({msg.linear.x, msg.angular.z}, monotonic_s());
    if ((previous == "AUTO" && p_.mode != "AUTO") || (!stopped && p_.estop_latched)) {
      cancel_navigation();
      last_report_ = 0.;
    }
  }

  void status(const String & msg)
  {
    const std::string previous = p_.mode;
    const bool stopped = p_.estop_latched;
    const std::string previous_status = p_.status;
    const std::string s = parse_gamepad_status(msg.data);
    p_.gamepad_status_input(s, monotonic_s());
    if ((previous == "AUTO" && p_.mode != "AUTO") || (!stopped && p_.estop_latched)) {
      cancel_navigation();
    }
    if (previous_status != s || previous != p_.mode) {last_report_ = 0.;}
  }

  void estop(const SetBool::Request & req, SetBool::Response & res)
  {
    const Result r = p_.set_estop(req.data, monotonic_s());
    res.success = r.success;
    res.message = r.message;
    if (res.success && req.data) {cancel_navigation();}
    last_report_ = 0.;
  }

  void wheel(const nav_msgs::msg::Odometry & msg)
  {
    p_.wheel = {msg.twist.twist.linear.x, msg.twist.twist.angular.z};
    const double age = static_cast<double>(now_ns()) * 1e-9 -
      (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9);
    if (-.05 <= age && age <= .2) {p_.wheel_at = monotonic_s();}
  }

  void mode(const SetBool::Request & req, SetBool::Response & res)
  {
    if (req.data) {
      const Result r = p_.set_auto(monotonic_s(), now_ns());
      res.success = r.success;
      res.message = r.message;
    } else {
      p_.set_manual();
      cancel_navigation();
      res.success = true;
      res.message = "cached automatic velocities cleared; fresh commands remain available";
    }
    last_report_ = 0.;
  }

  void source(const SetBool::Request & req, SetBool::Response & res)
  {
    const std::string previous = p_.automatic_source;
    const Result r = p_.set_person_source(req.data, monotonic_s(), now_ns());
    res.success = r.success;
    res.message = r.message;
    if (res.success && previous != p_.automatic_source) {last_report_ = 0.;}
  }

  void tick()
  {
    const double now = monotonic_s();
    const std::string previous = p_.mode;
    const Velocity v = p_.output(now);
    if (previous == "AUTO" && p_.mode == "MANUAL") {
      cancel_navigation();
      last_report_ = 0.;
    }
    Twist msg;
    msg.linear.x = v.first;
    msg.angular.z = v.second;
    out_->publish(msg);
    if (now - last_report_ > .2) {
      last_report_ = now;
      report(now, v);
    }
  }

  void report(double now, const Velocity & v)
  {
    nlohmann::ordered_json j;
    j["control_session"] = control_session_;
    j["stamp_ns"] = now_ns();
    j["mode"] = p_.mode;
    j["armed"] = p_.armed;
    j["mode_required"] = false;
    j["velocity_passthrough"] = true;
    j["priority"] = {"MANUAL", "PERSON", "NAV"};
    j["nav_manual_coexist"] = p_.nav_manual_coexist;
    j["output_source"] = p_.output_source;
    j["manual_ready"] = p_.manual_ready(now);
    j["automatic_authorized"] = p_.mode == "AUTO" && !p_.estop_latched;
    j["estop_latched"] = p_.estop_latched;
    j["estop_reason"] = p_.estop_reason;
    j["automatic_source"] = p_.automatic_source;
    j["source_generation"] = p_.source_generation;
    j["person_revoke_generation"] = p_.person_revoke_generation;
    j["person_request_id"] = p_.person_request_id;
    j["follow_session"] = p_.person_session;
    j["gamepad"] = p_.status;
    j["linear"] = v.first;
    j["angular"] = v.second;
    j["navigation_fresh"] = p_.command_fresh("NAV", now);
    j["person_fresh"] = p_.command_fresh("PERSON", now);
    String msg;
    // Gamepad status is external text; never let invalid UTF-8 throw here.
    msg.data = j.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
    report_->publish(msg);
  }

  VelocityPolicy p_;
  std::string control_session_;
  double last_report_{0.};
  rclcpp::Publisher<Twist>::SharedPtr out_;
  rclcpp::Publisher<String>::SharedPtr report_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::vector<rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr> cancel_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wla_manual_control

int main(int argc, char ** argv)
{
  // Keep the context alive until the final zero publication, as in the Python
  // node; SIGINT and SIGTERM are taken by a sigwait thread instead.
  sigset_t signals;
  sigemptyset(&signals);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &signals, nullptr);
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<wla_manual_control::CommandMux>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::atomic<bool> signalled{false};
  std::thread waiter([&] {
      int sig = 0;
      sigwait(&signals, &sig);
      signalled = true;
      executor.cancel();
    });
  executor.spin();
  // spin() may also end on context shutdown; release the waiter either way.
  if (!signalled) {pthread_kill(waiter.native_handle(), SIGTERM);}
  if (rclcpp::ok()) {
    for (int i = 0; i < 5; ++i) {node->publish_zero();}
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  waiter.join();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
