// C++ port of scripts/velocity_policy.py together with the control_policy.py
// members it inherits. Behaviour, constants and evaluation order match the
// Python policy; test/test_cpp_policy_equivalence.py checks this event by event.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace wla_manual_control
{

using Velocity = std::pair<double, double>;

struct Result
{
  bool success;
  std::string message;
};

// json.loads equivalent; returns a discarded value on malformed input.
nlohmann::json decode(const std::string & data);
// msg.data.split(']', 1)[0].lstrip('[') from command_mux.py.
std::string parse_gamepad_status(const std::string & data);

class VelocityPolicy
{
public:
  static constexpr const char * STATUS_SCHEMA = "wla-person-status/v1";
  static constexpr double PERSON_STATUS_AGE = .20;
  static constexpr double NEVER = -1e9;

  VelocityPolicy(double linear, double reverse, double angular);

  bool manual_input(const Velocity & v, double t);
  bool automatic_input(const Velocity & v, double t);
  bool candidate(const std::string & source, const nlohmann::json & message, double t, int64_t now_ns);
  bool person_input(const nlohmann::json & message, double t, int64_t now_ns);
  bool person_status_input(const nlohmann::json & message, double t, int64_t now_ns);
  void gamepad_status_input(const std::string & status, double t);
  Result set_estop(bool enabled, double t);
  Result set_auto(double t, int64_t now_ns);
  Result set_person_source(bool enabled, double t, int64_t now_ns);
  void set_manual();
  Velocity output(double t);

  bool manual_ready(double t) const;
  bool manual_requested(double t) const;
  bool connected(double t) const;
  bool stopped(double t) const;
  bool command_fresh(const std::string & source, double t) const;

  // Python-style max(abs(x) for x in v) < .01, including its NaN ordering.
  static bool zero(const Velocity & v);

  // Public like the Python attributes the mux node reads and writes.
  double limits[3];
  std::string mode{"AUTO"};
  bool armed{false};
  bool estop_latched{false};
  std::string estop_reason;
  bool gamepad_stop_asserted{false};
  double manual_ready_after{NEVER};
  Velocity manual{0., 0.};
  double manual_at{NEVER};
  std::string status{"DISCONNECTED"};
  double status_at{NEVER};
  Velocity wheel{0., 0.};
  double wheel_at{NEVER};
  std::string automatic_source{"NAV"};
  int64_t source_generation{0};
  int64_t person_revoke_generation{0};
  int64_t source_ros_ns{0};
  std::string output_source{"NONE"};
  bool nav_manual_coexist{true};

  Velocity person{0., 0.};
  double person_at{NEVER};
  double person_status_at{NEVER};
  double person_until{NEVER};
  double person_status_until{NEVER};
  int64_t person_stamp_ns{0};
  int64_t person_status_stamp_ns{0};
  std::string person_request_id;
  bool person_enabled{false};
  std::string person_session;
  int64_t person_activation_ns{0};
  bool person_candidate_only{true};
  bool person_motion_permitted{false};
  bool person_fault{false};
  std::string blocked_person_request_id;
  std::string blocked_person_session;

  struct Command
  {
    Velocity values;
    double received;
    double until;
  };
  std::map<std::string, Command> commands;
  std::map<std::string, int64_t> last_stamps;
  bool manual_override{false};
  double resume_after;

private:
  bool store(const std::string & source, const nlohmann::json & linear, const nlohmann::json & angular,
             double t, double until);
  void latch_stop(const std::string & reason);
  void disarm();
  void clear_manual();
  void clear_person();
  void revoke_person();
};

}  // namespace wla_manual_control
