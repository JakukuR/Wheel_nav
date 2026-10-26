// Reads policy events as JSON lines on stdin and prints the policy state after
// each one; test_cpp_policy_equivalence.py feeds the Python policy the same events.
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include "wla_manual_control/velocity_policy.hpp"

using nlohmann::json;
using wla_manual_control::Velocity;
using wla_manual_control::VelocityPolicy;

namespace
{
// Non-finite values travel as strings because strict JSON has no NaN/Infinity.
double in(const json & v)
{
  if (v.is_string()) {
    const auto & s = v.get_ref<const std::string &>();
    if (s == "nan") {return std::numeric_limits<double>::quiet_NaN();}
    return s == "-inf" ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
  }
  return v.get<double>();
}

json out(double v)
{
  if (std::isnan(v)) {return "nan";}
  if (std::isinf(v)) {return v > 0 ? "inf" : "-inf";}
  return v;
}

json pair(const Velocity & v)
{
  return json::array({out(v.first), out(v.second)});
}

json snapshot(VelocityPolicy & p, double t)
{
  json commands = json::object();
  for (const auto & [k, c] : p.commands) {
    commands[k] = {out(c.values.first), out(c.values.second), out(c.received), out(c.until)};
  }
  return {
    {"mode", p.mode}, {"armed", p.armed}, {"estop_latched", p.estop_latched},
    {"estop_reason", p.estop_reason}, {"gamepad_stop_asserted", p.gamepad_stop_asserted},
    {"manual_ready_after", out(p.manual_ready_after)}, {"manual", pair(p.manual)},
    {"manual_at", out(p.manual_at)}, {"status", p.status}, {"status_at", out(p.status_at)},
    {"wheel", pair(p.wheel)}, {"wheel_at", out(p.wheel_at)},
    {"automatic_source", p.automatic_source}, {"source_generation", p.source_generation},
    {"person_revoke_generation", p.person_revoke_generation}, {"output_source", p.output_source},
    {"person", pair(p.person)}, {"person_at", out(p.person_at)},
    {"person_status_at", out(p.person_status_at)}, {"person_until", out(p.person_until)},
    {"person_status_until", out(p.person_status_until)}, {"person_stamp_ns", p.person_stamp_ns},
    {"person_status_stamp_ns", p.person_status_stamp_ns}, {"person_request_id", p.person_request_id},
    {"person_enabled", p.person_enabled}, {"person_session", p.person_session},
    {"person_activation_ns", p.person_activation_ns}, {"person_candidate_only", p.person_candidate_only},
    {"person_motion_permitted", p.person_motion_permitted}, {"person_fault", p.person_fault},
    {"blocked_person_request_id", p.blocked_person_request_id},
    {"blocked_person_session", p.blocked_person_session},
    {"commands", commands}, {"last_stamps", p.last_stamps}, {"manual_override", p.manual_override},
    {"resume_after", out(p.resume_after)}, {"manual_ready", p.manual_ready(t)},
    {"connected", p.connected(t)}, {"stopped", p.stopped(t)},
    {"nav_fresh", p.command_fresh("NAV", t)}, {"person_fresh", p.command_fresh("PERSON", t)},
  };
}
}  // namespace

int main()
{
  std::unique_ptr<VelocityPolicy> p;
  std::string line;
  while (std::getline(std::cin, line)) {
    const json e = json::parse(line);
    const std::string op = e.at("op").get<std::string>();
    const double t = e.value("t", 0.);
    const int64_t now_ns = e.value("now_ns", int64_t{0});
    json result = nullptr;
    if (op == "reset") {
      p = std::make_unique<VelocityPolicy>(in(e["limits"][0]), in(e["limits"][1]), in(e["limits"][2]));
    } else if (op == "manual") {
      result = p->manual_input({in(e["v"][0]), in(e["v"][1])}, t);
    } else if (op == "auto") {
      p->automatic_input({in(e["v"][0]), in(e["v"][1])}, t);  // Python returns None
    } else if (op == "person") {
      result = p->person_input(wla_manual_control::decode(e["data"].get<std::string>()), t, now_ns);
    } else if (op == "person_status") {
      result = p->person_status_input(wla_manual_control::decode(e["data"].get<std::string>()), t, now_ns);
    } else if (op == "status") {
      p->gamepad_status_input(wla_manual_control::parse_gamepad_status(e["data"].get<std::string>()), t);
    } else if (op == "wheel") {
      p->wheel = {in(e["v"][0]), in(e["v"][1])};
      if (e["fresh"].get<bool>()) {p->wheel_at = t;}
    } else if (op == "estop") {
      const auto r = p->set_estop(e["data"].get<bool>(), t);
      result = {r.success, r.message};
    } else if (op == "mode") {
      if (e["data"].get<bool>()) {
        const auto r = p->set_auto(t, now_ns);
        result = {r.success, r.message};
      } else {
        p->set_manual();
        result = {true, "cleared"};
      }
    } else if (op == "source") {
      const auto r = p->set_person_source(e["data"].get<bool>(), t, now_ns);
      result = {r.success, r.message};
    } else if (op == "output") {
      result = pair(p->output(t));
    }
    json state = p ? snapshot(*p, t) : json::object();
    state["result"] = result;
    std::cout << state.dump(-1, ' ', false, json::error_handler_t::replace) << '\n';
  }
  return 0;
}
