#include "wla_manual_control/velocity_policy.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

namespace wla_manual_control
{
namespace
{
using nlohmann::json;

bool ready_status(const std::string & s)
{
  return s == "ACTIVE" || s == "SPIN" || s == "STANDBY";
}

// type(v) in (int, float) and math.isfinite(v); JSON booleans are excluded.
bool finite_number(const json & v)
{
  return v.is_number() && std::isfinite(v.get<double>());
}

// type(stamp) is int and stamp > 0, as int64 for the age arithmetic.
bool positive_int(const json & v, int64_t & out)
{
  if (v.is_number_unsigned()) {
    const auto u = v.get<uint64_t>();
    if (u == 0 || u > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {return false;}
    out = static_cast<int64_t>(u);
    return true;
  }
  if (v.is_number_integer()) {
    out = v.get<int64_t>();
    return out > 0;
  }
  return false;
}

const json & field(const json & message, const char * key)
{
  static const json missing;
  if (!message.is_object()) {return missing;}
  const auto it = message.find(key);
  return it == message.end() ? missing : *it;
}

bool string_equals(const json & v, const std::string & expected)
{
  return v.is_string() && v.get_ref<const std::string &>() == expected;
}

double age_s(int64_t now_ns, int64_t stamp)
{
  return static_cast<double>(now_ns - stamp) * 1e-9;
}

bool valid_stamp(const json & message, int64_t now_ns, double max_age, int64_t & stamp)
{
  if (!positive_int(field(message, "stamp_ns"), stamp)) {return false;}
  const double age = age_s(now_ns, stamp);
  return -.05 <= age && age <= max_age;
}

double clamp(double low, double high, double v)
{
  return std::max(low, std::min(high, v));  // max(low, min(high, v)) in Python order
}
}  // namespace

json decode(const std::string & data)
{
  // Python's json.loads also accepts NaN, Infinity and -Infinity. Every policy
  // check rejects them exactly as it rejects null, so map them to null.
  std::string text;
  text.reserve(data.size());
  bool in_string = false;
  for (size_t i = 0; i < data.size(); ++i) {
    const char c = data[i];
    if (in_string) {
      text += c;
      if (c == '\\' && i + 1 < data.size()) {text += data[++i];} else if (c == '"') {in_string = false;}
      continue;
    }
    if (c == '"') {in_string = true;}
    size_t skip = 0;
    if (data.compare(i, 3, "NaN") == 0) {skip = 3;}
    else if (data.compare(i, 8, "Infinity") == 0) {skip = 8;}
    else if (data.compare(i, 9, "-Infinity") == 0) {skip = 9;}
    if (skip) {
      text += "null";
      i += skip - 1;
    } else {
      text += c;
    }
  }
  return json::parse(text, nullptr, false);
}

std::string parse_gamepad_status(const std::string & data)
{
  const std::string head = data.substr(0, data.find(']'));
  const auto first = head.find_first_not_of('[');
  return first == std::string::npos ? std::string() : head.substr(first);
}

VelocityPolicy::VelocityPolicy(double linear, double reverse, double angular)
: limits{linear, reverse, angular}, resume_after(-std::numeric_limits<double>::infinity())
{
}

bool VelocityPolicy::zero(const Velocity & v)
{
  double m = std::fabs(v.first);
  const double b = std::fabs(v.second);
  if (b > m) {m = b;}
  return m < .01;
}

bool VelocityPolicy::store(
  const std::string & source, const json & linear, const json & angular, double t, double until)
{
  if (!finite_number(linear) || !finite_number(angular)) {
    commands.erase(source);
    return false;
  }
  commands[source] = Command{{linear.get<double>(), angular.get<double>()}, t, until};
  return true;
}

bool VelocityPolicy::automatic_input(const Velocity & v, double t)
{
  return store("NAV", json(v.first), json(v.second), t, t + .15);
}

bool VelocityPolicy::command_fresh(const std::string & source, double t) const
{
  const auto it = commands.find(source);
  if (it == commands.end()) {return false;}
  return it->second.received > resume_after && t <= it->second.until;
}

bool VelocityPolicy::candidate(
  const std::string & source, const json & message, double t, int64_t now_ns)
{
  std::string lower = source;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {return std::tolower(c);});
  int64_t stamp = 0;
  if (!message.is_object() || !string_equals(field(message, "schema"), "wla-" + lower + "-candidate/v1") ||
    !valid_stamp(message, now_ns, .15, stamp))
  {
    commands.erase(source);
    return false;
  }
  const auto last = last_stamps.find(source);
  if (stamp <= (last == last_stamps.end() ? 0 : last->second)) {return false;}
  last_stamps[source] = stamp;
  const double until = t + .15 - std::max(0., static_cast<double>(now_ns - stamp) / 1e9);
  return store(source, field(message, "linear"), field(message, "angular"), t, until);
}

bool VelocityPolicy::person_input(const json & message, double t, int64_t now_ns)
{
  return candidate("PERSON", message, t, now_ns);
}

bool VelocityPolicy::manual_input(const Velocity & v, double t)
{
  if (!std::isfinite(v.first) || !std::isfinite(v.second)) {
    latch_stop("invalid_manual_input");
    return false;
  }
  manual = v;
  manual_at = t;
  return false;  // Temporary priority, no task revocation or mode transition.
}

void VelocityPolicy::gamepad_status_input(const std::string & s, double t)
{
  const bool was_ready = ready_status(status) && t - status_at < .5;
  // A press of the e-stop button while connected: ESTOP directly followed by a
  // ready status. A reconnect or restarted gamepad node passes through a stale
  // or DISCONNECTED status and never counts as a release.
  const bool released = status == "ESTOP" && t - status_at < .5;
  status = s;
  status_at = t;
  const bool ready = ready_status(s);
  if (!ready || !was_ready) {
    clear_manual();
    manual_ready_after = t;
  }
  if (s == "ESTOP") {
    gamepad_stop_asserted = true;
    // Pressing the gamepad e-stop while another stop is latched keeps that stop
    // (and its explicit reset); only a latch the gamepad set alone is releasable.
    const bool gamepad_only = !estop_latched || gamepad_only_latch;
    latch_stop(gamepad_only ? "gamepad_estop" : estop_reason);
    gamepad_only_latch = gamepad_only;
  } else if (ready || s == "ARMING") {
    gamepad_stop_asserted = false;
    // Releasing the gamepad e-stop is the reset for the stop it latched, under the
    // same conditions as set_estop(false): stopped chassis, neutral manual input.
    if (released && estop_latched && gamepad_only_latch && stopped(t) && !(connected(t) && !zero(manual))) {
      estop_latched = false;
      estop_reason.clear();
      gamepad_only_latch = false;
      disarm();
    }
  } else if (s != "DISCONNECTED") {
    latch_stop("invalid_gamepad_status");
  }
}

void VelocityPolicy::clear_manual()
{
  armed = false;
  manual = {0., 0.};
  manual_at = NEVER;
}

bool VelocityPolicy::connected(double t) const
{
  return t - status_at < .5 && ready_status(status) && t - manual_at < .15;
}

bool VelocityPolicy::manual_ready(double t) const
{
  return !estop_latched && connected(t) && manual_at > manual_ready_after;
}

bool VelocityPolicy::manual_requested(double t) const
{
  return manual_ready(t) && !zero(manual);
}

bool VelocityPolicy::stopped(double t) const
{
  return t - wheel_at <= .2 && zero(wheel);
}

void VelocityPolicy::latch_stop(const std::string & reason)
{
  gamepad_only_latch = false;
  estop_latched = true;
  estop_reason = reason;
  disarm();
}

void VelocityPolicy::disarm()
{
  set_manual();
  clear_manual();
}

void VelocityPolicy::set_manual()
{
  // Legacy explicit stop request: clear caches, never latch a drive mode.
  commands.clear();
  revoke_person();
  mode = "AUTO";
}

void VelocityPolicy::clear_person()
{
  person = {0., 0.};
  person_at = person_status_at = NEVER;
  person_until = person_status_until = NEVER;
  person_stamp_ns = person_status_stamp_ns = 0;
  person_request_id.clear();
  person_enabled = false;
  person_session.clear();
  person_activation_ns = 0;
  person_candidate_only = true;
  person_motion_permitted = false;
  person_fault = false;
}

void VelocityPolicy::revoke_person()
{
  if (!person_request_id.empty()) {
    blocked_person_request_id = person_request_id;
    blocked_person_session = person_session;
  }
  ++person_revoke_generation;
  clear_person();
}

Result VelocityPolicy::set_estop(bool enabled, double t)
{
  if (enabled) {
    latch_stop("explicit_estop");
    return {true, "stop_latched; explicit reset required"};
  }
  if (!estop_latched) {return {true, "stop_already_clear"};}
  if (gamepad_stop_asserted) {return {false, "gamepad_estop_still_asserted"};}
  if (!stopped(t)) {return {false, "chassis_not_stopped"};}
  if (connected(t) && !zero(manual)) {return {false, "manual_input_not_neutral"};}
  estop_latched = false;
  estop_reason.clear();
  gamepad_only_latch = false;
  disarm();
  return {true, automatic_source == "NAV" ? "stop_cleared; waiting for fresh commands" :
    "stop_cleared; MANUAL; explicit AUTO required"};
}

Result VelocityPolicy::set_auto(double, int64_t)
{
  if (estop_latched) {return {false, "estop_latched"};}
  return {true, "velocity priority active; no AUTO switch required"};
}

Result VelocityPolicy::set_person_source(bool enabled, double, int64_t)
{
  // Compatibility hint for older producers only; output ignores this field.
  automatic_source = enabled ? "PERSON" : "NAV";
  mode = "AUTO";
  return {true, "compatibility hint only; velocity priority does not select sources"};
}

bool VelocityPolicy::person_status_input(const json & message, double t, int64_t now_ns)
{
  int64_t stamp = 0;
  const json & request = field(message, "request_id");
  const json & session_field = field(message, "follow_session");
  const bool has_fault = message.is_object() && message.contains("fault");
  const bool valid = message.is_object() &&
    string_equals(field(message, "schema"), STATUS_SCHEMA) &&
    valid_stamp(message, now_ns, PERSON_STATUS_AGE, stamp) &&
    request.is_string() &&
    session_field.is_string() && !session_field.get_ref<const std::string &>().empty() &&
    (!has_fault || field(message, "fault").is_boolean()) &&
    field(message, "enabled").is_boolean() && field(message, "candidate_only").is_boolean() &&
    field(message, "motion_permitted").is_boolean();
  if (!valid) {
    person_status_at = person_status_until = NEVER;
    return false;
  }
  if (stamp <= source_ros_ns || stamp <= person_status_stamp_ns) {return false;}
  const std::string request_id = request.get<std::string>();
  const std::string session = session_field.get<std::string>();
  const bool enabled = field(message, "enabled").get<bool>();
  if (enabled && (request_id.empty() ||
    (session == blocked_person_session && request_id == blocked_person_request_id)))
  {
    person_status_at = person_status_until = NEVER;
    return false;
  }
  if (session != person_session || request_id != person_request_id) {
    person = {0., 0.};
    person_at = person_until = NEVER;
    person_stamp_ns = 0;
    person_activation_ns = stamp;
  }
  person_session = session;
  person_request_id = enabled ? request_id : std::string();
  person_status_at = t;
  person_status_stamp_ns = stamp;
  person_status_until = t + PERSON_STATUS_AGE - std::max(0., age_s(now_ns, stamp));
  person_enabled = enabled;
  person_candidate_only = field(message, "candidate_only").get<bool>();
  person_motion_permitted = field(message, "motion_permitted").get<bool>();
  person_fault = has_fault && field(message, "fault").get<bool>();
  if (!person_enabled || person_candidate_only) {
    person = {0., 0.};
    person_at = person_until = NEVER;
  }
  return true;
}

Velocity VelocityPolicy::output(double t)
{
  output_source = "NONE";
  if (estop_latched) {
    commands.clear();
    armed = false;
    return {0., 0.};
  }
  armed = manual_ready(t);
  const bool override = manual_requested(t);
  if (manual_override && !override) {
    // After release, wait for a command received after the handover.
    resume_after = t;
  }
  manual_override = override;
  Velocity values{0., 0.};
  if (override) {
    values = manual;
    output_source = "MANUAL";
  } else {
    for (const char * source : {"PERSON", "NAV"}) {
      const auto it = commands.find(source);
      if (it != commands.end() && it->second.received > resume_after && t <= it->second.until &&
        !zero(it->second.values))
      {
        values = it->second.values;
        output_source = source;
        break;
      }
    }
  }
  return {clamp(-limits[1], limits[0], values.first), clamp(-limits[2], limits[2], values.second)};
}

}  // namespace wla_manual_control
