// Mode 3 final-output envelope, in the same wheel reference as the native map.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wla_manual_control/velocity_policy.hpp"

namespace wla_manual_control
{
class PointStopGuard
{
  using Json = nlohmann::json;
  struct Grid
  {
    std::string session;
    int64_t reference{}, generation{}, stamp{};
    int width{}, height{};
    double cell{}, x{}, y{}, radius{};
    std::vector<uint8_t> free, blocked;

    bool bit(const std::vector<uint8_t> & bits, int i, int j) const
    {
      const auto index = static_cast<size_t>(i * height + j);
      return (bits[index / 8] & (0x80 >> (index % 8))) != 0;
    }
  };
  std::optional<Grid> grid_;
  bool mode3_{false};

  static std::vector<uint8_t> decode_bits(const std::string & input, size_t expected)
  {
    constexpr std::string_view alphabet =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    if (input.size() != 4 * ((expected + 2) / 3)) {return {};}
    std::vector<uint8_t> out;
    out.reserve(expected);
    uint32_t value = 0;
    int bits = 0;
    const size_t symbols = (expected * 8 + 5) / 6;
    for (size_t i = 0; i < input.size(); ++i) {
      if (i >= symbols) {if (input[i] != '=') {return {};} continue;}
      const size_t code = alphabet.find(input[i]);
      if (code == std::string_view::npos) {return {};}
      value = (value << 6) | static_cast<uint32_t>(code);
      bits += 6;
      if (bits >= 8) {
        bits -= 8;
        out.push_back(static_cast<uint8_t>(value >> bits));
      }
    }
    return out.size() == expected ? out : std::vector<uint8_t>{};
  }

  // Conservative cell-circle intersection. Starting occupied space is exempt
  // from missing-free evidence only; a measured obstruction still wins.
  static double cell_distance(double x, double y, double left, double bottom, double cell)
  {
    return std::hypot(std::max({left - x, 0., x - left - cell}),
                     std::max({bottom - y, 0., y - bottom - cell}));
  }

  static bool capsule_hits(double ax, double ay, double bx, double by,
    double left, double bottom, double cell, double radius)
  {
    const double dx = bx-ax, dy = by-ay, length2 = dx*dx+dy*dy;
    if (cell_distance(ax,ay,left,bottom,cell) <= radius ||
      cell_distance(bx,by,left,bottom,cell) <= radius) {return true;}
    if (length2 == 0.) {return false;}
    double first = 0., last = 1.;
    const auto clip = [&](double start, double delta, double min) {
      if (delta == 0.) {return min <= start && start <= min+cell;}
      const double near = (min-start)/delta, far = (min+cell-start)/delta;
      first = std::max(first,std::min(near,far));
      last = std::min(last,std::max(near,far));
      return first <= last;
    };
    if (clip(ax,dx,left) && clip(ay,dy,bottom)) {return true;}
    for (double x : {left,left+cell}) {
      for (double y : {bottom,bottom+cell}) {
        const double t = std::clamp(((x-ax)*dx+(y-ay)*dy)/length2,0.,1.);
        const double ux = x-(ax+t*dx), uy = y-(ay+t*dy);
        if (ux*ux+uy*uy <= radius*radius) {return true;}
      }
    }
    return false;
  }

  static bool clear_capsule(const Grid & g, double ax, double ay, double bx, double by,
    double start_x, double start_y, double radius)
  {
    const double low_x = std::min(ax,bx)-radius, high_x = std::max(ax,bx)+radius;
    const double low_y = std::min(ay,by)-radius, high_y = std::max(ay,by)+radius;
    if (low_x < g.x || low_y < g.y || high_x >= g.x+g.width*g.cell ||
      high_y >= g.y+g.height*g.cell) {return false;}
    const int i0 = std::max(0, static_cast<int>(std::ceil((low_x-g.x)/g.cell))-1);
    const int i1 = static_cast<int>(std::floor((high_x-g.x)/g.cell));
    const int j0 = std::max(0, static_cast<int>(std::ceil((low_y-g.y)/g.cell))-1);
    const int j1 = static_cast<int>(std::floor((high_y-g.y)/g.cell));
    for (int i = i0; i <= i1; ++i) {
      for (int j = j0; j <= j1; ++j) {
        const double left = g.x+i*g.cell, bottom = g.y+j*g.cell;
        if (!capsule_hits(ax,ay,bx,by,left,bottom,g.cell,radius)) {continue;}
        if (g.bit(g.blocked,i,j)) {return false;}
        if (!g.bit(g.free,i,j) &&
          cell_distance(start_x,start_y,left,bottom,g.cell) > g.radius) {return false;}
      }
    }
    return true;
  }

  static bool safe_arc(const Grid & g, double x, double y, double yaw,
    const Velocity & v, double pose_age, double decel)
  {
    const double travel = std::abs(v.first) * (pose_age + .10) +
      v.first*v.first/(2.*decel);
    const double turn_per_m = v.first == 0. ? 0. : v.second/v.first;
    const double angle = std::abs(travel*turn_per_m);
    if (!std::isfinite(travel) || !std::isfinite(angle)) {return false;}
    const int steps = std::max(1, static_cast<int>(std::ceil(angle/.25)));
    const double quarter = angle/(4.*steps);
    // A chord capsule expanded by the sagitta covers the continuous curved
    // sweep. A straight sweep needs no padding around either endpoint.
    const double sinc_quarter = quarter == 0. ? 1. : std::sin(quarter)/quarter;
    const double padding = (travel/steps)*quarter/2.*sinc_quarter*sinc_quarter;
    double ax = x, ay = y;
    for (int i = 1; i <= steps; ++i) {
      const double s = std::copysign(travel*i/steps, v.first), turn = s*turn_per_m;
      const double sinc = std::abs(turn) < 1e-6 ? 1. : std::sin(turn/2.)/(turn/2.);
      const double px = x+s*sinc*std::cos(yaw+turn/2.);
      const double py = y+s*sinc*std::sin(yaw+turn/2.);
      if (!clear_capsule(g,ax,ay,px,py,x,y,g.radius+padding+1e-9)) {return false;}
      ax = px;
      ay = py;
    }
    return true;
  }

public:
  void observe(const Json & status)
  {
    mode3_ = status.contains("mode") && status.contains("enabled") &&
      status["mode"] == "3" && status["enabled"] == true;
    grid_.reset();
    if (!mode3_) {return;}
    try {
      const auto & packet = status.at("l4_clearance");
      const auto & map = packet.at("controller_map");
      const auto & proof = map.at("stop_guard");
      if (proof.at("schema") != "kalo-mode3-stop-grid/v1" ||
        map.at("encoding") != "bitset-row-major/v1" ||
        proof.at("follow_session") != status.at("follow_session") ||
        map.at("reference_generation") != status.at("reference_generation")) {return;}
      Grid g;
      g.session = proof.at("follow_session").get<std::string>();
      g.generation = proof.at("transport_generation").get<int64_t>();
      g.reference = map.at("reference_generation").get<int64_t>();
      g.stamp = map.at("depth_stamp_ns").get<int64_t>();
      g.width = map.at("width").get<int>();
      g.height = map.at("height").get<int>();
      g.cell = map.at("cell_m").get<double>();
      g.x = map.at("origin_xy").at(0).get<double>();
      g.y = map.at("origin_xy").at(1).get<double>();
      g.radius = packet.at("capabilities").at("radius_m").get<double>();
      if (g.stamp <= 0 || g.generation < 0 || g.reference < 0 || g.width <= 0 ||
        g.width > 256 || g.height <= 0 || g.height > 256 || g.cell < .02 || g.cell > .1 ||
        !std::isfinite(g.cell) || !std::isfinite(g.radius) ||
        !std::isfinite(g.x) || !std::isfinite(g.y) || g.radius < .15 || g.radius > .5) {return;}
      const size_t bytes = (static_cast<size_t>(g.width*g.height)+7)/8;
      g.free = decode_bits(proof.at("observed_free_bits").get<std::string>(),bytes);
      g.blocked = decode_bits(map.at("blocked_bits").get<std::string>(),bytes);
      if (g.free.empty() || g.blocked.empty()) {return;}
      // The raw packet can be newer than this map. Preserve positive first
      // returns in the same reference while the next native map is prepared.
      if (packet.at("follow_session") != g.session || packet.at("reference_generation") != g.reference ||
        packet.at("transport_generation") != g.generation ||
        packet.at("cell_m") != g.cell) {return;}
      const double dx = (packet.at("origin_xy").at(0).get<double>()-g.x)/g.cell;
      const double dy = (packet.at("origin_xy").at(1).get<double>()-g.y)/g.cell;
      if (!std::isfinite(dx) || !std::isfinite(dy) || std::abs(dx)>512 || std::abs(dy)>512 ||
        std::abs(dx-std::round(dx))>1e-5 || std::abs(dy-std::round(dy))>1e-5) {return;}
      for (const auto & row : packet.at("blocked")) {
        const int64_t i = static_cast<int64_t>(row.at(0).get<int>())+std::llround(dx);
        const int64_t j = static_cast<int64_t>(row.at(1).get<int>())+std::llround(dy);
        if (i>=0 && i<g.width && j>=0 && j<g.height) {
          const auto k = static_cast<size_t>(i*g.height+j);
          g.blocked[k/8] |= static_cast<uint8_t>(0x80 >> (k%8));
        }
      }
      grid_ = std::move(g);
    } catch (const Json::exception &) {
      // Bad evidence leaves the guard engaged with no usable grid.
    }
  }

  const char * apply(const VelocityPolicy & p, Velocity & command, int64_t now_ns,
    double now, double decel) const
  {
    if (p.output_source != "PERSON" || command == Velocity{0.,0.}) {return "";}
    const auto it = p.commands.find("PERSON");
    const bool point = it != p.commands.end() && it->second.mode3_origin.is_object();
    if (!mode3_ && !point) {return "";}
    const Velocity selected = command;
    command = {0.,0.};
    if (!grid_ || !point || !mode3_ || now-p.wheel_at > .20 ||
      !std::isfinite(decel) || decel < .15 || decel > .65 ||
      !std::isfinite(p.wheel.first) || std::abs(p.wheel.first) > 2. ||
      !std::isfinite(p.wheel.second) || std::abs(p.wheel.second) > 4.) {return "mode3_evidence_unavailable";}
    const auto & o = it->second.mode3_origin;
    const Grid & g = *grid_;
    if (o.at("follow_session") != g.session || o.at("request_id") != p.person_request_id ||
      o.at("reference_generation") != g.reference || o.at("transport_generation") != g.generation) {
      return "mode3_reference_mismatch";
    }
    const auto & pose = o.at("control_pose");
    const double age = (now_ns-pose.at("stamp_ns").get<int64_t>())*1e-9;
    if (age < 0. || age > .45 || now_ns < g.stamp || now_ns-g.stamp > 450000000) {
      return "mode3_evidence_stale";
    }
    const auto & xy = pose.at("xy_yaw");
    const double x = xy[0].get<double>(), y = xy[1].get<double>(), yaw = xy[2].get<double>();
    // Cover motion already underway even if the new command asks for a turn or reversal.
    if (!safe_arc(g,x,y,yaw,p.wheel,age,decel)) {return "mode3_current_stop_envelope";}
    for (const double scale : {1., .65, .35, .15}) {
      const Velocity trial{selected.first*scale, selected.second*scale};
      if (safe_arc(g,x,y,yaw,trial,age,decel)) {
        command = trial;
        return scale == 1. ? "mode3_clear" : "mode3_slowed";
      }
    }
    return "mode3_stop_envelope";
  }
};
}  // namespace wla_manual_control
