#pragma once
#include <cmath>
#include <cstddef>
#include <deque>
#include "wla_r680_navigation/relocalization_prior.hpp"

namespace wla_r680_navigation {
// Whole-map relocalization: one PnP-verified match can still be a look-alike place,
// and the same stationary view repeats the same wrong answer. Admit only when two
// matches from different viewpoints imply the same map->odom correction, i.e. the
// two map poses differ exactly by the robot's own odometry motion between them.
class ViewConsistency {
public:
  struct Limits {
    double correction_m{0.30}, correction_rad{0.17};  // agreement of the two corrections
    double view_m{0.10}, view_rad{0.25};              // minimum viewpoint change in odom
    double window_s{30.0};
  };
  ViewConsistency() = default;
  explicit ViewConsistency(Limits limits) : limits_(limits) {}

  // correction(map, odom) = T_map_base * inverse(T_odom_base), planar.
  static PlanarPose correction(PlanarPose map, PlanarPose odom) {
    const double yaw = wrapYaw(map.yaw - odom.yaw);
    return {map.x - (std::cos(yaw)*odom.x - std::sin(yaw)*odom.y),
            map.y - (std::sin(yaw)*odom.x + std::cos(yaw)*odom.y), yaw};
  }
  void reset() {views_.clear(); verified_ = false;}
  // stamp: ROS seconds of the matched frame; map: verified map pose; odom: base in odom.
  bool add(double stamp, PlanarPose map, PlanarPose odom) {
    if (!std::isfinite(stamp + map.x + map.y + map.yaw + odom.x + odom.y + odom.yaw) ||
        (!views_.empty() && stamp <= views_.back().stamp)) return verified_;
    while (!views_.empty() && stamp - views_.front().stamp > limits_.window_s) views_.pop_front();
    const View view{stamp, correction(map, odom), odom};
    for (const auto & previous : views_) {
      const bool distinct = std::hypot(odom.x - previous.odom.x, odom.y - previous.odom.y) >= limits_.view_m ||
        std::abs(wrapYaw(odom.yaw - previous.odom.yaw)) >= limits_.view_rad;
      const bool agree = std::hypot(view.correction.x - previous.correction.x,
                                    view.correction.y - previous.correction.y) <= limits_.correction_m &&
        std::abs(wrapYaw(view.correction.yaw - previous.correction.yaw)) <= limits_.correction_rad;
      if (distinct && agree) verified_ = true;
    }
    views_.push_back(view);
    return verified_;
  }
  bool verified() const {return verified_;}
  std::size_t views() const {return views_.size();}

private:
  struct View {double stamp; PlanarPose correction, odom;};
  Limits limits_{};
  std::deque<View> views_;
  bool verified_{false};
};
}  // namespace wla_r680_navigation
