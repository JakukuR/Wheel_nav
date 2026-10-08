#pragma once
#include <cmath>
#include <optional>

namespace wla_r680_navigation {
struct PlanarPose {double x, y, yaw;};
inline double wrapYaw(double a) {return std::atan2(std::sin(a), std::cos(a));}
// Compose T_map_base(anchor) * inverse(T_wheel_base(anchor)) * T_wheel_base(now).
// Wheel feedback is a bounded short-stop prior, never map-match evidence.
inline std::optional<PlanarPose> stoppedRelocalizationPrior(
  PlanarPose map, PlanarPose wheel_anchor, PlanarPose wheel_now,
  double max_translation, double max_yaw)
{
  const double dx = wheel_now.x-wheel_anchor.x, dy = wheel_now.y-wheel_anchor.y;
  const double yaw = wrapYaw(wheel_now.yaw-wheel_anchor.yaw);
  if (!std::isfinite(map.x+map.y+map.yaw+dx+dy+yaw) ||
      std::hypot(dx,dy) > max_translation || std::abs(yaw) > max_yaw) return {};
  const double a = map.yaw-wheel_anchor.yaw;
  return PlanarPose{map.x+std::cos(a)*dx-std::sin(a)*dy,
    map.y+std::sin(a)*dx+std::cos(a)*dy, wrapYaw(map.yaw+yaw)};
}
}
