#pragma once
#include <cmath>
namespace wla::follow {
// User-confirmed footprint, centred on the existing base reference; metres.
inline constexpr double kRobotLengthM = .29;
inline constexpr double kRobotWidthM = .22;
inline const double kRobotRadiusM = std::hypot(kRobotLengthM, kRobotWidthM) / 2.;
}
