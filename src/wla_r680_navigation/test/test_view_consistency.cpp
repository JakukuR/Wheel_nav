#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include "wla_r680_navigation/view_consistency.hpp"
namespace wla = wla_r680_navigation;
void require(bool value, const char * what) {if (!value) {std::cerr << what << "\n"; std::exit(1);}}
// The robot's true map pose for an odom pose, with a fixed map<-odom correction.
wla::PlanarPose truth(wla::PlanarPose odom) {
  const wla::PlanarPose c{3.0, -1.0, 0.8};
  return {c.x + std::cos(c.yaw)*odom.x - std::sin(c.yaw)*odom.y,
          c.y + std::sin(c.yaw)*odom.x + std::cos(c.yaw)*odom.y, wla::wrapYaw(c.yaw + odom.yaw)};
}
int main() {
  const wla::PlanarPose start{0.4, 0.2, 0.1};
  // Stationary repeats of one view are not independent evidence.
  wla::ViewConsistency still;
  for (int n = 0; n < 10; ++n) still.add(n, truth(start), start);
  require(!still.verified(), "repeated stationary view admitted");
  // The same place seen after a 30 degree turn agrees with odometry.
  wla::ViewConsistency turned;
  turned.add(1, truth(start), start);
  const wla::PlanarPose rotated{0.4, 0.2, 0.1 + 0.52};
  require(turned.add(2, truth(rotated), rotated), "consistent turned views rejected");
  // Forward 0.15 m is also a distinct viewpoint.
  wla::ViewConsistency forward;
  forward.add(1, truth(start), start);
  const wla::PlanarPose ahead{0.55, 0.2, 0.1};
  require(forward.add(2, truth(ahead), ahead), "consistent forward views rejected");
  // A look-alike place matched from the second view implies a different correction.
  wla::ViewConsistency alias;
  alias.add(1, truth(start), start);
  auto wrong = truth(rotated); wrong.x += 2.0; wrong.yaw += 0.4;
  require(!alias.add(2, wrong, rotated), "aliased second view admitted");
  // Small residuals inside the limits are accepted; larger ones are not.
  wla::ViewConsistency noisy;
  noisy.add(1, truth(start), start);
  auto near = truth(rotated); near.x += 0.2; near.yaw += 0.1;
  require(noisy.add(2, near, rotated), "residual inside limits rejected");
  wla::ViewConsistency off;
  off.add(1, truth(start), start);
  auto far = truth(rotated); far.x += 0.4;
  require(!off.add(2, far, rotated), "residual beyond limits admitted");
  // Views older than the window, reordered stamps and non-finite input do not count.
  wla::ViewConsistency stale;
  stale.add(1, truth(start), start);
  require(!stale.add(40, truth(rotated), rotated), "view outside window admitted");
  wla::ViewConsistency order;
  order.add(5, truth(start), start);
  require(!order.add(4, truth(rotated), rotated), "reordered view admitted");
  wla::ViewConsistency invalid;
  invalid.add(1, truth(start), start);
  auto nan = rotated; nan.x = std::numeric_limits<double>::quiet_NaN();
  require(!invalid.add(2, truth(rotated), nan), "non-finite view admitted");
  turned.reset();
  require(!turned.verified() && turned.views() == 0, "reset kept evidence");
  std::cout << "stationary repeats, turned/forward agreement, aliasing, limits, window, order and reset passed\n";
}
