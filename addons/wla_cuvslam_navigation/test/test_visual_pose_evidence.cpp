#include <cassert>
#include "visual_pose_evidence.hpp"

int main() {
  using wla_vio::visualPoseObserved;
  // The failed run retained hundreds of observations while PnP rejected the frame.
  assert(!visualPoseObserved(true, false, 100, 100, 680, 20));
  assert(visualPoseObserved(false, false, 100, 100, 26, 20));
  assert(!visualPoseObserved(false, true, 100, 100, 680, 20));
  assert(!visualPoseObserved(false, false, 99, 100, 680, 20));
  assert(!visualPoseObserved(false, false, 100, 100, 19, 20));
}
