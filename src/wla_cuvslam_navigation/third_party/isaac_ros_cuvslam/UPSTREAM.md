# Official ROS covariance implementation

Source: NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
Commit: 9058506efd0b967ac894a717bdbe98df017637e5
License: Apache-2.0; original file notices are retained.

The following files are copied byte-for-byte, without algorithm changes:

- `pose_cache.cpp` from `isaac_ros_cuvslam/src/impl/pose_cache.cpp`
- `include/isaac_ros_cuvslam/impl/pose_cache.hpp` from
  `isaac_ros_cuvslam/include/isaac_ros_cuvslam/impl/pose_cache.hpp`

Upstream integration reference:
`isaac_ros_cuvslam/src/impl/visual_slam_impl.cpp`, `UpdatePose()`.
Odometry covariance uses pose and velocity caches, falling back to identity
until the caches contain enough samples. SDK pose covariance is not used for
the Odometry message. An empty SDK pose resets the pose cache. A Track exception
logs a warning and ends that frame's callback, without stopping the tracker.

Local integration uses the same cache implementations and fallback behavior.
It keeps the existing body-frame filtered velocity calculation and feeds the
published twist into VelocityCache. This change does not replace the complete
ROS frontend with Isaac ROS, nor modify the SDK or its inertial state machine.
The existing SDK covariance diagnostic is retained only as a log; it cannot
discard a frame, publish unhealthy status, or stop the processing loop.

The SDK reference audited for comparison is nvidia-isaac/cuVSLAM commit
2a0882cfada6448632f37182a4b8a270adb2ec3d. The ROS wrapper and SDK repositories
have different API versions; the cache code has no SDK dependency.
