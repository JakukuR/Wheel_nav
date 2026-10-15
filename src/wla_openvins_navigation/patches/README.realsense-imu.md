# RealSense 4.58.1 IMU mode dispatch fix

The D455 motion-sensor callback in upstream tag 4.58.1 calls
`imu_callback_sync(frame)`. Its declaration defaults the second argument to
`COPY`, so `unite_imu_method=2` enables united output but still takes COPY.
The patch passes the configured `_imu_sync_method` explicitly (one changed line).
It keeps modes 0/1/2 and the upstream interpolation algorithm unchanged.

Source: https://github.com/realsenseai/realsense-ros/tree/4.58.1
Archive SHA256: d48d0591f5a575a546521cf1fed50f240f2a4c9d5705b6d836ad00a8b689e651
Relevant files: realsense2_camera/src/rs_node_setup.cpp,
include/base_realsense_node.h, src/base_realsense_node.cpp.

From /home/orin/ros2_ws:

```bash
bash src/wla_openvins_navigation/scripts/build_realsense_imu_fix.sh
```

The build installs a separate ROS wrapper under
`.runtime/realsense_openvins/install`. It does not replace /opt/ros or librealsense.
The workspace `r680_nav.sh --openvins` entry sources this overlay when using D455
IMU; cuVSLAM/RGB-D and wheel-IMU OpenVINS retain their existing package selection.
For a direct ROS launch, source this overlay's `local_setup.bash` after the
normal workspace setup. Existing processes must be restarted to load the new
library. Do not restart a camera owned by another active navigation session.

The wrapper uses the existing private OpenVINS cv_bridge (OpenCV 4.8).
Its own installed RPATH resolves that library before the system OpenCV 4.6
bridge, without changing global LD_LIBRARY_PATH for other navigation nodes.
The build requires that existing private bridge.

The build script runs a hardware-free C++ regression using the actual upstream
callback, default-argument declaration and interpolation/copy functions.
For accel samples (0,0,0) at 100 ms and (10,20,30) at 110 ms, a gyro sample at
105 ms must produce (5,10,15) in mode 2 and (0,0,0) in mode 1.
Mode 0 must not publish united data. Restoring the old call is a negative
control: the mode-2 assertion must fail.

The estimator config in config/r680_d455_20261013 now follows OpenVINS commit
69488123ed9362dd44b6f28e7f4680abbff1442b, config/rs_d455/estimator_config.yaml.
The only semantic override within that file is max_cameras=2 for IR stereo.
The existing device-specific camera/IMU calibration, topics and 200 Hz rate
remain unchanged; this is not a copy of the example RGB camera calibration.
Official defaults disable ZUPT, enable camera intrinsic/extrinsic/time-offset
online estimation, set max_slam=50 and init_imu_thresh=1.5.
Static initialization may wait for motion under these defaults.
Calibration remains commissioning/unvalidated.

## Validation on Orin, 2026-10-15

- Wrapper rebuilt and installed; source dispatch regression passed all three
  modes and correctly rejected the old call in the negative control.
- Installed library SHA256:
  dfe2203bbf92b2be5d3c113c9359348ba4fc3d7699440d2389509e733a726540.
- Loader inspection resolved the private cv_bridge and only OpenCV 4.8,
  while retaining /home/orin/robot_sensors/realsense/current/lib librealsense.
- Existing 3 calibration-contract tests and 11 startup-evidence tests passed.
  Isolated domain-198 mock startup sequencing passed. Domain-219 OpenVINS
  config smoke test loaded both cameras and was stopped by SIGINT after 10 s.
- After the pre-existing navigation/camera session exited, a bounded IMU-only
  test ran on domain 219 with no images, navigation or chassis commands.
  Firmware 5.17.3.10; wrapper 4.58.1; SDK 2.58.1; gyro 200 / accel 100 Hz;
  unite_imu_method=2. The test stopped its own camera process afterward.
- Captured accel=1880, gyro=3702, united IMU=3698 messages. United rate was
  199.717 Hz. All united gyro vectors matched raw gyro data.
- 3695 of 3698 united accelerations matched the timestamp-bracketed linear
  interpolation. The 3 exceptions were at 24.965, 29.958 and 34.951 ms after
  the first united sample. They exactly match extrapolation from earlier
  accelerometer pairs (alpha 1.535, 2.903, 4.271), consistent with the unchanged
  upstream arrival-order algorithm. Strict full-window interpolation acceptance
  therefore did NOT pass; this is not concealed by the steady-state check.
- After the first second, all 3507 samples matched linear interpolation,
  maximum vector error 1.8e-15 m/s^2. Median error against COPY was
  0.01608 m/s^2. This verifies mode-2 dispatch on hardware.
- Raw stamps were normalized to the upstream CimuData double-nanosecond
  representation before matching (observed rounding <=128 ns).
- SDK also warned that IMU calibration was unavailable and default intrinsic/
  extrinsic values would be used. This separate warning was not changed here.

Capture, raw samples and results are under .runtime/realsense_openvins/:
hardware_interpolation.log, hardware_interpolation_samples.json,
hardware_interpolation_result.json. No full stereo VIO accuracy or motion
acceptance was performed. Startup extrapolation and SDK calibration availability
remain separate follow-ups; the requested driver fix remains one line.


## Direct OpenVINS integration retest, 2026-10-15 19:55:56

Run: /home/orin/ros2_ws/.runtime/openvins_official/interpolation-live-20261015-195556

Real D455 serial 260922306083, stereo IR 640x480/30 Hz with emitter off,
gyro 200 Hz, accel 100 Hz, unite_imu_method=2. The patched wrapper fed the
official OpenVINS ROS2 node and odometry adapter in isolated ROS domain 219.
The config was copied from the committed default into the run directory.
No navigation stack, chassis interface or motion-command publisher was started.
The user manually moved the rig and explicitly confirmed when it was stationary.
All test processes were stopped at completion.

The first 120-second attempt (interpolation-live-20261015-195157) did not
initialize: image-disparity checks never exceeded 10 px (old/new window maxima
8.674/8.455 px). The message "no accel jerk detected" is issued by the visual
motion gate in InertialInitializer.cpp; it alone does not prove insufficient IMU
excitation. The user then requested a restart because the cue had been missed.

The second 180-second attempt initialized and produced 27,355 sampled raw
OpenVINS odometry messages. All saved poses/velocities were finite. Raw pose
norm stayed within 0.721 m and peak speed was 0.365 m/s for this manual motion.
There were no "No IMU measurements", "Missing inertial measurements" or
"ZERO DT" messages. This bounded trial did not reproduce the earlier rapid
divergence; it does not establish ground-truth accuracy or isolate which of the
driver/config changes contributed.

After the user-confirmed stop plus a 2-second settling period:
- Stationary window: 64.965 s, 13,006 raw odometry samples.
- Raw endpoint position drift: 0.007489 m; maximum displacement: 0.007955 m.
- Raw speed median / P95 / max: 0.001176 / 0.002190 / 0.003281 m/s.
- Adapter endpoint drift: 0.007606 m.
- Estimated camera-IMU offset: 2.14 to 3.85 ms, ending at 2.20 ms.

After the first second of received IMU data, all 35,100 checked united samples
matched interpolation from bracketed raw accelerometer measurements (maximum
vector error 5.33e-15 m/s^2); gyro vectors matched exactly. Median error against
COPY was 0.0270 m/s^2. Runtime mode-2 behavior is therefore verified during
stereo/OpenVINS processing as well as in the earlier IMU-only capture.

Arrival rate was approximately 200.2 Hz for IMU and 29.9 Hz for sampled images.
A 366.7 ms image-header gap in the Python diagnostic was cross-checked against
the independent rosbag: bag images had a maximum gap of only 40.401 ms.
Do not attribute the diagnostic gap to camera hardware or the estimator.
The bag continued briefly during process shutdown, so raw count differences
also include that additional recording interval.

Remaining observations:
- During the first 0.11 seconds of received data, IMU headers jumped forward by
  16.258 s while receive time advanced only 76.8 ms. This was a timestamp
  discontinuity, not a 16-second delivery stall. After the first 5 seconds,
  maximum IMU header spacing was 5.005 ms. The earlier attempt also had a
  startup timestamp jump. No timestamp behavior was changed in the one-line fix.
- The SDK warning about unavailable factory IMU calibration remained.
- Full navigation/load testing, long trajectories and externally measured
  accuracy remain unverified. Device calibration is still marked unvalidated.

Evidence: run/config/, camera.log, openvins.log, bag/, samples.json,
stationary_marker.json, result.json, analysis.json, bag_crosscheck.json.
