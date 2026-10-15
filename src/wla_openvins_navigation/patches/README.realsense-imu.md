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
