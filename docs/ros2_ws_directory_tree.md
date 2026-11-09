# ros2_ws 源码与文档完整目录树

2026-11-09 快照。完整展开项目源码、配置、接口、文档与 src 内二进制图片/遮罩的文件名。
不展开 .git、build/install/log、虚拟环境、.runtime、模型/地图数据和历史原始测试日志；这些目录职责见 [框架](ros2_ws_architecture.md)。
原有文本文件逐项清单/行数/哈希见 [读取清单](ros2_ws_source_inventory.tsv)。新文档不在初始读取哈希中；本表不是实时目录。

```text
ros2_ws/
├── config/
│   └── agent_semantic/
│       ├── actual-map-preview-policy.json
│       ├── list-map.json
│       ├── list-visible.json
│       └── navigate-table-preview.json
├── docs/
│   ├── validation/
│   │   ├── maintenance-history/
│   │   │   └── README.md
│   │   ├── vio-health-20261013/
│   │   │   ├── manifest.json
│   │   │   ├── vio-finish-build.txt
│   │   │   ├── vio-fix-build.txt
│   │   │   ├── vio-nav-tests.txt
│   │   │   └── vio-open-test.txt
│   │   ├── 20261015-nav-manual-coexist.md
│   │   ├── continuity-restart-removal-20261015.md
│   │   ├── cuvslam-feature-preview-20261016.md
│   │   ├── cuvslam-initialization-authority-20261015.md
│   │   ├── cuvslam-initialization-speed-20261015.md
│   │   ├── cuvslam-official-covariance-20261016.md
│   │   ├── cuvslam-startup-match-20261015.md
│   │   ├── modeless-nav-20261015.md
│   │   ├── object-follow-20261013.md
│   │   ├── openvins-official-20261015.md
│   │   └── openvins-zupt-20261014.md
│   ├── Agent共享检测与YOLO模型接入.md
│   ├── Agent语义导航接入.md
│   ├── Agent附近探索执行映射.md
│   ├── VIO健康与地图定位确认_20261013.md
│   ├── package-history.md
│   ├── ros2_ws_architecture.md
│   ├── ros2_ws_audit_20261109.md
│   ├── ros2_ws_directory_tree.md
│   ├── ros2_ws_launch_reference.md
│   ├── ros2_ws_operations.md
│   ├── ros2_ws_source_inventory.tsv
│   └── 人物跟随合入与系统链路核验.md
├── src/
│   ├── explore/
│   │   ├── config/
│   │   │   ├── params.yaml
│   │   │   └── params_costmap.yaml
│   │   ├── doc/
│   │   │   ├── architecture.dia
│   │   │   ├── screenshot.png
│   │   │   └── wiki_doc.txt
│   │   ├── include/
│   │   │   └── explore/
│   │   │       ├── costmap_client.h
│   │   │       ├── costmap_tools.h
│   │   │       ├── explore.h
│   │   │       ├── frontier_search.h
│   │   │       └── frontier_standoff.hpp
│   │   ├── launch/
│   │   │   └── explore.launch.py
│   │   ├── src/
│   │   │   ├── costmap_client.cpp
│   │   │   ├── explore.cpp
│   │   │   └── frontier_search.cpp
│   │   ├── test/
│   │   │   ├── frontier_standoff_test.cpp
│   │   │   └── test_explore.cpp
│   │   ├── .gitignore
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   └── package.xml
│   ├── explore_lite_msgs/
│   │   ├── msg/
│   │   │   └── ExploreStatus.msg
│   │   ├── .gitignore
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   └── package.xml
│   ├── gamepad_contorl/
│   │   ├── config/
│   │   │   └── gamepad_params.yaml
│   │   ├── gamepad_control/
│   │   │   ├── __init__.py
│   │   │   ├── gamepad_teleop.py
│   │   │   └── keyboard_sim.py
│   │   ├── launch/
│   │   │   └── gamepad_teleop.launch.py
│   │   ├── resource/
│   │   │   └── gamepad_control
│   │   ├── test/
│   │   │   └── probe_gamepad_hotplug.py
│   │   ├── .gitignore
│   │   ├── README.md
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   ├── setup.py
│   │   └── start.sh
│   ├── open_vins/
│   │   ├── config/
│   │   │   ├── euroc_mav/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── kaist/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── kaist_vio/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── rpng_aruco/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── rpng_ironsides/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   ├── kalibr_imucam_chain.yaml
│   │   │   │   ├── mask_ironsides0.png
│   │   │   │   ├── mask_ironsides0.xcf
│   │   │   │   ├── mask_ironsides1.png
│   │   │   │   └── mask_ironsides1.xcf
│   │   │   ├── rpng_plane/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── rpng_sim/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── rs_d455/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   ├── kalibr_imucam_chain.yaml
│   │   │   │   └── launch_d455.example
│   │   │   ├── rs_t265/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   ├── kalibr_imucam_chain.yaml
│   │   │   │   ├── launch_t265.example
│   │   │   │   ├── mask_t265_cam0.png
│   │   │   │   ├── mask_t265_cam0.xcf
│   │   │   │   ├── mask_t265_cam1.png
│   │   │   │   └── mask_t265_cam1.xcf
│   │   │   ├── tum_vi/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   ├── kalibr_imucam_chain.yaml
│   │   │   │   ├── mask_tumvi0.png
│   │   │   │   ├── mask_tumvi0.xcf
│   │   │   │   ├── mask_tumvi1.png
│   │   │   │   └── mask_tumvi1.xcf
│   │   │   ├── uzhfpv_indoor/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── uzhfpv_indoor_45/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── uzhfpv_outdoor/
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   ├── kalibr_imucam_chain.yaml
│   │   │   │   ├── mask_uzhfpv_outdoor_mask0.png
│   │   │   │   ├── mask_uzhfpv_outdoor_mask0.xcf
│   │   │   │   ├── mask_uzhfpv_outdoor_mask1.png
│   │   │   │   └── mask_uzhfpv_outdoor_mask1.xcf
│   │   │   └── uzhfpv_outdoor_45/
│   │   │       ├── estimator_config.yaml
│   │   │       ├── kalibr_imu_chain.yaml
│   │   │       └── kalibr_imucam_chain.yaml
│   │   ├── ov_core/
│   │   │   ├── cmake/
│   │   │   │   ├── ROS1.cmake
│   │   │   │   └── ROS2.cmake
│   │   │   ├── src/
│   │   │   │   ├── cam/
│   │   │   │   │   ├── CamBase.h
│   │   │   │   │   ├── CamEqui.h
│   │   │   │   │   └── CamRadtan.h
│   │   │   │   ├── cpi/
│   │   │   │   │   ├── CpiBase.h
│   │   │   │   │   ├── CpiV1.cpp
│   │   │   │   │   ├── CpiV1.h
│   │   │   │   │   ├── CpiV2.cpp
│   │   │   │   │   └── CpiV2.h
│   │   │   │   ├── feat/
│   │   │   │   │   ├── Feature.cpp
│   │   │   │   │   ├── Feature.h
│   │   │   │   │   ├── FeatureDatabase.cpp
│   │   │   │   │   ├── FeatureDatabase.h
│   │   │   │   │   ├── FeatureHelper.h
│   │   │   │   │   ├── FeatureInitializer.cpp
│   │   │   │   │   ├── FeatureInitializer.h
│   │   │   │   │   └── FeatureInitializerOptions.h
│   │   │   │   ├── plot/
│   │   │   │   │   ├── LICENSE
│   │   │   │   │   └── matplotlibcpp.h
│   │   │   │   ├── sim/
│   │   │   │   │   ├── BsplineSE3.cpp
│   │   │   │   │   └── BsplineSE3.h
│   │   │   │   ├── track/
│   │   │   │   │   ├── Grider_FAST.h
│   │   │   │   │   ├── Grider_GRID.h
│   │   │   │   │   ├── TrackAruco.cpp
│   │   │   │   │   ├── TrackAruco.h
│   │   │   │   │   ├── TrackBase.cpp
│   │   │   │   │   ├── TrackBase.h
│   │   │   │   │   ├── TrackDescriptor.cpp
│   │   │   │   │   ├── TrackDescriptor.h
│   │   │   │   │   ├── TrackKLT.cpp
│   │   │   │   │   ├── TrackKLT.h
│   │   │   │   │   ├── TrackSIM.cpp
│   │   │   │   │   └── TrackSIM.h
│   │   │   │   ├── types/
│   │   │   │   │   ├── IMU.h
│   │   │   │   │   ├── JPLQuat.h
│   │   │   │   │   ├── Landmark.cpp
│   │   │   │   │   ├── Landmark.h
│   │   │   │   │   ├── LandmarkRepresentation.h
│   │   │   │   │   ├── PoseJPL.h
│   │   │   │   │   ├── Type.h
│   │   │   │   │   └── Vec.h
│   │   │   │   ├── utils/
│   │   │   │   │   ├── colors.h
│   │   │   │   │   ├── dataset_reader.h
│   │   │   │   │   ├── opencv_lambda_body.h
│   │   │   │   │   ├── opencv_yaml_parse.h
│   │   │   │   │   ├── print.cpp
│   │   │   │   │   ├── print.h
│   │   │   │   │   ├── quat_ops.h
│   │   │   │   │   └── sensor_data.h
│   │   │   │   ├── dummy.cpp
│   │   │   │   ├── test_profile.cpp
│   │   │   │   ├── test_tracking.cpp
│   │   │   │   └── test_webcam.cpp
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   └── rosdoc.yaml
│   │   ├── ov_init/
│   │   │   ├── cmake/
│   │   │   │   ├── ROS1.cmake
│   │   │   │   └── ROS2.cmake
│   │   │   ├── launch/
│   │   │   │   ├── display.rviz
│   │   │   │   ├── dynamic.launch
│   │   │   │   └── mle.launch
│   │   │   ├── src/
│   │   │   │   ├── ceres/
│   │   │   │   │   ├── Factor_GenericPrior.cpp
│   │   │   │   │   ├── Factor_GenericPrior.h
│   │   │   │   │   ├── Factor_ImageReprojCalib.cpp
│   │   │   │   │   ├── Factor_ImageReprojCalib.h
│   │   │   │   │   ├── Factor_ImuCPIv1.cpp
│   │   │   │   │   ├── Factor_ImuCPIv1.h
│   │   │   │   │   ├── State_JPLQuatLocal.cpp
│   │   │   │   │   └── State_JPLQuatLocal.h
│   │   │   │   ├── dynamic/
│   │   │   │   │   ├── DynamicInitializer.cpp
│   │   │   │   │   └── DynamicInitializer.h
│   │   │   │   ├── init/
│   │   │   │   │   ├── InertialInitializer.cpp
│   │   │   │   │   ├── InertialInitializer.h
│   │   │   │   │   └── InertialInitializerOptions.h
│   │   │   │   ├── sim/
│   │   │   │   │   ├── SimulatorInit.cpp
│   │   │   │   │   └── SimulatorInit.h
│   │   │   │   ├── static/
│   │   │   │   │   ├── StaticInitializer.cpp
│   │   │   │   │   └── StaticInitializer.h
│   │   │   │   ├── utils/
│   │   │   │   │   └── helper.h
│   │   │   │   ├── dummy.cpp
│   │   │   │   ├── test_dynamic_init.cpp
│   │   │   │   ├── test_dynamic_mle.cpp
│   │   │   │   └── test_simulation.cpp
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   └── rosdoc.yaml
│   │   ├── ov_msckf/
│   │   │   ├── cmake/
│   │   │   │   ├── ROS1.cmake
│   │   │   │   └── ROS2.cmake
│   │   │   ├── launch/
│   │   │   │   ├── display.rviz
│   │   │   │   ├── display_driving.rviz
│   │   │   │   ├── display_ros2.rviz
│   │   │   │   ├── serial.launch
│   │   │   │   ├── simulation.launch
│   │   │   │   ├── subscribe.launch
│   │   │   │   └── subscribe.launch.py
│   │   │   ├── scripts/
│   │   │   │   ├── run_ros_eth.sh
│   │   │   │   ├── run_ros_kaist.sh
│   │   │   │   ├── run_ros_tumvi.sh
│   │   │   │   ├── run_ros_uzhfpv.sh
│   │   │   │   ├── run_sim_calib.sh
│   │   │   │   ├── run_sim_cams.sh
│   │   │   │   └── run_sim_featrep.sh
│   │   │   ├── src/
│   │   │   │   ├── core/
│   │   │   │   │   ├── VioManager.cpp
│   │   │   │   │   ├── VioManager.h
│   │   │   │   │   ├── VioManagerHelper.cpp
│   │   │   │   │   └── VioManagerOptions.h
│   │   │   │   ├── ros/
│   │   │   │   │   ├── ROS1Visualizer.cpp
│   │   │   │   │   ├── ROS1Visualizer.h
│   │   │   │   │   ├── ROS2Visualizer.cpp
│   │   │   │   │   ├── ROS2Visualizer.h
│   │   │   │   │   ├── ROSVisualizerHelper.cpp
│   │   │   │   │   └── ROSVisualizerHelper.h
│   │   │   │   ├── sim/
│   │   │   │   │   ├── Simulator.cpp
│   │   │   │   │   └── Simulator.h
│   │   │   │   ├── state/
│   │   │   │   │   ├── Propagator.cpp
│   │   │   │   │   ├── Propagator.h
│   │   │   │   │   ├── State.cpp
│   │   │   │   │   ├── State.h
│   │   │   │   │   ├── StateHelper.cpp
│   │   │   │   │   ├── StateHelper.h
│   │   │   │   │   └── StateOptions.h
│   │   │   │   ├── update/
│   │   │   │   │   ├── UpdaterHelper.cpp
│   │   │   │   │   ├── UpdaterHelper.h
│   │   │   │   │   ├── UpdaterMSCKF.cpp
│   │   │   │   │   ├── UpdaterMSCKF.h
│   │   │   │   │   ├── UpdaterOptions.h
│   │   │   │   │   ├── UpdaterSLAM.cpp
│   │   │   │   │   ├── UpdaterSLAM.h
│   │   │   │   │   ├── UpdaterZeroVelocity.cpp
│   │   │   │   │   └── UpdaterZeroVelocity.h
│   │   │   │   ├── utils/
│   │   │   │   │   └── NoiseManager.h
│   │   │   │   ├── dummy.cpp
│   │   │   │   ├── ros1_serial_msckf.cpp
│   │   │   │   ├── run_simulation.cpp
│   │   │   │   ├── run_subscribe_msckf.cpp
│   │   │   │   ├── test_sim_meas.cpp
│   │   │   │   └── test_sim_repeat.cpp
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   └── rosdoc.yaml
│   │   └── LICENSE
│   ├── wla_behavior_bridge/
│   │   ├── config/
│   │   │   ├── policy.example.json
│   │   │   └── yolo_realtime_whitelist.json
│   │   ├── env-hooks/
│   │   │   └── agent_runtime.dsv.in
│   │   ├── launch/
│   │   │   ├── agent_person_api.launch.py
│   │   │   ├── agent_semantic_api.launch.py
│   │   │   └── behavior_bridge.launch.py
│   │   ├── scripts/
│   │   │   ├── agent_person_api
│   │   │   ├── agent_semantic_api
│   │   │   ├── nav_perception_adapter
│   │   │   ├── navigation_task_gateway
│   │   │   └── task_velocity_gate
│   │   ├── test/
│   │   │   └── test_gateway_foreign_goal.py
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   └── package.xml
│   ├── wla_behavior_bt/
│   │   ├── src/
│   │   │   └── behavior_executor.cpp
│   │   ├── trees/
│   │   │   └── navigate_semantic.xml
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   └── package.xml
│   ├── wla_behavior_interfaces/
│   │   ├── action/
│   │   │   └── BehaviorTask.action
│   │   ├── msg/
│   │   │   └── TaskLease.msg
│   │   ├── srv/
│   │   │   ├── AgentSemanticCommand.srv
│   │   │   ├── PersonCommand.srv
│   │   │   └── ResolveTarget.srv
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   └── package.xml
│   ├── wla_cuvslam_navigation/
│   │   ├── config/
│   │   │   └── cuvslam.yaml
│   │   ├── src/
│   │   │   ├── cuvslam_odometry.cpp
│   │   │   ├── feature_overlay.hpp
│   │   │   ├── feature_visualizer.hpp
│   │   │   ├── health_evidence.hpp
│   │   │   ├── initialization_sampling.hpp
│   │   │   ├── odometry_covariance.hpp
│   │   │   ├── visual_pose_evidence.hpp
│   │   │   └── visual_quality.hpp
│   │   ├── test/
│   │   │   ├── test_feature_overlay.cpp
│   │   │   ├── test_feature_preview_ros.cpp
│   │   │   ├── test_health_evidence.cpp
│   │   │   ├── test_initialization_sampling.cpp
│   │   │   ├── test_odometry_covariance.cpp
│   │   │   ├── test_upstream_frame_recovery.py
│   │   │   ├── test_visual_pose_evidence.cpp
│   │   │   └── test_visual_quality.cpp
│   │   ├── third_party/
│   │   │   └── isaac_ros_cuvslam/
│   │   │       ├── include/
│   │   │       │   └── isaac_ros_cuvslam/
│   │   │       │       └── impl/
│   │   │       │           └── pose_cache.hpp
│   │   │       ├── LICENSE
│   │   │       ├── UPSTREAM.md
│   │   │       └── pose_cache.cpp
│   │   ├── .gitignore
│   │   ├── CMakeLists.txt
│   │   ├── COVARIANCE-DIAGNOSTICS-20261009.md
│   │   ├── DEGRADED-ODOM-20261009.md
│   │   ├── DIAGNOSTICS-20261008.md
│   │   ├── POSE-OUTPUT-20261009.md
│   │   ├── README.md
│   │   ├── build_frontend.sh
│   │   └── package.xml
│   ├── wla_cuvslam_validation/
│   │   ├── config/
│   │   │   └── validation.yaml
│   │   ├── launch/
│   │   │   └── validation.launch.py
│   │   ├── scripts/
│   │   │   ├── check_input_guards.py
│   │   │   ├── r680_vio_test.sh
│   │   │   ├── replay_validation.py
│   │   │   ├── run_validation.py
│   │   │   └── sensor_probe.py
│   │   ├── src/
│   │   │   └── cuvslam_validation.cpp
│   │   ├── .gitignore
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── VALIDATION_200HZ_RESULTS.md
│   │   ├── VALIDATION_RESULTS.md
│   │   ├── package.xml
│   │   └── sdk_manifest.json
│   ├── wla_detection_bridge/
│   │   ├── resource/
│   │   │   └── wla_detection_bridge
│   │   ├── test/
│   │   │   └── test_stream.py
│   │   ├── wla_detection_bridge/
│   │   │   ├── __init__.py
│   │   │   └── stream.py
│   │   ├── README.md
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   └── setup.py
│   ├── wla_diff_mpc/
│   │   ├── config/
│   │   │   └── mpc_example.yaml
│   │   ├── include/
│   │   │   └── wla_diff_mpc/
│   │   │       ├── obstacle_braking.hpp
│   │   │       ├── qp.hpp
│   │   │       ├── reference.hpp
│   │   │       ├── solver.hpp
│   │   │       └── terminal_alignment.hpp
│   │   ├── src/
│   │   │   ├── controller.cpp
│   │   │   ├── qp.cpp
│   │   │   ├── reference.cpp
│   │   │   └── solver.cpp
│   │   ├── test/
│   │   │   ├── check_terminal_controller.py
│   │   │   ├── obstacle_braking_regression.cpp
│   │   │   ├── qp_regression.cpp
│   │   │   ├── reference_regression.cpp
│   │   │   ├── terminal_alignment_regression.cpp
│   │   │   └── test_solver.cpp
│   │   ├── CHANGES-20261008.md
│   │   ├── CHANGES-TERMINAL-20261008.md
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── package.xml
│   │   └── plugins.xml
│   ├── wla_explore_wander/
│   │   ├── launch/
│   │   │   └── explore.launch.py
│   │   ├── resource/
│   │   │   └── wla_explore_wander
│   │   ├── test/
│   │   │   └── test_explorer.py
│   │   ├── wla_explore_wander/
│   │   │   ├── __init__.py
│   │   │   ├── explorer.py
│   │   │   ├── memory.py
│   │   │   ├── node.py
│   │   │   └── policy.py
│   │   ├── README.md
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   └── setup.py
│   ├── wla_follow_local/
│   │   ├── include/
│   │   │   └── wla_follow/
│   │   │       ├── rolling_map.h
│   │   │       └── tracking.h
│   │   ├── src/
│   │   │   ├── follow_core_module.cc
│   │   │   ├── follow_core_tracking.cc
│   │   │   ├── grid_kernels.h
│   │   │   ├── isolated_clusters.hpp
│   │   │   ├── numpy_blas.h
│   │   │   ├── reference_path.cc
│   │   │   ├── rolling_map.cc
│   │   │   ├── sim_geometry.cpp
│   │   │   └── tracking.cc
│   │   ├── test/
│   │   │   └── cpp/
│   │   │       ├── test_rolling_map.cpp
│   │   │       └── test_tracking.cpp
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   └── package.xml
│   ├── wla_follow_transport/
│   │   ├── include/
│   │   │   └── wla_follow_transport/
│   │   │       ├── channel.hpp
│   │   │       └── policy.hpp
│   │   ├── src/
│   │   │   └── follow_transport.cpp
│   │   ├── test/
│   │   │   └── test_transport_policy.cpp
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   └── package.xml
│   ├── wla_manual_control/
│   │   ├── config/
│   │   │   ├── chassis.yaml
│   │   │   ├── cyclonedds-local.xml
│   │   │   └── gamepad.yaml
│   │   ├── deploy/
│   │   │   └── r680-manual-base.service
│   │   ├── include/
│   │   │   └── wla_manual_control/
│   │   │       └── velocity_policy.hpp
│   │   ├── launch/
│   │   │   └── manual_base.launch.py
│   │   ├── scripts/
│   │   │   ├── control_policy.py
│   │   │   ├── start_manual_base.sh
│   │   │   └── velocity_policy.py
│   │   ├── src/
│   │   │   ├── command_mux.cpp
│   │   │   └── velocity_policy.cpp
│   │   ├── test/
│   │   │   ├── test_cpp_policy_equivalence.py
│   │   │   ├── test_mux_impl_ros.py
│   │   │   ├── test_nav_manual_shared.py
│   │   │   ├── test_object_control_handoff.py
│   │   │   ├── test_object_mux_ros.py
│   │   │   ├── test_object_policy.py
│   │   │   ├── test_policy.py
│   │   │   ├── test_relative_mux_ros.py
│   │   │   ├── test_velocity_mux_ros.py
│   │   │   ├── test_velocity_policy.py
│   │   │   └── velocity_policy_replay.cpp
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   └── package.xml
│   ├── wla_openvins_navigation/
│   │   ├── config/
│   │   │   ├── r680_d455_20261013/
│   │   │   │   ├── calibration.json
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── frontend.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── r680_wheel_20261013/
│   │   │   │   ├── calibration.json
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── frontend.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── synthetic/
│   │   │   │   ├── calibration.json
│   │   │   │   ├── estimator_config.yaml
│   │   │   │   ├── frontend.yaml
│   │   │   │   ├── kalibr_imu_chain.yaml
│   │   │   │   └── kalibr_imucam_chain.yaml
│   │   │   ├── calibration_manifest.example.yaml
│   │   │   ├── estimator_template.yaml
│   │   │   ├── r680_d455_20261013_manifest.yaml
│   │   │   ├── r680_wheel_20261013_manifest.yaml
│   │   │   └── synthetic_manifest.yaml
│   │   ├── include/
│   │   │   └── wla_openvins_navigation/
│   │   │       └── adapter.hpp
│   │   ├── launch/
│   │   │   └── openvins.launch.py
│   │   ├── patches/
│   │   │   ├── README.realsense-imu.md
│   │   │   ├── openvins-jazzy-headers.patch
│   │   │   └── realsense-4.58.1-imu-mode.patch
│   │   ├── scripts/
│   │   │   ├── build_isolated.sh
│   │   │   ├── build_realsense_imu_fix.sh
│   │   │   ├── generate_config.py
│   │   │   ├── inspect_config.py
│   │   │   ├── record_d455_debug.sh
│   │   │   ├── record_metrics.py
│   │   │   └── stereo_fault_injector.py
│   │   ├── src/
│   │   │   └── odometry_adapter.cpp
│   │   ├── test/
│   │   │   ├── test_adapter.cpp
│   │   │   ├── test_calibration.py
│   │   │   └── test_realsense_imu_mode.py
│   │   ├── .gitignore
│   │   ├── CMakeLists.txt
│   │   ├── COPYING
│   │   ├── README.md
│   │   ├── VERSION.json
│   │   ├── openvins.repos
│   │   └── package.xml
│   ├── wla_person_follow/
│   │   ├── config/
│   │   │   ├── reid_model.json
│   │   │   └── sim_follow.yaml
│   │   ├── docs/
│   │   │   ├── 2026-10-22_几何绑定与ReID职责拆分设计_v2.md
│   │   │   ├── FOLLOW_DIAGNOSTICS_20261022.md
│   │   │   ├── TRIAL_TARGET_PICKER.md
│   │   │   ├── continuous_follow_client.md
│   │   │   ├── continuous_local_free.md
│   │   │   ├── follow_scenarios_and_known_issues.md
│   │   │   ├── reid_recovery.md
│   │   │   ├── shared_depth_filter.md
│   │   │   ├── stall2_runtime_fix.md
│   │   │   ├── startup_free_prior.md
│   │   │   └── tracker_follow.md
│   │   ├── launch/
│   │   │   └── follow.launch.py
│   │   ├── neupan_bridge/
│   │   │   ├── CMakeLists.txt
│   │   │   ├── README.md
│   │   │   ├── bridge.cpp
│   │   │   ├── build.sh
│   │   │   └── r_la.yaml
│   │   ├── resource/
│   │   │   └── wla_person_follow
│   │   ├── scripts/
│   │   │   └── person_follow
│   │   ├── test/
│   │   │   ├── test_diagnostics.py
│   │   │   ├── test_follow_chain_ros.py
│   │   │   ├── test_goal_modes.py
│   │   │   ├── test_hybrid_protocol.py
│   │   │   ├── test_hybrid_ros.py
│   │   │   ├── test_native_map.py
│   │   │   ├── test_native_rolling_map.py
│   │   │   ├── test_native_tracking.py
│   │   │   ├── test_neupan_follower.py
│   │   │   ├── test_observation_path.py
│   │   │   ├── test_output_continuity.py
│   │   │   ├── test_person_cluster.py
│   │   │   ├── test_person_trail_clear.py
│   │   │   ├── test_reference_bspline.py
│   │   │   ├── test_reid_ros.py
│   │   │   ├── test_relative_control.py
│   │   │   ├── test_relative_geometry.py
│   │   │   ├── test_review_expansion.py
│   │   │   ├── test_review_topic.py
│   │   │   ├── test_sim_port.py
│   │   │   ├── test_sim_ros.py
│   │   │   ├── test_target_ghost_clear.py
│   │   │   ├── test_target_kalman.py
│   │   │   ├── test_tracking_fast_paths.py
│   │   │   ├── test_trial_client.py
│   │   │   ├── test_turn_detour.py
│   │   │   └── test_wide_recovery.py
│   │   ├── tools/
│   │   │   └── map_replay/
│   │   │       ├── compare.py
│   │   │       ├── compare_tracking.py
│   │   │       ├── rec_main.py
│   │   │       └── rec_tracking.py
│   │   ├── wla_person_follow/
│   │   │   ├── sim_core/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── approach_goal.py
│   │   │   │   ├── common.py
│   │   │   │   ├── detour.py
│   │   │   │   ├── hybrid_controller.py
│   │   │   │   ├── rolling_depth_map.py
│   │   │   │   ├── target_reference.py
│   │   │   │   └── tracking.py
│   │   │   ├── __init__.py
│   │   │   ├── camera.py
│   │   │   ├── depth_proc.py
│   │   │   ├── diagnostics.py
│   │   │   ├── follow_config.py
│   │   │   ├── follow_session.py
│   │   │   ├── hybrid_worker.py
│   │   │   ├── motion_memory.py
│   │   │   ├── native_map.py
│   │   │   ├── native_tracking.py
│   │   │   ├── neupan_follower.py
│   │   │   ├── person_cluster.py
│   │   │   ├── sim_node.py
│   │   │   ├── sim_runtime.py
│   │   │   ├── status_recorder.py
│   │   │   ├── task_modes.py
│   │   │   └── task_session.py
│   │   ├── FULL_CHAIN_TESTS.md
│   │   ├── HYBRID_TRANSPORT.md
│   │   ├── LATENCY_OPTIMIZATION.md
│   │   ├── PY_CPP_ARCHITECTURE.md
│   │   ├── README.md
│   │   ├── REID_ENROLLMENT_FIX.md
│   │   ├── SIM_CORE_PROVENANCE.json
│   │   ├── SIM_DEPLOYMENT.md
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   └── setup.py
│   ├── wla_r680_navigation/
│   │   ├── addons/
│   │   │   ├── cuvslam-sdk/
│   │   │   │   ├── .gitattributes
│   │   │   │   ├── README.md
│   │   │   │   ├── sdk-base-sha256.json
│   │   │   │   └── visual-pose-source.patch
│   │   │   ├── COLCON_IGNORE
│   │   │   └── README.md
│   │   ├── config/
│   │   │   ├── agent_observation_window.json
│   │   │   ├── chassis.yaml
│   │   │   ├── cuvslam.yaml
│   │   │   ├── d455_cuvslam.yaml
│   │   │   ├── d455_emitter_alternate.yaml
│   │   │   ├── d455_emitter_off.yaml
│   │   │   ├── d455_emitter_on.yaml
│   │   │   ├── d455_points.yaml
│   │   │   ├── d455_scan.yaml
│   │   │   ├── degraded_odometry.yaml
│   │   │   ├── dynamic_obstacles.yaml
│   │   │   ├── ekf_vo_imu.yaml
│   │   │   ├── exploration.yaml
│   │   │   ├── goal_approach_limiter.yaml
│   │   │   ├── interfaces.yaml
│   │   │   ├── kalo_bridge.yaml
│   │   │   ├── mission.yaml
│   │   │   ├── nav2.yaml
│   │   │   ├── nav2_mpc.yaml
│   │   │   ├── nav2_recovery_test.yaml
│   │   │   ├── nav2_test.yaml
│   │   │   ├── nav2_test_scan_only.yaml
│   │   │   ├── navigate_through_poses_no_spin.xml
│   │   │   ├── navigate_to_pose_no_spin.xml
│   │   │   ├── odom_fusion.yaml
│   │   │   ├── path_speed_profile.yaml
│   │   │   ├── perception.botsort.yaml
│   │   │   ├── r680_mapping.rviz
│   │   │   ├── r680_navigation.rviz
│   │   │   ├── rtabmap.yaml
│   │   │   ├── storage.yaml
│   │   │   ├── vio_initializer.yaml
│   │   │   ├── vio_initializer_cuvslam.yaml
│   │   │   ├── vo_watchdog.yaml
│   │   │   ├── web_gateway.yaml
│   │   │   └── yolo_mapping_whitelist.json
│   │   ├── docs/
│   │   │   ├── changes/
│   │   │   │   ├── R680-DEGRADED-ODOM-001.md
│   │   │   │   ├── R680-DEGRADED-ODOM-002.md
│   │   │   │   ├── R680-FIXES-20261009.md
│   │   │   │   ├── R680-MPC-STATIC-GLOBAL-001.md
│   │   │   │   ├── R680-OBSTACLE-001.md
│   │   │   │   ├── R680-RECOVERY-001.md
│   │   │   │   ├── R680-RECOVERY-002.md
│   │   │   │   ├── R680-TERMINAL-001.md
│   │   │   │   └── R680-VIO-OUTPUT-001.md
│   │   │   ├── AGENT_OBSERVATION_WINDOW.md
│   │   │   ├── BOXMOT_MIGRATION.md
│   │   │   ├── CUVSLAM-RECOVERY-20261016.md
│   │   │   ├── GMC_RECOVERY.md
│   │   │   ├── R680-CUVSLAM-001.md
│   │   │   ├── R680-CUVSLAM-002.md
│   │   │   ├── R680-CUVSLAM-003.md
│   │   │   ├── R680-CUVSLAM-004.md
│   │   │   ├── R680-CUVSLAM-005.md
│   │   │   ├── R680-CUVSLAM-006.md
│   │   │   ├── R680-MAP-EDIT-001.md
│   │   │   ├── R680-MAP-EDIT-002.md
│   │   │   ├── R680-NAV-007-mapless.md
│   │   │   ├── R680-NAV-008-odom-fusion.md
│   │   │   ├── R680-NAV-009-lost-recovery.md
│   │   │   ├── R680-NAV-010-emitter-alternate.md
│   │   │   ├── R680导航建图启动链路与代码目录树.md
│   │   │   ├── debug_web_smoothing.md
│   │   │   ├── openvins_20261013.md
│   │   │   ├── openvins_motion_init.md
│   │   │   ├── openvins_navigation_motion.md
│   │   │   ├── openvins_visual_health_20261014.md
│   │   │   ├── perception_debug.md
│   │   │   └── planning_topdown.md
│   │   ├── include/
│   │   │   └── wla_r680_navigation/
│   │   │       ├── degraded_odometry.hpp
│   │   │       ├── isolated_clusters.hpp
│   │   │       ├── map_localization_evidence.hpp
│   │   │       ├── odom_fusion.hpp
│   │   │       ├── recovery_evidence.hpp
│   │   │       ├── relocalization_prior.hpp
│   │   │       ├── scan_projection.hpp
│   │   │       ├── view_consistency.hpp
│   │   │       ├── vio_init_policy.hpp
│   │   │       └── vo_recovery_policy.hpp
│   │   ├── kalo_web/
│   │   │   ├── index.html
│   │   │   ├── kalo.css
│   │   │   ├── kalo.jpg
│   │   │   └── kalo.js
│   │   ├── launch/
│   │   │   ├── bringup.launch.py
│   │   │   └── d455_camera.launch.py
│   │   ├── scripts/
│   │   │   ├── accept_navigation_map_edit
│   │   │   ├── agent_observation_window
│   │   │   ├── build_native_botsort
│   │   │   ├── build_osnet_engine
│   │   │   ├── build_yoloe_engine
│   │   │   ├── emitter_frame_splitter
│   │   │   ├── furniture_semantic.py
│   │   │   ├── kalo_debug_web
│   │   │   ├── mapping_mission
│   │   │   ├── nav_perception_bringup
│   │   │   ├── nav_yolo_detector
│   │   │   ├── navigation_goal_bridge
│   │   │   ├── navigation_map_mode
│   │   │   ├── navigation_motion_session.py
│   │   │   ├── openvins_motion_init.py
│   │   │   ├── openvins_nav_startup.py
│   │   │   ├── perception_debug_standalone
│   │   │   ├── prepare_mapping_run
│   │   │   ├── r680_mapping.sh
│   │   │   ├── r680_mode_lock.sh
│   │   │   ├── r680_nav.sh
│   │   │   ├── r680_teleop.sh
│   │   │   ├── resolve_navigation_map
│   │   │   ├── save_manual_map
│   │   │   ├── wait_mapping_ready
│   │   │   └── web_gateway
│   │   ├── src/
│   │   │   ├── command_guard.cpp
│   │   │   ├── degraded_odometry.cpp
│   │   │   ├── depth_to_points.cpp
│   │   │   ├── dynamic_obstacle_memory.cpp
│   │   │   ├── goal_approach_limiter.cpp
│   │   │   ├── imu_conditioner.cpp
│   │   │   ├── interface_monitor.cpp
│   │   │   ├── kalo_jpeg.cpp
│   │   │   ├── mapping_odom_gate.cpp
│   │   │   ├── odom_fusion.cpp
│   │   │   ├── odom_fusion_core.cpp
│   │   │   ├── path_speed_profile.cpp
│   │   │   ├── point_to_scan.cpp
│   │   │   ├── timed_obstacle_layer.cpp
│   │   │   ├── vio_initializer.cpp
│   │   │   └── vo_watchdog.cpp
│   │   ├── test/
│   │   │   ├── check_degraded_odometry.py
│   │   │   ├── check_global_obstacle_replan.py
│   │   │   ├── check_relocalization_prior.py
│   │   │   ├── check_rtabmap_recovery_parameters.py
│   │   │   ├── check_staged_relocalization.py
│   │   │   ├── mux_process.py
│   │   │   ├── probe_cuvslam_navigation.py
│   │   │   ├── probe_mapping_workflow.py
│   │   │   ├── test_boxmot_runtime.py
│   │   │   ├── test_continuity_restart.py
│   │   │   ├── test_cuvslam_feature_gateway.py
│   │   │   ├── test_cuvslam_initialization_authority.py
│   │   │   ├── test_cuvslam_launch_parameters.py
│   │   │   ├── test_cuvslam_startup_match.py
│   │   │   ├── test_cuvslam_startup_timeout.py
│   │   │   ├── test_cuvslam_transient_recovery.py
│   │   │   ├── test_cuvslam_watchdog.py
│   │   │   ├── test_debug_standalone.py
│   │   │   ├── test_degraded_odometry.cpp
│   │   │   ├── test_emitter_frame_splitter.py
│   │   │   ├── test_frontend_interface.py
│   │   │   ├── test_identity_features.py
│   │   │   ├── test_imu_conditioner_timing.py
│   │   │   ├── test_isolated_clusters.cpp
│   │   │   ├── test_kalo_debug_ros.py
│   │   │   ├── test_map_localization_evidence.cpp
│   │   │   ├── test_mapless_navigation.py
│   │   │   ├── test_mapping_cuvslam_ros.py
│   │   │   ├── test_mapping_run.py
│   │   │   ├── test_nav_detector_ros.py
│   │   │   ├── test_navigation_motion_session.py
│   │   │   ├── test_navigation_startup_evidence.py
│   │   │   ├── test_navigation_static_map.py
│   │   │   ├── test_no_degraded_launch.py
│   │   │   ├── test_observation_export.py
│   │   │   ├── test_odom_fusion.cpp
│   │   │   ├── test_odom_fusion_ros.py
│   │   │   ├── test_openvins_control.py
│   │   │   ├── test_openvins_motion_init.py
│   │   │   ├── test_openvins_startup.py
│   │   │   ├── test_perception_geometry.py
│   │   │   ├── test_raw_image.py
│   │   │   ├── test_recovery_evidence.cpp
│   │   │   ├── test_reid_engine.py
│   │   │   ├── test_relocalization_prior.cpp
│   │   │   ├── test_rgbd_watchdog.py
│   │   │   ├── test_scan_projection.cpp
│   │   │   ├── test_slam_image_source.py
│   │   │   ├── test_startup_view_consistency.py
│   │   │   ├── test_storage.py
│   │   │   ├── test_target_masks.py
│   │   │   ├── test_timed_obstacle_layer.cpp
│   │   │   ├── test_view_consistency.cpp
│   │   │   ├── test_vio_init_policy.cpp
│   │   │   ├── test_vio_init_ros.py
│   │   │   ├── test_visual_odom_sensor_independence.py
│   │   │   ├── test_vo_recovery_policy.cpp
│   │   │   ├── test_web_status_semantics.py
│   │   │   └── test_yoloe_engine.py
│   │   ├── web/
│   │   │   ├── app.js
│   │   │   ├── index.html
│   │   │   ├── robot-sticker.png
│   │   │   ├── style.css
│   │   │   ├── vio.html
│   │   │   └── vio.js
│   │   ├── wla_r680_navigation_py/
│   │   │   ├── __init__.py
│   │   │   ├── archive.py
│   │   │   ├── boxmot_reid.py
│   │   │   ├── boxmot_runtime.py
│   │   │   ├── identity_features.py
│   │   │   ├── mission.py
│   │   │   ├── mission_core.py
│   │   │   ├── observation_export.py
│   │   │   ├── perception_geometry.py
│   │   │   ├── raw_image.py
│   │   │   ├── reid_engine.py
│   │   │   ├── startup_evidence.py
│   │   │   ├── storage.py
│   │   │   ├── target_masks.py
│   │   │   └── yoloe_engine.py
│   │   ├── .gitignore
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── costmap_plugins.xml
│   │   └── package.xml
│   └── wla_terrain_analysis/
│       ├── config/
│       │   └── terrain_analysis.yaml
│       ├── figures/
│       │   ├── live_compare.png
│       │   ├── live_overlay_after.png
│       │   └── live_overlay_before.png
│       ├── include/
│       │   └── wla_terrain_analysis/
│       │       └── terrain_core.hpp
│       ├── launch/
│       │   └── terrain_analysis.launch.py
│       ├── src/
│       │   ├── terrain_analysis_node.cpp
│       │   ├── terrain_core.cpp
│       │   └── terrain_odometry.cpp
│       ├── test/
│       │   ├── test_terrain_core.cpp
│       │   ├── test_terrain_node_ros.py
│       │   └── test_terrain_odometry_ros.py
│       ├── CMakeLists.txt
│       ├── README.md
│       └── package.xml
├── .gitattributes
├── .gitignore
├── AGENTS.md
├── FOLLOW_TRIAL.md
├── README.md
├── person_follow_trial.py
├── r680_agent.sh
├── r680_camera.sh
├── r680_follow_bag.sh
├── r680_follow_replay.sh
├── r680_follow_trial.sh
├── r680_mapping.sh
├── r680_nav.sh
├── r680_perception.sh
├── r680_person_follow.sh
├── r680_teleop.sh
├── r680_vio_test.sh
├── start_nav.md
├── maps/         （地图数据，不展开）
├── models/       （本机引擎/跟踪库，不展开）
├── build/        （构建产物，不展开）
├── install/      （安装产物，不展开）
├── log/          （构建日志，不展开）
├── .runtime/     （运行和独立构建数据，不展开）
├── .venv-boxmot/ （本机环境，不展开）
├── .venv-perception/ （本机环境，不展开）
└── .git/         （版本历史，不展开）
```
