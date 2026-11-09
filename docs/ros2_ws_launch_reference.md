# ROS launch 参数完整索引（2026-11-09）

从源码 AST 提取，不执行 launch。下表是 launch 的声明值；根 r680_*.sh、YAML、OpaqueFunction 和传入参数可能再次覆盖，常用最终入口见 [操作手册](ros2_ws_operations.md)。表达式保留源码，必填表示没有 default_value。

## src/explore/launch/explore.launch.py

源码：[文件](../src/explore/launch/explore.launch.py)。

| 行 | 参数 | 默认声明 | 可选值 |
| --- | --- | --- | --- |
| 19 | `'use_sim_time'` | `'true'` | `—` |
| 22 | `'namespace'` | `''` | `—` |

## src/gamepad_contorl/launch/gamepad_teleop.launch.py

源码：[文件](../src/gamepad_contorl/launch/gamepad_teleop.launch.py)。

无逐字写出的 DeclareLaunchArgument；可能由循环生成或只组合固定节点。

## src/open_vins/ov_msckf/launch/subscribe.launch.py

源码：[文件](../src/open_vins/ov_msckf/launch/subscribe.launch.py)。

| 行 | 参数 | 默认声明 | 可选值 |
| --- | --- | --- | --- |
| 11 | `'namespace'` | `'ov_msckf'` | `—` |
| 12 | `'ov_enable'` | `'true'` | `—` |
| 15 | `'rviz_enable'` | `'false'` | `—` |
| 18 | `'config'` | `'euroc_mav'` | `—` |
| 23 | `'config_path'` | `''` | `—` |
| 28 | `'verbosity'` | `'INFO'` | `—` |
| 33 | `'use_stereo'` | `'true'` | `—` |
| 38 | `'max_cameras'` | `'2'` | `—` |
| 43 | `'save_total_state'` | `'false'` | `—` |

## src/wla_behavior_bridge/launch/agent_person_api.launch.py

源码：[文件](../src/wla_behavior_bridge/launch/agent_person_api.launch.py)。

无逐字写出的 DeclareLaunchArgument；可能由循环生成或只组合固定节点。

## src/wla_behavior_bridge/launch/agent_semantic_api.launch.py

源码：[文件](../src/wla_behavior_bridge/launch/agent_semantic_api.launch.py)。

| 行 | 参数 | 默认声明 | 可选值 |
| --- | --- | --- | --- |
| 14 | `'map_directory'` | `必填` | `—` |
| 14 | `'policy_file'` | `必填` | `—` |
| 15 | `'realtime_whitelist'` | `whitelist` | `—` |
| 16 | `'annotations'` | `''` | `—` |
| 17 | `'execution_enabled'` | `'false'` | `—` |

## src/wla_behavior_bridge/launch/behavior_bridge.launch.py

源码：[文件](../src/wla_behavior_bridge/launch/behavior_bridge.launch.py)。

| 行 | 参数 | 默认声明 | 可选值 |
| --- | --- | --- | --- |
| 13 | `'map_directory'` | `必填` | `—` |
| 13 | `'policy_file'` | `必填` | `—` |
| 14 | `'selected_map_record'` | `必填` | `—` |
| 15 | `'annotations'` | `''` | `—` |
| 16 | `'output_topic'` | `'/r680_nav/task_safe_preview'` | `—` |
| 17 | `'allow_guarded_initialization'` | `'false'` | `—` |

## src/wla_cuvslam_validation/launch/validation.launch.py

源码：[文件](../src/wla_cuvslam_validation/launch/validation.launch.py)。

| 行 | 参数 | 默认声明 | 可选值 |
| --- | --- | --- | --- |
| 13 | `'use_imu'` | `'true'` | `—` |
| 14 | `'statistics_path'` | `'/tmp/cuvslam_validation_statistics.json'` | `—` |

## src/wla_explore_wander/launch/explore.launch.py

源码：[文件](../src/wla_explore_wander/launch/explore.launch.py)。

| 行 | 参数 | 默认声明 | 可选值 |
| --- | --- | --- | --- |
| 11 | `'session_budget_s'` | `'0.0'` | `—` |
| 11 | `'boundary_radius_m'` | `'4.0'` | `—` |
| 11 | `'seed'` | `'-1'` | `—` |

该文件通过循环声明：session_budget_s=0.0、boundary_radius_m=4.0、seed=-1。

## src/wla_manual_control/launch/manual_base.launch.py

源码：[文件](../src/wla_manual_control/launch/manual_base.launch.py)。

无逐字写出的 DeclareLaunchArgument；可能由循环生成或只组合固定节点。

## src/wla_openvins_navigation/launch/openvins.launch.py

源码：[文件](../src/wla_openvins_navigation/launch/openvins.launch.py)。

| 行 | 参数 | 默认声明 | 可选值 |
| --- | --- | --- | --- |
| 57 | `'config_dir'` | `必填` | `—` |
| 58 | `'production_topics'` | `'false'` | `['true', 'false']` |
| 59 | `'allow_test_config'` | `'false'` | `['true', 'false']` |
| 60 | `'allow_commissioning'` | `'false'` | `['true', 'false']` |
| 61 | `'publish_tf'` | `'false'` | `['true', 'false']` |
| 62 | `'use_sim_time'` | `'false'` | `['true', 'false']` |
| 63 | `'mode'` | `'mapping'` | `['mapping', 'localization']` |
| 64 | `'contact_topic'` | `'/r680_nav/contact_state'` | `—` |

## src/wla_person_follow/launch/follow.launch.py

源码：[文件](../src/wla_person_follow/launch/follow.launch.py)。

| 行 | 参数 | 默认声明 | 可选值 |
| --- | --- | --- | --- |
| 58 | `'diagnostic_log_directory'` | `str(Path.home() / 'ros2_ws/.runtime/follow-diagnostics')` | `—` |
| 59 | `'transport'` | `'cpp'` | `—` |
| 60 | `'python'` | `os.environ.get('WLA_FOLLOW_PYTHON', '/usr/bin/python3')` | `—` |
| 61 | `'config'` | `''` | `—` |
| 62 | `'enable_motion'` | `'false'` | `—` |
| 63 | `'local_planner'` | `'mpc'` | `['mpc', 'neupan']` |
| 64 | `'person_filter'` | `'cylinder'` | `['cylinder', 'cluster']` |
| 65 | `'depth_speckle'` | `'0'` | `—` |
| 66 | `'grid_route'` | `'dijkstra'` | `['dijkstra', 'astar']` |
| 67 | `'numeric_backend'` | `'python'` | `['python', 'cpp']` |
| 68 | `'enable_reid'` | `'true'` | `—` |
| 69 | `'output_mode'` | `'mux'` | `—` |
| 70 | `'direct_topic'` | `'/r680_base/cmd_vel'` | `—` |
| 71 | `'terrain'` | `'false'` | `['true', 'false']` |

## src/wla_r680_navigation/launch/bringup.launch.py

源码：[文件](../src/wla_r680_navigation/launch/bringup.launch.py)。

| 行 | 参数 | 默认声明 | 可选值 |
| --- | --- | --- | --- |
| 568 | `'mode'` | `'mapping'` | `['mapping', 'localization']` |
| 569 | `'start_d455'` | `'false'` | `['true', 'false']` |
| 570 | `'start_chassis'` | `'false'` | `['true', 'false']` |
| 571 | `'start_nav2'` | `'true'` | `['true', 'false']` |
| 572 | `'obstacle_scan'` | `'false'` | `['true', 'false']` |
| 573 | `'terrain_analysis'` | `'false'` | `['true', 'false']` |
| 574 | `'start_dynamic_obstacles'` | `'true'` | `['true', 'false']` |
| 575 | `'start_navigation_servers'` | `'true'` | `['true', 'false']` |
| 576 | `'agent_control'` | `'false'` | `['true', 'false']` |
| 577 | `'nav_params_file'` | `str(config / 'nav2.yaml')` | `—` |
| 579 | `'start_state_estimation'` | `'true'` | `['true', 'false']` |
| 580 | `'auto_vio_init'` | `'false'` | `['true', 'false']` |
| 581 | `'vio_init_result_path'` | `'/tmp/r680_vio_init.json'` | `—` |
| 582 | `'odom_source'` | `'rgbd'` | `['rgbd', 'cuvslam', 'openvins']` |
| 583 | `'openvins_config_dir'` | `''` | `—` |
| 584 | `'openvins_allow_commissioning'` | `'false'` | `['true', 'false']` |
| 585 | `'openvins_contact_topic'` | `'/r680_nav/contact_state'` | `—` |
| 586 | `'chassis_imu_topic'` | `'/wheel/imu/data_raw'` | `—` |
| 587 | `'chassis_odom_topic'` | `'/wheel/odom'` | `—` |
| 588 | `'cuvslam_params_file'` | `str(config / 'cuvslam.yaml')` | `—` |
| 589 | `'enable_degraded_odom'` | `'true'` | `['true', 'false']` |
| 590 | `'degraded_odom_params_file'` | `str(config / 'degraded_odometry.yaml')` | `—` |
| 591 | `'odom_backend'` | `'fusion'` | `['fusion', 'degraded']` |
| 592 | `'dark_infra'` | `'false'` | `['true', 'false']` |
| 594 | `'slam_image'` | `'color'` | `['color', 'infra']` |
| 596 | `'odom_fusion_params_file'` | `str(config / 'odom_fusion.yaml')` | `—` |
| 597 | `'cuvslam_statistics_path'` | `'/tmp/cuvslam_navigation_statistics.json'` | `—` |
| 598 | `'start_web'` | `'true'` | `['true', 'false']` |
| 599 | `'start_semantics'` | `'false'` | `['true', 'false']` |
| 600 | `'semantic_output'` | `'/tmp/wla-semantic.geojson'` | `—` |
| 601 | `'semantic_map_id'` | `'unassigned'` | `—` |
| 602 | `'semantic_whitelist'` | `str(config / 'yolo_mapping_whitelist.json')` | `—` |
| 603 | `'semantic_mark_home'` | `'false'` | `['true', 'false']` |
| 604 | `'semantic_python'` | `'/usr/bin/python3'` | `—` |
| 606 | `'semantic_model'` | `''` | `—` |
| 608 | `'web_host'` | `'0.0.0.0'` | `—` |
| 609 | `'web_port'` | `'8080'` | `—` |
| 610 | `'web_map_yaml'` | `''` | `—` |
| 613 | `'navigation_map_yaml'` | `web_map_yaml` | `—` |
| 616 | `'use_d455_imu'` | `'false'` | `['true', 'false']` |
| 617 | `'use_chassis_imu'` | `'false'` | `['true', 'false']` |
| 618 | `'enable_hardware_output'` | `'false'` | `['true', 'false']` |
| 619 | `'publish_mount_tf'` | `'false'` | `['true', 'false']` |
| 620 | `'database_path'` | `'/home/orin/.local/share/wla/r680-navigation/rtabmap.db'` | `—` |
| 622 | `'initial_pose'` | `''` | `—` |

## src/wla_r680_navigation/launch/d455_camera.launch.py

源码：[文件](../src/wla_r680_navigation/launch/d455_camera.launch.py)。

| 行 | 参数 | 默认声明 | 可选值 |
| --- | --- | --- | --- |
| 78 | `'infra'` | `'false'` | `['true', 'false']` |
| 80 | `'imu'` | `'false'` | `['true', 'false']` |
| 82 | `'emitter'` | `'on'` | `['on', 'off', 'alternate']` |
| 85 | `'mount'` | `'true'` | `['true', 'false']` |

## src/wla_terrain_analysis/launch/terrain_analysis.launch.py

源码：[文件](../src/wla_terrain_analysis/launch/terrain_analysis.launch.py)。

无逐字写出的 DeclareLaunchArgument；可能由循环生成或只组合固定节点。
