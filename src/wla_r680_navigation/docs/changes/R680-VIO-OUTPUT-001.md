# cuVSLAM 位姿输出参数同步

2026-10-09。

配合 wla_cuvslam_navigation 移除相邻位姿运动幅度停止条件，删除实际导航加载的 config/cuvslam.yaml 中 max_pose_speed_mps、max_pose_angular_speed_radps。外部 vo_watchdog.yaml、恢复逻辑和 MPC 参数保持原样。

前端继续保留时间戳、无效数值、四元数、协方差及惯性初始化检查。本次只编译和软件验证，不启动车辆。重新启动建图或导航后生效。
