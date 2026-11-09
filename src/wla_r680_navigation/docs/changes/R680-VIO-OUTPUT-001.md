# cuVSLAM 位姿输出参数同步

> 2026-11-09 核对：本文保留所述版本的设计/变更与验证证据；其中旧参数、包名、IP、启动/权限步骤不自动代表当前部署。当前框架见 [工作区总览](../../../../docs/ros2_ws_architecture.md)，操作见 [启动与参数](../../../../docs/ros2_ws_operations.md)。历史实测结果不因本轮文档核对而重新获得验收。

2026-10-09。

配合 wla_cuvslam_navigation 移除相邻位姿运动幅度停止条件，删除实际导航加载的 config/cuvslam.yaml 中 max_pose_speed_mps、max_pose_angular_speed_radps。外部 vo_watchdog.yaml、恢复逻辑和 MPC 参数保持原样。

前端继续保留时间戳、无效数值、四元数、协方差及惯性初始化检查。本次只编译和软件验证，不启动车辆。重新启动建图或导航后生效。
