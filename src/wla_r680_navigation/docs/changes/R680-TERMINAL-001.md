# 终点到达需确认实际停稳

日期：2026-10-08。适用于 `nav2_mpc.yaml`、`nav2_test.yaml`、`nav2_recovery_test.yaml`。

三个 MPC 参数文件把 GoalChecker 从 SimpleGoalChecker 改为 `nav2_controller::StoppedGoalChecker`，保持 XY 容差 0.20 m、朝向容差 0.18 rad；新增线速度 0.025 m/s、角速度 0.04 rad/s 的停车条件，避免车仍在转动时报告到达。

FollowPath 配合 `wla_diff_mpc` 新终点状态机：停车连续稳定 0.35 秒后独立对齐朝向，考虑实际角速度和响应时间提前制动，进入容差后保持零速。参数直接写在三个 YAML 的 FollowPath 下；速度源为 `/d455_slam/odom`，新鲜度 0.20 秒。实现和验证见 MPC 包 `CHANGES-TERMINAL-20261008.md`。

启动命令不变，例如：

```bash
cd ~/ros2_ws
./r680_nav.sh --map map-2026-10-08-1 \
  --test-recovery --cuvslam --auto-vio-init --enable-motion
```

已编译并在隔离 ROS 域中用实际控制器验证；未驱动实车。默认 MPPI 参数文件 `nav2.yaml` 和原有用户修改未改动。当前运行进程不会自动加载新插件和参数，需结束后重新启动导航。

观察本次日志的 `MPC terminal`：应先出现 stopping，再进入 align/brake，最后 hold。反复 stopping 且 fresh=0 表示速度来源或新鲜度未满足；持续 brake 时检查实测角速度是否真正降低。
