# VIO 重启后的自动重定位初值

日期：2026-10-08。

## 行为

`vo_watchdog` 保留故障前最近可信的地图位姿和同一时刻轮式里程计。前端重启后，轮速连续低于 0.02 m/s、0.04 rad/s 达 0.5 秒，视觉跟踪连续正常 1 秒，才向 `/d455_slam/initialpose` 发送一次位置提示：

```text
T_map_base_now = T_map_base_anchor × inverse(T_wheel_base_anchor) × T_wheel_base_now
```

只允许故障后 20 秒内、轮式位移不超过 0.50 m、转角不超过 0.80 rad 的短时提示。没有可信锚点、轮速反馈过期或运动量超限时不发送。提示只帮助 RTAB-Map 在附近搜索，不能替代真实地图匹配，也不修改运动锁。

解锁仍要求重启后的真实 RTAB-Map 匹配、地图定位消息、地图 TF 与故障前锚点/轮式位移一致，以及连续稳定 2 秒。没有地图匹配时保持零速；原导航目标的恢复仍由既有任务逻辑处理。

## 配置与日志

参数文件：`config/vo_watchdog.yaml`。新参数为 `relocalization_prior_enabled`、`initial_pose_topic`、`prior_max_translation_m`、`prior_max_yaw_rad`、`prior_stop_stable_s`、`prior_tracking_stable_s`、`prior_max_wait_s`。

日志关键字：`relocalization prior sent`、`relocalization prior refused`、`recovery remains locked`。后者每 5 秒给出跟踪、地图匹配、地图位姿、TF、锚点及提示状态。

## 验证与回退

纯 C++ 测试验证 SE(2) 变换、角度归一化和限幅。`test/check_relocalization_prior.py` 在 ROS_DOMAIN_ID=74 用虚拟前端及合成消息验证：未停稳不发送、停稳只发送一次、缺少地图匹配不解锁、匹配后稳定才解锁。测试不发布底盘速度。

关闭功能：把 `config/vo_watchdog.yaml` 中 `relocalization_prior_enabled` 改为 `false`，重新启动导航。未进行实车运动验证；视觉场景缺少纹理或旧地图不匹配时，自动提示仍可能无法恢复定位。
