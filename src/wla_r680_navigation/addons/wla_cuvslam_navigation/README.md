# cuVSLAM 导航与建图前端

ROS 2 Jazzy 的 `wla_cuvslam_navigation` 使用 cuVSLAM v17，输入 D455 左右红外图像、CameraInfo 和 `/wheel/imu/data_raw`。IMU 频率参数为 200 Hz；外参、时间偏移和噪声仍是候选配置，需要实测标定。D455 内部 IMU 不用于本链路。

## 数据输出

导航模式的链路为：

```text
infra1 / infra2 + CameraInfo + 车身 IMU
  -> cuvslam_odometry -> /r680_nav/vio_raw_odom
  -> degraded_odometry + /wheel/odom + 车身 IMU
  -> /d455_slam/odom + d455_floor_odom -> r680_mapping_floor
  -> RTAB-Map RGB-D localization -> map -> d455_floor_odom
  -> Nav2 MPC -> 速度平滑 / 碰撞监控 / 最终指令门禁
```

导航 launch 启用 `publish_visual_only` 并关闭前端 TF，由连续里程计节点拥有 odom -> base。初始化阶段保留必要的位姿 passthrough；建立可信视觉后，SDK 的纯 IMU 推算不能作为视觉锚点。通过新增的 `IsLastPoseInertialOnly()` 读取求解器实际分支，总观测点数不等于 PnP 求解成功。

建图模式不经过导航连续接管；前端发布 `/d455_slam/odom` 和 TF，再由 `mapping_odom_gate` 过滤到 `/r680_nav/mapping_odom` 供 RTAB-Map 建图。

## SDK 与构建

必须先应用导航仓库的 `addons/cuvslam-sdk/visual-pose-source.patch` 并重新构建 SDK，再构建本前端。未修补的 SDK 缺少新 API，公共头文件和运行时库必须来自一致构建。SDK 原始文件校验值和补丁说明与源码一并归档；完整 SDK 需单独准备。

既有 Orin 路径为 `~/ros2_ws/.runtime/cuvslam_validation/{cuVSLAM,sdk_build}`。本包置于 `~/ros2_ws/src/wla_cuvslam_navigation` 后执行：

```bash
bash ~/ros2_ws/src/wla_cuvslam_navigation/build_frontend.sh
```

脚本构建前端到 `.runtime/cuvslam_navigation/{build,install,log}`，并构建导航包、同步导航启动脚本。MPC 依赖和插件需按导航仓库的构建说明另行准备。原 `wla_cuvslam_validation` 包保留。

## 启动

```bash
cd ~/ros2_ws
./r680_nav.sh --map map-2026-10-08-1 --cuvslam
```

默认不执行底盘运动。VIO 初始化需要运动激励；重力、偏置、跟踪与速度估计满足稳定窗口后才报告健康。静止几秒或出现 IMU state 不能单独证明初始化完成。

由操作者确认起步条件后使用受限初始化与导航：

```bash
./r680_nav.sh --map map-2026-10-08-1 --cuvslam --auto-vio-init --enable-motion
```

退出当前运行后可用 `--rgbd` 回退到原 RGB-D VO / EKF，不在运行中热切换里程计原点。导航脚本默认前端仍为 RGB-D、控制器为 MPC。

## 健康与诊断

- `config/cuvslam.yaml` 是前端默认参数；导航包配置中的同名文件是导航脚本加载来源，以运行目录快照和进程参数为准。
- `/r680_nav/vio_tracking_healthy` 表示前端惯性初始化健康；`vio_status`、`vio_health_reason` 和每进程 `vio_health-*.jsonl` 提供诊断。
- `/r680_nav/continuous_odom_healthy`、`continuous_odom_status`、`odom_degraded` 由连续里程计节点发布，区分正常视觉、短时接管与阻断。
- `/r680_nav/vo_watchdog_healthy` 加入地图匹配和故障恢复条件；前端恢复输出不等于已经完成地图重定位。
- 新鲜视觉返回时的三帧一致性确认属于连续里程计；地图重定位接受一次几何验证匹配，两者不同。
- `navigation.json`、`cuvslam_statistics.json` 和本次配置快照记录实际前端与运行证据。

SDK API、前端规则与隔离 ROS 场景已验证；未完成真实视觉退化、长期轮速 / IMU 漂移及端到端实车恢复验收。
