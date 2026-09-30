# cuVSLAM 实车导航前端（试验接入）

使用已固定的 cuVSLAM v17 SDK，输入 D455 左右红外图像和车身原始 IMU。
原 `wla_cuvslam_validation` 验证包保留，不依赖 Agent 链路。

```text
infra1/infra2 + CameraInfo + /wheel/imu/data_raw
    -> cuvslam_odometry -> /d455_slam/odom + d455_floor_odom -> r680_mapping_floor
RGB/Depth + odom + 地图工作数据库 -> RTAB-Map -> map -> d455_floor_odom
静态地图 + 局部 Scan -> Nav2 MPC -> 原速度/碰撞/最终指令保护 -> 底盘
```

## 构建

```bash
bash ~/ros2_ws/src/wla_cuvslam_navigation/build_frontend.sh
```

构建产物位于 `~/ros2_ws/.runtime/cuvslam_navigation/{build,install,log}`。
复用 `~/ros2_ws/.runtime/cuvslam_validation/{cuVSLAM,sdk_build}`，不修改 SDK。
导航包仍安装到 `~/ros2_ws/install`。

## 启动与回退

先停止独立底盘/手柄会话，避免重复占用串口。先关闭运动输出验证：

```bash
cd ~/ros2_ws
./r680_nav.sh --map map-2026-09-29-1 --test-scan-only --cuvslam
```

需要惯性初始化激励时可在预览中手推；保留纹理、缓慢移动和转向。
不是每次启动只静止五秒就可以确认 VIO 初始化。当前外参、时间偏移和噪声仍是候选值。
SDK 的 IMU state 可早于重力初始化出现，因此不能单独作为放行依据。

完成定位验证后由操作者启动运动测试：

```bash
./r680_nav.sh --map map-2026-09-29-1 --test-scan-only --cuvslam --enable-motion
```

回退：先 Ctrl+C 退出当前运行，再执行：

```bash
./r680_nav.sh --map map-2026-09-29-1 --test-scan-only --rgbd --enable-motion
```

不加前端选项仍默认 `rgbd`。失败时不在运行中热切换坐标原点。
源地图数据库只复制到本次 `nav_run`，不覆盖 maps 下的原件。

## 配置与健康

- 实际加载参数：`wla_r680_navigation/config/cuvslam.yaml`。
- 相机：`wla_r680_navigation/config/d455_cuvslam.yaml`，被动双目，关闭投射器。
- `initialization_stable_s: 2.0`：连续有效重力、偏置、跟踪和速度估计窗口。
- 重力模长要求 8～11.5 m/s²；偏置范数上限 gyro 0.25 rad/s、accel 3 m/s²。
  这些是试验健康阈值，应在标定/实测后审核，不能以调大阈值替代标定。
- 不运行原 VO、EKF、Madgwick 和 IMU conditioner，不重复融合 IMU。
- 速度由相邻位姿在车体坐标系下估计，低通时间常数 0.08 s；不是轮速替代值。
  SDK 未公开速度协方差，使用带下限的位姿差分不确定度；首帧速度标为未知。
- `/r680_nav/vio_tracking_healthy`：前端是否完成稳定惯性初始化。
- `/r680_nav/vio_status`：等待初始化、就绪、输入断流、无效位姿等状态。
- `/r680_nav/vo_watchdog_healthy`：加入 RTAB 地图匹配和漂移诊断后的最终健康。
- 初始地图匹配需要有效 localization_pose、匹配事件和新鲜地图 TF。
- 看门狗只重启本前端明确匹配的一个进程，仍要求地图重新匹配后恢复运动。
- `navigation.json` 记录 `odom_source`；本次目录保存 cuvslam.yaml 和统计 JSON。

## 验证边界

传感器/无运动输出验证不代表已经通过实车运动导航。尚需确认惯性初始化与
地图匹配持续稳定，并在短距离目标测试中检查制动、目标收敛和故障恢复。
