# R680 cuVSLAM 自动运动初始化试验版

2026-09-30。实现位于 Orin 的 `/home/orin/ros2_ws/src/wla_r680_navigation`，不修改 Agent 链路。默认仍使用原 RGB-D VO + EKF；自动初始化必须同时指定 `--cuvslam --auto-vio-init`。

## 启动

先关闭独立底盘驱动、手柄和上一轮导航，避免串口冲突。相机朝向家具、门框等有纹理的场景。

只预览，不向底盘发送指令：

```bash
cd /home/orin/ros2_ws
./r680_nav.sh --map map-2026-09-29-1 --test-scan-only --cuvslam --auto-vio-init
```

预览状态机可以生成运动请求，但最终保护的真实输出保持关闭；静止时 cuVSLAM 可能无法完成惯性初始化，超时属于预期结果。

确认起步区空旷、人在车旁且可以急停后，执行实车试验：

```bash
cd /home/orin/ros2_ws
./r680_nav.sh --map map-2026-09-29-1 --test-scan-only --cuvslam --auto-vio-init --enable-motion
```

这条命令会在启动阶段自主低速移动，不需要先在 Web 发送目标。成功交接后再发送导航目标，控制器沿用 MPC。

退出本次运行后回到原前端：

```bash
./r680_nav.sh --map map-2026-09-29-1 --test-scan-only --rgbd --enable-motion
```

## 初始化过程

```text
waiting_inputs         等待相机、VO、轮速、IMU 和安全节点可用
    ↓
stationary             连续确认轮速静止 0.8 秒
    ↓
moving                 低速前进，同时小幅左右转向，提供运动激励
    ↓
stopping               清零请求，连续确认轮速停稳 0.5 秒
    ↓
waiting_localization   等待 cuVSLAM 惯性初始化稳定及 RTAB 地图定位通过
    ↓
succeeded              接管新收到的导航指令，开放正常导航门控

任意活动阶段发生故障 → failed → 持续零输出，人工退出后重新启动
```

已经完成惯性初始化时跳过运动，仍须停稳及地图定位通过。失败不自动重试；不会通过降低 cuVSLAM 初始化阈值、伪造健康状态或重复播放缓存导航指令来启动车辆。

## 配置与边界

源码配置：`/home/orin/ros2_ws/src/wla_r680_navigation/config/vio_initializer.yaml`。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `linear_speed` | 0.06 m/s | 前进请求上限，实际请求在 0.03～0.06 m/s 之间变化 |
| `angular_speed` | 0.20 rad/s | 正弦变化的小幅转向请求上限 |
| `target_travel_m` | 0.16 m | 累计轮式里程达到此值即结束激励 |
| `motion_duration_s` | 8 s | 单次激励最多持续时间 |
| `max_travel_m` / `max_radius_m` | 0.25 / 0.25 m | 累计轮式路程 / VO 与轮式位移的较大值，达到即故障锁定 |
| `max_yaw_rad` | 0.35 rad | 相对起步朝向的最大偏转，达到即锁定 |
| `max_measured_linear` / `max_measured_angular` | 0.12 m/s / 0.35 rad/s | 实测轮速异常边界 |
| `startup_timeout_s` | 45 s | 等待输入及连续静止的最长时间 |
| `initialization_timeout_s` | 20 s | 激励开始后完成初始化、停稳、地图定位的总时间预算 |
| `sensor_timeout_s` | 0.35 s | 输入接收及传感器时间戳新鲜度 |
| `minimum_imu_rate` | 100 Hz | 原始车身 IMU 的最低采样率，当前目标约 200 Hz |
| `minimum_depth_valid_ratio` | 0.35 | 深度中央区域的有效采样占比门限 |
| `obstacle_forward_distance` / `obstacle_half_width` | 0.65 / 0.40 m | 前方安全点云检测范围 |

结束激励不等于初始化成功；激励结束后仍须等待 cuVSLAM 报告惯性就绪。硬编码试验上限限制初始化请求不超过 0.08 m/s、0.20 rad/s，路程和位移预算不超过 0.25 m，偏角不超过 0.35 rad，总初始化预算不超过 20 s。

位移限制依赖里程计测量，不能保证车体实际停止距离。单个前向 D455 无法覆盖侧后方盲区；深度有效率也不能证明所有空间可通行，因此这一版用于预先确认空旷的起步区。起步区安全点云出现障碍或数据不连续时，不会强行开始运动。

## 指令与健康接口

```text
正常控制器 / 恢复行为
    → /r680_nav/nav_command_input
    → C++ vio_initializer 仲裁（成功前忽略导航指令）
    → /cmd_vel_nav
    → velocity_smoother
    → collision_monitor
    → /r680_nav/cmd_vel_collision_checked
    → command_guard
    → /r680_nav/chassis_cmd_vel
    → 底盘驱动
```

自动初始化开启时，`vio_initializer` 是 `/cmd_vel_nav` 的唯一发布节点。最终 `command_guard` 才能发布驱动指令。

初始化阶段使用独立的限速授权，持续检查输入、近场障碍和安全节点状态。最终保护进一步限制当前碰撞检查结果不得超过当前初始化请求；请求清零后，速度平滑残留不能继续推动小车。初始化节点失联时，授权在约 0.20 s 后失效，实际停车响应仍取决于底盘。

成功后回到原有地图定位健康、任务授权和指令超时门控。停止、等待和失败状态均禁止转发导航运动。

| 接口 | 用途 |
|---|---|
| `/r680_nav/vio_init_state` | 状态字符串，20 Hz |
| `/r680_nav/vio_init_request` | 带时间戳的初始化速度请求 |
| `/r680_nav/vio_init_permit` | 初始化阶段短时授权 |
| `/r680_nav/cmd_vel_safe_preview` | 最终保护后的预览速度 |
| `/r680_nav/vio_tracking_healthy` | cuVSLAM 惯性及跟踪稳定状态 |
| `/r680_nav/localization_ready` | 原有导航定位健康状态 |

## 日志与代码

本次运行目录：`/home/orin/nav_run/nav-<时间>-<PID>/`。

```text
nav-<时间>-<PID>/
├── vio_initializer.yaml        # 源码初始化参数快照
├── vio_init.json               # 状态、失败原因、请求记录、轮式路程及各输入门控
├── cuvslam_statistics.json     # SDK 跟踪、惯性状态和时序统计
├── nav2.yaml                   # 实际控制器配置快照
└── logs/bringup.log            # 节点启动、生命周期和状态转换
```

`requested_motion: true` 只说明生成过非零请求，不证明实车发生了移动。

```text
wla_r680_navigation/
├── config/vio_initializer.yaml
├── include/wla_r680_navigation/vio_init_policy.hpp  # 纯状态机及最终请求限幅规则
├── src/vio_initializer.cpp                         # 传感器校验、状态机、指令仲裁和日志
├── src/command_guard.cpp                           # 初始化授权与正常授权的最终门控
├── launch/bringup.launch.py                        # 开关、话题切换及实际参数合并
├── scripts/r680_nav.sh                             # 命令行选项、就绪等待及失败退出
├── test/test_vio_init_policy.cpp                   # 状态机和限制回归
├── test/test_vio_init_ros.py                       # 隔离 ROS 图的故障与交接测试
└── test/probe_cuvslam_navigation.py                # 无真实输出的完整栈预览
```

cuVSLAM 的标定参数可以复用，但本版不会把上次的偏置、重力和速度状态直接当作本次初始化结果。进程重启后仍需由当前数据初始化；启动激励的成功率和真实停车距离须进一步实车验证。

## 本轮验证

- Orin Release 编译成功，4 项 CTest 通过。
- ROS_DOMAIN_ID=74 隔离测试通过：成功交接、障碍出现、IMU 断流、VO 突跳、深度无效、损坏点云、生命周期失效、超时、初始化节点退出，以及原门控兼容性。测试均关闭硬件输出；模拟成功不代表实车初始化成功。
- 原 RGB-D 与 cuVSLAM 的看门狗健康、重定位及丢失门控回归通过。
- 完整无运动输出预览：`/home/orin/nav_run/cuvslam-nav-preview-20260930-210405/`。五个被检查的 Nav2 生命周期节点均为 active，TF 链连接正常；IMU 200.60 Hz、里程计 29.70 Hz、Scan 8.67 Hz。
- 确认 `/cmd_vel_nav` 只有 `r680_vio_initializer` 发布，原限速节点及恢复行为已切至 `/r680_nav/nav_command_input`，底盘速度消息数为 0。
- 本轮实车输入预览的结果为 `failed / startup_inputs_timeout`：输入均新鲜、安全节点 active、轮速静止，但原始安全点云间歇或持续触发 `cloud_blocked`，未能连续满足启动条件。没有生成非零请求，cuVSLAM 惯性初始化也没有完成。这证明门控生效，不构成运动初始化成功证据。

launch 使用 `RewrittenYaml` 合并初始化配置及话题覆盖值，避免节点专用 YAML 覆盖匿名 launch 参数；完整预览已检查这一修正。

本功能之前的导航包基线提交：`b60a4d9`。运行级回退使用 `--rgbd`，不需要改 Git，也不会删除现有的用户配置修改。完整源码回退须先保存用户未提交工作，不能直接使用破坏性的工作区重置。
