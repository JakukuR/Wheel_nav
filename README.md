# R680 导航与建图

本仓库是 ROS 2 Jazzy 的 `wla_r680_navigation` 包，运行在 Orin 的 `~/ros2_ws`。
导航与建图进程独立于 WLA Agent；复用 R680 底盘驱动、D455、RTAB-Map 和 Nav2。
`addons/` 归档配套 MPC、cuVSLAM 前端源码及 SDK 补丁。

本文按 2026-10-09 的 cuVSLAM 导航链路说明。**导航脚本默认控制器是 MPC，但里程计前端仍默认 RGB-D；使用当前双目＋车身 IMU 链路必须显式加 `--cuvslam`。** 导航默认关闭真实底盘输出，`--enable-motion` 才开放运动。

## 当前导航链路

```text
D455 左右红外图像 + CameraInfo + /wheel/imu/data_raw
  -> cuvslam_odometry：双目视觉 / 车身 IMU
  -> /r680_nav/vio_raw_odom：提供视觉位姿与初始化输出
  -> degraded_odometry：可信视觉锚点 + 有预算的轮速 / IMU 接管
       同时输入 /wheel/odom、车身 IMU 和健康状态
  -> /d455_slam/odom
       + TF d455_floor_odom -> r680_mapping_floor

D455 RGB + aligned depth + CameraInfo + /d455_slam/odom
  -> RTAB-Map localization：在地图数据库中重定位
  -> TF map -> d455_floor_odom
       + /d455_slam/localization_pose、/d455_slam/info

已保存的 map.yaml / map.pgm -> map_server -> /r680/d455/map
D455 aligned depth -> depth_to_points
  -> points / clearing_points -> 局部代价地图
  -> points_safety -> collision_monitor

静态全局地图 + 局部障碍 -> SmacPlanner2D -> Nav2 MPC
  -> /r680_nav/cmd_vel_controller -> goal_approach_limiter
  -> /cmd_vel_nav -> velocity_smoother -> collision_monitor
  -> /r680_nav/cmd_vel_collision_checked -> command_guard
  -> /r680_nav/chassis_cmd_vel -> 底盘驱动
```

启用自动 VIO 初始化时，初始化节点在 `goal_approach_limiter` 与 `/cmd_vel_nav` 之间接管命令，再交还导航；仍经过后续平滑、碰撞监控和最终指令门禁。

全局规划使用已保存的二维地图，RTAB-Map 数据库栅格放在 `/d455_slam/localization_map` 供定位调试，不覆盖导航地图。当前默认 `config/nav2_mpc.yaml` 的全局代价地图仅包含静态层和膨胀层，局部 D455 障碍检测保持开启。本链路不依赖 M10P。

| 数据 / 状态 | 接口 |
| --- | --- |
| 双目输入 | `/r680/d455/infra1/image_rect_raw`、`infra2/image_rect_raw` |
| RGB / 对齐深度 | `/r680/d455/color/image_raw`、`/r680/d455/aligned_depth_to_color/image_raw` |
| 车身 IMU / 轮式里程计 | `/wheel/imu/data_raw`、`/wheel/odom` |
| 连续里程计 | `/d455_slam/odom` |
| 前端惯性初始化健康 | `/r680_nav/vio_tracking_healthy` |
| 连续里程计健康 / 状态 | `/r680_nav/continuous_odom_healthy`、`/r680_nav/continuous_odom_status`；降级标志 `/r680_nav/odom_degraded` |
| 地图匹配和故障恢复健康 | `/r680_nav/vo_watchdog_healthy`、`/r680_nav/vo_watchdog_status` |
| 综合就绪 / 运动许可 | `/r680_nav/localization_ready`、`/r680_nav/mission_motion_allowed` |
| 导航目标请求 | `/r680_nav/goal_request` -> `/navigate_to_pose` |

TF 主链为 `map -> d455_floor_odom -> r680_mapping_floor -> d455_link`，相机驱动发布 optical frame。导航连续里程计节点拥有 odom -> base；RTAB-Map 拥有 map -> odom。不要同时运行另一套发布相同 TF 的定位进程。

## 依赖与构建

依赖 ROS 2 Jazzy、候选底盘工作区 `~/r680_chassis_candidate_ws`、RealSense ROS 驱动、RTAB-Map、Nav2、Eigen3、OSQP / OsqpEigen，以及 cuVSLAM v17。Orin 的现有 OSQP 依赖前缀为 `~/.local/wla_mpc_deps`；cuVSLAM 源码 / SDK 构建分别位于 `~/ros2_ws/.runtime/cuvslam_validation/{cuVSLAM,sdk_build}`。

仓库根目录是一个 ROS 包。`addons/COLCON_IGNORE` 避免嵌套包被重复发现；新部署时将配套源码复制为同级包：

```bash
cd ~/ros2_ws/src/wla_r680_navigation
mkdir -p ../wla_diff_mpc ../wla_cuvslam_navigation
cp -a addons/wla_diff_mpc/. ../wla_diff_mpc/
cp -a addons/wla_cuvslam_navigation/. ../wla_cuvslam_navigation/
```

已有独立源码仓库时，先核对本地修改，再同步 `addons/`，避免覆盖未提交工作。

**先按 [SDK 补丁说明](addons/cuvslam-sdk/README.md) 应用视觉来源 API 补丁并重建 SDK。** 本前端依赖新增的 `IsLastPoseInertialOnly()`；未修补的 SDK 不能与它混用。

```bash
source /opt/ros/jazzy/setup.bash
source ~/r680_chassis_candidate_ws/install/setup.bash
export CMAKE_PREFIX_PATH="$HOME/.local/wla_mpc_deps:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$HOME/.local/wla_mpc_deps/lib:${LD_LIBRARY_PATH:-}"
cd ~/ros2_ws
colcon build --packages-select wla_diff_mpc wla_r680_navigation \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash src/wla_cuvslam_navigation/build_frontend.sh
install -m 755 src/wla_r680_navigation/scripts/r680_nav.sh ./r680_nav.sh
install -m 755 src/wla_r680_navigation/scripts/r680_mapping.sh ./r680_mapping.sh
```

前端安装在 `.runtime/cuvslam_navigation/install`，MPC 与导航包安装在 `install`。
启动脚本按此顺序加载前缀；具体依赖见 [配套源码说明](addons/README.md) 和各包 `package.xml`。
默认 ROS 域为 73，RMW 为 CycloneDDS；DDS 配置使用 `~/Wheel_Legged_Agent/deploy/r680/cyclonedds-local.xml`。这些可通过 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、`CYCLONEDDS_URI` 覆盖。

## 启动导航

当前地图的定位、规划与命令预览：

```bash
cd ~/ros2_ws
./r680_nav.sh --map map-2026-10-08-1 --cuvslam
```

VIO 需要运动激励完成惯性初始化，单纯静止五秒不保证就绪。`--auto-vio-init` 使用受限初始化流程；不加 `--enable-motion` 时不向真实底盘执行初始化运动。由操作者确认起步区条件后启动当前运动链路：

```bash
./r680_nav.sh --map map-2026-10-08-1 \
  --cuvslam --auto-vio-init --enable-motion
```

初始化会检查传感器、静止反馈和近场障碍，并限制时间、距离及转角；参数见 `config/vio_initializer.yaml`。失败或取消后停车。启动结果保存在本次运行目录的 `vio_init.json`。

启动脚本处理受管理的定位 / 相机服务，拒绝与已有独立底盘或 RealSense 会话争抢串口和相机。初始化和启动就绪后才开放任务运动许可，最终仍由健康门控和 `command_guard` 决定是否输出。

| 参数 | 当前行为 |
| --- | --- |
| `--map NAME` | 选择地图目录；不指定时读取 `storage.yaml` 的 `active_map` |
| `--initial-pose "X Y Z R P Y"` | 提供 RTAB-Map 重定位初值，单位米 / 弧度 |
| `--cuvslam` / `--rgbd` | 双目＋车身 IMU / 原 RGB-D VO＋EKF；默认 `rgbd` |
| `--mpc` / `--mppi` | MPC / 选择 `nav2.yaml` 配置；脚本默认 MPC，具体插件以加载 YAML 为准 |
| `--scan-obstacles` | 局部标记改用派生 LaserScan，保留点云清障和碰撞监控 |
| `--test-scan-only` | MPC＋局部 Scan，选择独立静态全局地图测试配置 |
| `--test-recovery` | MPC＋局部 Scan＋有时限的全局动态障碍测试配置 |
| `--no-degraded-odom` | 禁用轮速＋IMU 接管，视觉失效时停止连续输出 |
| `--no-rviz` | 不启动 RViz，Web 网关仍保留 |

RGB-D 回退链为 RGB-D VO -> 原 EKF / 车身 IMU 整理 -> `/d455_slam/odom`。cuVSLAM 模式不重复运行该 EKF、Madgwick 或 IMU conditioner；使用车身 IMU，未启用 D455 内部 IMU。脚本必须保持运行，导航 action 才存在。`Ctrl+C` 撤销运动许可、关闭本次节点并恢复受管理服务。

## 开局地图匹配与故障恢复

cuVSLAM 提供局部里程计；机器人在旧地图中的位置由 RTAB-Map 使用 RGB-D 观测与数据库做重定位得到。`--initial-pose` 或 RViz 的 `2D Pose Estimate` 只提供搜索初值，不等于匹配成功。

cuVSLAM 导航的开局健康要求前端 / 连续里程计就绪、RTAB-Map 报告有效地图匹配事件、新鲜且合法的 `localization_pose`、可用的地图 TF 等。地图匹配事件依据 RTAB-Map 的回环、邻近或地标匹配字段；仅收到图像、地图栅格或存在 TF 不能代替匹配证据。

真正故障后，看门狗锁定运动并重启匹配到的前端进程。具备故障前可信锚点时，停止状态下用有界轮式位移提供一次重定位提示，再按近域 1 m、扩域 3 m、VPR＋RGB-D 几何验证的阶段搜索。**一次新鲜、合法、几何验证通过的地图重定位即可形成匹配证据，不要求机器人移动或在同一位置连续匹配三次。** 解除锁定仍需原有健康稳定、地图 TF / 位姿一致性、轮速及可信锚点检查。缺少可信锚点或恢复失败时保留锁定，需要操作者校正定位。

`recovery_stable_s=2.0` 是既有健康稳定时间，和连续地图匹配次数不同。参数见 `config/vo_watchdog.yaml`，实现细节见 [恢复说明](docs/changes/R680-RECOVERY-002.md)。

## 短时视觉退化

导航启用连续里程计时，前端区分实际视觉求解与纯 IMU 推算。初始化阶段保留必要的原始位姿输出；建立可信视觉后，纯 IMU 推算不能作为新的视觉锚点。总观测点数量不等于 PnP 求解成功。

迟到帧直接丢弃；协方差过大的有限视觉估计视为弱证据，也不能刷新可信锚点或接管预算。已有可信视觉 / 地图锚点、轮速与 IMU 新鲜且状态合格时，轮速＋IMU 在原有预算内维持连续输出：

| 门限 | 默认值 |
| --- | --- |
| 原始视觉输出新鲜度 | 0.15 s |
| IMU / 轮速新鲜度 | 0.10 s / 0.15 s |
| 接管时间 / 距离 / 转角 | 2.0 s / 0.30 m / 0.80 rad |
| 位置 / 朝向标准差上限 | 0.10 m / 0.15 rad |
| 降级命令线速度 / 角速度上限 | 0.15 m/s / 0.30 rad/s |

预算内收到新鲜视觉位姿，经过协方差和预测一致性检查后接回。`visual_return_frames=3` 指连续里程计内部的视觉返回确认，**不是地图重定位匹配三次，也不要求移动三次**。传感器故障、非法位姿或预算耗尽仍停车。接管参数见 `config/degraded_odometry.yaml`；长期无纹理导航尚不在当前能力范围。

## 终点位置与朝向

MPC 在位置接近终点后先确认停止，再独立完成低速朝向对齐。当前配置进入距离为 0.20 m，退出距离为 0.28 m，最大对齐角速度为 0.22 rad/s；最终由 `StoppedGoalChecker` 验收位置、朝向和停止反馈。

开始对齐后，中途停车沿用退出距离，避免小幅位置修正造成“对齐 -> 路径跟踪 -> 对齐”反复切换。日志记录 `robot_local`、`goal_local`、`plan_tf`，用于区分本地运动与地图修正。此前运行未记录完整地图变换历史，不能确认历史反向转动都由重定位修正触发。修复和隔离验证见 [2026-10-09 变更说明](docs/changes/R680-FIXES-20261009.md)。

## 地图、建图与运行产物

`config/storage.yaml` 默认保存完整运行到 `~/nav_run`，精简地图到 `~/ros2_ws/maps`，默认地图选择 `latest`：

```text
maps/map-YYYY-MM-DD-N/
  map.pgm + map.yaml        二维导航栅格
  rtabmap.db               视觉重定位数据库
  map_info.json
  semantic.geojson
  navigation.yaml         本地图的统一入口和相对路径
  manifest.json           地图完整性校验
```

导航先校验地图，再把数据库复制到本次运行目录；只写工作副本，不覆盖源地图数据库。若修改了导航栅格，按 `accept_navigation_map_edit` 的流程显式接受修改后再运行。

cuVSLAM 手柄建图预览：

```bash
./r680_mapping.sh --cuvslam --dry-run
```

由操作者开始运动初始化和手柄建图：

```bash
./r680_mapping.sh --cuvslam --auto-vio-init
```

**建图脚本与导航不同：默认允许硬件输出，`--dry-run` 才关闭。** 建图模式使用 `mapping_odom_gate` 向 RTAB-Map 提供 `/r680_nav/mapping_odom`，不启用导航的轮速＋IMU 连续接管。退出时先撤销运动许可、等待停止，在建图节点仍运行时备份数据库和二维地图；未就绪或保存失败时保留日志与工作库，不发布有效导航地图。

每次导航目录形如 `~/nav_run/nav-YYYYMMDD-HHMMSS-PID/`。排查时优先读取实际加载快照：`navigation.json`、`selected_map.json`、`nav2.yaml`、`cuvslam.yaml`、`degraded_odometry.yaml`、`cuvslam_statistics.json`、`vio_health-*.jsonl` 和 `logs/bringup.log`。源码 YAML 不等于当前进程的实际参数。

## 操作界面与诊断

RViz 的 `2D Pose Estimate` 发布 `/d455_slam/initialpose`，用于重定位初值；`2D Goal Pose` 发布 `/r680_nav/goal_request`，由目标桥接节点调用导航 action。MPC 预测轨迹位于 `/FollowPath/predicted_path`，全局路径位于 `/plan`。

Web 网关默认随 bringup 启动。当前 Orin 地址为 `http://172.31.18.156:8080`，用于地图、相机、位姿、路径、导航和重定位操作；可通过 `start_web:=false`、`web_host`、`web_port` 设置。用户地图插画须与地图 ID、栅格校验值及尺寸一致；TF 超时隐藏机器人贴图。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/Wheel_Legged_Agent/deploy/r680/cyclonedds-local.xml"
ros2 topic echo /r680_nav/continuous_odom_status --once
ros2 topic echo /r680_nav/vo_watchdog_status --once
ros2 topic echo /r680_nav/localization_ready --once
ros2 lifecycle get /controller_server
```

检查降级是否是预算耗尽、输入断流或入口锚点被拒绝，应查看连续里程计的具体原因和各输入年龄。TF error / costmap 超时可能是里程计停止输出的后续结果，不能直接作为最初原因。

本包还保留自主探索编排 `mapping_mission` 和家具语义收集功能；其启动、地图归档与任务配置位于 `scripts/`、`config/` 和 `docs/`。本轮修复通过软件规则和隔离 ROS 图测试，尚未完成真实退化场景及终点转向的实车运动验收。
