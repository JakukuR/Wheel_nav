# R680 独立导航建图包

`wla_r680_navigation` 位于手柄工作区 `~/ros2_ws`，复用本机仿真的 RGB-D VO、
RTAB-Map、Nav2、速度平滑、碰撞监控和最终命令门禁。它独立于 WLA Agent。
底盘驱动和硬件速度输出默认关闭。

## 实车数据链

```text
D455 RGB + aligned depth + CameraInfo
  ├─> RTAB RGB-D VO -> /r680_nav/vo_odom
  │                     └─> robot_localization EKF
  │                          └─> /d455_slam/odom
  │                              + TF d455_floor_odom -> r680_mapping_floor
  ├─> RTAB-Map -> map -> d455_floor_odom + /r680/d455/map
  └─> C++ depth_to_points -> /r680_nav/d455/points
                               ├─> Nav2 local obstacle layer
                               └─> collision_monitor

Nav2 /cmd_vel_nav -> velocity_smoother -> collision_monitor -> C++ command_guard
                                                        ├─> 安全预览
                                                        └─> 底盘原始入口（默认关闭）
```

本包不启动也不依赖 M10P。近场动态障碍来自 D455 对齐深度，保持与仿真
`/home_sim/d455/points` 动态层相同的数据职责。

## VO / VIO 状态

D455 驱动的 `unite_imu_method=2` 只把不同频率的 gyro/accel 线性插值为同一条
`sensor_msgs/Imu`，它不是相机内部完成的 VIO，也不提供融合姿态。正常情况下，本包使用
Madgwick 整理 IMU，再由 `robot_localization` 把 RGB-D VO 与 IMU gyro Z 松耦合，形成
兼容现有 `/d455_slam/odom` 接口的 VIO。

2026-09-18 实机检查表明：D455 视频设备的 librealsense 序列号为 `260922306083`，
USB/HID 描述符序列号为 `254343063587`；即使补齐 `/dev/hidraw0` 的 `video` 组权限，
驱动仍报告 `No HID info provided, IMU is disabled`，没有发布 gyro、accel 或统一 IMU。
因此 `use_d455_imu` 默认必须保持 `false`，当前可运行链为纯 RGB-D VO。修复 HID 枚举并
完成六面标定前，不能宣称 VIO 已启用。EKF 已按缺少 IMU 时可继续使用 VO 的方式配置。

## 已核实接口

- RGB：`/r680/d455/color/image_raw`，约 30 Hz。
- 对齐深度：`/r680/d455/aligned_depth_to_color/image_raw`，约 30 Hz。
- VO：`/r680_nav/vo_odom`。
- EKF 输出：`/d455_slam/odom`。
- 地图：`/r680/d455/map`。
- D455 障碍点：`/r680_nav/d455/points`。
- 底盘驱动执行入口：本包重映射为 `/r680_nav/chassis_cmd_vel`。

足迹为前 `0.30 m`、后 `0.25 m`、左右各 `0.24 m`，padding `0.02 m`。其外接半径约 `0.412 m`，所以实车局部/全局膨胀分别使用 `0.45/0.50 m`；`0.20 m` 会关闭 MPPI/Smac 快速碰撞检查。实车初始速度
限制为 `0.30 m/s`、`0.50 rad/s`。硬件输出默认关闭。

## 构建

```bash
source ~/.bashrc
source ~/r680_chassis_candidate_ws/install/setup.bash
cd ~/ros2_ws
colcon build --packages-select wla_r680_navigation --symlink-install
source install/setup.bash
```

## 安全静止联调

现有定位服务和本包不能同时拥有 `/d455_slam/*` 与对应 TF。先停止旧服务，再复用已运行的
D455 图像启动新链；下面的命令不会启动底盘驱动，也不会向底盘输出速度：

```bash
systemctl --user stop r680-d455-localization-stack.service

ros2 launch wla_r680_navigation bringup.launch.py \
  mode:=mapping \
  start_d455:=false \
  start_chassis:=false \
  start_nav2:=true \
  start_state_estimation:=true \
  use_d455_imu:=false \
  publish_mount_tf:=true \
  enable_hardware_output:=false
```

检查：

```bash
ros2 topic hz /r680_nav/vo_odom
ros2 topic hz /d455_slam/odom
ros2 topic hz /r680_nav/d455/points
ros2 topic echo /r680_nav/localization_ready --once
ros2 lifecycle get /controller_server
ros2 param get /r680_command_guard hardware_output_enabled
```

退出新链后恢复旧定位服务：

```bash
systemctl --user restart r680-d455-localization-stack.service
```


## 车身 IMU 简单融合测试

D455 HID 尚不可用时，可设置 `use_chassis_imu:=true`，将底盘 MPU6050 的
`/wheel/imu/data_raw` 先经启动静止陀螺零偏估计和 Madgwick，再接入同一个 3D EKF。
当前融合三轴姿态、三轴角速度和三轴加速度；IMU 被假定在 `r680_mapping_floor` 车体中心且
与车体轴对齐。不要同时启用 `use_d455_imu`。测试时必须同时设置 `start_chassis:=true`，并在
启动后的前 5 秒保持车辆完全静止。该模式用于桥接短时视觉退化，持续退化仍需轮速观测约束。

```bash
ros2 launch wla_r680_navigation bringup.launch.py \
  mode:=mapping start_d455:=false start_chassis:=true start_nav2:=false \
  start_state_estimation:=true use_d455_imu:=false use_chassis_imu:=true \
  publish_mount_tf:=true enable_hardware_output:=false
```

## 一键手柄建图

```bash
cd ~/ros2_ws
./r680_mapping.sh
```

脚本使用 `deadman_enabled=false`，但会在开放运动授权前检查手柄必须持续处于零位。
有图形会话时自动打开预配置 RViz。建图完成后在脚本终端按 `Ctrl+C`，脚本会先停车，
在建图节点仍存活时保存二维栅格和 RTAB-Map 数据库，然后关闭节点并恢复原定位服务。

## 手柄遥控建图与归档

手动模式让手柄发布到 `/cmd_vel_nav`，继续经过速度平滑、D455 碰撞监控和最终命令门禁。
完成后停止手柄与运动授权，再执行：

```bash
ros2 run wla_r680_navigation save_manual_map --run-dir "$RUN_DIR"
```

该命令等待车辆稳定停止，调用 RTAB-Map backup，并原子生成与自主任务一致的
`map_archive/`。启动前约 5 秒必须保持车辆静止，以完成车身 IMU 陀螺零偏估计。

## 地图与完整运行产物

默认路径由 `config/storage.yaml` 管理：

```yaml
run_root: ~/nav_run
maps_root: ~/ros2_ws/maps
map_prefix: map
active_map: latest
```

每次建图的日志、数据库、参数快照和完整归档保存在
`~/nav_run/run-YYYYMMDD-HHMMSS/`。保存成功后，系统原子生成精简导航地图：

```text
~/ros2_ws/maps/map-YYYY-MM-DD-N/
├── map.pgm
├── map.yaml
├── rtabmap.db          # RTAB-Map 开局/漂移重定位的只读源库
├── map_info.json
├── semantic.geojson
├── navigation.yaml     # 本地图的统一导航启动入口
└── manifest.json
```

`semantic.geojson` 初始为空，供后续房间、物体、命名点和区域规则扩展；它不计入
不可变几何文件的校验值。修改存储路径后重新构建或显式传入自定义 `--storage-config`。
一键建图也可以直接使用另一份配置：

`navigation.yaml` 记录二维栅格、视觉定位数据库、地图元数据、语义层和默认初始位姿的
相对路径。导航脚本先读取并校验这份文件，因此地图目录可以整体复制到另一台机器，不依赖
原建图运行目录。控制器、底盘和相机参数仍由 ROS 包统一维护，避免每张地图各存一份后失配。

```bash
./r680_mapping.sh --storage-config /path/to/storage.yaml
```

导航侧读取 `active_map`（默认 `latest`），也可指定地图名称：

```bash
ros2 run wla_r680_navigation resolve_navigation_map
ros2 run wla_r680_navigation resolve_navigation_map --map map-2026-09-20-1
```

命令输出 `map_yaml` 和 `database_path`。二维 Nav2 使用 `map.yaml`，RTAB-Map
localization 使用同目录的 `rtabmap.db`，从而保证几何地图与视觉重定位数据库属于同一版本。

## 一键导航

默认读取 `storage.yaml` 的 `active_map`（当前为 `latest`），启动 RGB-D VO、车身 IMU
松耦合、RTAB-Map localization、完整 Nav2、D455 障碍层、速度平滑、碰撞监控和 RViz：

```bash
cd ~/ros2_ws
./r680_nav.sh
```

默认是安全预览模式，能定位、规划和检查安全链，但 `command_guard` 不向底盘发布真实速度。
选择地图和显式开放实车运动：

```bash
./r680_nav.sh --map map-2026-09-20-1 --enable-motion
```

脚本必须持续运行，`/navigate_to_pose` action 才存在。RViz 中两个箭头工具职责不同：

- `2D Pose Estimate` 只向 `/d455_slam/initialpose` 发布开局重定位初值，不会导航；
- `2D Goal Pose` 向 `/r680_nav/goal_request` 发布目标，由 `navigation_goal_bridge` 转成
  `/navigate_to_pose` action，并把红色目标箭头保持在地图上。下发前确认右侧
  `Navigation 2` 面板显示 Nav2 为 active。

RViz 默认显示青色全局路径 `/plan`、黄色局部路径 `/local_plan`、全局/局部代价地图、
绿色实际 footprint、橙色碰撞预测区域和按深度着色的 D455 点云。局部代价地图中高代价值
区域即 D455 障碍层与膨胀层的合成结果。

已知出生点时可以给 RTAB-Map 六维初始位姿；未知时在 RViz 使用 `2D Pose Estimate`
进行校正，然后再用 `2D Goal Pose` 下发目标：

```bash
./r680_nav.sh --map map-2026-09-20-1 \
  --initial-pose "0 0 0 0 0 0" --enable-motion
```

脚本启动前校验导航地图 manifest，启动后等待以下条件：

- D455 RGB、深度、相机内参、融合里程计和障碍点云连续健康；
- `/r680/d455/map` 已产生有效二维栅格；
- `map -> r680_mapping_floor` TF 可查询；
- controller、planner、BT navigator、velocity smoother 和 collision monitor 均为 active。

这些条件证明数据链和 Nav2 已就绪，不证明当前相机图像已经与旧地图正确匹配。开放实车
运动前仍要在 RViz 核对机器人位姿。按 `Ctrl+C` 会撤销运动许可、发送零速、关闭本次节点，
再恢复原定位服务。脚本先校验 `maps` 中不可变地图，再把 `rtabmap.db` 复制到本次运行目录；
localization 只写工作副本，不会污染地图包。日志和工作数据库写入
`run_root/nav-YYYYMMDD-HHMMSS-PID/`。

## 自主探索任务（沿用仿真逻辑）

`explore_lite` 使用与仿真相同的改版：相邻 10 cm 目标合并、到点后 360° 扫描、失败目标
冷却重试、局部倒车恢复。`mapping_mission` 继续负责稳定前沿复查、图优化锚定的返航、停止确认、
RTAB-Map 备份和原子地图归档。实车话题和 frame 已改为本包接口。

创建一次全新的任务目录：

```bash
RUN_DIR=$(ros2 run wla_r680_navigation prepare_mapping_run)
CONFIG_DIR=$(ros2 pkg prefix wla_r680_navigation)/share/wla_r680_navigation/config
```

启动 bringup 后，在第二个终端启动任务编排：

```bash
ros2 run wla_r680_navigation mapping_mission \
  --output "$RUN_DIR/exploration.json" \
  --config-dir "$CONFIG_DIR"
```

任务归档写入 `$RUN_DIR/map_archive/`，包含 `map.pgm`、`map.yaml`、完整栅格 JSON、
经完整性检查的 `rtabmap.db`、参数快照、任务结果和 SHA-256 manifest。当前默认硬件输出为
false，因此这两条命令可验证启动和接口，但不会让车运动；开放实车运动前还要完成有人值守的
急停、制动距离和 D455 盲区验收。

## 后续开放底盘

只有完成静止联调、D455 深度盲区检查、制动距离和急停验证后，才启动候选底盘驱动并把
`enable_hardware_output` 改为 `true`。原厂 gamepad 与 Nav2 不能同时直接占用原始
`/cmd_vel`；实车导航必须经过本包的 collision monitor 和 command guard。

## Web 导航与建图网关

`bringup.launch.py` 默认启动轻量 Web 网关，建图和导航脚本无需增加参数。浏览器可显示二维
栅格地图、机器人实时位姿、全局/局部路径、即时/确认障碍点、D455 彩色画面、速度和导航
状态；在地图上左键拖拽可发送导航目标或 `/d455_slam/initialpose`，并可取消导航、清理代价地图。

同一局域网直接访问（以当前 Orin 地址为例）：

```text
http://172.31.18.156:8080
```

不开放局域网端口时，可启动仅绑定回环地址的网关，并从本机建立 SSH 隧道：

```bash
# Orin：启动时覆盖监听地址
ros2 launch wla_r680_navigation bringup.launch.py web_host:=127.0.0.1

# 本机：保留此 SSH 会话，然后访问 http://127.0.0.1:8080
ssh -L 8080:127.0.0.1:8080 orin@172.31.18.156
```

可用 `start_web:=false` 关闭，或用 `web_port:=18080` 修改端口。该页面覆盖日常二维导航、
建图和调试视图；完整三维点云、TF 树和插件面板仍使用 RViz/Foxglove。
