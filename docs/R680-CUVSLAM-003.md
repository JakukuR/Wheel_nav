# cuVSLAM 建图接入

2026-10-08。Orin 源码：`/home/orin/ros2_ws/src/wla_r680_navigation`。使用 ROS 2 建图链路，不修改 Agent。

## 启动

先关闭独立底盘、手柄和上一轮导航/建图，避免串口或 D455 被重复打开。相机朝向有纹理的家具、门框或墙角。

关闭真实底盘输出，验证启动链路：

```bash
cd /home/orin/ros2_ws
./r680_mapping.sh --cuvslam --auto-vio-init --dry-run
```

此命令不发送底盘指令；静止时可能无法完成惯性初始化。可以手推验证，但不要另外启动一个底盘驱动抢占串口。

确认起步区空旷、人在车旁且可随时急停后，启动真实建图：

```bash
cd /home/orin/ros2_ws
./r680_mapping.sh --cuvslam --auto-vio-init
```

**这条命令允许自动初始化运动及后续手柄控制。** 启动自动前进/小幅转向的请求上限为 0.06 m/s、0.20 rad/s，保持原初始化距离、偏角和超时限制。失败不自动重试。

VIO 稳定后先停稳，再等待第一张地图、`map -> r680_mapping_floor` TF 与安全节点就绪，最后启动手柄。手柄死人开关保持 false；默认上限仍为前进 1.00 m/s、倒车 0.70 m/s、转向 0.50 rad/s，起步前必须通过摇杆零位检查。

家具语义及出生点标注默认保留。出生点为 VIO 初始化后首次建立有效地图位姿时的位置；自动初始化阶段的临时 VO 位姿不进入地图，因此标记位置可能与通电时的位置相差这段初始化位移。

可选参数：

```bash
./r680_mapping.sh --cuvslam --auto-vio-init --no-rviz --no-semantics
./r680_mapping.sh --cuvslam --auto-vio-init --storage-config /绝对路径/storage.yaml
```

有本机图形会话时自动打开 RViz；纯 SSH 默认使用 Web：`http://172.31.18.156:8080`。`--no-semantics` 关闭家具及出生点采集，保留地图保存和空语义侧文件。

不启用自动运动，可用 `--cuvslam --dry-run` 手推完成初始化；VIO 就绪前不开放手柄。退出当前运行后，用原前端回退：

```bash
./r680_mapping.sh --rgbd
```

未指定前端时仍使用原 RGB-D VO + EKF。

## 数据链路

```text
D455 左右红外图像 + 原始车身 IMU /wheel/imu/data_raw
    → cuVSLAM
    → /d455_slam/odom、odom -> 车体 TF
    ├─ 自动初始化节点：读取临时 VO 与轮式反馈，执行受限激励
    └─ mapping_odom_gate：仅放行稳定惯性里程计，自动模式还要求已停稳
         → /r680_nav/mapping_odom
         + D455 RGB/Depth/CameraInfo
         → RTAB-Map（增量建图，不加载旧地图、不等待旧地图匹配）
         → /r680/d455/map、map -> odom TF、rtabmap.db

手柄 → /r680_nav/nav_command_input（自动初始化模式）
     → vio_initializer 仲裁 → /cmd_vel_nav
     → velocity_smoother → collision_monitor → command_guard
     → /r680_nav/chassis_cmd_vel → 底盘驱动
```

cuVSLAM 路线不运行旧 RGB-D VO、EKF 和车身 IMU conditioner/Madgwick；RTAB-Map 不重复融合 IMU，只消费稳定里程计和 RGB-D。

## 停止与保存

建图结束时，在启动终端按 **Ctrl+C**。脚本取消初始化、撤销运动授权并停止手柄；建图节点暂时保留，等待新鲜轮速反馈确认停稳，再调用 RTAB-Map backup、校验数据库并原子发布地图，最后关闭本次节点。

初始化未通过、建图从未就绪时，只保留日志和工作数据库，不向 `maps/` 发布无效导航地图。保存失败时保留运行目录并报告失败，不伪造成功。

VIO 在建图过程中失效或断流时，建图里程计门控会锁定，同时关闭手柄运动门控，避免把重新初始化后的坐标直接接入旧地图。退出可保存已完成的部分；继续建图需要重新启动，本版不自动拼接两次 VIO 会话。

默认存储策略：`/home/orin/ros2_ws/src/wla_r680_navigation/config/storage.yaml`。`run_root` 和 `maps_root` 支持自定义。
自定义文件中的相对路径按该文件所在目录解析；本次运行保存原文件 `source_storage.yaml` 和转换为绝对路径的实际 `storage.yaml`，避免退出保存时路径偏移。

```text
/home/orin/nav_run/run-<时间>/
├── run.json                    # 前端、IMU、数据库、路径、是否真实输出
├── cuvslam.yaml                # 前端配置快照
├── vio_initializer.yaml        # 初始化配置快照
├── cuvslam_statistics.json     # SDK 跟踪与惯性统计
├── vio_init.json               # 初始化状态、失败原因及门控
├── mapping_readiness.json      # 地图、TF、VIO、生命周期就绪结果
├── semantic.geojson            # 家具与出生点（启用语义时）
├── logs/                       # 建图、手柄、RViz、运动授权日志
├── rtabmap.db                  # 当前工作数据库
├── map_archive/                # 完整归档及实际配置快照
└── navigation_map_result.json  # maps 下的最终发布目录

/home/orin/ros2_ws/maps/map-YYYY-MM-DD-N/
├── map.pgm / map.yaml          # 二维导航地图
├── rtabmap.db                  # 导航重定位所需数据库
├── map_info.json               # 分辨率、原点、来源及语义文件信息
├── navigation.yaml            # 导航启动文件索引
├── semantic.geojson            # 语义侧文件
└── manifest.json               # 地图文件校验值
```

## 代码与诊断

新增 `src/mapping_odom_gate.cpp`，修改建图脚本、里程计健康门控及运行元数据。`scripts/wait_mapping_ready` 统一检查建图就绪。`/r680_nav/mapping_odom_ready` 与 `/r680_nav/mapping_odom_status` 用于诊断；`/r680_nav/vio_init_cancel` 用于取消初始化并锁定，不是恢复授权。

单个前向 D455 仍不能证明侧后方盲区安全；初始化位移限制依赖里程计测量，并非真实停车距离保证。IMU 外参、噪声及时间偏移仍沿用先前试验配置，实车初始化成功率与地图精度需要进一步验证。

## 验证记录

- Release 编译通过；状态机、存储/地图发布、运行元数据及相对路径测试通过。
- 隔离 ROS_DOMAIN_ID=74 验证了：初始化前/运动中的位姿不写图，惯性丢失或断流后写图与运动门控关闭且锁定，取消与无效位姿不放行，就绪检查等待地图、TF、安全节点及 VIO，失败状态准确记录。
- 原初始化测试加入取消分支后通过；原 RGB-D 与 cuVSLAM 看门狗回归通过。
- 实际 cuVSLAM 建图入口 `--dry-run`：`/home/orin/ros2_ws/.runtime/mapping_cuvslam_validation/20261008-111529/`。IMU 200.03 Hz、里程计 29.69 Hz；近场安全云触发障碍门控，45 秒后锁定。惯性尚未就绪，因此 `/r680_nav/mapping_odom`、建图地图和底盘速度消息均为 0。退出保留工作库及日志，没有发布无效地图。
- 原 RGB-D 前端实际输入建图、RTAB backup 和地图归档：`/home/orin/ros2_ws/.runtime/mapping_cuvslam_validation/20261008-111726/`。IMU 200.01 Hz、里程计 29.23 Hz，底盘速度消息为 0；测试地图成功保存为该测试目录下的 `maps/map-2026-10-08-1`，包含地图、数据库和导航索引，未写入用户正式 `maps/`。

本轮没有执行电机运动，也没有验证真实 cuVSLAM 惯性初始化成功或移动建图精度。当前实际设备检查未找到 `/dev/input/js0`；真实手柄建图前需要接上手柄。

接入前基线为 `2a302d4`，运行级回退使用 `--rgbd`。原有 Nav2、RViz、Web 未提交修改保留，不纳入本次提交。
