# ros2_ws 当前代码框架（2026-11-09）

基准：Orin `192.168.9.129`，`/home/orin/ros2_ws`，外层 Git `22230b1` 加读取时已有的两处工作树改动。
系统为 Ubuntu 24.04 / aarch64、ROS 2 Jazzy、Python 3.12.3。21 个 ROS 包分布在 19 个一级源码目录。
启动脚本默认 ROS 域 73、CycloneDDS 本机回环发现；直接 source 环境不会自动设置域和 RMW。

阅读入口：[启动与常用参数](ros2_ws_operations.md) · [完整源码目录树](ros2_ws_directory_tree.md) ·
[启动参数完整索引](ros2_ws_launch_reference.md) · [本轮核对记录](ros2_ws_audit_20261109.md)。

## 目录和职责

```text
/home/orin/ros2_ws/
├── r680_*.sh / person_follow_trial.py   根目录脚本是实际启动入口
├── src/                               包源码、launch、config、接口、测试和网页
│   ├── wla_manual_control/             底盘常驻 launch、C++ 速度仲裁与策略参考
│   ├── gamepad_contorl/                ROS 包名 gamepad_control；目录拼写保留
│   ├── wla_r680_navigation/            D455、感知、Web、RTAB-Map、Nav2 编排与地图存储
│   ├── wla_detection_bridge/           共享检测 JSON 协议与校验，不运行模型
│   ├── wla_follow_transport/           C++ ROS 收发、轮速积分、IPC 和最终输出门
│   ├── wla_person_follow/              任务、跟踪、身份恢复、局部地图与采样 MPC
│   ├── wla_follow_local/               pybind11 几何、地图、跟踪与参考曲线扩展
│   ├── wla_explore_wander/             无预存地图的闲逛/好奇决策，用跟随后端执行
│   ├── wla_terrain_analysis/           可选坡面/台阶点云、地形里程计
│   ├── wla_diff_mpc/                   Nav2 的差速 MPC 插件（与跟随采样 MPC 不同）
│   ├── wla_cuvslam_navigation/         cuVSLAM 导航前端
│   ├── wla_cuvslam_validation/         cuVSLAM 独立输入/SDK 验证工具
│   ├── wla_openvins_navigation/        官方 OpenVINS 的启动、配置检查和坐标转换
│   ├── open_vins/                     ov_core / ov_init / ov_msckf 官方估计器源码
│   ├── explore/                       ROS 包 explore_lite；Nav2 前沿探索
│   ├── explore_lite_msgs/             前沿探索状态接口
│   ├── wla_behavior_interfaces/        Agent/BT/跟随公共 srv/action
│   ├── wla_behavior_bt/                语义导航行为树执行器
│   └── wla_behavior_bridge/            Agent API、导航任务网关、任务速度门
├── config/agent_semantic/              地图/语义策略与请求样例
├── docs/                              当前总览和历史验证资料
├── maps/                              精简导航地图；不等于运行日志
├── models/                            本机 TensorRT 引擎/跟踪库等运行资源
├── build/ install/ log/                构建、部署、构建测试日志
├── .venv-boxmot/ .venv-perception/      本机 Python 环境，不能整体提交
└── .runtime/                          锁、跟随日志/bag、独立 SDK/驱动构建等
```

`src/wla_r680_navigation/scripts/r680_{nav,mapping,teleop}.sh` 是转发器，默认调用根目录脚本；
可用 `WLA_NAV_WORKSPACE` 指定完整工作区。**不要把这些转发器复制覆盖根目录脚本**，否则会触发递归保护而无法启动。
`build_frontend.sh` 只构建 cuVSLAM 前端，不再构建导航包或覆盖根入口。

## 常驻底盘与最终速度链

`r680-manual-base.service` → `start_manual_base.sh` → `manual_base.launch.py`：

```text
gamepad_teleop ── /r680_manual/cmd_vel ───────────┐
follow_transport ─ /r680/person_follow/command_candidate ─┤
command_guard / task_velocity_gate ─ /r680_nav/chassis_cmd_vel ─┤
                                              ↓
                                  command_mux（C++，50 Hz）
                                              ↓ /r680_base/cmd_vel
                                  wheeltec_robot_node → 底盘
                                              ↓
                               /wheel/odom、/wheel/imu/data_raw
```

常驻服务的驱动 remap 到 `/r680_base/cmd_vel`；该部署中 mux 是此话题的预期唯一发布者。
手柄非零输入 > PERSON 非零输入 > NAV 非零输入；零速释放本路优先级，过期的单一路输入只退出本路，其他有效输入仍可输出。
手柄松杆后等交接之后的新自动命令；拨杆本身不终止跟随任务。
`velocity_passthrough=true` 时无需切 AUTO/PERSON。旧服务保留兼容用途，持续停止使用锁存急停或任务 stop。
当前 mux 默认限幅前进 1.0 m/s、后退 0.3 m/s、转向 1.5 rad/s；跟随算法的 1.2 m/s 上限还会经过这个最终限幅。

## 相机、共享感知与观察窗口

```text
r680_camera.sh → d455_camera.launch.py → 唯一 D455 驱动 + 安装 TF
                        └─ emitter=alternate 时加 emitter_frame_splitter
r680_perception.sh run → nav_perception_bringup
  ├─ nav_yolo_detector（.venv-boxmot：YOLOE + BoT-SORT + OSNet）
  │   ├─ /r680/perception/detections       检测框/类别/ID，采集时间不改写
  │   ├─ /r680/perception/relative_objects 机器人系 yaw/distance
  │   ├─ /r680/perception/target_mask      绑定任务/ID/深度帧的目标掩码
  │   ├─ /r680/person_follow/reid_features 同帧复用外观特征
  │   └─ /r680_nav/perception/{status,timing}
  ├─ nav_perception_adapter → Agent 白名单反馈
  ├─ agent_person_api → /wla_agent/person_command
  └─ agent_observation_window（可选）← 有界本机 IPC 的同帧观察
```

模型权重/词表/文本编码器在 `~/Wheel_Legged_Agent`；本工作区可使用有校验 sidecar 的 TensorRT 引擎和有界 C++ BoT-SORT 库。
YOLOE、OSNet、BoT-SORT 各有自己的运行后端，不能只凭文件存在断言加速已加载；查看 `perception/timing` 的 backend 字段。
检测默认空闲目标频率 10 Hz，跟随请求活跃时 20 Hz（`nav_yolo_detector` 的 `--idle-hz` / `--hz`）；相机仍为 30 Hz。
跟随人物使用几何跟踪；ReID 用于初始登记/身份丢失恢复，其他人始终参与动态避障。
Kalo 网页由 foxglove_bridge + JPEG 编码 + 静态 Web/WS 转发组成，默认 `http://192.168.9.129:18081/`。

## 跟随：收发层和数值层是两个独立选择

```text
follow.launch.py：transport=cpp（默认）
  follow_transport（ROS 节点 /person_follow）
    ↔ 两个 16 MiB 共享槽 + UNIX SOCK_SEQPACKET
  hybrid_worker（Python，无 ROS Node/Executor）
    ├─ 复用 sim_node.FollowEngine / TaskSession / FollowTracker
    └─ sim_runtime.process_step → SimulationEngine
         ├─ 深度投影/目标过滤 → 滚动地图
         ├─ 目标和其他行人跟踪 → 参考曲线 / DetourPlanner
         └─ HybridFollower 采样 MPC；可选 mode 0 NeuPAN 外部进程
  返回结果 → C++ 再核对 request/代次/源帧/期限/轮速/控制权 → 候选速度
```

`transport=python` 为兼容 ROS 适配器，共用算法但使用原计算进程结构。
`numeric_backend=python` 是默认数值实现；`cpp` 替换滚动地图、目标/路人跟踪和参考曲线，采样 MPC 仍在 Python。
它不是把全部跟随算法改为 C++；`_wla_sim_geometry` 在默认数值后端下也使用。
C++ 数值扩展缺失时报错，不静默回退；混合收发只支持 `output_mode=mux` 和系统时间。

`PersonCommand` 的 `mode`：0 人物跟随，1 人物靠近，2 物体靠近，3 相对点目标，4 相对转角。
后两者供闲逛动作使用；交互试验客户端 `r680_follow_trial.sh` 只提供 0/1/2。
任务账本保证具名请求、有限租约、幂等、终态不复活；新任务 ID 与目标必须明确绑定。
`wla_explore_wander` 只挑选和串联这些动作，不直接发底盘速度，也不使用 `explore_lite`。

核心阅读位置：`task_session.py`（任务账本）、`follow_session.py`（目标/身份恢复）、
`sim_node.py`（共享执行壳/到达与输出）、`sim_runtime.py`（地图和规划编排）、
`sim_core/hybrid_controller.py`（采样控制）、`sim_core/detour.py`（绕行/栅格路线）。
本轮读取时 `follow_session.py` 已有未提交的“贴边距离新息”修改，文档按当前工作树描述，未提交该源码。

## 导航与建图：同一编排，三种里程计

根 `r680_nav.sh` / `r680_mapping.sh` 负责参数校验、模式锁、相机复用、服务暂停/恢复、地图选择、快照和退出清理。
`bringup.launch.py` 负责节点图。根导航入口默认 `rgbd + MPC`；直接 launch 的默认参数文件为 `nav2.yaml`，不要混淆两种入口的默认值。

| 分支 | 里程计路径 | 使用说明 |
| --- | --- | --- |
| RGB-D（默认） | RGB-D VO → 车身 IMU 整理/Madgwick + EKF → `/d455_slam/odom` | 原路线 |
| cuVSLAM 导航 | 双目 + 车身 IMU → `/r680_nav/vio_raw_odom` → `odom_fusion` → `/d455_slam/odom` | 默认滑窗融合后端；`--no-fusion-backend` 换旧 `degraded_odometry` |
| cuVSLAM 禁接管 | cuVSLAM 直接 `/d455_slam/odom` + odom TF | `--no-degraded-odom` 同时关闭两种连续后端 |
| cuVSLAM 建图 | cuVSLAM odom → `mapping_odom_gate` → `/r680_nav/mapping_odom` → RTAB-Map | 不启用导航连续融合；写图前检查前端状态 |
| OpenVINS | 官方 `ov_msckf/run_subscribe_msckf` → `odometry_adapter` → 机器人 odom | 成套配置检查、Nav2 分阶段激活；D455 IMU 需驱动修复 overlay |

cuVSLAM 定位时默认 `odom_fusion` 用 GTSAM 的 3 s 滑窗，正常 GOOD、无视觉时 BRIDGING、超不确定度或输入条件不满足时 LOST。
旧 `degraded_odometry` 的 2 s / 0.30 m / 0.80 rad 预算只描述旧后端，不能写成滑窗融合的默认预算。
`vio_initializer` 的初始化/LOST 恢复、`vo_watchdog` 的地图匹配、连续里程计健康各负不同职责。
cuVSLAM 看门狗不再因短暂健康丢失重启前端；可选初始化/LOST 恢复器有独立的有界恢复逻辑。

```text
里程计 + RGB-D/红外深度 → RTAB-Map → map→odom 修正 / 地图匹配证据
保存的 map.yaml → map_server → /r680/d455/map
navigation_map_mode → /r680_nav/navigation_map（定位地图或无图占位栅格）
深度 → depth_to_points（或 terrain_analysis）→ 点云/清障/碰撞输入
Nav2 全局/局部代价地图 → SmacPlanner2D → wla_diff_mpc
→ goal_approach_limiter → 可选 vio_initializer 命令仲裁
→ velocity_smoother → collision_monitor → command_guard
→ 普通导航：/r680_nav/chassis_cmd_vel → 常驻 mux
→ Agent：task_guard_input → task_velocity_gate → 配置输出 → mux
```

`motion_ready` 是里程计与输入链就绪；`localization_ready` 才表示旧地图匹配健康。
无图模式拒绝网页 map 目标，可接受符合协议的 odom/机器人系目标；不能把能出 odom 当作已定位到旧地图。
TF 主链：`map → d455_floor_odom → r680_mapping_floor → d455_link → 相机 optical frames`。
跟随的局部轮速参考不依赖 RTAB-Map 的 map TF，不要为跟随额外启动导航。

## Agent、探索与存储

`wla_behavior_interfaces` 定义 PersonCommand、AgentSemanticCommand、BehaviorTask、TaskLease 等公共协议；
`wla_behavior_bridge` 薄桥依赖工作区外 `Wheel_Legged_Agent/src/wla_sim` 的语义/策略实现。
`behavior_executor` 执行 BT，`navigation_task_gateway` 将语义目标解析为 Nav2 请求并维护租约；
`task_velocity_gate` 对任务链输出再门控。`--agent-control` 禁用普通目标桥，不能同时让两套入口派发导航。

`explore_lite` 是基于 Nav2 代价地图的前沿探索；`mapping_mission` 负责准备→探索→返航→停稳→保存，
与 `wla_explore_wander` 的“跟随局部地图 + mode 3/4”是两条不同路线。

`storage.py` 负责地图选择、索引/manifest 校验和地图发布，`archive.py` 负责归档；
`prepare_mapping_run`、`wait_mapping_ready`、`save_manual_map` 是建图脚本配套工具。
导航复制 `rtabmap.db` 到 `~/nav_run/nav-*` 后使用工作副本；建图成功保存到 `maps/map-*`。
源码 YAML、运行目录快照、install 副本、进程实际参数是四个层次；核对运行值请用参数查询和日志。

## 构建和维护边界

外部依赖：`~/r680_chassis_candidate_ws` 的驱动、`~/Wheel_Legged_Agent` 的 Agent/模型、
OSQP/OsqpEigen `~/.local/wla_mpc_deps`、GTSAM、cuVSLAM SDK 和 D455 IMU 修复 overlay。
详细命令见操作手册。内嵌 `.git` 保留原历史，统一在外层工作区提交，不能为消除 dirty 状态重复提交内仓。
这份框架说明来自源码/配置静态核对和只读环境检查，不代表已启动/验收本轮的视觉、规划或实车运动。
