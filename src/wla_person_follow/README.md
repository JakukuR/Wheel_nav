# 目标跟随与靠近（wla_person_follow）

一个包同时实现三种目标任务：跟人、走到人面前、走到某个物体前；另有两个探索用的动作原语（点目标、原地转向）。

- **mode 0（人物跟随）**：选定一个人，保持约 1.2 m 跟随，一直跟到操作者停止，或租约 / 权限被撤销。
- **mode 1（人物靠近）**：走到人面前，人的表面距机器人参考原点约 0.95 m 时到达，然后结束。
- **mode 2（物体靠近）**：靠近非人物体，表面距车身外廓 0.15 m 时到达；不可达时停在当前局部地图里最近的可达点。
- **mode 3（点目标）**：`target_id` 为机器人系下的 `dx,dy`（≤ 3 m），开始时按轮速里程计固定；沿滚动地图上的带间隙代价网格路线走，限速 0.8 m/s，距目标 0.15 m 内到达。供探索闲逛使用。
- **mode 4（原地转向）**：`target_id` 为相对转角（弧度，左正，≤ 2π+0.5），以 0.35–0.5 rad/s 原地转，转到或转过目标角即到达。供探索环顾使用。

mode 3/4 没有跟踪目标，但仍要求感知门开着且检测新鲜（其他人仍是动态障碍）。

后端启动后只等待命令，不会自己选目标。

## 快速开始

在车上（`orin@<车 IP>`）依次运行，每条命令一个终端：

```bash
cd ~/ros2_ws
./r680_camera.sh run
./r680_perception.sh run
./r680_person_follow.sh run transport:=cpp enable_motion:=true
./r680_follow_trial.sh follow --mode 0 --continuous --allow-motion
```

- 第 1 行：D455 相机和安装 TF。
- 第 2 行：检测器，输出 `/r680/perception/relative_objects`。
- 第 3 行：跟随后端。省略 `enable_motion` 时只预览，不出速度。
- 第 4 行：选目标 ID 并下发任务，Ctrl+C 停止。

从自己的电脑操作时，第 4 步要在车上执行，例如 `ssh -t orin@<车 IP> 'cd ~/ros2_ws && ./r680_follow_trial.sh follow --mode 0 --continuous --allow-motion'`。

在后台启动后端（断开 SSH 后不退出）：

```bash
ssh -n orin@<车 IP> 'cd ~/ros2_ws && L=.runtime/follow-backend-$(date +%Y%m%d-%H%M%S).log && setsid -f ./r680_person_follow.sh run transport:=cpp enable_motion:=true > $L 2>&1 < /dev/null'
```

停止后端：

```bash
pkill -INT -f follow.launch.py
```

**注意**

- **只能运行一个跟随后端。** Wheel_Legged_Agent 的 `tools/run_all_actions.py` 会自己再起一套跟随后端和 `high_action_target_egress`，两者都向 `/r680/person_follow/command_candidate` 发布。和这里的后端同时运行会抢输出，表现为跟随明显变慢。
- **改了本包的 Python 代码，要先构建再重启后端**：`colcon build --packages-select wla_person_follow`。install 里放的是文件副本，不是指向源码的软链接，不构建就重启仍会运行旧代码。

## 链路

```text
D455 → nav_yolo_detector（YOLOE 分割 + BoT-SORT + OSNet）
     → /r680/perception/relative_objects
     → follow_transport（C++：收发、时效、租约、权限、输出门禁）
     → hybrid_worker（Python：FollowEngine = 跟踪 + 身份恢复 + 滚动深度地图 + 采样 MPC）
     → /r680/person_follow/command_candidate → command_mux → 底盘
```

- C++ 层负责最终的时效、撤销、租约和权限检查。计算放在独立的 Python 进程里。
- `transport:=python` 保留原来的 `SimulationFollow` 适配器作为兼容入口，用的是同一套算法。
- 拆分方式和接口见 [HYBRID_TRANSPORT.md](HYBRID_TRANSPORT.md)。

**检测行格式**

- 每一行的字段严格固定为 `{detected, label, id, yaw, distance}`，`TaskSession` 和 Agent 的 `relative_adapter` 都按这个集合精确检查。
- 额外信息放在消息顶层，例如 `image_edge_ids`：贴左右画面边缘的检测 ID 列表。

**命令接口**

- 服务 `/r680/person_follow/command`，以及 Agent 门面 `/wla_agent/person_command`。
- 消息类型都是 `PersonCommand`，支持 start、heartbeat、stop、status。
- 租约 0.1–2 s。每个新任务使用新的 `request_id`。`mode` 只接受字符串 `0`、`1`、`2`。

## 人物跟随的行为

### 跟踪状态

由 `follow_session.FollowTracker` 维护：

| 状态 | 含义 | 运动 |
| --- | --- | --- |
| `tracking` | 当前帧看到了目标 | 按 5 s 轨迹（0.4 s 采样，三阶 Bézier）跟随，保持 1.2 m |
| `coasting` | 短暂没看到，不超过 2 s | 按 Kalman 预测继续走 |
| `holding` | 检测相对预测跳得太远，被拒绝 | 只丢掉这一次检测；mode 0/1 按最后可信轨迹继续走（`holding_last_trusted`），mode 2 停车 |
| `search_last_seen` | 丢失超过 2 s | 以 0.4 m/s 开往人消失的位置 |
| `search_heading` | 到达消失点 | 原地扫视找人 |

**提前结束"开往消失点"**

只看规划器信号，不看离消失点的距离：

- 出现深度覆盖恢复（车头已经贴到障碍）：立即算作到达。
- 没有可行路径或正在倒车脱困：累计 2 s 后算作到达。

这样原地转向、绕行、短暂停顿都不会被误判为到达。- 既没移动也没转动持续 3 s：算作到达，避免停在到达范围外一直不开始扫视。

状态字段 `search_arrived_by` 记录到达方式：`distance`、`coverage`、`blocked` 或 `stalled`。

**扫视方式**

- 相对人的行走方向，先转约 90°（1.5 rad）到人离开的一侧（`search_side`），然后沿同一方向继续转，直到找到人。
- 转速 0.6 rad/s，转满一圈约 10.5 s，车后方也能看到。
- 相关参数：`search_exit_turn`、`search_turn_rate`。

### 身份恢复（ReID）

- ReID 只在目标丢失时用来重新绑定，正常跟随完全靠几何。
- 丢失期间，对画面里所有可见的人计算外观特征，取离预测位置最近的 4 个，不再限制位置。
- 选相似度最高的人，并要求同时满足：
  - 相似度 ≥ 0.55（`reid_search_similarity`）；画面里只有一个人时 ≥ 0.45（`reid_alone_similarity`）；
  - 比第二名高出至少 0.08；
  - 同一个 ID 连续确认 3 次，相邻两次间隔不超过 2 s。
- 特征不可用的帧不计数，也不清零已有确认。这类帧包括遮挡、贴边、低置信度、未检测到，原因见 `feature_diagnostics`。
- 初次登记模板的配对阈值是 0.80，后续更新模板的阈值是 0.90，都与恢复阈值分开。
- `reid_similarity`、`reid_search_radius_m`、`reid_search_confirm_frames` 仍保留在配置里供校验，但恢复逻辑已不再使用。

### 人物位置

- 人物取躯干带深度的 20% 分位（`perception_geometry.person_point`：框宽中间 40%、框高 20–60%）。这样人半出画面时，不会读到身后的墙面。
- 贴左右画面边缘的人（`image_edge_ids`）只采用方位，距离用预测值。状态字段 `edge_range_observations` 计数这类观测。
- 目标自身的点云按同帧分割掩码从障碍地图里屏蔽，已确认的历史占据也会清理。其他人仍作为动态障碍。

### 避障

规划器是在滚动深度地图上的采样 MPC（`sim_core/hybrid_controller.py`）。车身按半径 0.19 m 的圆处理，障碍余量 0.10 m。

| 情况 | 行为 | `planner_reason` |
| --- | --- | --- |
| 直线被挡 | `DetourPlanner` 选左右边缘或栅格路线作为绕行航点；看不到出口时原地转向侧面观察（look） | `tracker_follow` |
| 走直线已经安全 | 沿已走过的轨迹取最远的、直线可通过的点（视线捷径），不再绕回人的足迹 | `tracker_follow` |
| 连续 ≥3 个周期没有向前的可行候选 | 沿自己走过的轨迹倒车，每个卡住阶段最多 0.6 m | `backing_out` |
| 倒车方向和车尾偏差 > 0.5 rad | 先原地转正（≥ 0.3 rad/s），再以 ≥ 0.12 m/s 倒车；倒车 2 s 仍未移动 5 cm 就放弃，改为原地转 | `backing_out` |
| 深度画面覆盖太少（车头贴着桌子等，进入相机最小量程） | 不前进，只倒车或原地转向人 | `depth_coverage_recovery` |
| 全部候选都不安全 | 停车 | `no_feasible_candidate` |

车身已经压进障碍余量时，仍然允许原地转（这时 `clear < 0`，圆形车身原地转不改变距离），倒车也允许，只要倒车路径不比当前位置更靠近障碍。11-02 之前这种情况下转向会被禁止，曾经停住 10 s 以上。

### 主要参数

在 `sim_core/common.py` 的 `Tuning` 中：

| 项 | 值 | 说明 |
| --- | --- | --- |
| `desired_gap` / `target_stop_gap` | 1.20 / 1.16 m | 车中心到人中心的距离 |
| `max_v` | 1.2 m/s | 跟随最高速度 |
| `search_speed` | 0.4 m/s | 开往消失点的速度 |
| `chassis_tau_w` / `chassis_tau_v` | 0.50 / 0.18 s | 底盘响应模型，增益 1.0，按 J02 实测拟合 |
| `backup_speed` / `backup_min_speed` / `backup_max_m` | 0.2 / 0.12 m/s，0.6 m | 倒车脱困 |
| `backup_stall_s` | 2 s | 倒车停滞多久放弃（倒车时边退边把车尾转向走过的路，不再原地先转正） |
| 人物滤波 | 测量噪声 0.05 m，加速度 0.5 m/s² | `follow_session.TARGET_MEAS_STD_M` / `TARGET_ACCEL_STD_MPS2` |
| `max_sensor_age_s` | 0.40 s | 观测时效；启动检查上限 0.45 s，C++、`sim_node`、`TaskSession` 三处一致 |

## 人物靠近与物体靠近

**mode 1**

- 朝人当前所在位置减速靠近，不沿人走过的路线绕。

**mode 2**

- 绑定检测类别和完整 ID，用同帧分割深度取物体的可见表面。
- 在滚动地图的八邻接图上找路，禁止斜穿障碍角，整条路径交给同一个 MPC。
- 外廓按 0.48 m 方形计算。地图范围 12 m，栅格 0.1 m，只在已知空间里求最近可达点。
- 保守外廓和栅格会让实际停车距离大于 0.15 m。
- 物体不可见或没有可用表面时等待；不使用人物 ReID，也不执行人物搜索。

**到达判定（mode 1 和 2）**

- 终点稳定至少 0.6 s。
- 至少三个不同的新鲜深度帧，时间跨度 ≥ 0.2 s。
- 同时满足：跟踪有效、方向误差在范围内、MPC 可行、轮速接近 0。
- 距离容差：mode 1 为 0.04 m，mode 2 为 0.065 m。
- 到达后状态锁存，不会因目标移动或重发同一个 `request_id` 而恢复。

**非活动任务**

- 停止、到达或租约结束后，只维护地图（`planner_activity=idle_map_only`），不做预测和 MPC，输出为零。

## 日志与诊断

| 内容 | 位置 |
| --- | --- |
| 后端终端输出 | `.runtime/follow-backend-*.log`（后台启动时） |
| 任务客户端输出 | `.runtime/follow-trials/follow-*.log` |
| 逐帧状态（`/r680/person_follow/status`） | `.runtime/follow-diagnostics/follow-status-*.jsonl*`，会滚动覆盖，要分析的数据请及时复制出来 |
| cProfile | 创建 `.runtime/follow-profile` 后再启动后端；删除该文件即关闭 |

分析时常用的状态字段：

- `geometry_binding.state`、`reason`
- `planner_candidates`：`feasible` / `forward` / `forward_total` / `forward_static`，`detour.side`，`detour.looking`
- `linear_mps` / `angular_rps`，`measured_motion.wheel.twist`
- `target_observation.surface_distance_m` / `bearing_rad`
- `identity_supervision.feature_diagnostics`
- `search_arrived_by`、`search_blocked_s`

确认运行中的进程加载了哪个版本，要看状态里的 `transport_backend` 和后端日志；源码和构建成功都不能证明这一点。

## 测试

在隔离域 198 运行，不连底盘：

```bash
cd ~/ros2_ws/src/wla_person_follow
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
RUN_SIM_ROS=1 ROS_DOMAIN_ID=198 python3 -m pytest -q test
```

检测器测试（在 `src/wla_r680_navigation` 下运行，需要 boxmot 虚拟环境）：

```bash
ROS_DOMAIN_ID=198 ~/ros2_ws/.venv-boxmot/bin/python -m pytest -q test/test_nav_detector_ros.py test/test_boxmot_runtime.py test/test_perception_geometry.py
```

- 2026-11-02 共 163 项通过。
- `test_follow_chain_ros` 在整套高负载运行时偶发 `control_tick` 断言失败，单独运行能通过。

## 验证边界

- **2026-10-30**：J02 实车验证过观测时效、观测链路和转向修正。
- **2026-11-02**：实车跑过这些改动：1.2 m 跟随距离、`person_point`、holding 继续行驶、搜索提速、90° 扫视、ReID 选最佳匹配、消失点不可达时提前搜索、贴障原地转和倒车修正。
- 17:08 重启后的那次运行，操作者反馈效果不错，但还没有逐帧分析。
- 车身按圆形处理；如果实际轮廓是长方形，贴障原地转时边角可能擦到障碍，仍需留意。
- 软件门禁不等于物理制动验收。

## 其他文档

- [HYBRID_TRANSPORT.md](HYBRID_TRANSPORT.md)：C++ 收发层与 Python 计算进程。
- [FULL_CHAIN_TESTS.md](FULL_CHAIN_TESTS.md)：全链路测试。
- [PY_CPP_ARCHITECTURE.md](PY_CPP_ARCHITECTURE.md)、[LATENCY_OPTIMIZATION.md](LATENCY_OPTIMIZATION.md)、[REID_ENROLLMENT_FIX.md](REID_ENROLLMENT_FIX.md)。
- [三模式操作](../../FOLLOW_TRIAL.md)。
