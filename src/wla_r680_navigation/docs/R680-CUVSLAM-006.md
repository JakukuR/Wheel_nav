# R680-CUVSLAM-006：开机初始化与全图重定位

日期：2026-10-27

## 问题

- 2026-10-27 静止实测：车静止 16 分钟，cuVSLAM 一直 `gravity_unavailable`。10-15/16 的几十次运行静止约 7 s 即 `tracking_inertial_ready`。
- 原因：`8bcf1de` 把 `initialization_keyframe_period_s` 从 0.25 改成 0.0。SDK 重力初始化需要 ≥20 个关键帧、跨度 ≥2 s，车静止时 SDK 自己几乎不建关键帧（R680-CUVSLAM-005：自动 1 个，采样 28 个）。
- 自动初始化要求 20 s 内同时完成地图定位，否则判失败并锁定；不在地图里的位置必然失败。
- 启动器 120 s 内未就绪就整套退出。
- RTAB-Map 开机以数据库里的“上次定位”为起点；每次运行用源库副本，这个点永远是建图结束时的位置，与车实际位置无关。按空间的近邻搜索围着这个点找。
- 开机只要一次地图匹配就放行；全图搜索时外观相似的地方可能认错，同一静止视角重复匹配不是独立证据。

## 改动

### 初始化（`vio_initializer`，`vio_init_policy.hpp`）

1. 恢复 `initialization_keyframe_period_s: 0.25`（`config/cuvslam.yaml`）。
2. 顺序：静止 0.8 s → 再静止等待静态初始化，最多 `static_init_wait_s`（10 s）→ 前进约 0.15 m（0.15 m/s）→ 原地左转约 30°（`turn_angle_rad: 0.52`，0.4 rad/s）→ 转回起始朝向 → 停稳 → 等惯性就绪。
   - 先等静态初始化，是为了转动时前端已在跟踪，转动的几个视角能计入重定位证据。
   - 每段有时间上限；总运动不超过 `motion_duration_s`（6 s）。路径预算 0.25 m、半径 0.25 m 不变；偏角预算放宽到 0.70 rad（转角 + 0.1 rad 余量，cuVSLAM 上限 0.75）。
   - 前方 0.65 m 空旷检查只约束前进段；原地转动在车体范围内，仍由碰撞监视约束。
   - 不倒车。实车输出仍需 `--enable-motion`、底盘授权、最终 guard 的初始化限幅（0.20 m/s、0.50 rad/s）。
3. 成功只看惯性初始化（状态 `waiting_inertial` → `succeeded`），不再要求同时定位；地图准入仍由 `localization_ready` 关闭导航输出。删除 `mapping_or_localization_timeout`。

### 启动器（`r680_nav.sh`）

- `--cuvslam` 默认开启自动初始化；`--no-auto-vio-init` 关闭。`--openvins` 仍需显式 `--auto-vio-init`。
- 120 s 期限只约束导航栈本身（生命周期、地图、TF、输入）。栈就绪后运动保持锁定，等待惯性初始化与重定位，不再超时退出；每 10 s 打印前端、初始化与 `重定位=` 状态。
- 因不再超时，`StartupEvidence` 在关键节点退出（rtabmap、vo_watchdog、interface_monitor、command_guard、vio_initializer、lifecycle_manager）时立即失败退出。

### 全图重定位（`vo_watchdog`，`view_consistency.hpp`）

- `startup_global_search`：首次准入前把 RTAB-Map `RGBD/ProximityBySpace` 设为 false，只用全图 BoW 检索（VPR）+ PnP 验证（地图库 `Vis/EstimationType=1`，≥20 内点），不受过期起点影响；准入后恢复原值。
- `startup_view_consistency`：首次准入要求两次匹配来自不同视角（odom 中位移 ≥0.10 m 或转角 ≥0.25 rad），且推出的 map→odom 修正一致（≤0.30 m、≤0.17 rad）。即两次地图位姿之差必须等于车自身里程计测得的运动。认错的地方通常换个视角就对不上，更不会与里程计一致。
- 只计入 RTAB-Map 同一时刻有回环/近邻匹配的帧；原有的位姿/TF 一致、前端跟踪、新鲜度检查不变。
- 操作者在网页/RViz 给出初始位姿后：恢复近邻搜索，附近 1.0 m、0.6 rad 内一次匹配即可准入。
- 状态发布在 `/r680_nav/relocalization_status`：`startup_global proximity_by_space=off views=N views_agree=false`。
- 故障后的分级恢复（1 m → 3 m → VPR）不变。

## 验证

- 单元：`vio_init_policy`（三段动作、仅惯性成功、静态等待、7 类锁定故障、超时）、`view_consistency`（静止重复、转动/前进一致、混淆、阈值、时间窗、乱序、重置）、`map_localization_evidence`（admit 只阻止新准入不撤销）、`navigation_startup_evidence`（关键节点退出）。
- ROS 合成：`startup_view_consistency`（静止重复不放行、混淆视角不放行、一致的第二视角放行、操作者位姿单次匹配放行）。
- 未实车验证：静态初始化恢复后的实际耗时、三段动作实车轨迹、真实地图上的多视角准入率。实车测试需在地图覆盖区域内、相机 `--infra` 启动。

## 限制

- 车不在地图覆盖范围内时无法重定位，会停在“等待重定位”；需在当前位置重新建图。
- 三段动作约提供 3–4 次 RTAB-Map 更新（1 Hz），不保证两次都匹配；不足时手推/手柄转动或在网页给初始位姿。
- 光照变化导致 BoW 召回不足时，后续可接入 GPU 全局描述子（NetVLAD/MixVPR）取 top-K，再对地图库 3D 词做 PnP，结果作为 initialpose 提示，仍经上述一致性检查。
