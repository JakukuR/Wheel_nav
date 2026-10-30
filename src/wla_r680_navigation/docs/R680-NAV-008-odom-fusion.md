# R680-NAV-008 滑窗融合里程计（odom_fusion）

## 问题

- `degraded_odometry` 是切换式：视觉一丢切到轮速+陀螺推算，预算 2 s / 0.3 m / 0.8 rad，还要求地图已验证；用完即 Blocked，解除要求车停着等 VIO 自己恢复。车停在无特征处时形成死锁。
- `vo_watchdog` 把 VO 与轮速持续不一致判为 VO 故障；轮子打滑时方向判反。
- cuVSLAM 内部跳变直接进入 odom。

## 设计

cuVSLAM 是闭源 SDK，内部加不了轮速因子；在它外面建平面滑窗因子图（GTSAM `BatchFixedLagSmoother`，3 s 窗口，10 Hz 关键帧），占据原 continuity 槽位（cuVSLAM 发 `vio_raw_odom` 且不发 TF）。

| 项 | 内容 |
|---|---|
| 状态 | 平面位姿 x, y, yaw；陀螺零偏 b_z |
| 轮速因子 | 关键帧间轮速 v 与去偏陀螺 ωz 预积分（转角只用陀螺）；对零偏的雅可比数值求取 |
| 视觉因子 | 同一 cuVSLAM 连续段内的相对位姿，Huber 核；与轮速预测相差超过 0.15 m / 0.15 rad 视为跳变，开新段，不进 odom |
| 零偏 | 随机游走；静止时零速约束（ZUPT）使零偏可观 |
| 视觉质量分 | cuVSLAM 新发 `/r680_nav/vio_visual_quality`（观测点数、左目 8×8 网格覆盖率）；分数 >0.7 停用视觉因子，<0.4 连续 3 帧恢复 |
| 打滑分 | 轮速角速度 vs 陀螺、轮速加速度 vs IMU 加速度（去慢变均值）、轮速 vs 视觉位移；连续加权 σ×(1+10·分数)；超过 0.8 持续 2 帧判打滑，轮速平移基本不计，预测改用平滑器估计的速度；低于 0.4 持续 5 帧解除 |
| 桥接不确定度 | 无视觉时累计：随机项平方和 + 轮速比例误差（3% × 盲行距离）+ 零偏误差（0.003 rad/s × 盲行运动时间）× 距离；静止不增长 |

输出：`/d455_slam/odom` 与 odom TF 始终连续（高度、横滚、俯仰沿用 cuVSLAM）；`/r680_nav/odom_state`（GOOD / BRIDGING / LOST）；`/r680_nav/odom_epoch`（latched，连续性丢失后由视觉重新锚定时加一）。沿用 `continuous_odom_healthy`（GOOD 或 BRIDGING）与 `odom_degraded`（BRIDGING），因此 guard 在 BRIDGING 时按原降级限速 0.15 m/s、0.3 rad/s，interface_monitor 无需改动。

| 状态 | 条件 |
|---|---|
| WAITING | 首个质量合格的视觉位姿之前；透传 cuVSLAM 位姿与 TF，不报健康 |
| GOOD | 视觉因子在用 |
| BRIDGING | 无视觉，σ ≤ 0.25 m 且 ≤ 0.25 rad |
| LOST | 超出 σ 上限，或底盘输入中断；中断超过 0.25 s 视为连续性丢失，视觉恢复后 epoch+1 |

在 0.15 m/s 的桥接限速下 σ 达到 0.25 m 约需 10–20 s（参数为临时值，需实车录包拟合）。

## 其他改动

- `vo_watchdog`：`wheel_consistency_check`（融合后端时由 launch 置 false）；收到新 odom_epoch 时清空地图证据并重新全图搜索。
- launch：`odom_backend:=fusion|degraded`（默认 fusion）、`odom_fusion_params_file`。
- `r680_nav.sh`：`--no-fusion-backend` 回到旧 degraded_odometry；`--no-degraded-odom` 两者都不启动。

## 验证

- `odom_fusion`（核心，合成数据，23 项）：有视觉时零偏估计与跟踪；3 s 视觉中断桥接、输出连续、恢复不换 epoch；长时间盲行进入 LOST、视觉恢复后 epoch=1；打滑检测且位姿不被空转轮带走、滞回解除；cuVSLAM 跳 1 m 不进 odom；静止盲态 ZUPT 保持位姿；质量分与积分。
- `odom_fusion_ros`：锚定 cuVSLAM 坐标、GOOD/epoch 0/TF；运动中丢视觉进入 BRIDGING；恢复 GOOD 不换 epoch；轮速中断 LOST；恢复后 epoch=1。
- `test_no_degraded_launch.py`（未注册 ctest，手动运行）：后端选择与 watchdog `wheel_consistency_check` 路由。
- `navigation_startup_evidence`：启动脚本传出 `odom_backend` 与 `odom_fusion_params_file`。
- cuVSLAM `visual_quality`：网格覆盖率。
- `wla_r680_navigation` 全部 32 项、`wla_cuvslam_navigation` 全部 8 项通过。

## 限制

- 未实车验证；噪声、打滑阈值、σ 上限都是临时值。
- 底盘 IMU 外参仍为单位阵，时间偏移为 0；视觉与底盘 IMU 时间偏移未在线估计。
- 只融合平面运动；坡道上平面假设会带来误差。
