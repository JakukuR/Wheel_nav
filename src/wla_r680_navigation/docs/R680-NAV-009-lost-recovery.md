# R680-NAV-009 里程计 LOST 后的恢复动作

## 背景

odom_fusion（R680-NAV-008）在无视觉时桥接，σ 超限或底盘输入中断才进入 LOST。LOST 时 guard 停止导航输出（`continuous_odom_healthy=false`）。停在无特征处的车如果不动，视觉不会自己回来。参照 z-localization 的 SUSPECT → LOST：先等，再找特征，最后重置前端。

车静止时 ZUPT 使不确定度不增长，所以停在白墙前只会一直 BRIDGING（可运动），不会进入 LOST；LOST 主要来自盲行过远、打滑或底盘输入中断。

## 设计

`vio_initializer` 已拥有 `/cmd_vel_nav` 与 guard 的有界 bootstrap 通道（平滑器 → 碰撞监控 → guard 限幅），恢复动作复用它，不新增速度源。状态机在 `vo_recovery_policy.hpp`（无 ROS，可单测）：

1. 初始化成功后，`/r680_nav/odom_state == LOST`（新鲜）进入 `waiting_for_vision`。
2. 等 `recovery_wait_s`（3 s）；车轮已停、近场无障碍、轮速/IMU/深度/安全点云新鲜、基座授权与安全链有效时开始原地扫：+60° → −60° → 回到 0°，0.3 rad/s，不平移（相机看不到后方）。每段 8 s 超时。
3. 扫完仍 LOST：重启 cuVSLAM 一次（只向本 ROS domain 的 `cuvslam_odometry` 发 SIGTERM，由 launch respawn），等 25 s（含静态惯性初始化）。
4. 失败或被阻断（近场障碍、授权丢失）进入 30 s 冷却；最多 3 次，之后 `exhausted`，等操作员（手柄始终可接管）。
5. 任何时候 odom_state 不再是 LOST（视觉回来或桥接恢复），立即停止并复位。

恢复期间 `/r680_nav/vio_init_state` 为 `recovering`：guard 的 bootstrap 通道接受它（线速度被压为 0，角速度按初始化上限限幅）；正常导航要求 `succeeded`，因此被挡住。状态在 `/r680_nav/vo_recovery_state`。

## 授权

`navigation_motion_session --cuvslam-init` 进入 `navigation_enabled` 后继续发布 `vio_init_base_authorized=true`，即同一次显式授权覆盖初始化和恢复扫动。原本"只初始化一次"的约定由此扩展为"初始化 + 有界恢复"。手柄优先级与急停不变；会话结束时授权撤销。

## 启用

`recovery_enabled` 仅在 cuVSLAM 且 `odom_backend:=fusion` 时由 launch 打开（需要 odom_state）。`recovery_angular_speed` 不得超过初始化角速度。

## 验证

- `vo_recovery_policy`：等待 → ±60° 扫 → 回正 → 一次重启；恢复即复位；车在动或未授权不启动；近场阻断中止；次数用尽后等待；再次丢失重新开始；限幅。
- `test_vio_init_ros.py`（domain 74 手动运行）新增 `recovery`：初始化成功后 LOST 3 s，出现 `recovering`，guard 预览只有原地角速度且有界，恢复后回到 `succeeded`，正常导航速度恢复。
- `cuvslam_initialization_authority`：基座授权与撤销流程不变。

## 限制

- 未实车验证；重启 cuVSLAM 后静态初始化需要车保持静止。
- 扫动航向来自底盘里程计（轮速/陀螺）。
