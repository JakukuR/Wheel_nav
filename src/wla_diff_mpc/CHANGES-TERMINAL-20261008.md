# 终点停车、独立朝向制动与零速保持

日期：2026-10-08。基线提交：`7e85424`。

## 控制行为

终点附近不再用跟踪 QP 同时调整位置和朝向。新增纯 C++ `TerminalAlignment` 状态机：

```text
跟踪与接近减速 → 停车确认 → 独立朝向对齐 → 制动等待 → 零速保持
```

距离进入 0.20 m 范围（最多不超过 GoalChecker 的 XY 容差）后先发布零速。直接订阅 `/d455_slam/odom` 的车体速度；消息来源时间戳和接收时间均必须新鲜，child_frame 必须匹配控制器车体帧，速度必须有限且非显式无效。线速度不超过 0.025 m/s、角速度不超过 0.04 rad/s，连续稳定 0.35 秒后才允许朝向对齐。

朝向对齐始终令线速度为零。最大角速度仍为 0.22 rad/s，根据实际角速度估计停止角度：

```text
停止角度 = |实际角速度| × 响应时间 + 实际角速度² / (2 × 角减速度)
```

默认响应时间 0.70 s、角减速度 0.40 rad/s²，是保守初值，尚未经过本次实车测量。剩余角度预留 25% 朝向容差余量，限制角速度；预计不能在余量内停下时立即请求零速，等待实际停稳。反向纠偏也要先完成停车确认。

朝向进入 GoalChecker 的容差后保持零速，避免追逐容差内的位姿噪声。只有停稳后仍超出朝向容差才继续纠偏；停稳后位置超出 XY 容差则重新接近目标。同一目标重规划保留状态，新目标或控制器停用重置状态。速度反馈失效时零速并重新确认停车。

独立转向仍经过连续足迹碰撞检查，预测考虑残余角速度与响应时间；碰撞返回 NoValidControl。接近目标时也加入基于距离的减速。原路径跟踪 QP 和障碍制动逻辑保持使用。

## 参数与诊断

Nav2 `controller_server.ros__parameters.FollowPath` 新参数：

| 参数 | 默认值 |
|---|---:|
| final_align_velocity_topic | /d455_slam/odom |
| final_align_velocity_timeout_s | 0.20 |
| final_align_stopped_linear_velocity | 0.025 |
| final_align_stopped_angular_velocity | 0.04 |
| final_align_stop_stable_s | 0.35 |
| final_align_angular_deceleration | 0.40 |
| final_align_reaction_time | 0.70 |
| final_align_yaw_gain | 1.0 |

状态转换会打印 `MPC terminal X -> Y`；每秒打印一次 `MPC terminal phase=...`，包括距离、角度误差、实测速度、反馈新鲜度及角速度输出。零速是停车请求，停车成功由实测速度确认。

导航 YAML 需使用 `nav2_controller::StoppedGoalChecker`，旋转/平移停车阈值与上述值一致。仅使用 SimpleGoalChecker 可能在车仍转动时就报告成功。

## 验证与回退

`terminal_alignment_regression` 验证先停车、停稳时间、反馈中断、提前制动、反向等待、容差内零速、位置超限重新接近，以及带 0.4 秒指令延迟和 0.5 秒响应时间的模拟底盘收敛。原 QP、参考轨迹和障碍制动测试一起通过。

`test/check_terminal_controller.py` 在 ROS_DOMAIN_ID=74 启动实际 Nav2 controller_server、StoppedGoalChecker 与生命周期管理器，用合成里程计/TF 和延迟底盘模型验证两个新目标完成且成功时已停稳。未启动底盘节点，没有实车运动测试。

回退：revert 本次 MPC 提交并重新编译 `wla_diff_mpc`，同时按需 revert 导航参数提交。切换 YAML 不会撤销插件代码。
