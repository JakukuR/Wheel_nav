# 独立底盘、手柄和自主速度选择

系统服务 `r680-manual-base.service` 使用 ROS 域 73，独占底盘串口。导航以 `start_chassis:=false` 复用 `/wheel/odom` 与 `/wheel/imu/data_raw`。手柄和速度选择器不依赖 VIO、地图或 Nav2。

`command_mux.py` 是 `/r680_base/cmd_vel` 的唯一发布者。手动输入 `/r680_manual/cmd_vel` 优先，并锁存到 MANUAL；放开摇杆不会恢复自主运动。开机、手柄失联、急停或输入过期都输出零，重连后必须归中约 0.3 秒才能重新接受手动输入。现有手柄按键、档位、限幅保持不变：当前默认前进上限 1.00 m/s、后退 0.30 m/s、转向 1.50 rad/s，档位和手柄侧加速度限制仍生效。

自主输入只有一个被选中的来源，启动默认为 NAV：

| 来源 | 输入 | 放行条件 |
| --- | --- | --- |
| NAV | `/r680_nav/chassis_cmd_vel`，Twist | 原 Nav2 安全链后输出；新鲜输入；显式 AUTO |
| PERSON | `/r680/person_follow/command_candidate`，String JSON | 新鲜启用状态、匹配进程和请求、允许实体输出、有效时间戳；显式 AUTO |

`/r680_base/set_person_source`（`std_srvs/srv/SetBool`）以 `true` 选择 PERSON、`false` 选择 NAV。实际切换要求手柄连接且归中、轮速反馈新鲜且车已停；成功后清空旧速度、回到 MANUAL，并请求取消 Nav2 目标。PERSON 被选中期间，NAV 输入不会被转发。重复选择同一来源是幂等操作。

选择来源不会自动授权运动。操作员仍需显式调用 `/r680_base/set_automatic`，`data: true`；此时必须已通过归中等待、车已停、所选来源有新鲜的零速度。PERSON 还要求已启用一个任务，`candidate_only=false`。AUTO 只接受在授权之后新产生的速度。`data: false` 随时回到 MANUAL。

人物候选业务由跟随节点提供，速度包格式：

```json
{"schema":"wla-person-candidate/v1","follow_session":"node-process-uuid","request_id":"task-uuid","stamp_ns":100000000000,"linear":0.12,"angular":0.0}
```

选择器要求候选不超过 150 ms，有限数值且绝对值不超过 0.20 m/s、0.45 rad/s；还必须匹配 200 ms 内 `/r680/person_follow/status` 的 `follow_session` 和 `request_id`。状态字段 `enabled`、`candidate_only`、`motion_permitted` 和 `fault` 决定放行。原 `/r680/person_follow/cmd_vel_candidate` Twist 仅供预览，选择器不订阅它。

正常任务停止、完成、租约到期或短时目标不可见时输出零；新鲜非故障状态下保留 AUTO，下一条显式人物任务可继续执行。状态超时、明确故障、手柄接管或断开时回到 MANUAL，并递增 `person_revoke_generation`；被撤销请求不能凭旧包恢复。来源切回 NAV 同样撤销人物任务。`/r680_base/control_state` 同时提供 `control_session`、`automatic_source`、`source_generation`、`person_revoke_generation` 和当前模式，供人物节点识别撤权与选择器重启。

人物任务启动接口只启动人物执行，不替操作员调用来源选择或 AUTO。检测进程启动也不授权运动；人物任务和底盘 AUTO 授权分别控制。联调保持人物 `enable_motion=false`，只看相对目标与预览候选；部署源代码或构建包不会自动重启已运行的底盘服务。

纯策略测试：`python3 test/test_policy.py`。隔离 DDS 测试必须在 `ROS_DOMAIN_ID=198` 下执行 `python3 test/test_relative_mux_ros.py`；只启动选择器和测试发布者，不启动串口驱动。隔离测试不替代真实手柄失联、制动距离、相机失效与人物靠近的现场验收。

测试维护：策略测试已接入 CMake/CTest；`test_relative_mux_ros.py` 统一覆盖 NAV/PERSON 仲裁、NAV 与手柄输入超时、急停恢复、热插拔和当前限速，替代旧版 `test_ros_mux.py`。`../gamepad_contorl/test/probe_gamepad_hotplug.py` 保留用于 ROS_DOMAIN_ID=174 下的合成输入流回归，检查真实读取线程、EOF、重连和急停锁存。这些测试不会随底盘服务启动，也不安装到运行目录。

手柄未接入时使用 Linux inotify 等待设备路径、udev 符号链接或权限变化事件，不再每秒尝试连接；启动时只尝试一次已存在的设备，拔出后清空输入并继续等待事件。事件监听失败时保持零输出并记录错误。0.3 秒归中等待适用于开机、重连及急停解除后的重新使能；它不是插拔后的固定倒计时。
