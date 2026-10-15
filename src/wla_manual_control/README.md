# 独立底盘、手柄和导航速度仲裁

系统服务 `r680-manual-base.service` 使用 ROS 域 73，独占底盘串口。
`command_mux.py` 是 `/r680_base/cmd_vel` 的唯一发布者。导航复用
`/wheel/odom` 和 `/wheel/imu/data_raw`，不另启底盘驱动。

## 手柄与 NAV 无需模式切换

启动后两路都可以提交速度，无需调用 `set_automatic`，也没有 NAV 的 MANUAL 锁存：

1. 有效、非零的手柄输入优先执行，两路速度不相加。
2. 手柄零输入不挡导航。松杆、断连或手动输入过期后，先清除交接前的导航命令，收到交接后的新鲜 NAV 命令即执行，不取消导航目标。
3. 手柄不可用时，NAV 仍可输出；NAV 不可用时，手柄仍可输出。
4. 没有有效输入时输出零。NAV 命令超过 150 ms 即过期；这只是速度命令有效期，不是视觉位姿门控。
5. 急停锁存停止两路。明确复位后只接受新收到的命令，不回放复位前的缓存。

NAV 输入为 `/r680_nav/chassis_cmd_vel`。Nav2 的定位、避障、命令限幅以及
`--enable-motion` 的导航侧输出开关仍由导航链路管理，底盘不再重复要求 AUTO 授权。
原 `nav_manual_coexist` 构造参数仅为调用兼容保留，不能恢复旧 NAV 模式门控；mux 不再声明此配置开关。

手柄包负责原始摇杆连续归中 0.3 秒和连接就绪；只有 ACTIVE/SPIN/STANDBY
状态及该状态之后的新鲜速度才参与仲裁。等待归中、断连和超时使手动输入失效，
不影响有效 NAV。现有限幅为前进 1.00 m/s、后退 0.30 m/s、转向 1.50 rad/s。

## 兼容接口与状态

`/r680_base/control_state` 中，NAV 的 `mode_required=false` 明确表示无模式门控。
旧字段 `mode=AUTO` 仅为现有上游消费者兼容保留，不表示有一次授权动作。
`output_source=MANUAL/NAV/NONE` 表示实际速度来源；`armed`/`manual_ready` 仅表示手柄是否就绪。

NAV 下 `/r680_base/set_automatic`（SetBool）仍接受旧客户端调用：true 是兼容确认，
不做模式切换；false 清空旧 NAV 输入并取消当前导航目标，不禁止之后新到达的命令。
需要持续停止两路时使用 `/r680_base/set_estop`，不能再用 set_automatic(false) 当作锁存停止。

急停服务 true、手柄 ESTOP 或非法手动数值均锁存停止并请求取消 Nav2。
false 仅在手柄急停输入已解除、轮速新鲜且停稳、手动输入中立或不可用时复位。
断开或重连手柄不能解除急停。锁存属于当前进程的软件状态，不跨进程重启持久化。

## 独立跟随任务

PERSON/OBJECT 为独立、互斥的任务来源，仍使用原有来源选择、任务身份、租约、
时间戳和激活协议；本次没有将跟随候选变成无条件运动命令。
选择 PERSON/OBJECT 后 NAV 不输出；有效手柄接管会撤销跟随任务。
切回 NAV 会清空旧输入，随后新鲜 NAV 可直接输出，无需再切 AUTO。

## 验证与加载

纯策略：`python3 -m pytest -q test/test_policy.py test/test_nav_manual_shared.py test/test_object_policy.py test/test_object_control_handoff.py`。
隔离 ROS：`ROS_DOMAIN_ID=198 python3 test/test_relative_mux_ros.py`，只启动仲裁器和合成输入，不启动串口驱动。
另有导航会话、初始化退出零速回归，位于 `wla_r680_navigation/test`。
这些检查不证明实车运动或制动已验收。

已运行的 Python 进程不会重新导入修改；加载前需停止导航任务、确认输出零速，
再重启常驻底盘服务。通过 `mode_required=false` 确认新版本已加载。
