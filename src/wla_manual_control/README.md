# 独立底盘、手柄和自主速度选择

系统服务 `r680-manual-base.service` 使用 ROS 域 73，独占底盘串口。导航以 `start_chassis:=false` 复用 `/wheel/odom` 与 `/wheel/imu/data_raw`。手柄和速度选择器不依赖 VIO、地图或 Nav2。

`command_mux.py` 是 `/r680_base/cmd_vel` 的唯一发布者，启动为 MANUAL、零输出。手柄包负责连接、原始摇杆连续归中 0.3 秒及手动就绪判断；等待归中时发布 ARMING 和零速度，只有 ACTIVE/SPIN/STANDBY 表示已就绪。仲裁要求状态与速度输入新鲜，并只接受就绪状态之后收到的手动输入。默认 `nav_manual_coexist=true`：已显式授权的 AUTO/NAV 会话允许手柄与导航共同提交命令。手柄有有效非零输入时执行手柄；摇杆归中、手柄断开或输入过期后，先输出零，再等待交接后新到达的导航命令恢复。手柄空闲零消息不屏蔽导航，两路速度不相加，不取消导航目标，也不撤销 NAV 授权。PERSON/OBJECT 仍保留手柄接管后锁存 MANUAL、撤销跟随任务的规则。

手柄缺席、断开、状态超时或等待归中只使手动输入失效，不撤销已显式授权的 AUTO。MANUAL 下发生这些情况则输出零；AUTO 下继续按所选 NAV/PERSON 的有效性决定输出。插入手柄本身不接管。手动限速保持为前进 1.00 m/s、后退 0.30 m/s、转向 1.50 rad/s，档位和手柄侧加速度限制仍生效。

自主输入只有一个被选中的来源，启动默认为 NAV：

| 来源 | 输入 | 放行条件 |
| --- | --- | --- |
| NAV | `/r680_nav/chassis_cmd_vel`，Twist | 原 Nav2 安全链后输出；新鲜输入；显式 AUTO；可与手柄共享 |
| PERSON | `/r680/person_follow/command_candidate`，String JSON | 新鲜启用状态、匹配进程和请求、允许实体输出、有效时间戳；显式 AUTO |

`/r680_base/set_person_source`（`std_srvs/srv/SetBool`）以 `true` 选择 PERSON、`false` 选择 NAV。实际切换要求无急停锁存、没有有效的非零手动接管输入、轮速反馈新鲜且车已停；不要求手柄存在。成功后清空旧速度、回到 MANUAL，并请求取消 Nav2 目标。PERSON 被选中期间，NAV 输入不会被转发。重复选择同一来源是幂等操作。

选择来源不会自动授权运动。受信任的操作端或自主运行管理端仍需显式调用 `/r680_base/set_automatic`，`data: true`；此时必须无急停锁存、没有有效的非零手动接管输入、车已停且所选来源有新鲜零速度。PERSON 还要求已启用一个任务，`candidate_only=false`。AUTO 只接受在授权之后新产生的速度。`data: false` 随时回到 MANUAL。无手柄时按相同流程授权，不需要伪造手柄状态或速度；本包不在开机时自动授权，也不替代 NAV 上游定位/避障或 PERSON 上游任务租约/碰撞检查。

`/r680_base/set_estop`（`std_srvs/srv/SetBool`）提供独立于手柄的软件停止接口：`true` 锁存停止、清除旧 NAV 速度、退回 MANUAL、请求取消 Nav2，并撤销正在运行的人物任务。手柄 ESTOP 或非法手动数据也触发同一全局停止。断开、超时、重新连接或重新发送 AUTO 不能清除锁存。`false` 仅在已知手柄急停输入已解除、轮速新鲜且停稳、手动指令为零或不可用时复位；复位后仍为 MANUAL，AUTO 必须重新授权。手柄 A 键解除本地急停后，也需显式复位该全局锁存。无手柄触发的软件停止可在停稳后直接复位。此锁存属于本进程的软件控制，不能代替独立硬件急停；进程重启总是回到未授权 MANUAL。

人物候选业务由跟随节点提供，速度包格式：

```json
{"schema":"wla-person-candidate/v1","follow_session":"node-process-uuid","request_id":"task-uuid","stamp_ns":100000000000,"linear":0.12,"angular":0.0}
```

选择器要求候选不超过 150 ms，有限数值且绝对值不超过 0.20 m/s、0.45 rad/s；还必须匹配 200 ms 内 `/r680/person_follow/status` 的 `follow_session` 和 `request_id`。状态字段 `enabled`、`candidate_only`、`motion_permitted` 和 `fault` 决定放行。原 `/r680/person_follow/cmd_vel_candidate` Twist 仅供预览，选择器不订阅它。

正常任务停止、完成、租约到期或短时目标不可见时输出零；新鲜非故障状态下保留 AUTO，下一条显式人物任务可继续执行。状态超时、明确故障、有效手柄接管或全局急停时回到 MANUAL，并递增 `person_revoke_generation`；被撤销请求不能凭旧包恢复。来源切回 NAV 同样撤销人物任务。`/r680_base/control_state` 提供 `control_session`、`automatic_source`、`source_generation`、`person_revoke_generation` 和当前模式，并增加 `manual_ready`、`automatic_authorized`、`estop_latched`、`estop_reason`、`nav_manual_coexist` 和 `output_source`。共享导航时 `mode=AUTO` 表示导航授权仍有效，`output_source=MANUAL/NAV/NONE` 表示当前实际选择的输出来源。兼容字段 `armed` 仅表示手动就绪；无手柄 AUTO 时可以为 false，不能据此否认自主授权。人物节点仍按 mode/source/代次与任务状态判断，不依赖 armed。

人物任务启动接口只启动人物执行，不替操作员调用来源选择或 AUTO。检测进程启动也不授权运动；人物任务和底盘 AUTO 授权分别控制。联调保持人物 `enable_motion=false`，只看相对目标与预览候选；部署源代码或构建包不会自动重启已运行的底盘服务。

纯策略测试：`python3 test/test_policy.py`。隔离 DDS 测试必须在 `ROS_DOMAIN_ID=198` 下执行 `python3 test/test_relative_mux_ros.py`；只启动选择器和测试发布者，不启动串口驱动。隔离测试不替代真实手柄失联、制动距离、相机失效与人物靠近的现场验收。

测试维护：策略测试已接入 CMake/CTest；`test_relative_mux_ros.py` 统一覆盖 NAV/PERSON 仲裁、NAV 与手柄输入超时、急停恢复、热插拔和当前限速，替代旧版 `test_ros_mux.py`。`../gamepad_contorl/test/probe_gamepad_hotplug.py` 保留用于 ROS_DOMAIN_ID=174 下的合成输入流回归，检查真实读取线程、EOF、重连和急停锁存。这些测试不会随底盘服务启动，也不安装到运行目录。

手柄未接入时使用 Linux inotify 等待设备路径、udev 符号链接或权限变化事件，不再每秒尝试连接；启动时只尝试一次已存在的设备，拔出后清空输入并继续等待事件。事件监听失败时手柄保持零输出并记录错误。0.3 秒原始摇杆归中由手柄包自行判断，适用于开机、重连及手柄急停解除后的重新使能；缺少摇杆轴快照或中途偏转都会阻止/重置等待。状态切换立即发布，普通状态保持低频心跳。

本次手柄与仲裁协议需要配套加载：更新源文件不会让现有 Python 进程重新导入代码，应在停机窗口统一重启独立底盘服务。隔离回归覆盖无手柄 NAV/PERSON 授权、未授权归零、未就绪手柄不得接管、就绪接管后的退出锁定、手动失联停车、急停跨断连锁存与显式复位、原任务/时效/限幅边界。未进行实车运动或制动验收。

## 手柄与导航共同输入

命令出口仍只有 `command_mux.py` 一个发布者，不允许多个进程直接向底盘串口或 `/r680_base/cmd_vel` 抢发。开机、急停复位、显式 `set_automatic(false)` 后仍不自动恢复导航；启动脚本的 `--enable-motion` 授权会话负责取得 AUTO。手柄在该会话中临时覆盖导航，不需要每次松杆重新授权。

自动 VIO 初始化器同样默认 `nav_manual_coexist=true`，有效手柄输入不再直接触发 `manual_takeover` 失败；实际轮速反馈仍受原有行程、转角、超速、时间和输入有效性限制。操作手柄可能消耗初始化预算，也可能因超速使本次激励失败。急停、显式取消和会话失联仍会停止初始化输出。此变更不绕过定位输入或碰撞保护生命周期条件，不能据此宣称惯性初始化一定成功。

如需旧的 NAV 手柄锁存接管行为，应在启动时同时为 mux 和初始化器设置 `nav_manual_coexist=false`。纯策略类 `Policy` 保留默认 false，运行节点显式默认 true；使用该类的独立测试应明确选择策略。

检查运行是否加载新版本：mux 的 `nav_manual_coexist` 参数应存在且为 true，`control_state` 应包含 `output_source`。源码更新不会使已运行的 Python 进程重新导入；需要在停稳且无导航任务时重启常驻底盘服务才生效。
