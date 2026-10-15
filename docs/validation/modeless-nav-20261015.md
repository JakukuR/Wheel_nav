# 手柄与 NAV 取消模式门控 — 2026-10-15

## 问题与改动

实车运行 `nav-20261015-161908-2358368` 已接收导航点，但底盘状态为
MANUAL、automatic_authorized=false、output_source=MANUAL。导航脚本的
`--enable-motion` 开放导航侧输出，却没有切换底盘；用户明确要求删除这种模式分隔。

NAV 分支现在直接仲裁有效手柄与导航速度，不读取 mode 决定放行，也不需要
set_automatic、停稳零速授权握手或手柄在线。有效非零手柄优先，松杆后使用交接后的
新鲜 NAV 命令；不把两路相加、不因手柄覆盖取消导航点。无有效输入输出零。
旧的 NAV 手动锁存分支和 AUTO 授权分支已移除。速度有效期、限幅、急停仍有效。

NAV 下 set_automatic(true) 仅为旧调用方返回兼容确认；false 清除当前缓存并取消
当前导航目标，不阻止后续新命令。持续停止两路使用 set_estop；复位清除旧输入，
接收复位后的新命令。手柄 ESTOP 和非法手动数值仍取消导航任务并锁存停止。

NAV 状态新增 mode_required=false。旧 mode=AUTO 字段为已有初始化器、界面和任务
消费者保留兼容，不是一个需要切换的控制模式。初始化/导航会话在该状态下不再等待
MANUAL 前置状态；上游定位、避障、初始化运动限制与输出许可保持原语义。

PERSON/OBJECT 仍是独立任务来源，任务激活、身份、代次和时效校验保持。
从 NAV 显式选择 OBJECT 不再要求事先调用 NAV 的 AUTO 服务，但仍须停稳且需要
另行绑定任务后激活；不能只凭选择来源输出跟随速度。

## 验证记录

- 53 项纯策略测试通过：NAV 启动即用、手柄优先、松杆后新命令恢复、超时归零、
  急停与复位、人物/物体任务有效性与来源互斥。
- 实际 mux 节点在隔离 ROS 域 198 测试通过：启动即接收新鲜 NAV，无模式调用；
  手柄覆盖不取消导航、恢复 NAV、断连、限幅和急停。该域没有底盘串口驱动，
  `/r680_base/cmd_vel` 仅有测试 mux 一个发布者。
- 证据保存在 `.runtime/openvins_official/modeless_ros.log`。
- 原有 OpenVINS 相机配置未提交改动不属于本次修改。

上述隔离检查不构成实车运动和制动验收。

## 补充验证与加载

- 导航会话隔离测试覆盖 normal、manual_takeover、estop、process_killed、
  delayed_health、cancel_health_drop、waiting_health、stale_health。
  首轮前七项通过；最后一项因 5 秒诊断间隔超过观察窗口失败，测试观察窗口
  从 6 秒改为 8 秒后单独重跑通过，生产阈值及停止断言未修改。
- 初始化助手的十项隔离用例分批通过：positive、no_gamepad、manual_takeover、
  sensor_report_lost、sigterm、future_sensor、stale_sensor、stale_image、
  guard_enabled、preview。修正测试提取启动脚本时缺少循环上下文的问题；
  future_sensor 的测试输入由提前 51 ms 改为 200 ms，避免传输延迟使其落入
  既有 50 ms 容差，生产阈值未改。详细记录在 modeless_init.log、
  modeless_init_remaining.log、modeless_handoff.log、modeless_stale.log。
- bash 语法检查和 git diff --check 通过。
- 旧导航会话已停止。随后发现独立 task_velocity_gate（PID 1485740）仍向
  /r680_nav/chassis_cmd_vel 持续发送零速，4 秒观测内没有任务输入或活动租约。
  用户明确要求杀掉该进程，核对命令行后发送 SIGTERM，确认进程退出。
  本次仅停止该运行进程，未删除任务门控源码或修改其启动配置。
- 确认 NAV 输入无剩余发布者后重启 r680-manual-base.service，服务 active，
  新 control_session=7eb91b2b65ef4f379e1eb0e756f6c1c2，mode_required=false。
  加载后 4 秒内底盘命令 201 帧全部零速，轮速线速度为零，角速度最大
  0.002 rad/s；NAV 输入 0 帧。证据：modeless_load.log、modeless_after_load.json。
- 没有重新启动导航、发送导航目标或进行实车运动测试。下次运行原导航命令
  即使用新逻辑，不需另调 set_automatic。
