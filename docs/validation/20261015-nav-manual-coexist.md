# 手柄与导航共同输入验证（2026-10-15）

用户要求手柄与导航都可输入，不再因正常手柄操作把导航锁回 MANUAL。运行节点默认启用 `nav_manual_coexist=true`：保留显式 AUTO/NAV 运动授权，非零有效手柄输入临时覆盖导航；归中、断开或输入过期后，先归零并等待新的导航命令恢复。速度不相加，空闲手柄零消息不取消目标。底盘速度仍由一个 mux 发布。

初始化器采用同一默认配置，有限的手柄速度不再直接触发 `manual_takeover`。实际轮速仍计入原有行程、转角、时间和超速预算；非法数值、急停、显式取消、授权失联保留停止行为。PERSON/OBJECT 的手柄接管与任务撤销规则保持原逻辑。纯 Policy 类保留默认旧策略，Mux 明确传入运行配置；设置两个节点启动参数 `nav_manual_coexist=false` 可使用锁存接管模式。

## 软件证据

- 52 项 Python 策略测试通过（原有 43 项，加共享导航 9 项）。覆盖同时输入顺序、空闲零消息、松杆新命令屏障、断连/超时、无隐式授权、限幅、急停复位与非法输入。
- `cmake --build build/wla_r680_navigation --target vio_initializer test_vio_init_policy --parallel 2` 完成；原初始化策略的成功交接、七类锁存故障、超时、行程预算和残留速度约束通过。
- manual_control CMake/CTest 两个测试目标通过。
- `test_relative_mux_ros.py` 隔离 DDS 测试通过：真实 mux 在导航中接受手柄，保持 AUTO、不取消 Nav2，松杆后恢复新导航命令；仍只有一个底盘速度发布者。
- `test_navigation_motion_session.py` 八个既有授权交接场景通过。
- `test_cuvslam_initialization_authority.py` 十个场景通过：成功、共享手柄、非法手柄、旧接管模式、缺授权、授权进程退出、显式 MANUAL、急停、障碍、延迟 guard。共享手柄场景使用真实 initializer/guard/session/mux 与合成传感器，观察 MANUAL 输出后回到 NAV，并最终进入 succeeded。
- 所有 ROS 主动发布测试在域 198 完成，未启动底盘串口驱动；模拟成功不是 cuVSLAM SDK 实车惯性初始化成功。
- `git diff --check` 通过。测试初次执行曾因缺少工作区环境、WLA_NAV_BIN 或未指定隔离域被拒绝；补齐正确环境后重新执行通过，未放宽断言。

## 常驻服务加载及只读实机验证

15:52:29（香港时间）重启 `r680-manual-base.service`，加载新的 Python 仲裁。重启前确认无导航/初始化进程，连续约四秒 MANUAL/NAV、无 AUTO 授权、无急停锁存，801 条轮速反馈最大 |v|=0、|w|=0.002 rad/s。

重启后服务 active，MainPID=2298809；运行参数 `nav_manual_coexist=true`，control_state 含 `output_source`。四秒观察收到 199 条底盘命令，全部为零；792 条车身 IMU，约 200.06 Hz；轮速最大 |v|=0、|w|=0.001 rad/s。底盘速度只有 r680_command_mux 一个发布者。最终 MANUAL、automatic_authorized=false，手柄当时未连接。

没有调用 AUTO 授权，没有发布实车非零速度，没有执行运动验收。手柄实物共同输入、制动和 cuVSLAM 惯性初始化仍待现场验证；原有传感器及碰撞节点启动依赖未被本变更绕过。

运行日志、前后状态 JSON 和源码备份位于 `/home/orin/ros2_ws/.runtime/nav_manual_coexist_20261015_154112/`；不提交这些运行产物。既有 OpenVINS 的 bringup.launch.py 未提交改动及 d455_openvins.yaml 未跟踪文件不属于本次变更。
