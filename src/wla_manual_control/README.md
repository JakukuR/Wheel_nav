# 底盘速度优先级 mux

`r680-manual-base.service` 常驻运行底盘、手柄与 mux，ROS 域 73。
`command_mux.py` 是 `/r680_base/cmd_vel` 的唯一发布者。
当前执行策略是 `VelocityPolicy`；`control_policy.Policy` 保留公用手柄/急停工具和旧策略回归。

## C++ 实现（可选）

`command_mux`（`src/command_mux.cpp` + `src/velocity_policy.cpp`）是同一策略的 C++ 移植：
话题、服务、QoS、50 Hz 输出、5 Hz `control_state` 字段与 Python 版相同，SIGINT/SIGTERM 后发 5 次零速。
默认仍运行 Python 版；`manual_base.launch.py mux_impl:=cpp` 或服务环境变量
`R680_MUX_IMPL=cpp` 切换，回退改回 `python`。2026-10-26 在域 198 模拟负载
（200 Hz odom、50 Hz 手柄、5 Hz 状态）下 CPU 约 5%，Python 版约 39%；尚未实车验证。

`test/test_cpp_policy_equivalence.py` 用随机事件序列（含过期、乱序、重复、畸形 JSON、
NaN/Inf）同时驱动两版策略，逐事件比较全部状态，已注册为 colcon 测试。
修改 `velocity_policy.py` 时须同步修改 C++ 版，否则该测试失败。
`test/test_mux_impl_ros.py <可执行文件>` 在域 198 检查单个实现并测量 CPU。

## 无模式切换

收到新鲜合法的速度即可参与仲裁，无需 PERSON/OBJECT 来源选择、AUTO 切换或状态握手。
非零输入优先级：手柄 > PERSON > OBJECT > NAV。零输入释放本路优先级；有多个自动
发布者时，较低优先级仍可在高优先级发零/过期后输出。无新鲜非零输入时输出零。
速度不相加，实际来源看 `control_state.output_source`。

输入：

- 手柄：`/r680_manual/cmd_vel`，Twist，沿用手柄就绪与消息新鲜度检查。
- 人物：`/r680/person_follow/command_candidate`，String JSON，schema `wla-person-candidate/v1`。
- 物品：`/r680/object_follow/command_candidate`，String JSON，schema `wla-object-candidate/v1`。
- 导航：`/r680_nav/chassis_cmd_vel`，Twist。

人物/物品命令需要 `stamp_ns`、有限数值 `linear` 和 `angular`。有效期 150 ms，
拒绝过期和乱序命令；不再由 mux 检查跟随任务 ID、租约或状态权限。
这些属于上游跟随器的职责，上游停止/失效应输出零或停止发布。
预览后端只向实际候选接口发零，非零计算结果仅在调试预览话题。

手柄接管不撤销自动任务；松杆后只接受交接之后的新命令，仍活跃的跟随可继续。
结束任务应向跟随器发 stop，或锁存急停；不能把短暂拨杆当成结束任务。
限幅沿用部署参数：前进 1.0 m/s、后退 0.3 m/s、角速度 1.5 rad/s。
上游人物策略自行限制最高前进 0.65 m/s、角速度 0.9 rad/s。

## 停止与旧接口

`/r680_base/set_estop`、手柄 ESTOP、非法手柄数值仍锁存停止。
复位需要停稳与手柄急停解除，不回放旧缓存。物理制动与硬件失联看门狗仍需实车验证。

旧 set_person_source/set_object_source/set_automatic(true) 只兼容调用，不控制仲裁；
`automatic_source` 和 `mode=AUTO` 是兼容字段，`mode_required=false`、
`velocity_passthrough=true` 表示新策略。set_automatic(false) 清空自动缓存并撤销任务，
不禁止后续的新速度；持续停止应使用 set_estop。

## 测试

```bash
python3 src/wla_manual_control/test/test_velocity_policy.py
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=198 python3 src/wla_manual_control/test/test_velocity_mux_ros.py
```

ROS 测试只启动 mux 与模拟输入，检查无状态握手的输出、固定优先级、零速释放、
过期/非法命令、手柄交接、急停和限幅，不启动底盘驱动。
旧 test_relative_mux_ros.py / test_object_mux_ros.py 转到此统一回归入口。
实际人物测试步骤见工作区根目录 FOLLOW_TRIAL.md。
