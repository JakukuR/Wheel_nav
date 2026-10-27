# C++ ROS / Python 计算拆分

2026-10-26 实现，默认入口 `transport:=cpp`。

```
相机深度 / CameraInfo / 轮速 / 检测结果 / 控制权状态
    ↓
wla_follow_transport/follow_transport（C++，ROS 节点 person_follow）
    接收与有界缓存、采集时刻 TF、同帧掩码配对、轮速积分
    ↓ 有界共享内存 + UNIX SOCK_SEQPACKET 通知
wla_person_follow.hybrid_worker（一个 Python 进程，不创建 ROS 节点）
    FollowEngine：任务租约、目标跟踪、身份恢复、深度处理、地图/MPC
    ↓ 带任务 ID、传输代次、轮速代次、源帧和截止时间的结果
同一个 C++ ROS 节点
    独立检查时效、任务撤销、租约、轮速、感知代次和控制权限
    ↓ 原有 /r680/person_follow/command_candidate 协议
现有 C++ mux → 底盘
```

## 接口与运行

`/r680/person_follow/command`（PersonCommand）、`set_estop`（SetBool）、
status、候选速度、ReID 请求、点云、栅格和路径的话题保持原接口。
三个任务模式仍为数字字符串 `0/1/2`，启动不会自动选择目标或恢复任务。

```bash
cd /home/orin/ros2_ws
source install/setup.bash
colcon build --packages-select wla_follow_transport wla_person_follow --cmake-args -DCMAKE_BUILD_TYPE=Release
./r680_person_follow.sh check
./r680_person_follow.sh run transport:=cpp enable_motion:=false output_mode:=mux
```

已有运动授权时，启动参数可使用原有 `enable_motion:=true`；该参数仍不等于目标任务
启动，也不替代 mux 的权限和租约。不要在旧跟随进程仍运行时重复启动生产前缀。

兼容入口 `transport:=python` 使用原来的 ROS Python 适配器和计算子进程，仍共用
同一 `FollowEngine`。混合入口当前只支持 `output_mode:=mux`、系统时间；请求 direct
或 `use_sim_time=true` 会明确失败，不静默回退。默认 launch 不再启动 Python ROS 主节点。

状态新增 `transport_backend=cpp_ros_python_compute`、`transport_generation`、
`transport_worker_pid`、`transport_worker_age_s`、`transport_task_active` 和
`transport_blocker`。最终 `motion_permitted`、实际发布命令、租约剩余时间及 C++
撤销后的终态由 C++ 更新。原数值后端字段仍如实标识 Python MPC / C++ 地图和几何算子。

## 数据与计算边界

- C++ 原始深度缓存最多 16 帧、CameraInfo 最多 24 条、轮速历史最多 128 条；
  轮速积分与 LocalMotion 的梯形速度平均、转角中点公式及原有效性限值一致。
- C++ 只在约 10 Hz 的交付上限内选择新深度。活动任务要求匹配同一深度时间戳和
  当前 request_id 的掩码；CameraInfo 同 frame/尺寸、时间差不超过 250 ms；TF
  按采集时刻查询。Python 保留原算法的身份、掩码、标定及有效深度二次校验。
- 状态增量最多约 50 Hz 交给 Python，包括已积分的新轮速样本，不在 Python
  重算积分。数值工作仍最多一个在途任务，超时结果不能排队后重新生效。
- IPC 每个方向一个 16 MiB 共享槽，接收方复制后确认，发送方收到确认前不覆盖。
  大帧及几何消息走共享槽；小型控制、服务回复和结果走有界本机消息通道。
  Python 不创建 Node/Executor，不订阅 DDS；ROS 消息类型和 CDR 编解码仍作为
  数据适配工具使用。因此这不是全 C++ 算法，也不是无复制实现。
- Python 数值工作改用同进程单线程执行器，保留计算线程和任务处理线程，取消原
  ProcessPool 的额外进程与序列化。GIL 竞争可能影响活动任务尾延迟；C++ 门禁独立
  运行，超出原期限就丢弃，不延长期限来提高“成功率”。
- 临时目录由 C++ 以私有权限创建，连接通过 SO_PEERCRED 验证为本次启动的同 UID
  Python 子进程；正常退出清理共享文件和 socket。子进程异常退出时 C++ 归零并以
  非零退出码结束，launch 不会自动恢复旧任务。

## 输出与撤销

结果包含单调序号、任务 ID、传输代次、轮速参考代次、检测 session/epoch、已交付
深度帧时间、观测时间、计算产生时间及单调时钟截止时间。重复序号、旧代次、未知帧、
过期源数据、非有限速度、Python 否决或延后的结果均不能输出。

任务 stop 和本地 estop 在 C++ 直接处理，不等待 Python。命名 stop 可以先于
在途 start 生效；终态请求不能通过重发或心跳复活。异步 start/heartbeat 有界等待，
租约按 C++ 收到请求的时间计算，迟到响应不能延后原截止时间。已经终止任务的清理
stop 不覆盖到达/撤销等首个终态。

沿用最大源龄 300 ms、默认计算结果预算 150 ms、轮速新鲜度 200 ms、控制权状态
500 ms 和命令租约 100 ms–2 s。Python 的原输出判断继续保留，C++ 另行执行最终
判断。控制权撤销、感知关闭/换代、超时和急停不会因数据重新健康自动恢复任务。

普通 ROS 软件门禁不等于已完成物理停车验收；下游 mux/底盘超时和独立停止手段
仍承担原职责。本轮只使用隔离 ROS 合成输入及真实传感器的禁止运动影子运行。

## 验证记录

截至 2026-10-27，隔离工作区已完成 Release 编译，C++ `transport_policy` 测试通过。
新增 Python 协议及 ROS 集成测试最终为 **21 通过**（12 项协议、9 项 ROS）。
模式 2 初次场景未产生非零速度，经完整状态确认是 `object_goal_unavailable`：
目标表面提取正常，但窄视野合成观测不足以提供保守外廓所需的连通路径。
保留原场景验证明确 holding/零输出；另以更宽视野的合成相机输入验证可达停车点
及非零候选。没有修改生产参数、算法门禁或取消可达场景的非零速度断言。
已通过的测试覆盖实际跟随计算、人物接近、取消、先 stop 后 start、Python 挂起
后的结果过期、独立 C++ estop、控制权撤销、轮速过期、感知关闭和子进程崩溃。

旧测试与未修改生产基线对照：选定单元模块均有相同 12 个失败，旧 ROS 集成模块
均有相同 5 个失败，没有新增失败 ID；另有缺失 `test_review_clock` 导致全量收集
失败。因此不能宣称完整测试套件通过。历史失败及本轮修正的测试场景分别保留记录。

真实传感器、禁止运动、无目标状态下，两轮各 20 秒影子测量：新链路 C++ 与
Python 合计平均 **44.56% 单核**，同时段旧链路主进程与计算子进程合计 **67.67%
单核**，降低约 **34.1%**。两轮所有输出均为零；实际更新地图。单独旧链路前后
基线平均 76.15% 单核，但并行负载、调频和额外订阅会影响测量，不能将其与影子
数据当作严格前后 A/B。该结果不代表活动跟随/MPC 或整车总 CPU 的降幅。

2026-10-27 按用户提供的新地址 `192.168.9.130` 恢复连接并完成上述复测。
本轮没有启动生产跟随或真实目标任务；软件验证不能代替活动跟随负载与实机运动验收。

## 2026-10-27 后续全链路回归

用户要求修正旧测试后完成全链路软件测试。旧 17 项失败及缺失测试模块已修正，跟随
测试目录最终 113 项全部通过、无跳过；其中 11 项连接真实 Agent API、C++ 跟随、
Python 计算、C++ mux 和模拟底盘接收端。另通过 mux 策略 6 项、独立 ROS 回归、
120,300 事件的 C++/Python 策略对照和 C++ 门禁测试。

新增到达用例发现并修复了终态同步问题：Python 的 arrived 不再被同步撤销覆盖，
迟到心跳仍拒绝续约但返回所属任务的 arrived 终态，terminal_arrival 与最终状态一致。
生产参数和规划算法未放宽。完整范围、旧测试迁移依据及复现命令见
[FULL_CHAIN_TESTS.md](FULL_CHAIN_TESTS.md)。上文初版失败记录为历史证据，由本节更新
当前验证状态；仍未进行真实视觉推理、整车运动和物理制动验收。

## 2026-10-27 收发层修正

- status 只由 20 Hz 定时器发布（以及 `fail()` 时发一次终态）。原来每个 Python 结果也
  发一次，实测约 48 Hz，状态记录器、观测窗口和 Kalo 都要按这个频率解析。候选速度仍在
  定时器、每个结果、stop 和 estop 时立即发布，mux 拿到的始终是最新门禁值。
- 订阅、轮速、estop 和命令服务回调里的任何异常都统一走 `fail()`：输出归零、发布
  `compute_transport_failure` 终态、结束 Python 子进程并以非零码退出。命令服务失败时
  调用方收到拒绝；estop 失败时 `fail()` 已锁存急停，所以急停请求仍回复成功。
- start/heartbeat 的在途回复只被取消类代次（stop、estop、撤权、租约/心跳超时、检测换代）
  拒绝。轮速参考重置等非取消的代次变化不再让心跳被拒；输出仍要求结果与当前代次一致。
