# 跟随全链路软件回归

2026-10-27 完成：跟随测试目录 **113 通过、0 失败、0 跳过**，含下述 11 个完整
进程链用例；mux 策略 6 项通过，C++ transport_policy 通过，独立 mux ROS 回归通过。
C++/Python mux 策略的 300 组、120,300 个事件逐项一致。此前列出的 17 个旧失败与
缺失模块收集错误已修正；不通过删除失败用例、跳过 ROS 或放宽生产门禁取得结果。

## 范围与隔离

`test_follow_chain_ros.py` 使用真实 Agent Person API、真实 C++ 跟随节点、Python
计算进程和真实 C++ mux；深度、相机内参、采集 TF、轮速、检测、分割及 ReID 特征
由合成输入提供。通过 Agent 服务发送任务，不注入已经求解的速度替代 MPC。

```
合成相机/轮速/检测/分割/ReID → C++ ROS → Python 跟踪/地图/MPC
Agent Person API → C++ 任务入口       ↓
                      C++ 结果检查 → C++ mux → 私有 Twist 接收端
```

强制 domain 198、localhost discovery；mux 输出重映射到
`/hybrid_test/chassis_cmd_vel`。测试检查只有真实 mux 发布该话题，原底盘话题没有发布者，
控制权状态只有一个真实 mux 发布者。没有启动底盘驱动、YOLO、RealSense 或 Agent 模型。
这是 L3 软件进程联调，不是物理停车、真实视觉准确率或实车运动验收。

## 覆盖

- 三模式 0/1/2：无任务仅维护地图、Agent 启动、实际计算非零候选、mux 最终输出、
  服务取消、迟到心跳/重用请求拒绝。物体接近还检查实际可达停车点。
- 深度断流、轮速断流、租约不续约、Python 挂起、C++ 进程被杀：检查最终接收端归零。
- 手动优先级和 mux 急停：急停清除后旧任务不能恢复。
- ReID：两个同特征候选不换绑，唯一候选经确认后恢复，原任务编号和选定 ID 保留。
- 人物接近到达：连续新鲜计算及静止轮速确认后终止，状态、到达标志、服务返回一致；
  后续目标移动不自动重启，清理 stop 不覆盖到达终态。

归零检查的测试上限为断流/进程故障 0.5 秒、取消/急停 0.3 秒、1 秒任务租约
不续约 1.25 秒；这些是隔离联调的断言上限，**不是生产阈值、实测停车距离或物理制动
时间**。到达最多等待 8 秒以取得连续有效帧，过期帧仍重置确认，不放宽生产 150 ms
计算结果期限。每例保存完整状态、候选、mux 输出、服务往返及归零检查记录。

## 旧测试迁移依据

模拟服务回复改用真实 PersonCommand.Response，补齐 request_id/state；模拟检测补
label；移除地图测试中与查询无关的未定义变量。旧 `follow` 模式改为 `0`，启动必须
检查 accepted；旧 ReID 模型改测当前 FollowTracker/RecoveryIdentity，包含模板间隔、
相似度、邻人歧义、预测期不训练及种子保留。不存在的 test_review_clock 依赖已消除。

未知前方地图仍禁止前进，但原地候选可行，不能把它等同于所有候选不可行；另一个
测试继续验证真正无可行解时丢弃非零返回。掩码 ROS 测试覆盖 bbox/segmentation 与
ReID 开关四组合，依据显式配置检查移除像素数。mux 优先级测试按统一后的 PERSON/NAV
入口检查，不再要求已移除的独立 OBJECT 来源参与输出。

到达全链路用例发现并修复传输层问题：C++ 同步 Python 自己产生的终态时，不再把
已到达状态覆盖成取消；C++ 拒绝终态任务的迟到续约时保留所属任务终态；最终发布的
terminal_arrival 与最终 state 一致。未改变路径规划算法、速度限制和门禁期限。

## 复现

先完成两个包的 Release 构建。不要把本测试域设置为实车使用的 domain 73。

```bash
cd /home/orin/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp RUN_SIM_ROS=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 -m pytest -q src/wla_person_follow/test --junitxml=/tmp/follow-tests.xml
python3 -m pytest -q src/wla_manual_control/test/test_velocity_policy.py
ctest --test-dir build/wla_follow_transport --output-on-failure
python3 src/wla_manual_control/test/test_velocity_mux_ros.py
python3 src/wla_manual_control/test/test_cpp_policy_equivalence.py build/wla_manual_control/velocity_policy_replay 300 20261027
```

没有 RUN_SIM_ROS=1 的跳过结果不能记为 ROS 联调通过；本测试也不等于整个 Nav/Agent
仓库全量测试。运行来源与提交以本次交付回执为准。
