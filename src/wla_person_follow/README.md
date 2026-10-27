# 统一目标跟随与靠近

任务和算法实现位于 `ros2_ws/src/wla_person_follow`。默认由 `wla_follow_transport`
的 C++ ROS 节点接入真实 RGB-D、检测和轮速，`hybrid_worker` 在一个独立 Python
进程中运行同一套 `FollowEngine`、目标跟踪和地图/MPC。C++ 负责最终时效、撤销、
租约和控制权限检查，再交给现有 C++ mux。启动只等待命令，不自动选择 ID。

`transport:=python` 保留原 `SimulationFollow` ROS 适配器作为兼容入口；
这不是另一套控制算法。拆分接口、边界与验证见 [HYBRID_TRANSPORT.md](HYBRID_TRANSPORT.md)。

| mode（ROS 字符串） | 目标 | 终止条件 |
| --- | --- | --- |
| `0` | 人物跟随 | 操作者停止或租约/权限撤销 |
| `1` | 人物靠近 | 观测表面距机器人参考原点约 0.95 m，到达即结束 |
| `2` | 非人物物体靠近 | 表面距外廓 0.15 m；不可达则当前局部地图最近可达停车点 |

人物：选定 ID → 几何/Kalman tracker → 同帧目标点云屏蔽与已确认历史占据清理 → MPC。
ReID 只收集人物模板、在丢失时建议重绑定；不参与正常几何跟随的许可。
恢复接受余弦相似度 ≥ 0.40，并通过图像质量、位置检查；在合格候选中取最高分，
有第二名时要求领先至少 0.08，同一 ID 连续确认 3 帧（相邻特征帧间隔不超过 0.6 秒）。
初始模板配对相似度仍为 0.80，后续模板更新仍为 0.90，与恢复阈值分开。
丢失预测 2 秒，然后去最后真实位置，朝消失前运动方向搜索；速度不可靠时使用最后观测方位。
模式 0 保留 5 秒窗口、0.4 秒等间隔采样及多段三阶 Bézier 参考。
模式 1 对当前人物位置减速靠近，不绕行其已走过的历史路径。

物体：绑定具体检测类别与完整 ID，使用同帧分割深度提取可见表面。
保留目标原始深度和占据，其他人物继续作为动态障碍。物体不可见或表面不可用时等待，
不调用人物 ReID，也不执行人物丢失搜索。

模式 2 使用滚动地图的八邻接连通图（禁止斜穿障碍角），从当前轮速参考位置选终点，
把整条路径送入同一个 MPC。所有图边检查已知空间与静态障碍；MPC 继续检查动态人物。
距离计算沿用原物体靠近的 0.48 m 方形外廓，扫掠用外接圆保守检查。
最近可达的范围是当前 12 m 滚动地图和 0.1 m 栅格、有效已知空间及外廓约束，
不是未知区域或全局连续空间的最优保证。保守外廓和栅格会使停车距离大于 0.15 m。
无安全连接时停车等待，不把规划失败当成到达。

终点至少稳定 0.6 秒；到达确认使用至少三个不同新鲜深度帧（跨度至少 0.2 秒）、
有效跟踪、方向误差、可行 MPC 和轮速近零。模式 1 距离容差 0.04 m；模式 2 终点容差
0.065 m。终态锁存，不因目标移动或重发同一 request_id 自动恢复。

## 入口

```bash
cd /home/orin/ros2_ws
./r680_person_follow.sh run transport:=cpp enable_motion:=false
./r680_follow_trial.sh follow --mode 0 --continuous
./r680_follow_trial.sh follow --mode 1 --continuous
./r680_follow_trial.sh follow --mode 2 --continuous
```

已有明确运动授权且后端以 `enable_motion:=true` 启动时，客户端加 `--allow-motion`。
不填 `--target-id` 可交互选择。终端日志自动保存到 `.runtime/follow-trials/`。
启动脚本只使用主工作区 install，不叠加旧 releases。

唯一服务 `/r680/person_follow/command`，Agent 门面 `/wla_agent/person_command`，均为
`PersonCommand`，支持 start/heartbeat/stop/status。租约 0.1–2 秒；每次新任务使用新的
request_id。mode 只接受数字字符串 `0/1/2`，不再接受旧英文模式。
统一速度通道仍使用 `/r680/person_follow/command_candidate`，由 mux 保留手柄优先和急停。
状态中的 `mode`、`target_label`、`approach_goal`、`arrival`、`output_blockers` 报告当前决策。

停止、到达或租约结束后的非活动任务只维护地图（深度处理、占据更新及过期清理），
跳过动态轨迹预测、靠近终点搜索和 MPC 调用，参考路径及候选为空、输出为零。
`planner_activity=idle_map_only` 表示本帧仅维护地图；活动任务进行候选计算时为
`candidate_evaluation`，尚无可规划目标时为 `waiting_target`。
地图维护仍按原来的约 10 Hz 上限运行；本改动未测量整机 CPU 或耗时收益。

## 验证边界

2026-10-26 的 C++ ROS 拆分构建、隔离 ROS 联调和性能证据见
[HYBRID_TRANSPORT.md](HYBRID_TRANSPORT.md)。没有启动实车目标任务或进行运动验收。
生产进程是否加载新版本须核对 `transport_backend`，不能由源码或构建成功推断。

2026-10-27 后续修正旧测试，并完成 113 项跟随回归（含 11 项 Agent API 到实际
C++ mux 的完整软件进程链）、mux 策略/ROS/一致性测试；见
[FULL_CHAIN_TESTS.md](FULL_CHAIN_TESTS.md)。新增用例同时修复了到达终态跨进程传递问题。
