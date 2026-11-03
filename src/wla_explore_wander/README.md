# 闲逛与好奇探索（wla_explore_wander）

不需要地图的短时探索：原地环顾一圈，有值得看的物体就走过去看一会儿，没有就往开阔、最近没去过的方向走 1–2 m，然后进入下一轮。

本包只做决策，不发布速度。每一步动作都作为一条 **leg** 交给人物运控后端（`wla_person_follow`）的 `PersonCommand` 服务，再经过它的 mux 门禁：

| 步骤 | 后端模式 | target_id |
|---|---|---|
| 原地环顾 / 转头看物体 | `4` 原地转向 | 相对转角，弧度，左正，如 `6.483` |
| 走近物体 | `2` 物体靠近 | 当前检测 ID（按记忆里的位置现找，不依赖旧 ID） |
| 闲逛 | `3` 点目标 | 机器人系下的 `dx,dy`，单位 m，≤ 3 m |

模式 3 在 MPC 的滚动深度地图上用 DetourPlanner 的带间隙代价网格路线（Dijkstra，代价 = 步长 + 间隙惩罚）做参考线，路线会贴着门洞中线走。模式 3/4 的目标在任务开始时按当时的轮速里程计位姿固定下来；轮速参考重置会直接取消该 leg。

## 记忆（只在本次运行内，里程计坐标）

- **去过哪里**：0.5 m 一格的停留热力图，时间常数 240 s 逐渐淡化。
- **看过什么**：检测按「类别 + 位置」（0.6 m 内合并）聚成物体实例。每个实例记录被看到的次数、是否去看过、上次看到的时间、是否被挪过、附近是否有人碰过。
- **哪边开阔**：12 个绝对方向扇区，在相机视野内时用后端发布的局部障碍栅格（`/r680/person_follow/local_obstacles`）测可走距离和通道宽度。

## 选择

- **好奇**：新奇度 + 类别稀有度 + 显著性（刚出现 / 被挪过 / 刚有人碰过）− 距离和转角代价 − 刚看过的惩罚。分数 ≥ 0.35 的候选按 softmax 随机选一个；看完后进入 300 s 冷却。
- **闲逛**：扇区可走距离、目标点未去过的程度、转角、通道宽度，加一点随机；离出发点超过边界 70% 时偏向回家方向。目标点必须在边界（默认 4 m）内。

## 停止与恢复

| 事件 | 结果 |
|---|---|
| 有人走近到 1.0 m 内并停留 1 s | 取消当前 leg，会话结束（`yielded_to_person`），不自动重启 |
| `pause`（对应"停下 / 暂停"） | 停车，保留记忆 |
| `resume`（对应"继续"） | 从记忆重新选目标，不要求原目标还在 |
| 手柄急停 / 切到手动 / 后端撤销运动权限 | 会话结束，解除后必须重新 `start` |
| 其他客户端的跟随任务占用后端 | 会话结束（`preempted_by_…`） |
| 超出预算（默认 600 s，暂停不计） | 会话结束 |

`start` 和 `resume` 要求 `/r680_base/control_state` 新鲜、处于 AUTO、未急停、自动源为 PERSON（或速度直通）。本节点不会去改 mux 的模式或来源。

## 运行

先按 `wla_person_follow/README.md` 启动相机、感知和跟随后端（`enable_motion:=true` 才会真正动），并由操作者把 mux 切到 AUTO + PERSON。然后：

```bash
ros2 launch wla_explore_wander explore.launch.py boundary_radius_m:=3.0 session_budget_s:=300.0
ros2 service call /r680/explore/start std_srvs/srv/Trigger
ros2 service call /r680/explore/pause std_srvs/srv/Trigger
ros2 service call /r680/explore/resume std_srvs/srv/Trigger
ros2 service call /r680/explore/stop std_srvs/srv/Trigger
ros2 topic echo /r680/explore/status
```

状态话题里有当前状态、leg、选择理由与分项分数、扇区可走距离、物体实例和最近 10 条事件。

## 测试

```bash
cd ~/ros2_ws/src/wla_explore_wander && python3 -m pytest -q test
```
