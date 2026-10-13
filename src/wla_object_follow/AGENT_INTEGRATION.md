# approach_confirm 接入物体靠近

本次车端源码位置：Agent `/home/orin/Wheel_Legged_Agent`；Nav `/home/orin/ros2_ws`。

## 执行链路

Agent 的 `approach_confirm` 是行为动作包。其“执行”阶段经 `HybridExecutionAdapter → PhysicalDispatcher → RelativeObjectExecutor → RosObjectTransport` 下发 `approach_object` 到 Nav 的 `wla_object_follow`。此链路使用 **ROS 2 Service**，不是 ROS Action；不经过 Nav2，也不要求 map 坐标。

底层距离控制沿用物体包：机器人前沿到可见物体表面目标约 0.15 m；速度随剩余间距降低，可经过 0.05 m/s 收敛到零。软件到位容差、避障、轮速停车证据等约束保持。实际距离精度仍待实车验证。

## 跟随什么物体

`GET /api/actions/state` 返回 `object_targets`，其中 `objects` 的字段为 `id,label,yaw,distance`，以及 `fresh`、`bindings`。yaw 单位 rad，distance 单位 m，来自 Nav 已有相对检测话题 `/r680/perception/relative_objects`。本次不新增 YOLO 推理或检测开关。

使用现有 Bearer token 调用 `POST /api/actions/object-target`：

```json
{"target_key":"object:chosen","target_id":"detector-session:0:7"}
```

`target_key` 必须与 Agent 被选中动作的目标键一致；`target_id` 必须是当前检测流的完整 `session:epoch:track_id`。接口先返回 `queued`，Engine 线程处理后在 `/api/actions/state` 的 `object_selection_result` 中返回成功或原因。**绑定不会启动运动，也不会创建 Engine 意图/世界对象或绕过动作仲裁。** 上游仍负责从已有感知/意图链选择动作和目标。本接口只是连接 Engine 目标键与检测 ID；新观察可见于列表不等于 LLM 已消费。

同名类别不会自动换目标；人物、无有效几何、过期检测及旧检测会话不可绑定。活动物体任务期间禁止重新绑定；更换物体先取消当前任务，待终止/停车确认后再绑定。检测有效期 300 ms。目标持续丢失时当前适配器撤销执行，不声称自动跨 ID 重识别。

## ROS 协议

| 接口 | 类型 | 用途 |
|---|---|---|
| `/wla_agent/object_command` | `wla_behavior_interfaces/srv/ObjectCommand` | `start / heartbeat / stop / status`，参数包含 `request_id,mode,target_id,lease_duration_s` |
| `/r680_base/object_control` | `wla_behavior_interfaces/srv/ObjectControl` | `select / activate / release`，绑定 mux 会话、源代次、撤销代次和任务身份 |
| `/r680/object_follow/status` | `std_msgs/msg/String`，`wla-object-status/v1` | 当前任务、enabled、状态、源时间与会话 |
| `/r680_base/control_state` | `std_msgs/msg/String` | 当前 MANUAL/AUTO、控制源、代次与撤销信息 |

`approach_confirm` 固定为有限任务 `approach_object`。`ObjectCommand` 仍保留 `follow_object` 模式和 `target_id`，供后续持续跟随动作接入；本次没有把其他 Agent 动作自动路由到持续跟随。

控制流程：已有 AUTO 授权且车轮停止 → 条件选择 OBJECT（暂时 MANUAL）→ 启动指定物体任务 → 等待有效零候选 → 对同一任务 activate → 仅由 Engine tick 续租。手动接管、急停、控制代次变更立即拒绝后续激活/续租。不会从初始 MANUAL 自动授权运动。启动脚本允许无地图，仅表示相对物体执行无需地图；首次自动授权仍需车端既有授权流程。

取消通过具名 stop 完成，再释放 OBJECT 控制权。服务通信结果不确定时保留执行占用，不能假装任务成功；底层租约到期负责撤销运动。未知 select 结果可能需要人工/上层确认控制状态后恢复。服务机制自行实现租约、取消与终态关联，没有 ROS Action 原生 feedback/result/cancel API。

## 入口与配置边界

Nav：`/home/orin/ros2_ws/r680_object_follow.sh check`；`run` 默认预览（`enable_motion=false`）。Agent 运行 `python -m wla_sim.r680.action_service --hybrid`，`WLA_VISUAL_TOKEN` 使用现有配置；可省略地图与策略文件用于本地物体动作，其他地图导航动作仍要求相应配置。提供地图与策略时必须成对，旧 `approach_confirm` 的静态导航路由需要移除，否则装载器会拒绝不匹配的执行类型。

Agent 入口不代为启动物体节点；需匹配版本的 ObjectControl 接口和 mux、物体节点、YOLO、RGB-D、相机 TF、轮速，以及显式运动配置/授权。当前 hybrid 保留原有渲染回执要求，无 UI 回执会失败，不能当作无界面自主服务。没有重启生产服务，也没有开启实车运动。

## 代码位置

- Agent `src/wla_sim/hybrid_catalog.py`：动作阶段和能力需求。
- Agent `src/wla_sim/behavior_bridge/object_targets.py`：目标 ID 缓存与绑定。
- Agent `src/wla_sim/behavior_bridge/object_executor.py`：动作到相对物体任务映射。
- Agent `src/wla_sim/behavior_bridge/object_transport.py`：ROS Service、续租与控制权切换。
- Agent `src/wla_sim/behavior_bridge/configuration.py`、`hybrid_execution.py`：分发及执行许可。
- Agent `src/wla_sim/r680/action_service.py`：实际入口、HTTP 接口、Engine 所属线程处理。
- Nav `src/wla_behavior_interfaces/srv/ObjectControl.srv`：条件控制权接口。
- Nav `src/wla_manual_control/scripts/control_policy.py`、`command_mux.py`：唯一底盘仲裁与退出处理。
- Nav `src/wla_object_follow`：原有目标靠近/跟随节点和局部控制。

测试证据见根证据报告及 Windows wheel_nav/output/object-agent-20261013/，提交哈希以本次交付记录为准。软件/隔离 ROS 通过不等于实车 15 cm 停靠验收。
