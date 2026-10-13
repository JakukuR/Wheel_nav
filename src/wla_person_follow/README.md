# Relative person approach and follow

人物执行仅使用机器人相对位置，提供 `follow` 与 `approach_interact` 两种模式。人物路径不再包含 map/odom 转换、Nav2 目标、世界坐标预测、自动换 ID、地图拐角搜索或 MPC。家具语义建图由导航侧独立维护，不受此包影响。

## 运行与数据

`/home/orin/ros2_ws/r680_person_follow.sh check` 检查依赖，`run` 启动服务。启动仅等待命令；不会自动选人。`indoor`/`outdoor` 旧 shell 别名均转入同一相对执行模式。旧 `autonomous_follow.launch.py` 与 `voxel_mpc_follow.launch.py` 已移除。

唯一检测来源为 nav 侧 YOLO；本包不加载模型、不订阅 RGB。输入 `/r680/perception/relative_objects` 使用 `wla-relative-detections/v1`：

```json
{
  "schema": "wla-relative-detections/v1",
  "stamp_ns": 10000000000,
  "session": "camera-session",
  "epoch": 0,
  "enabled": true,
  "base_frame": "r680_mapping_floor",
  "objects": [
    {"detected": true, "label": "person", "id": "camera-session:0:7", "yaw": 0.2, "distance": 2.0}
  ]
}
```

业务字段只有是否识别、类别、ID、yaw、distance。外壳时间戳与 session/epoch 是时效和身份安全元数据。yaw 使用 rad，机器人左侧为正；distance 是机器人平面距离，单位 m。无有效深度或安装 TF 时 yaw/distance 同时为 null，不伪造坐标。输入源龄最多 0.30 s，传输时间计入此预算；检测状态 heartbeat 不会刷新目标证据。

## Agent 执行接口

内部服务 `/r680/person_follow/command` 类型 `wla_behavior_interfaces/srv/PersonCommand`。Agent 应经正式桥接 facade 调用；内部契约如下。

| command | 作用 |
| --- | --- |
| start | 原子设置 request_id、mode、target_id、lease_duration_s；需要当前可见且相对位置有效的 person |
| heartbeat | 仅为匹配且尚未到期的 request_id 续租 |
| stop | 停止匹配 request_id；空 request_id 全停 |
| status | 只读当前任务 |

租约范围 0.1–2.0 s，建议 1 s 并约每 0.3 s 续租。重复同 request_id、mode、target_id 的 start 幂等且不续租；终止/到期的 request_id 不可复活，新任务需新 ID。同 request_id 携带不同参数会被拒绝。具名 stop 即使先于同 ID 的 start 到达，也会记录取消，阻止迟到 start；不会停止其他 ID 的当前任务。新 request_id 可以原子切换目标/模式。识别到多人时只执行完整 target_id 匹配者，绝不自动改跟另一 ID。

`follow` 默认保持 1.40 m；偏角大时只转向；距离过近不倒车。`approach_interact` 默认靠近到 0.95 m，加 0.10 m 到达容差；至少三个不同源帧满足距离且绝对 yaw≤0.12 rad 后进入 `arrived`，锁存停止，后续人走开不会自动追赶。

空检测、目标丢失、源证据过期、无效距离、深度不可用均立即零候选；不会沿预测目标继续走。同一 detector session/epoch、同一 ID 恢复且执行租约仍有效时可以继续。检测只读健康状态不可用或 detector 代际变化会取消任务，恢复后必须重新 start。NAV 检测进程有新鲜图像就持续推理，不提供 Agent 检测启停服务；本包仅消费 `/r680_nav/perception/status`，人物 stop 不会停止 YOLO。

## 深度安全与运动权限

本包保留本地 D455 aligned depth 安全门：深度、内参匹配，图像时刻 camera→`r680_mapping_floor` TF；无 map/odom 要求。无新鲜深度、深度覆盖不足、附近可见障碍、前进走廊占用时停止相应候选。人物本身仍是障碍。这是前向相机局部安全门，不提供完整屋内路径规划，也未建立侧后方完整可观测性或台阶通行能力。

默认 `enable_motion=false`，`/r680/person_follow/cmd_vel_candidate` Twist 仅用于预览；正式 `/r680/person_follow/command_candidate` String 始终为零。`enable_motion:=true` 只允许生成正式候选，仍需要底盘显式选择 PERSON 自动源和 AUTO 授权；不会直接发布 `/r680_base/cmd_vel`。正式候选和 status 都有进程级 follow_session 与 request_id，防止跨重启/任务迟到速度。手动接管、切回 NAV 或运动控制状态失效会撤销执行。

默认最大候选速度 0.20 m/s、0.45 rad/s；内部加速斜率有限，任何安全停立即置零。启动、退出、没有 Agent 命令时都持续零输出。现有相机/底盘实测制动、障碍和手动接管需在分级实车验收中验证，隔离回放不能代替。

底盘控制状态正常约 5 Hz；最多允许源时间 0.50 s，传输时间从这 0.50 s 预算扣除。迟到 AUTO 状态不得覆盖新 MANUAL，跨底盘进程旧会话状态也不得重新授权。

## 测试

`python3 -m unittest discover -s test -p 'test_relative_control.py' -v` 验证：无输入、安全门、源时间 TTL、检测健康兼容状态、会话/epoch、指定 ID、租约、原子模式切换、接近完成锁存、角度正负号和失目标立即停。
