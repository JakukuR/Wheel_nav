# YOLO 物体靠近与跟随

包位置：`/home/orin/ros2_ws/src/wla_object_follow`。复用 Nav 侧正在运行的 YOLO、RGB-D 和 C++ 局部避障，不新增推理实例，不依赖 map、Nav2 或全局定位。

## 行为

- `approach_object`：靠近指定非 person 目标；机器人前沿与可见物体表面约 0.15 m，容差 ±0.02 m。需要轮速 |v|≤0.015 m/s、|w|≤0.03 rad/s，并连续三个不同 YOLO 源帧确认，才结束任务。
- `follow_object`：接近到同一距离后保留任务；物体远离时继续接近。目标变得过近时停车，不自动倒退。目标丢失、深度失效或租约过期即输出零，不在近距离按预测盲走。
- 距离默认从配置的机器人前沿 0.24 m 算起，不从相机、底盘中心或目标中心算起。此尺寸来自现有 collision_monitor 外包络，尚未实测。
- 远处使用共享 C++ 栅格/Dijkstra 局部绕行；目标相对距离 ≤1.2 m 时切换 C++ 表面间距控制与矩形 footprint 扫掠。末段路径受阻时停车，不保证在狭窄区继续绕过。

## 速度与停车

默认最大前进速度 0.20 m/s；末段上限 `close_max_linear_mps=0.20`，比例增益 `close_gain=0.8`。速度按剩余间距平滑下降，可经过 0.05 m/s 继续趋零，没有最低速度钳位。加速上限 0.4 m/s²；避障/制动可立即请求零速。

候选速度还受 `v*t + v²/(2*a)` 余量限制，并用实际轮速判断是否需要提前请求停车。`response_delay_s=0.7` 参考现有 Nav 配置中的响应延迟记录；`braking_deceleration_mps2=0.3` 是待实测的假设。碰撞检查保留至少 1 s 的扫掠窗口，制动包络更长时延长窗口。代码发零不等于车已停稳，0.15 m 实物精度仍须验证制动、相机近距盲区及外包络。

## 数据链

1. Nav `nav_yolo_detector` → `/r680/perception/detections`（`wla-detections/v1` 原始框）和 `/r680/perception/relative_objects`（类别、ID、yaw、distance）。
2. Agent 选择完整 ID `session:epoch:track_id`。ObjectFollow 固定该 ID 和类别，拒绝 person，检测器换代不跨代继承任务。
3. 原始框按采集时间匹配对齐深度（≤35 ms），CameraInfo 与深度≤250 ms；要求尺寸、frame、session/epoch 一致。无匹配、重复/倒序、过期、丢失目标时撤销旧表面坐标。
4. 使用采集时刻的相机 TF 和轮速积分参考系保存目标表面，控制时重投影到当前机器人系；不查询 map。最新深度的完整障碍点也参与末段检查，不等待下一次 YOLO；任何时候都不挖掉整个人/物体框的障碍。
5. C++ 控制器 → `/r680/object_follow/command_candidate` → 底盘 mux 的独立 `OBJECT` 源 → 底盘。PERSON、OBJECT、NAV 不混合输出，切源时回 MANUAL，需要重新显式 AUTO。

默认障碍高度 0.01–0.50 m，低于地面 -0.10 m 的落差点也阻挡；选中物体表面取框内 0.01–2.5 m 的有效深度。框不是实例 mask，框内其他物体可能导致提早停车；透明、反光、无深度的物体不能保证靠到 15 cm。轮速参考系仅补偿平面运动，不证明抱起/俯仰情况下可用。末段依赖新鲜局部可见几何，实车侧后盲区仍需验证。

## Agent 接口

两个同类型服务入口：`/wla_agent/object_command`、`/r680/object_follow/command`，类型 `wla_behavior_interfaces/srv/ObjectCommand`。这是 Nav 提供的 Agent 执行接口；不意味着已经修改 Agent 的 LLM 动作路由。

请求字段：`command`（start/heartbeat/stop/status）、`request_id`、`mode`、`target_id`、`lease_duration_s`。每次新任务使用新 request_id；先 stop 后迟到的同 ID start 不得复活。定期 heartbeat 续租，默认建议 1 s 租约、0.3 s 心跳。停止可按 request_id 精确取消。

```yaml
command: start
request_id: object-task-001
mode: approach_object
target_id: '<从当前检测流读取的 session:epoch:track_id>'
lease_duration_s: 1.0
```

状态：`/r680/object_follow/status`，schema `wla-object-status/v1`。包括 request_id、target_id、target_label、state、reason、enabled、object_surface_gap_m、object_surface_fresh、linear_mps、angular_rps、motion_permitted。`arrived` 才是成功；waiting 表示暂不能行动，expired/cancelled 表示需新任务。

`/r680/object_follow/cmd_vel_candidate` 是预览 Twist，不得接到底盘；只有带 session/request/源时间的 `command_candidate` 给 mux。默认 `enable_motion=false` 时后者始终为零。

运动授权顺序：确保停车且轮速新鲜 → `/r680_base/set_object_source`（SetBool=true）→ 下发新 ObjectCommand → 等待新鲜状态与零候选 → `/r680_base/set_automatic`（SetBool=true）。切源不自动开 AUTO。enable_motion 必须显式为 true；手动接管/急停/状态失效会撤销任务，旧 request_id 不能自动恢复。

## 启动与验证

```bash
cd /home/orin/ros2_ws
./r680_object_follow.sh check
./r680_object_follow.sh run  # 默认仅预览；需要外部已有检测、相机 TF 和 wheel/odom
```

生产底盘 mux 本轮没有重启，现有进程不具备新 OBJECT 服务；需在安排好的停车维护窗口加载新代码，之后再做受控实车验收。

构建：`colcon build --packages-select wla_behavior_interfaces wla_follow_local wla_person_follow wla_object_follow wla_manual_control`，随后 source install/setup.bash。

隔离测试使用 ROS_DOMAIN_ID=198、RUN_RELATIVE_ROS=1。`test_object_control.py` 验证间距、障碍、速度收敛、轮速停车和延迟植物模型；`test_object_ros.py` 验证真实 ROS 服务及采集时间 RGB-D 联接；`wla_manual_control/test/test_object_mux_ros.py` 验证独立源、AUTO、超时、撤销，输出重映射到测试话题。均不是实车运动验收。
