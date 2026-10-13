# Relative person approach and follow

人物执行仅使用机器人相对位置，提供 `follow` 与 `approach_interact` 两种模式。人物路径不依赖 map、全局里程计位姿、Nav2 或 MPC。使用轮速机体 twist 补偿短时局部障碍记忆；不自动换 ID。家具语义建图由导航侧独立维护，不受此包影响。

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

源证据过期、深度不可用等仍输出零候选。新鲜检测批次中短时缺少指定目标时，允许下述有界 Kalman 丢失恢复；不把预测点当成新观测。同一 detector session/epoch、同一 ID 恢复且执行租约仍有效时可以继续。检测只读健康状态不可用或 detector 代际变化会取消任务，恢复后必须重新 start。NAV 检测进程有新鲜图像就持续推理，不提供 Agent 检测启停服务；本包仅消费 `/r680_nav/perception/status`，人物 stop 不会停止 YOLO。

## 深度安全与运动权限

本包保留本地 D455 aligned depth 安全门：深度、内参匹配，图像时刻 camera→`r680_mapping_floor` TF；无 map 或全局定位要求；新增 `/wheel/odom` 的机体速度输入，child_frame_id 必须为 `r680_mapping_floor`，忽略其 pose。无新鲜深度、深度覆盖不足、附近可见障碍进入紧急距离时停车；直达通道受阻时尝试局部绕行，没有已观测通路则停车。人物本身仍是障碍。这是前向相机局部安全门，不提供完整屋内路径规划，也未建立侧后方完整可观测性或台阶通行能力。

默认 `enable_motion=false`，`/r680/person_follow/cmd_vel_candidate` Twist 仅用于预览；正式 `/r680/person_follow/command_candidate` String 始终为零。`enable_motion:=true` 只允许生成正式候选，仍需要底盘显式选择 PERSON 自动源和 AUTO 授权；不会直接发布 `/r680_base/cmd_vel`。正式候选和 status 都有进程级 follow_session 与 request_id，防止跨重启/任务迟到速度。手动接管、切回 NAV 或运动控制状态失效会撤销执行。

默认最大候选速度 0.20 m/s、0.45 rad/s；内部加速斜率有限，任何安全停立即置零。启动、退出、没有 Agent 命令时都持续零输出。现有相机/底盘实测制动、障碍和手动接管需在分级实车验收中验证，隔离回放不能代替。

底盘控制状态正常约 5 Hz；最多允许源时间 0.50 s，传输时间从这 0.50 s 预算扣除。迟到 AUTO 状态不得覆盖新 MANUAL，跨底盘进程旧会话状态也不得重新授权。

## 测试

`python3 -m unittest discover -s test -p 'test_relative_control.py' -v` 验证：无输入、安全门、源时间 TTL、检测健康兼容状态、会话/epoch、指定 ID、租约、原子模式切换、接近完成锁存、角度正负号和失目标立即停。


## 独立局部绕障（2026-10-13）

`wla_follow_local/src/local.cpp` 实现栅格射线、障碍膨胀、Dijkstra 与弧线碰撞检查，Python `local_avoidance.py` 仅做数组适配，不保留 Python 搜索回退。通过 pybind11 调用，主要计算释放 GIL；状态与 ROS 回调仍在 Python 中，不宣称硬实时。深度按离地高度筛选，默认 0.01–0.50 m，下限可改为 0.02 m；阈值不是相机 1 cm 检出精度的证明。高度依赖图像时刻相机到 floor 坐标的安装变换，本版不估计动态地面平面，适用于标定正确的平地姿态。低于地面 0.10 m 的有效点也保留为危险占据；缺测不能证明没有落差。

局部栅格分辨率 0.10 m、中心范围约 ±3 m；仅实测射线与未过期历史提供可通行中心点。障碍规划膨胀为 max(obstacle_inflation_m, emergency_distance_m+0.10)，默认 0.70 m，用来提前绕开 0.60 m 紧急停车区。机器人当前位置只补 1 格连接种子，不能据此声称已观测侧后方完整扫掠体积。

直达通路优先，否则使用 8 邻接 Dijkstra、禁止对角穿角、保留绕行侧偏好；路径前视点约 0.45 m，比例角速度控制，实际输出弧线检查约 0.8 s。限速仍为 0.20 m/s、0.45 rad/s，不倒车。目标保持碰撞对象，不从深度中整框删除人物。

`local_memory.py`：轮速机体 twist 积分只建立任意原点的局部运动参考，保存约 2 s 原始观测；不递归刷新历史寿命。轮速源龄超过 0.20 s、数值异常或运动代次重置时停止使用旧场景。新鲜深度和轮速恢复后可恢复仍有效的任务。

诊断输出 `/r680/person_follow/local_path`（Path）和 `/r680/person_follow/local_obstacles`（OccupancyGrid）均为机器人坐标；后者显示原始占据，控制器内部另外膨胀。status 包含规划原因、绕行侧、轮速新鲜度、记忆时间及实际规划净空。20 Hz 是控制定时器配置，不等于实测整链路帧率。

验证：核心/几何/记忆/隔离 ROS 用例以及闭环运动学场景。不使用预测的基线宽视野场景能绕开箱体；86 度全视野场景出现目标出视野并停车。新增 Kalman 层另有机体运动补偿和隔离 ROS 漏检测试；不将这两组测试拼接声称真实相机已完成全程绕障。未进行真实相机低障碍精度、侧后盲区、底盘制动或动态人物实车验收；默认 enable_motion=false。

测试命令（隔离域 198，无底盘）：`RUN_RELATIVE_ROS=1 ROS_DOMAIN_ID=198 python3 -m pytest -q src/wla_person_follow/test/test_relative_control.py src/wla_person_follow/test/test_relative_geometry.py src/wla_person_follow/test/test_local_avoidance.py src/wla_person_follow/test/test_local_memory.py src/wla_person_follow/test/test_local_closed_loop.py src/wla_person_follow/test/test_relative_ros.py`。

车端 Release 构建与安装包验证：58 项测试通过，覆盖原相对控制、几何、C++ 局部规划、轮速短时记忆、闭环场景、隔离 ROS。未启用实车运动。


## C++ Kalman 与短时丢失恢复

`wla_follow_local/src/target_kalman.hpp` 实现局部平面匀速状态 [x,y,vx,vy]，Eigen 固定尺寸矩阵预测/更新，Joseph 协方差更新；Python `target_tracking.py` 只负责身份与机体坐标适配。检测采集时刻的轮速局部位姿将相对观测转换到短时运动参考，输出再转换回当前机器人坐标。不是 map 坐标，也不依赖外部 VIO。

默认 `target_prediction_horizon_s=2.0`（可调为 1.0，最大 2.0）；从最后一次被滤波器接受的观测起算，空检测、预测、拒绝的异常值不会续期。位置最大轴标准差门限 `target_prediction_max_std_m=0.75`；协方差先超限时会提前停止，2 秒是上限而非固定放行时间。当前测量噪声标准差 0.08 m、加速度过程噪声模型 1 m/s²、二维创新门限 9.21、目标速度上限 3 m/s，均为待实测标定的模型假设。

行为顺序：

1. 人可见且测量有效：跟踪滤波位置，按 follow / approach_interact 正常执行。
2. 新鲜检测帧缺少原目标或测量被创新门拒绝：平移目标固定为最后一次可靠观测位置，绝不逐帧前移到外推位置；用现有局部绕障到达该位置附近。默认 `lost_target_stop_distance_m=0.35`，另有 0.10 m 到达容差；感知中的障碍仍保留，可能阻止到达。
3. 行进朝向遵循绕障路径；到达最后可见位置附近后，朝向滤波器预测的目标方位角。该 yaw 是相对机器人的视线方位，不是人体朝向或行走速度方向。
4. 同一完整 ID 重现时恢复普通跟随；仅预测的帧不能完成 approach_interact 的到达确认。预测超时或不确定度超限、无安全通路、轮速/深度失效则零输出。轮速代次、请求 ID、人物 ID 或检测器会话变化均清空旧跟踪器。

检测流整体停更超过原 0.30 s 门限时仍停车；2 秒恢复只适用于检测流和深度仍新鲜的目标短时丢失。运动授权与 Agent 租约不能由 Kalman 续期。最大线速度 0.20 m/s，因此 2 秒最多行进约 0.40 m（实际受加速与避障限制），不保证走到较远的消失位置。

status 增加 `target_tracking`、`target_prediction_age_s`、`target_position_std_m`、`target_filter_reason`、`last_seen_yaw`、`last_seen_distance`。Agent 原检测协议和 person_command 接口不变，预测不伪装为 YOLO 识别结果。C++ 数值计算加速不等于硬实时：ROS 任务状态和定时回调仍由 Python 管理。


### 坐标参考与相机运动

人物滤波和历史障碍均保存在同一任意原点的局部参考系中；该参考系在同一轮速代次内不随机器人刚性平移/转动。观测链为 `p_local = T_local_base(t_image) * T_base_camera(t_image) * p_camera`，控制使用 `p_base_now = inverse(T_local_base(t_now)) * p_local_predicted`。相机位姿也按 `T_local_camera(t_image)=T_local_base(t_image)*T_base_camera(t_image)` 计算，status 输出相机局部位置、旋转矩阵、采集时间与参考代次。

YOLO 投影和深度绕障均按采集时刻查询相机 TF；相机独立云台运动需要对应的动态 TF 数据，不能用固定安装 TF 冒充。当前 local_base 来自轮速平面运动，补偿平移与偏航；它不是完整六自由度惯性世界系。轮滑、被抱起及未测得的底盘俯仰/横滚仍需可靠 IMU/VIO 或姿态反馈，不能宣称轮速已解决。参考系重置时清空人物与障碍历史，禁止拼接不同代次坐标。

新增合成回归：机器人与相机安装位姿同时变化、人物静止时，局部系人物速度应接近零；机器人和人同时运动时，应恢复人的独立平面速度。未因此改变 Agent 的五个相对检测业务字段。

障碍记忆保存原始观测的局部参考坐标；每次控制 tick 使用当前轮速位姿重新投影到机器人坐标，不复用图像时刻的机体网格。回投影不刷新观测寿命，路径与网格诊断时间为控制时刻，相机参考位姿仍携带原采集时间。
