# R680 D455 视觉人物跟随

安装路径：`/home/orin/ros2_ws/src/wla_person_follow`。本包复用已运行的 D455 ROS 2 驱动，不独占 USB 相机。RGB、已对齐彩色的深度和 CameraInfo 来自 `/r680/d455/*`；检测模型为现有 `/home/orin/Wheel_Legged_Agent/models/yolo11n.pt`。YOLO11 仅检测 COCO `person`，Ultralytics ByteTrack 维护候选 ID，锁定目标再叠加躯干颜色、三维位置与 odom 平面 Kalman 门控。

## 工作方式

- D455 驱动负责深度对齐；感知线程取最新同步 RGB-D 帧，3×3 空间中值与稳定像素时域滤波；车端只读测试中预热约 4 s、稳定单帧 40–45 ms（约 22–25 FPS）；不承诺 40 FPS。超过 0.30 s 的帧直接丢弃。
- 框内中央躯干的有效深度中值投影到 `r680_mapping_floor`。需要有效的相机光学坐标系到该基坐标系 TF，以及新鲜的 `/wheel/odom`；缺任一项时不产生跟随速度。
- 深度点按 `z=0.10–1.50 m` 高度带投影，形成 6×6 m、0.10 m 栅格和 0.45 m 膨胀。发布的未观察格为 unknown；单前向相机无法认证侧后方为空。
- 控制定时器 20 Hz。目标消失后最多 1.2 s 用 odom 平面匀速 Kalman 预测，线速封顶 0.08 m/s、角速封顶 0.25 rad/s。只在相机、TF、里程计、深度覆盖仍有效且前方畅通时继续。超时、身份歧义、双侧阻挡或障碍过近输出零。
- 不能保证衣着相同、长时间完全遮挡或穿越人群时零误认。本包遇到相近候选会停下；超出预测窗后不自动重新获取目标，须人工重选。ByteTrack ID 本身不作为身份的唯一证据。

本包**仅**发布 `/r680/person_follow/cmd_vel_candidate`。默认 `enable_motion=false`，即使选中人物也只输出零速度；未接到 `/cmd_vel` 或 `/r680_nav/chassis_cmd_vel`。将来接入底盘时，应由现有唯一命令仲裁、安全监督、碰撞监测、速度守卫与硬件急停链审查后接线；不能直接把候选话题重映射到 `/cmd_vel`。

## 构建与观察

车端已有 Jazzy 和 `.venv-perception`（包含 CUDA PyTorch、Ultralytics、cv_bridge），无需安装其他模型：

```bash
cd /home/orin/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wla_person_follow
source install/local_setup.bash
ros2 launch wla_person_follow follow.launch.py publish_mount_tf:=true
```

仅在没有其他发布者负责该外参时使用 `publish_mount_tf:=true`；它复用车上已通过几何门限的外参脚本。
独立终端先确认 `/wheel/odom` 和 `r680_mapping_floor <- d455_color_optical_frame` TF 新鲜，再看：

```bash
ros2 topic echo /r680/person_follow/status
ros2 topic echo /r680/person_follow/detections
ros2 topic hz /r680/person_follow/debug/compressed
```

`detections` 给出当前 `id`、`bbox_xyxy` 和米制相对坐标；订阅 `debug/compressed` 时生成带框图像。绿色框是已锁定目标，青色框是其他人。局部障碍图在 `/r680/person_follow/obstacles`。

## 选择和清除目标

在实时检测列表中选择一个 ID，或使用当前 RGB 图像的像素坐标点击单个人框。仅接受当前可见且深度有效的候选，不会自动选择最近者：

```bash
ros2 topic pub --once /r680/person_follow/select_track_id std_msgs/msg/Int32 '{data: 7}'
ros2 topic pub --once /r680/person_follow/select_pixel geometry_msgs/msg/Point '{x: 320.0, y: 240.0, z: 0.0}'
ros2 topic pub --once /r680/person_follow/clear std_msgs/msg/Empty '{}'
```

`/r680/person_follow/status` 包含 `mode`（idle/tracking/predicting/recovering/ambiguous/lost）、停驶原因、传感器新鲜度、推理耗时和实际候选速度。仅在已完成隔离/仲裁及有人值守的实机验收后，才考虑启用候选输出：

```bash
ros2 topic pub --once /r680/person_follow/enable std_msgs/msg/Bool '{data: true}'
```

该使能在进程内有效；启动默认关闭，`clear` 也会关闭。它本身不会连接或启动车轮。驱动端必须具备独立的命令超时与停止通道。

## 参数与依赖

参数见 `config/follow.yaml`。默认使用导航栈的 `/wheel/odom`；独立运行原始驱动只提供 `/odom` 时，把 `odom_topic` 改为 `/odom`；里程计消息必须表示与 `base_frame` 一致的车体平面位姿。相机外参和高度必须经过现场核对，尤其是深度高度带与真正可碰撞物的关系。D455 盲区、低矮障碍、玻璃及逆光不受此格图覆盖。

测试：

```bash
cd /home/orin/ros2_ws/src/wla_person_follow
PYTHONPATH=. /home/orin/Wheel_Legged_Agent/.venv-perception/bin/python -m unittest discover -s test -v
```
