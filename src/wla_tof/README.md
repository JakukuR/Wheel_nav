# 四路 ToF、关节骨架和地面过滤（C++20）

## 当前链路

`ESP32 TF4 → tof_serial → /r680/tof/points_raw → tof_obstacles → /r680/tof/points`。
两节点均为 C++20/rclcpp。状态适配器 `wla_robot_state/robot_state_adapter` 复用电控的 IMU 与实测 `q_rad`，不发送电机指令。ToF 直接通过 FT232R 接 ESP32，raw 921600/8N1，按持久设备路径打开，不发串口命令、不主动翻转 DTR/RTS。四路固件 ID 与实际安装对应：

| 端口 / sensor_id | 安装位置 / mount | 图层颜色 |
|---|---|---|
| 1 / 0 | front_right 前右 | 粉色 |
| 2 / 1 | front_left 前左 | 蓝色 |
| 3 / 2 | rear_right 后右 | 黄色 |
| 4 / 3 | rear_left 后左 | 绿色 |

原固件的 front_left/front_right 等标签是历史逻辑名，以 status 中的 mount 为准。

## 数据、话题与过滤

- `/r680/tof/<mount>/raw`：JSON 原始 64 区测距、status、targets、sigma 和 MCU 时间。
- `/r680/tof/<mount>/points`：光学预览系中的基础有效性筛选点。
- `/r680/tof/points_raw`：同样的有效点，按 CAD 安装位合并到 `base_Link`；不是完全未筛选的测距。
- `/r680/tof/points`：在合并点基础上，再做支撑平面地面剔除与障碍物高度筛选；保留全部原字段和 sensor_id。
- `/r680/tof/status`：四路新鲜度、有效点数、CRC、时间、丢帧及测距范围。
- `/r680/tof/ground_status`：地面模型、输入/输出点数、地面净空/超高剔除计数和过滤可用状态。
- `/robot_state/geometry`：同一 C++ 运动学产生的 8 关节、IMU 原点、3 相机、4 ToF、2 轮几何姿态，供网页绘制。

接收器校验 TF4 444/460 字节协议、CRC32、序号和 MCU 时间语义，拒绝重复/倒序，支持重连与 boot 变化。基础点条件：targets>0、status=5、`min_distance_m ≤ distance ≤ max_distance_m`，当前 **0.15～4.0 m**。超过 `stale_after_s=0.30` 的通道不合并。每路最多64点，合并最多256点，66 ms定时发布。sigma 保留但尚未另设阈值；没有体素降采样、孤立点簇剔除、补洞和时间平滑。

## 实时关节地面模型

只读导出当前电控 `/home/onana/wheel_leg_nav20261105_lqr10_cpp/assets/Motor35_WF.urdf` 的关节原点、轴向和车轮碰撞圆柱，保存在 `config/tof/robot_geometry.json`，记录原文件 SHA-256。不修改电控模型或零位，不取目标关节角、`reference_height_m`、standby 的默认轮腿长度和未校验接触观测器。

FK 严格使用 `T_child = T_parent × T_origin × R_axis(q_measured)`。IMU 四元数给出机身到重力参考的旋转，机身中的平地法向为 `n = Rᵀ [0,0,1]`。每轮圆柱轴 a、半径 r、半宽 w 对地面的支撑距离为：

`support = r sqrt(1-(n·a)²) + w |n·a|`

两侧轮底给出的机身高度分别 `h_i = support_i - n·wheel_center_i`。当前用户确认双轮接触同一平地，`support_mode=both`，取两侧均值；差值超过 0.05 m 则不产生可用地面。点到支撑平面的高度为 `height = n·point + h`，当前保留 **0.05～0.50 m**（含端点）的障碍点。

这是**双轮接地、局部水平地面假设下的运动学支撑平面**，不是点云拟合、接触传感器实测或 SLAM 地图地面；两个轮子无法唯一确定任意坡面的法向。机器人被架起、悬空、单轮接地或进入坡面时必须重新配置/核对工况；不要继续将 both 当作已确认接触。`support_mode` 支持 both/left/right/air/unknown；修改 YAML 后重启本链路，air/unknown 不发布过滤点云。接触来源在 status 中标成操作者确认，不冒充电控接触测量。

输入使用源 header 与主机接收近似时间，新鲜度上限 0.20 s、匹配偏差上限0.10 s；合并云时间戳取参与通道最早的主机接收时刻，避免把旧点重新标成当前测量；各通道仍为近似时间组，没有硬件采样同步或点云运动补偿。缺 IMU/关节、非法值、过期/偏差超限、轮底高度冲突时，停止过滤点云输出并报告原因；不把原始点云冒充过滤结果，不用假空云表示自由空间。地面可用但没有障碍点时才正常输出空障碍点云。

### 几何边界

CAD 传感器安装位仍是预览值，`base_link`/`base_Link` 原点对接、ToF 光轴和8×8网格方向尚未实物标定。沿用45°等角区中心、测距值作为光学Z的预览投影。因此 `geometry_calibrated=false`；过滤已执行不等于几何精度或跟随避障验收通过。网页的 Roll/Yaw 反向约定只作用于显示，后端使用原 ROS 四元数，不反向发布。

## 启动与网页

已有链路运行时无需重复启动。手动启动入口（前台，可 Ctrl+C 停止所启动的只读子节点）：

```bash
cd /home/onana/ros2_ws
./robot_tof.sh check
./robot_tof.sh run
```

入口持有 `.runtime/tof-pipeline.lock`，启动接收器、过滤器，并在状态适配器尚未运行时启动它；不停止已有电控。另一个终端启动网页：

```bash
cd /home/onana/ros2_ws
./robot_debug_web.sh run
```

访问 `http://192.168.50.1:18081/`，刷新以加载新 JS。机身3D骨架可以拖动旋转和滚轮缩放，显示8关节、四路ToF、已安装的上右RGB与下方左右灰度相机；未安装上左相机不显示。轮辐随轮关节角旋转，传感器方向线来自CAD安装轴，不当作已标定光轴。

俯视图的“**ToF 障碍物**”默认显示过滤结果，“**ToF 原始预览**”可独立开关比较；“2 m 居中”适合近场。后方某路若仍在线却0点，先看原始测距是否低于0.15m，不能据此认定掉线。原始通道统计、关节数据和地面平面分别过期，不将旧骨架或旧云持续标为在线。

后台日志：

```bash
tail -F /home/onana/ros2_ws/.runtime/robot_tof.log /home/onana/ros2_ws/.runtime/robot_debug_web.log
```

如需 CLI 观察，使用相同 ROS/DDS 环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/onana/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/onana/ros2_ws/config/uvc_rig/fastdds_uvc.xml
ros2 topic echo /r680/tof/ground_status --no-daemon
ros2 topic echo /r680/tof/status --no-daemon
```

参数集中在 `config/tof/new_orin.yaml`。尚未设置开机自启；尚未把该输出接入人物跟随、Nav2 或地形分析。`follow_input_enabled=false`。

## 验证

```bash
colcon build --packages-select wla_tof --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
ctest --test-dir build/wla_tof --output-on-failure
ROS_DOMAIN_ID=211 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 src/wla_tof/test/test_ros.py --executable "$PWD/install/wla_tof/lib/wla_tof/tof_serial"
ROS_DOMAIN_ID=212 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 src/wla_tof/test/test_obstacles.py --executable "$PWD/install/wla_tof/lib/wla_tof/tof_obstacles" --geometry "$PWD/config/tof/robot_geometry.json"
node src/wla_r680_navigation/test/test_kalo_robot_state.cjs
```

隔离测试不打开真实串口、不发布真实域控制消息。覆盖协议、0.15m边界、完整FK、IMU法向、车轮圆柱支撑、左右链独立、缺关节/接触不明/高度冲突、地面及超高点剔除、IMU继续而关节过期时停止输出、恢复和网页数据过期。真实点云几何与实际地面还需用已知尺寸物体核验。
