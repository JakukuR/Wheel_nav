# 独立相机、检测、局部体素与路径调试

默认链路不依赖 Nav2、地图、OpenVINS 或里程计。窗口只读取数据；独立入口仅启动相机、安装 TF、检测器、深度障碍转换和窗口，不启动运动控制或发送跟随任务。

## 一条命令

```bash
cd /home/orin/ros2_ws
./r680_perception.sh debug-camera
```

- 检测窗口：`http://<robot-ip>:18081/detections`
- 体素/局部路径窗口：`http://<robot-ip>:18081/planning`
- 页面“打开另一窗口”可同时打开第二视图；支持独立图层开关和暂停。
- 默认 ROS 域 73；`ROS_DOMAIN_ID=...` 可覆盖。`--port 18082` 可换端口。
- `debug-camera --check` 仅检查现有发布者、打印实际启动计划，不打开相机。
- RGB、检测、局部障碍点发布者存在时复用；安装 TF 已存在时不重复发布。端口被相同配置的局部调试窗口占用时复用，否则明确报冲突。发现其他域的 RealSense 进程时拒绝再占用设备。
- Ctrl+C 只结束本入口创建的进程；复用的相机/检测/窗口由原终端管理。未设置开机自启。

相机与数据源已经运行，只需要窗口时：

```bash
./r680_perception.sh debug
```

## 两窗口显示什么

| 内容 | 数据源 | 含义 |
|---|---|---|
| RGB/内参 | `/r680/d455/color/image_raw`、`/r680/d455/color/camera_info` | 相机原始 RGB、K 和畸变系数 D |
| 人物/物品 | `/r680/perception/detections`、`/r680/perception/relative_objects` | 同帧识别框、类别、置信度、跟踪 ID、水平距离和方位 |
| 当前局部障碍/体素 | `/r680/perception/local_obstacles` | 当前对齐深度帧，地面/高度及孤立簇过滤后，显示点或 0.10 m 体素线框 |
| 人物局部路径 | `/r680/person_follow/local_path` | 人物跟随自身的局部规划结果 |
| 物品局部路径 | `/r680/object_follow/local_path` | 物品跟随自身的局部规划结果 |
| 二维局部障碍格 | `/r680/{person,object}_follow/local_obstacles` | 跟随模块的二维占用栅格，仅画地面轮廓 |

`depth_to_points` 是复用的独立深度处理可执行文件，虽然位于 `wla_r680_navigation` 包中，但此入口不启动该包的整套 bringup，也不启动 Nav2 节点。局部体素在显示端从当前障碍点按 `floor(point / voxel_size)` 计算，不是跨帧累积地图或确认障碍记忆，不依赖 `d455_floor_odom`。

跟随节点尚未启动时，对应图层显示“尚未收到数据”；没有有效跟随任务/目标时路径为空，显示“暂无路径”。单纯看相机不会自动发起跟随，也不会伪造一条规划路径。需要运行对应跟随模块时，可在另一终端使用现有预览入口：

```bash
./r680_object_follow.sh run enable_motion:=false
# 或人物：./r680_person_follow.sh run enable_motion:=false
```

预览入口仍需原有明确的任务/目标输入才产生跟随路径；这里不替代任务协议，不改变运动授权。

## 对齐与限制

- 检测框匹配 RGB 的 `stamp_ns + frame_id + width + height`；距离进一步匹配 `session + epoch + track_id + stamp_ns`。检测代次变化后不复用旧 ID。
- 默认 `--mode local`：选择与当前障碍点时间最接近的 RGB，只允许车体 `r680_mapping_floor` 或该相机光学坐标系的几何，使用安装外参投影。每个图层与 RGB 的时间差必须 ≤30 ms；超出则不叠加。**这个小时间区间内的运动没有里程计补偿**，快速运动时可能仍有投影误差，页面明确标记该模式和时间差。不把车体坐标系伪装成静止世界坐标系。
- 跟随路径/栅格保留有限时间历史以匹配图像；新的空路径会立即清除旧路径。源数据和本地接收时间都要新鲜。
- RGB 最长 0.8 秒有效；CameraInfo 与 RGB 差 ≤0.3 秒，帧名/尺寸匹配。支持 plumb_bob、rational_polynomial、equidistant 畸变；ROI 裁剪和 binning 暂不支持并明确报错。
- 负深度、NaN/Inf 不绘制；裁剪近裁剪面和画面外线段。**不做深度遮挡消除**，后方几何可能显示在前景物体上，不能据此判断通行安全。
- RGB 缓存上限 24 帧/64 MiB；最多显示 600 个体素、4000 个障碍点、500 个二维格/图层、500 条路径细分段。目标 5 Hz，5 秒没有页面访问就暂停投影计算。断流清空实时画面，主动暂停保留画面并标记非实时。
- 窗口 HTTP 只读，限可信机器人局域网；仅本机访问可用 `debug --host 127.0.0.1`。

已有世界系障碍记忆/里程计数据需跨时间转换时，可显式用 `debug --mode odom --port 18082`。这是可选兼容模式，使用 `lookup_transform_full` 和实际固定里程计坐标系，不自动启动任何导航节点；默认局部模式不订阅 Nav2 路径。

## 测试

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 src/wla_r680_navigation/test/test_debug_projection.py
/usr/bin/python3 src/wla_r680_navigation/test/test_debug_standalone.py
ROS_DOMAIN_ID=197 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  /usr/bin/python3 src/wla_r680_navigation/test/test_debug_viewer_ros.py --local
```

隔离测试的图像明确标记 SYNTHETIC FIXTURE。测试覆盖独立局部投影、无 map/odom TF、无 Nav2 节点、同帧检测、时间差/世界坐标拒绝、CameraInfo 错配、断流恢复和 HTTP 拒绝写操作。软件测试不等于实车标定/投影精度验收。

2026-10-16 真实 D455 静态观察测试：独立入口只启动 camera、mount、detector、local_obstacles，复用窗口；未启动 Nav2/定位/运动控制。RGB 640×480、检测数据和局部体素投影接通，记录到 person 检测；采样记录 308 个体素、2981 个障碍点、几何/RGB 时间差 0 ms。测试结束后本次相机、检测、深度处理进程已退出，窗口保留。未下发跟随任务，真实跟随路径和运动中的投影精度尚未验收。
