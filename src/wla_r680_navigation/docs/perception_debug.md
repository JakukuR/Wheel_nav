# RGB 检测与体素路径调试窗口

调试器只订阅 ROS 数据，不调用动作/服务，不发布运动指令，不占用相机设备，不加载第二份 YOLO。原导航 Web 与地图不受影响。

## 启动

相机、检测和规划链路已经启动时，在 `/home/orin/ros2_ws`：

```bash
./r680_perception.sh debug
```

打开两张独立页面（可拖出浏览器成为两个窗口）：

- 检测：`http://<robot-ip>:18081/detections`
- 体素与路径：`http://<robot-ip>:18081/planning`

页面的“打开另一窗口”链接可同时打开另一视图。支持暂停、恢复和独立图层开关。

需要同时启动检测时：

```bash
./r680_perception.sh run --no-agent --debug
```

两条命令二选一，不要重复占用 18081；可分别用 `debug --port 18082` 或 `run --debug --debug-port 18082` 改端口。默认 ROS 域 73，可通过 `ROS_DOMAIN_ID` 覆盖。`run` 与 `debug` 均不自动启动相机或导航。复用已有相机/导航启动入口，不启动第二个 RealSense 驱动。

独立 `debug` 在终端 Ctrl+C 退出；组合 `run --debug` 跟随本次 perception 进程组退出。无开机自启。HTTP 仅用于可信机器人局域网；需要仅本机可见时用 `debug --host 127.0.0.1`。

## 数据与显示语义

| 内容 | 订阅话题 | 显示方式 |
|---|---|---|
| RGB、内参 | `/r680/d455/color/image_raw`、`/r680/d455/color/camera_info` | 原尺寸 RGB；使用 K 与 D |
| 人物/物品 | `/r680/perception/detections`、`/r680/perception/relative_objects` | 同帧识别框、类别、置信度、ID、水平距离、方位 |
| 确认的 3D 体素 | `/r680_nav/d455/points_confirmed` | 三维格子线框，默认边长 0.10 m |
| 当前障碍点 | `/r680_nav/d455/points` | 投影点；不用零时间戳的 RViz `points_viz` |
| 跟随路径 | `/r680/person_follow/local_path`、`/r680/object_follow/local_path` | 两种颜色的路径线 |
| 跟随障碍栅格 | `/r680/person_follow/local_obstacles`、`/r680/object_follow/local_obstacles` | 二维占用格地面轮廓，不补造高度 |
| Nav2 路径/轨迹 | `/plan`、`/optimal_trajectory` | 全局/局部路径线 |

没有执行任务时，跟随路径可能为空；没有对应规划节点时，显示“尚未收到数据”。部分启动模式关闭全局动态障碍记忆时，`points_confirmed` 没有数据属正常，此时仍可显示当前障碍点。查看图层自身状态，不能用 RGB 在线推断规划器已经工作。

确认点云保存的是体素内代表点：显示端先在源坐标系按 `floor(point / voxel_size)` 找到真实格子，再变换八个角点。`--voxel-size` 必须与 `dynamic_obstacle_memory.voxel_size` 一致，默认 0.10 m。该话题是最长约 4 秒保留的障碍记忆快照，不是同一 RGB 帧的瞬时测量；当前上游未发布逐点采集时间，且融合使用 latest TF，其误差不由显示端修复。

## 对齐、失效与边界

- 检测框必须匹配 RGB 的 `stamp_ns + frame_id + width + height`；距离进一步匹配 `session + epoch + track_id + stamp_ns`。检测代次变化后不复用旧 ID。
- 规划视图留约 100 ms 图像延迟供 TF 到达。跨时刻几何使用 `lookup_transform_full`，默认固定坐标系 `d455_floor_odom`；没有所需 TF 时不退回 latest。可用 `--fixed-frame` 指定当前链路实际的连续里程计坐标系。
- RGB 最长 0.8 秒有效；CameraInfo 与 RGB 时间差 ≤0.3 秒，帧名/尺寸需匹配。支持 plumb_bob、rational_polynomial、equidistant 畸变。ROI 裁剪或 binning 尚不支持，明确拒绝叠加。
- 体素/跟随几何约 1 秒失效，实时障碍点 0.8 秒，全局路径 5 秒；同时检查源时间和本地接收时间。零时间戳、超龄、缺 TF、无效相机参数均在页面报原因。路径的时间采用 Path header，不把各 pose 的未来轨迹时间当采集时间；不接受混合 frame。
- 负深度、非有限坐标不会绘制；线段经过近裁剪面与图像边界裁剪。路径细分以近似畸变下的曲线。**不做深度遮挡消除**，后方几何可能叠在前景物体上，不能据此判断通行安全。
- RGB 缓存同时限制 24 帧/64 MiB；最多显示 600 个确认体素、4000 个障碍点、500 个二维占用格/图层、500 条路径细分段。稠密输入会抽样，路径过长时截断。渲染目标 5 Hz，两窗口共用 JPEG/投影结果；5 秒没有页面访问就停止投影计算。
- HTTP 断开或数据过期时清除画面；主动暂停时保留画面并显示“非实时”。页面仅提供 GET 读取，不提供控制接口。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 src/wla_r680_navigation/test/test_debug_projection.py
ROS_DOMAIN_ID=197 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  /usr/bin/python3 src/wla_r680_navigation/test/test_debug_viewer_ros.py
```

2026-10-16：7 项纯几何/身份/缓存测试通过；隔离域 197 的真实 ROS→HTTP 测试通过，覆盖两路检测、不同时间的体素/点/路径/二维格投影、CameraInfo 错配、无 TF、旧数据重放、零时间戳、相机断流与恢复、HTTP 拒绝写操作。浏览器实查两窗口、图层开关和暂停恢复，未见脚本错误。合成测试画面明确标注 SYNTHETIC FIXTURE，不代表实车标定/重投影精度验收。
