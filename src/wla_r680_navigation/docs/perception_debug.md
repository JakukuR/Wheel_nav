# Kalo 调试台

只读的感知调试网页。车上只负责转发已经序列化的消息，投影、按时间配对和绘制都在浏览器里完成（WebGL）。不启动 Nav2、定位或运动控制，也不发送跟随任务。

## 启动

```bash
cd /home/orin/ros2_ws
bash r680_camera.sh run                # 相机单独一个终端
./r680_perception.sh debug-camera      # 检测器（如未运行）+ Kalo 调试台
```

- 页面：`http://<robot-ip>:18081/`。`--port 18082` 可以换端口；默认 ROS 域 73，可以用 `ROS_DOMAIN_ID=...` 覆盖。
- `debug-camera --check`：只做检查，打印实际启动计划，不启动任何进程。
- 已经在运行的部分直接复用：检测器、安装 TF、桥接器（8765 端口能完成 `foxglove.sdk.v1` 握手）、画面节点（`/r680/debug/color/compressed` 已有发布者）、网页（端口返回本域的 `kalo-debug/v1` 配置）。端口被其他程序占用时直接报错。
- 不启动相机。没有 D455 话题时报错，并提示先运行 `r680_camera.sh run`。安装 TF 由 `r680_camera.sh run` 发布；只有相机用 `--no-mount` 启动且没有其他发布者时，这里才补发。
- `depth_to_points` 只在加 `--with-points` 时才启动；如果导航已经在发布 `/r680_nav/d455/points`，也不启动。
- `nav_perception_bringup --debug` 和 `r680_perception.sh debug` 启动的是同一个页面。

## 组成

| 进程 | 作用 |
| --- | --- |
| `foxglove_bridge`（节点名 `kalo_bridge`） | 只监听 `127.0.0.1:8765`，按 `config/kalo_bridge.yaml` 的白名单转发话题。不开放客户端发布、服务调用和参数修改，公布的能力列表为空。只有浏览器订阅了某个话题，它才去订阅对应的 ROS 话题 |
| `kalo_jpeg` | 只在有人订阅输出时才订阅彩色原图，最多 10 Hz，用 Orin NVJPG 硬件编码（GStreamer `nvjpegenc`），建不起来时退回 OpenCV CPU 编码。输出 `/r680/debug/color/compressed`，消息头与原图相同 |
| `kalo_debug_web` | 发静态文件和 `/config.json`，并把同端口的 `/ws` 原样转发给本机桥接器（只搬运字节，不解析消息）。所以只需要开放一个端口，浏览器也不必单独访问 8765 |

## 图层

| 图层 | 话题 | 画法 |
| --- | --- | --- |
| 相机画面 | `/r680/debug/color/compressed`、`/r680/d455/color/camera_info` | JPEG 解码后作为 WebGL 纹理 |
| 检测框 | `/r680/perception/detections`、`relative_objects` | 选与当前画面时间最接近的一帧（相差 150 ms 以内），标出类别、跟踪 ID 和距离；俯视图上标出目标位置 |
| 目标掩膜 | `/r680/perception/target_mask` | zlib + packbits 解码后作为半透明叠加层 |
| 障碍体素 | `/r680/person_follow/depth_obstacles` | 浏览器把障碍点归入立方格：1.2 m 内边长 3 cm，往外依次 6、12、20 cm，格内至少 2 个点才画。按高度从薄荷绿、黄到樱花粉着色，按针孔模型透视投影到画面（相差 150 ms 以内；点云比画面晚 60–100 ms 到达，先用上一帧，本帧到达后立即重画），近处方块遮挡远处方块。俯视图把最新一帧画成同色方块 |
| 障碍栅格 / 跟随路径 | `/r680/person_follow/local_obstacles`、`local_path` | 俯视图，栅格作为纹理绘制 |
| 导航点云 / 路径 | `/r680_nav/d455/points`、`/plan`、`/optimal_trajectory` | 导航运行时才有数据；通过 `/tf` 用最新变换转到车体坐标系 |

- 关掉的图层会退订对应话题，选择保存在浏览器本地。
- 图层按钮上显示实际频率，以及与当前画面的时间差。
- 状态卡片显示检测频率、相机到检测发布的延迟（中位数）、跟随状态，以及画面帧率和帧龄（相机时间戳到桥接器收到的时间）。

## 资源（2026-10-26 实车，30 s 平均，CPU 调频为 schedutil）

| 状态 | 桥接器 | 画面节点 | 网页 / 转发 | 合计 |
| --- | --- | --- | --- | --- |
| 一个浏览器打开页面（约 1.5 MB/s） | 7.0% | 6.5% | 2.9% | 约 16% |
| 没人打开页面 | 0.3% | 0.1% | 0.4% | 不到 1%，并且不订阅相机 |

原来的 Python 服务端渲染网页一直占 26–33%，而且不管有没有人看都在订阅 30 Hz 彩色图。

## 限制

- 投影使用针孔模型和内参 K，不做畸变校正（D455 彩色相机畸变很小）。导航点云不做深度遮挡消除，后方的点可能画在前景物体上，不能据此判断能否通行。
- `depth_obstacles` 只含离地约 0.05–0.5 m 的障碍点（不含地面），并且已经去掉了跟随目标本人的轮廓，所以目标人物不会显示成方块。
- 画面只有 10 Hz，点云与画面的时间差最大约 60 ms，这段时间内的车体运动没有补偿。
- 导航相关图层用的是最新 TF，没有按消息时间回查。
- 网页和桥接器都不需要登录，只在可信的机器人局域网内使用。

## 测试

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 src/wla_r680_navigation/test/test_debug_standalone.py
ROS_DOMAIN_ID=196 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 src/wla_r680_navigation/test/test_kalo_debug_ros.py
```

第二个测试在隔离域里验证：画面节点按需订阅、频率不超过 10 Hz、JPEG 有效且消息头正确；同时验证网页的配置和静态文件。

2026-10-26 起，这个页面替代了原来的 Python 服务端渲染网页（`debug_viewer.py`、`debug_web/`、`nav_perception_debug`）。
