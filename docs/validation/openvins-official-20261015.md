# OpenVINS 官方源码替换与接口说明（2026-10-15）

## 范围与来源

工作区 `/home/orin/ros2_ws`，修改前外层 Git 提交 `dad3300`。
按用户要求删除旧前端，估计器直接使用
[rpng/open_vins](https://github.com/rpng/open_vins/tree/69488123ed9362dd44b6f28e7f4680abbff1442b)，
固定提交 `69488123ed9362dd44b6f28e7f4680abbff1442b`。
实际启动 `ov_msckf/run_subscribe_msckf`，源码入口为
`src/open_vins/ov_msckf/src/run_subscribe_msckf.cpp`。

保留的上游差异仅为 Jazzy 头文件文件名兼容：
`ROS2Visualizer.h`、`ROSVisualizerHelper.h` 中 image_transport、cv_bridge、
tf2_geometry_msgs 的 `.h` 改为 `.hpp`，四处 include。补丁位于
`src/wla_openvins_navigation/patches/openvins-jazzy-headers.patch`。
官方源文件与固定版本逐文件比较时忽略 CRLF/LF；保留外层仓库已有换行风格，避免无关差异。

## 完整数据接口

默认生产配置为 `src/wla_openvins_navigation/config/r680_d455_20261013`。

| 顺序 | 生产话题/配置 | 类型、坐标与语义 | 消费者 |
|---|---|---|---|
| 1 | `/r680/d455/infra1/image_rect_raw`、`/r680/d455/infra2/image_rect_raw` | `sensor_msgs/msg/Image`，左右目；官方读取相机链 YAML 的 `rostopic` | 官方 ROS2Visualizer 立体同步订阅 |
| 2 | `/r680/d455/imu` | `sensor_msgs/msg/Imu`，D455 IMU；官方读取 IMU 链 YAML 的 `rostopic` | 官方 IMU 回调 |
| 3 | `estimator_config.yaml`、`kalibr_imucam_chain.yaml`、`kalibr_imu_chain.yaml` | 官方配置、相机/IMU 标定、噪声与时间偏置 | 官方 YamlParser；不依赖私有前端解析器 |
| 4 | `/openvins/poseimu` | `geometry_msgs/msg/PoseWithCovarianceStamped`，`global` 中的 IMU 滤波位姿 | 可选既有运动初始化辅助程序观察官方初始化输出 |
| 5 | `/openvins/odomimu` | `nav_msgs/msg/Odometry`，`global → imu`；IMU 时刻传播状态，twist 在 IMU 局部坐标系 | `odometry_adapter` |
| 6 | `/d455_slam/odom` | `nav_msgs/msg/Odometry`，`d455_floor_odom → r680_mapping_floor`；保留官方时间戳 | RTAB-Map、Nav2 及已有接口监测 |
| 7 | `/tf` | 同一转换后的里程计位姿 | 导航 TF 树 |
| 8 | `/openvins/pathimu`、`/openvins/trackhist` | 官方轨迹、特征跟踪图 | 可视化/记录，可选 |

`CameraInfo` 不作为官方估计器的订阅输入；标定来自 YAML。
切换底盘 IMU 配置时从该配置读取输入话题，不把 D455 与底盘 IMU 混用。
坐标转换节点只使用配置中的 `T_imu_base`；位姿、杆臂线速度、角速度与协方差转换
复用已有数学实现。没有额外滤波、外推、轮速融合、时间戳重写或状态机。
原始里程计话题仍可直接观测。官方 TF 通过其原生参数关闭，以免重复发布坐标树。
上层 launch 保留 `mode`、`contact_topic` 参数以兼容现有调用；它们不传给官方估计器。
传入官方节点的 ROS 参数是 `config_path`、`use_sim_time`、
`publish_global_to_imu_tf=false` 和 `publish_calibration_tf=false`。

## 官方行为与现有机器人逻辑的边界

官方入口使用 MultiThreadedExecutor，并将 `use_multi_threading_subs` 设为 true。
配置模板及三组既有配置中旧的 `multi_threading_pubs/subs: false` 已删除，
恢复官方 ROS 节点默认线程设置；相应配置哈希同步更新，标定数据不变。
IMU 回调推进状态，官方后台图像处理负责图像更新；不再使用旧前端串行回调包装。
官方 `visualize_odometry()` 在未初始化、初始化后不足约 1 秒、或 fast propagation
失败时不发布 `odomimu`，并且需要有订阅者。适配节点提供该订阅。
这些是官方条件，没有添加新的初始化或超时策略。

已删除：`frontend.cpp`、`quality_gate.py`、`EstimatorStatus.msg`、
`ImuDiagnostics.msg`、`visual_evidence.hpp`、旧 `openvins-v1.patch`，以及仅验证
这些已删除私有行为的样例。官方源中的私有 ZUPT/MSCKF 计数、传播修改均已撤销。
不再提供 `/openvins/estimator_status`、`tracking_healthy`、`mapping_ready`、
`motion_ready`、`gate_status` 或私有 reset 服务；不保留 `imu_timeout_requires_reset` 锁死。

官方位姿/里程计存在不能被解释为独立“视觉健康”证明，因此不伪造兼容 Bool 状态。
RTAB-Map 直接消费转换后的里程计，OpenVINS 路径不再要求私有前端健康/运动许可。

以下是原有机器人集成条件，保留但不控制官方估计器输入或重置：

- Nav2 启动辅助：等到新鲜里程计（原有 0.2 s 条件）、正确坐标、map TF 和生命周期服务，
  再发送一次 STARTUP；没有里程计时继续等待，未初始化不会触发旧前端故障锁死。
- 通用接口监测：检查图像、深度、CameraInfo、里程计、障碍点云等接收新鲜度，
  开启运动时也检查轮速接收；默认 0.5 s。它影响机器人导航许可，不停止 OpenVINS 计算。
- 已有底盘自动权限、任务权限、手柄接管、碰撞停车与速度/距离预算保持原有职责。
- 显式启用的运动初始化辅助：改读官方 pose 和实际传感器；原有传感器新鲜度和
  运动边界保留（其中 Python 辅助的 IMU 0.1 s 条件仍只约束这次运动辅助）。
  它不是默认估计器链路，也不能杀死官方输入处理。

## 构建与复现

Orin 的系统 cv_bridge 二进制依赖 OpenCV 4.6，OpenCV 开发配置是 4.8。
为官方节点单独构建官方
[cv_bridge 4.1.0](https://github.com/ros-perception/vision_opencv/tree/f5b738d9694f0cee5904440d03912fc249943f8a/cv_bridge)，
提交 `f5b738d9694f0cee5904440d03912fc249943f8a`，源码没有修改。
依赖安装于 `.runtime/openvins_official/cv_bridge_install/cv_bridge`。
launch 仅为 OpenVINS 子进程设置该库路径，不全局覆盖机器人其他节点的 cv_bridge。

现有 `bash src/wla_openvins_navigation/scripts/build_isolated.sh /绝对路径/独立工作区`
已更新为固定官方源码、仅应用头文件补丁、编译上述官方 cv_bridge 与 OpenVINS。
该脚本只编译，不运行测试或节点。使用官方 `ENABLE_ARUCO_TAGS=OFF` 选项，
因为机器上的 OpenCV 不带 ArUco 开发头文件，现有配置也未启用 ArUco。

生产工作区关键构建参数：

```sh
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export MAKEFLAGS=-j2
colcon build --executor sequential --packages-select ov_core ov_init ov_msckf \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DENABLE_ARUCO_TAGS=OFF -DOpenCV_DIR=/usr/lib/cmake/opencv4 \
  -Dcv_bridge_DIR="$PWD/.runtime/openvins_official/cv_bridge_install/cv_bridge/share/cv_bridge/cmake" \
  -DCMAKE_INSTALL_RPATH="$PWD/.runtime/openvins_official/cv_bridge_install/cv_bridge/lib"
colcon build --symlink-install --executor sequential \
  --packages-select wla_openvins_navigation wla_r680_navigation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

## 源码迁移阶段验证边界（启动测试之前）

按用户“别测试”要求，只做源码比较、静态接口/语法检查、编译和 ELF 依赖检查。
没有启动 ROS 节点、隔离 ROS 样例、传感器、仿真、服务切换或实车运动。
上游默认构建的 test 命名可执行文件不代表它们被运行。
现有启动/运动辅助样例只更新接口，未执行，不能声称运行回归通过。
标定仍为既有 commissioning 数据，未宣称实车定位精度或遮挡恢复验收。

QoS 静态契约：官方 IMU 为 SensorDataQoS；官方立体图像为 message_filters 默认订阅；
官方 odom/pose 为默认可靠发布，转换器用 SensorDataQoS 接收、可靠发布里程计。
安装的 RealSense ROS 版本为 4.58.1，其
[官方默认图像 QoS](https://github.com/realsenseai/realsense-ros/blob/4.58.1/realsense2_camera/include/constants.h)
为 SYSTEM_DEFAULT、IMU 为 SENSOR_DATA；本次未查询实时 DDS 端点或更改驱动 QoS。
因此这是源码与配置核对，不是实时传输验收。

最终结果：

- 官方 cv_bridge、`ov_core`、`ov_init`、`ov_msckf` 编译安装成功。
- `wla_openvins_navigation`、`wla_r680_navigation` 编译安装成功，均关闭 BUILD_TESTING。
- 202 份上游文件与固定提交比较，仅两份头文件有上述兼容差异；算法源码一致。
- 修改的 Python 文件 AST 解析、两个 shell 脚本语法、全量 `git diff --check` 通过。
- 三组配置的传感器话题与官方 Kalibr 链一致；两组实机候选配置已有哈希均一致。
  synthetic 既有元数据未记录配置哈希，不将其写成“哈希验收通过”。
- 安装后的官方可执行文件和坐标转换程序 `ldd` 无缺失依赖；官方节点只链接
  OpenCV 4.8，加载独立构建的官方 cv_bridge 4.1.0。
- 旧前端、quality_gate、生成的私有消息不在安装包内；官方库无旧私有计数接口符号。
- 安装后的 launch 与默认配置与源码一致。未运行 ROS 测试或进行任何实车启动。

构建和静态检查记录保存在工作区 `.runtime/openvins_official/`：
`build.log`、`integration.log`、`final-source-audit.json`、`final-install-audit.json`、
`official-node-ldd.txt`。这些机器运行产物不纳入源码提交。

## 用户授权后的启动预览测试（2026-10-15）

用户随后要求“启动测试一下”，故进行两次不启用运动输出的实机传感器预览。
上节“没有启动 ROS 节点”仅描述此前源码迁移阶段；本节是后续运行证据。
源码提交为 `0a4fe19`。未执行单元测试或运动验收，没有新增估计器逻辑。

### 测试命令及退出

第一次：

```sh
./r680_nav.sh --openvins --map map-2026-10-08-1 --test-scan-only
```

日志目录：`/home/orin/nav_run/nav-20261015-143247-1928057`。
官方节点与坐标转换器启动，地图和 Scan 可用，但官方位姿/里程计无输出，
在现有 300 秒初始化等待超时后退出。初期特征历史不足的日志不能解释持续失败。

第二次使用同一命令，额外指定：

```sh
--openvins-config /home/orin/ros2_ws/.runtime/openvins_official/debug_config
```

日志目录：`/home/orin/nav_run/nav-20261015-144053-2002219`。
此运行配置只是既有配置的副本，把官方 verbosity 改为 DEBUG，并更新副本的配置哈希。
未改生产配置、算法或阈值。收集证据后 Ctrl-C 退出；脚本报告已清理并恢复原定位服务。
随后进程检查未发现本次官方节点、转换器或导航启动脚本；`r680-manual-base.service` 为 active。
脚本恢复消息不等于原定位服务恢复了有效定位，本次没有对此做额外验收。

### 20 秒只读采样结果

第二次预览的 `startup_probe.json`：

| 项目 | 结果 |
| --- | --- |
| 左目图像 | 598 条，约 29.9 Hz |
| 右目图像 | 599 条，约 29.9 Hz |
| D455 IMU `/r680/d455/imu` | 3861 条，约 193.0 Hz |
| Scan | 165 条 |
| 官方 `/openvins/poseimu` | 0 条 |
| 官方 `/openvins/odomimu` | 0 条 |
| 坐标转换输出 `/d455_slam/odom` | 0 条 |
| map/odom 到底盘 TF | 均不可用 |
| hardware_output_enabled | false |
| `/r680_nav/chassis_cmd_vel` | 1000 条，非零 0 条 |

实际相机发布端与官方立体图像订阅端均为 RELIABLE，兼容。
末条消息在探针收到时的年龄约为：IMU 14 ms，左右目 73–80 ms。
探针收到数据不等于官方回调收到全部消息；官方日志另行证实图像追踪和静态初始化在执行。
探针未保存完整时间戳序列，因此这些末条年龄不能证明期间没有乱序、跳变或缺口。
第一次探针与关闭过程重叠，不把其消息计数当作稳态频率。

### 已确认阻塞位置及未决问题

第二次 `logs/bringup.log` 持续出现：

```text
InertialInitializer.cpp:139 [init]: USING STATIC INITIALIZER METHOD!
StaticInitializer.cpp:51 [init-s]: unable to select window of IMU readings, not enough readings
```

实际固定版本的官方源码 `src/open_vins/ov_init/src/static/StaticInitializer.cpp:49-52`：

```cpp
if (newesttime - oldesttime < params.init_window_time) {
  PRINT_INFO(YELLOW "[init-s]: unable to select window of IMU readings, not enough readings\n" RESET);
  return false;
}
```

运行日志显示 `init_window_time = 2.00`。这里检查的是初始化缓冲区最后与第一条 IMU 的
时间戳跨度，既不是“单帧超过 100 ms 就杀死节点”，也不是累计运行时长。
判定返回本次初始化失败，官方节点继续接收数据并重试。下游因无里程计而无法就绪。
该判定来自官方代码，不是已删除的私有健康门控。

官方 `VioManager.cpp` 的 IMU 接收路径和 `ov_init/src/init/InertialInitializer.cpp`
的图像初始化路径都会按各自时间基准清理旧 IMU。当前证据尚不能解释为什么保留数据的
跨度长期不足 2 秒：未测量官方内部缓冲区首尾具体值，不能断言低频、缺乏运动激励、
时间戳乱序或上游缓冲区缺陷是根因。此轮没有改官方源码、标定时间偏移或窗口阈值。
下一步应只读采集完整相机/IMU 时间戳序列，核对到达顺序、时间基准与官方缓存清理路径。

结论：官方源码集成和构建已完成，启动预览未通过；未达到定位/导航或实车运动验收。
运行日志及探针 JSON 保存在上述 nav_run 目录，不纳入源码 Git。