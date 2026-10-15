# 官方 OpenVINS ROS 2 接入

估计器与传感器处理直接使用 `rpng/open_vins` 的
`ov_msckf/run_subscribe_msckf`，固定提交
`69488123ed9362dd44b6f28e7f4680abbff1442b`。

`src/open_vins` 相对该版本只保留 Jazzy 头文件兼容修改：
`image_transport`、`tf2_geometry_msgs`、`cv_bridge` 的 `.h` 引用改为 `.hpp`。
具体差异见 `patches/openvins-jazzy-headers.patch`。没有修改官方估计器、
初始化判定、MSCKF/ZUPT、IMU 积分、图像队列或线程调度。

原来的 `frontend.cpp`、质量门控、私有状态消息与估计器行为补丁已删除。
不再使用 `imu_timeout_requires_reset`，也不以自定义超时停止处理输入。
旧的关闭多线程配置已移除，使用官方 ROS 节点默认线程设置；标定数据保持不变。

## 接口

| 接口 | 类型与用途 |
|---|---|
| 左右目输入 | `sensor_msgs/Image`；由 `kalibr_imucam_chain.yaml` 中的 `rostopic` 指定 |
| IMU 输入 | `sensor_msgs/Imu`；由 `kalibr_imu_chain.yaml` 中的 `rostopic` 指定 |
| `/openvins/poseimu` | 官方 `PoseWithCovarianceStamped`，IMU 在 `global` 中的滤波后位姿 |
| `/openvins/odomimu` | 官方 `Odometry`，`global → imu`，包含 IMU 时刻的传播状态 |
| `/openvins/pathimu`、`/openvins/trackhist` | 官方轨迹与特征跟踪图 |
| `/d455_slam/odom` | 纯刚体转换后的 `Odometry`，`d455_floor_odom → r680_mapping_floor` |
| `/tf` | 由坐标转换节点发布同一转换后位姿，保留输入时间戳 |

默认 D455 配置的输入是 `/r680/d455/infra1/image_rect_raw`、
`/r680/d455/infra2/image_rect_raw` 与 `/r680/d455/imu`。
显式选择底盘 IMU 配置时，读取该配置的 `rostopic`，不把两者混用。

`odometry_adapter` 只执行既有 `T_imu_base` 刚体转换：位姿、包含杆臂项的
线速度、角速度和协方差。它不订阅传感器、不执行估计、不重置官方节点，
也不包含年龄门控、健康状态机或运动权限。无效数值导致单条消息无法转换时
只记录错误，不锁定后续消息。

官方位姿与里程计输出本身遵循官方初始化条件。不得将这些输出伪装成
“视觉匹配健康”或“ZUPT 已接受”等官方未发布的私有状态。
RTAB-Map 直接订阅转换后的 `/d455_slam/odom`，不再经过旧质量门控。
已有 Nav2 生命周期启动、输入就绪监测、底盘权限与手柄接管仍属于机器人
集成层，不修改官方估计器行为。

原 `EstimatorStatus`、`ImuDiagnostics`、`tracking_healthy`、`mapping_ready`、
`motion_ready`、`gate_status` 和自定义 `reset` 服务不再提供。
启动器和原有可选运动初始化辅助程序改为读取官方输出及实际传感器话题，
不伪造兼容状态。运动辅助程序只有显式启用时才运行，其原有运动边界保留。

## 编译依赖

Orin 的 OpenCV 开发库为 4.8；系统 ROS `cv_bridge` 二进制链接 4.6。
为避免一个进程混用两版，使用官方 `ros-perception/vision_opencv` 的
`cv_bridge 4.1.0`（提交 `f5b738d9694f0cee5904440d03912fc249943f8a`），
在 `.runtime/openvins_official/cv_bridge_install` 单独编译。
官方 OpenVINS 链接该依赖；launch 只给 OpenVINS 子进程设置对应库路径，
不向 RTAB-Map、相机驱动等其他节点注入此覆盖。

本轮只进行源码、接口和构建检查；未启动 ROS 测试、相机或实车运动。
