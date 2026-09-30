# cuVSLAM + 车身 IMU 独立验证

只验证传感器输入与里程计，不向 `/cmd_vel` 发布、不发布 TF、不接管导航。
依赖 NVIDIA cuVSLAM v17.0.0，commit `57f42cc92d93eef47577d726789852a350b2a369`。
SDK 源码、依赖、编译产物位于 `~/ros2_ws/.runtime/cuvslam_validation/`，不提交 Git。

## 启动

已有 `/r680/d455` 相机驱动、底盘串口空闲时：

```bash
cd ~/ros2_ws
./r680_vio_test.sh --seconds 60
# 纯双目基线
./r680_vio_test.sh --stereo-only --image-decimation 1 --seconds 30
```

脚本临时启用 D455 两个 640×480@30 Hz 红外流、关闭红外投射器，退出时恢复。
相机参数通过一个原子事务批量更新两侧红外流，避免驱动同步器在单侧停流时卡住。
本测试目录使用文件锁，上一轮完成恢复前不能开始下一轮。
临时底盘驱动的速度和充电命令输入映射到测试专用空闲话题。
不要与导航、手柄底盘驱动并行打开串口；测试时保持车辆静止。
Ctrl+C 或达到时限后关闭本脚本创建的节点。

## 输入输出

| 类型 | 话题 | 用途 |
|---|---|---|
| 输入 | `/r680/d455/infra{1,2}/image_rect_raw` | 同步左右目 mono8 图像 |
| 输入 | `/r680/d455/infra{1,2}/camera_info` | 校正后内参与基线 |
| 输入 | `/r680_vio_test/chassis/imu_raw` | 车身原始角速度、加速度 |
| 输入 | `/tf`, `/tf_static` | 仅读相机驱动内部外参 |
| 输出 | `/r680_vio_test/odom` | 测试位姿；不用于当前导航 |
| 输出 | `/r680_vio_test/status` | 初始化、丢失、输入异常状态 |

输出 frame 为 `cuvslam_validation_odom → r680_mapping_floor`，不会广播此 TF。
仅输出位姿，twist 未测量并设置大协方差，不能当作底盘速度反馈。

## 配置与产物

`config/validation.yaml`：相机安装矩阵、IMU 外参、噪声、时间偏移和输入话题。
车身 IMU 暂按车体中心且轴向一致假设；没有把原始加速度先经过 Madgwick/EKF。
SDK 使用 `Inertial` 模式联合估计；`--stereo-only` 使用 `Multicamera` 模式。
默认 `image_decimation: 2`，将图像选帧到 15 Hz，IMU 保留真实约 20 Hz 的全部样本。
`--image-decimation 1` 可复现 30 Hz 图像相对于 20 Hz IMU 的覆盖不足问题。
`imu_state_frames > 0` 不是惯性初始化成功的证据；需要 `inertial_initialized: true`、
重力可用及 SDK `IMU INIT DONE` 日志，并继续验证运动与遮挡恢复。

```text
~/nav_run/cuvslam_validation/run-YYYYMMDD-HHMMSS-PID/
├── run.json             # 模式、命令、进程和环境
├── validation.yaml      # 配置快照
├── camera_original.json # 原始相机参数（自动恢复）
├── tracker.log          # cuVSLAM 日志
├── chassis.log          # 独立底盘反馈日志
├── statistics.json      # 跟踪、IMU 初始化、丢帧、耗时、静止漂移
└── resources.json       # CPU、RSS、线程采样；100% 为一个 CPU 核
```

车身串口反馈实际约 20 Hz，时间戳为上位机发布时间，尚无硬件采样时间戳。
噪声参数仅为静止样本估值，外参、时间偏移及完整 IMU 标定待验证。
静止通过不代表动态、遮挡或高速转弯可靠；不应直接替换生产导航里程计。

## 编译

SDK 源码与固定版本依赖准备完成后：

```bash
cmake --build ~/ros2_ws/.runtime/cuvslam_validation/sdk_build --target cuvslam --parallel 2
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon --log-base .runtime/cuvslam_validation/ros_log build \
  --packages-select wla_cuvslam_validation \
  --build-base .runtime/cuvslam_validation/ros_build \
  --install-base .runtime/cuvslam_validation/ros_install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCUVSLAM_SOURCE_DIR=$HOME/ros2_ws/.runtime/cuvslam_validation/cuVSLAM \
  -DCUVSLAM_BUILD_DIR=$HOME/ros2_ws/.runtime/cuvslam_validation/sdk_build
```
