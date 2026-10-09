# cuVSLAM + 车身 IMU 验证结果

日期：2026-09-30。设备：`orin@172.31.18.156`，ROS Jazzy / L4T R39.2.1 / CUDA 13.2.86。

**输入链路、C++ 节点与静止跟踪通过；完整惯性融合尚未验证成功。不能直接替换导航里程计。**

## 实际实现

```text
D455 左右红外图像 + CameraInfo + 相机内部 TF ─┐
                                          ├→ cuVSLAM Inertial → /r680_vio_test/odom
车身原始陀螺仪 + 加速度 ───────────────────────┘
底盘轮速 → 仅检查测试期间是否静止，不参与估计
```

使用 NVIDIA cuVSLAM **v17.0.0** C++ SDK，固定源码提交：
`57f42cc92d93eef47577d726789852a350b2a369`。
SDK 官方的 `Inertial` 模式使用双目与 IMU；未接入 HF-Net，未经过现有 EKF。
[官方模式说明](https://github.com/nvidia-isaac/cuVSLAM#tracking-modes)

独立包：`/home/orin/ros2_ws/src/wla_cuvslam_validation`。
配置：`/home/orin/ros2_ws/src/wla_cuvslam_validation/config/validation.yaml`。
SDK、依赖及 build/install/log：`/home/orin/ros2_ws/.runtime/cuvslam_validation/`，不提交 Git。

只发布测试位姿，不发布 TF、不发布速度命令。twist 标记为未测量，不能拿它当底盘速度反馈。
原导航入口、Agent 链路及导航包已有的未提交修改没有改动。

## 输入实测

| 项目 | 结果 |
|---|---|
| 左右目 | mono8，640×480，约 29.85 Hz |
| 双目同步 | 同帧时间戳一致，测得差值 0 ms |
| 双目基线 | 0.095073 m，由校正后投影矩阵提取 |
| 车身 IMU | 约 20 Hz，间隔约 50 ms |
| IMU 时间戳 | 上位机发布时间，不是下位机采样时间 |
| 静止加速度模长 | 约 9.7306 m/s²，完整标定待完成 |
| IMU 外参 | 暂按车体中心且轴向一致；相机安装沿用当前导航矩阵 |

相机安装 `base_from_camera_link` 为：
`xyz = [0.184428484, 0.059803590, 0.501660007] m`，
`xyzw = [0.000096773, 0.077125800, -0.000560247, 0.997021207]`。
再乘相机驱动的 `d455_link ← 左目光学帧` TF，得到实际左目外参。
IMU 噪声仅使用静止样本估值，时间偏移暂为 0；这些是验证配置，不是已完成标定的参数。

## 跟踪与资源实测

以下是静止测试，CPU 为**整个测试节点进程**，100% 对应一个 CPU 核；RSS 包含共享映射。
资源数据不包含相机驱动、底盘驱动、RTAB-Map、Nav2 或 Agent。

| 模式 | 时长 | 输出频率 | Track P95 | 输出帧年龄 P95 | 平均 CPU | RSS 峰值 | IMU 覆盖告警 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 纯双目，30 Hz 图像 | 30 s | 约 30 Hz | 2.56 ms | 35.71 ms | 10.6% | 492 MiB | 0 |
| 惯性模式，30 Hz 图像 + 20 Hz IMU | 60 s | 29.97 Hz | 4.66 ms | 50.83 ms | 14.5% | 507 MiB | 592 次 |
| 惯性模式，15 Hz 选帧 + 20 Hz IMU | 60 s | 14.98 Hz | 5.66 ms | 50.70 ms | 10.7% | 507 MiB | 0 |

三轮均没有跟踪丢失或非法位姿。轮速反馈线速度始终为 0，角速度最大约 0.001 rad/s。
后两轮静止最大位移分别约 7.1 mm、5.2 mm；这些结果不能用来评价行驶精度。

30 Hz 图像比 20 Hz IMU 快，部分图像间隔没有真实 IMU 样本。
SDK 将这样的间隔判为 IMU 覆盖不足；**592 条告警不等于实际丢了 592 个串口包**。
图像选帧到 15 Hz 后，每个处理间隔能够获得真实 IMU 样本，告警消失；没有插值制造高频 IMU。

两轮惯性模式的 `gravity_frames=0`，没有 `IMU INIT DONE`，SDK 日志保持 `imu_state=0`、`integrated=0`。
`GetImuState()` 返回值只说明状态对象存在，并不证明惯性融合初始化完成。
SDK 初始化还需要足够的有效关键帧，本次静止测试没有验证运动激励条件。
因此只能确认视觉跟踪和 IMU 输入有效，**没有证明遮挡时能靠 IMU 持续跟踪**。
完整融合开始后的开销也可能不同于本次数据。

## 启动与检查

在 Orin 上，已有 `/r680/d455` 相机驱动且底盘串口空闲时：

```bash
cd ~/ros2_ws
./r680_vio_test.sh --seconds 60

# 纯双目基线
./r680_vio_test.sh --stereo-only --image-decimation 1 --seconds 30

# 复现原始 30 Hz 图像 + 20 Hz IMU 的覆盖问题
./r680_vio_test.sh --image-decimation 1 --seconds 60
```

默认使用 15 Hz 选帧，并保留所有实际 IMU 样本。脚本不允许与现有底盘节点同时打开串口。
脚本临时启用两路红外并关闭投射器，结束或 Ctrl+C 后恢复相机参数并关闭自己的节点。
禁止同时启动两轮测试；文件锁保持到相机恢复结束。

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=73
ros2 topic echo /r680_vio_test/status
```

`tracking_visual_waiting_inertial_initialization` 表示尚未完成惯性初始化。
需要确认 `statistics.json` 的 `inertial_initialized=true`、重力可用及 SDK 初始化日志，
再继续验证运动与短时遮挡；仅看到 `/r680_vio_test/odom` 有数据不算通过。

## 产物与检查结果

```text
/home/orin/nav_run/cuvslam_validation/
├── input_probe.json               # 30 秒传感器频率、时间戳与静止统计
├── input_bag/                     # 约 14 秒，248 MiB，同步图像/IMU/轮速/TF 原始录包
├── input_guard_regression.json    # 图像格式、错误 frame、NaN、乱序输入拒绝检查
├── cleanup_regression.json        # Ctrl+C 清理与重复启动拦截检查
├── results_summary.json           # 三轮测试统计
├── run-20260930-132827-56225/      # 纯双目 30 Hz
├── run-20260930-133522-57226/      # 惯性模式 30 Hz
├── run-20260930-133948-57672/      # 惯性模式 15 Hz
└── run-20260930-134451-61234/      # 中断清理验证
    ├── run.json
    ├── validation.yaml
    ├── camera_original.json
    ├── tracker.log / chassis.log
    ├── statistics.json
    └── resources.json
```

输入保护检查通过。限时退出、Ctrl+C 退出、重复启动拦截及原子参数恢复均通过。
第一轮退出时，逐个关闭红外流使 RealSense 卡在 `Close Sensor`，已重启原相机服务恢复，
并将脚本改为一个原子事务更新两路红外配置。修改后两轮及中断验证都没有恢复错误。
最终恢复两路红外关闭、原红外 profile `848x480x30`、投射器开启，测试节点已关闭。

## 下一步

1. 采集小范围真实平移、转向数据，确认 SDK 产生足够关键帧并完成惯性初始化；本次没有控制车辆运动。
2. 工程使用优先让下位机输出至少约 100 Hz 的真实 IMU 与采样时间戳，再恢复 30 Hz 图像。15 Hz 选帧只是当前 20 Hz IMU 的验证折中。
3. 校验 IMU 轴向、外参、零偏、加速度比例和相机/IMU 时间偏移，然后测试短时遮挡与漂移。
4. 这些通过后再接 RTAB-Map，明确 odom TF 的唯一发布者与速度语义；当前测试输出不接生产导航。
