# 车身 IMU 200 Hz 修正与 cuVSLAM 复测

日期：2026-09-30。设备：orin@172.31.18.156，ROS 2 Jazzy，ROS_DOMAIN_ID=73。

## 已修正的原有代码

| 位置（Orin） | 修正 | 验证 |
|---|---|---|
| `~/r680_chassis_candidate_ws/src/turn_on_wheeltec_robot/src/Quaternion_Solution.cpp` | 去掉固定 20 Hz 积分步长，使用本次有效串口反馈间隔；无效、非正或大于 0.2 s 的间隔不积分 | 20/200 Hz 同一秒转动积分一致；抖动间隔、异常间隔和非有限值测试通过 |
| 同包 `src/wheeltec_robot.cpp`、`src/v650_wheeltec_robot.cpp` | 将 `Sampling_Time` 传入姿态解算 | 实际构建目标 `wheeltec_robot_node` 编译通过；原反馈连续性测试通过。v650 源码同步更新接口，但不是本次运行目标 |
| `~/ros2_ws/src/wla_r680_navigation/src/imu_conditioner.cpp` | 从“收到 100 个样本就完成”改为连续静止至少 5 s 且至少 100 个样本；同时检查消息时间与单调墙钟时间；运动、时间回退或采样中断重置采集 | 20 Hz 首次输出 5.053 s；200 Hz 首次输出 5.007 s；运动打断后重新等待通过 |
| 同包 `launch/bringup.launch.py` | 显式设置 `calibration_duration_s: 5.0`；修正旧的“只融合 Z 轴”注释 | 编译及安装完成 |

本地 Git 提交：底盘 `7fa14a9`；导航包 `e52d517`。
独立 cuVSLAM 验证包提交：`5e0eb51`。
未修改 Agent 代码。导航仓库原有 `nav2.yaml`、RViz、Web 的未提交修改保留。

## 其他 IMU 消费链路

```text
原建图/导航：车身原始 IMU → imu_conditioner → Madgwick → EKF / RTAB-Map
独立 cuVSLAM 验证：车身原始 IMU + D455 左右红外图像 → cuVSLAM Inertial
```

- Madgwick 使用消息时间戳计算 dt；`constant_dt=0.0`。不需要按频率乘十或除十修改增益。
- 实际原始 IMU → conditioner → Madgwick 测试：输出各 2057 条，输出时间戳频率均为 206.54 Hz；校准等待 5.042 s。短窗口频率随反馈调度变化，长窗口约 200 Hz。
- 校正后静止三轴陀螺均值约 `[-1.44e-5, 6.12e-6, -2.82e-5] rad/s`。
- `config/ekf_vo_imu.yaml` 的 30 Hz 是融合输出频率，保留；输入 IMU 约 200 Hz 不要求输出同频。
- `imu0_queue_size: 20` 保留。在 30 Hz 更新间隔内通常收到约 6～7 条 IMU；本次未进行带 Nav2 负载的 EKF 压力测试。
- 原驱动四元数的 20 Hz 问题不会直接改变 cuVSLAM 输入：cuVSLAM 读取原始角速度/加速度，conditioner 也明确丢弃原驱动姿态。
- 原始时间戳仍是上位机发布时间，尚不是 MCU 采样时间。外参、时间偏移、噪声密度没有被本次频率修改视为已标定。

## cuVSLAM 验证配置与结果

独立包：`~/ros2_ws/src/wla_cuvslam_validation`。
SDK：cuVSLAM v17.0.0，固定源码 SHA `57f42cc92d93eef47577d726789852a350b2a369`。

默认改为图像 30 Hz、IMU 标称 200 Hz；未插值伪造 IMU。
排障使用同步 SBA。异常位姿增量超过 2 m/s 或 4 rad/s 时停止测试位姿输出；这些是缓慢采集测试限值，不能代替生产导航安全验证。
输入类型、重复时间戳、非有限 IMU、缺内参禁止输出等回归测试通过。

| 数据目录（`~/nav_run/cuvslam_validation/`） | 结果 |
|---|---|
| `run-20260930-161311-14325` | 90 s 旧异步诊断：出现惯性初始化日志后严重发散，验证失败；不能只凭 `IMU INIT DONE` 判定可用 |
| `run-20260930-162807-17891` | 第一轮 60 s 人工采集；IMU 200.02 Hz，左右目 29.98 Hz。图像主要为近距离白墙；同一数据的纯双目和惯性模式回放均出现不合理位姿增量 |
| `run-20260930-163616-20020` | 有纹理场景 60 s：1791 帧，跟踪丢失/异常/IMU 覆盖不足均为 0；最大位移约 9.7 mm，未完成惯性初始化 |
| `run-20260930-164229-21212` | 复用手柄底盘 120 s：3590 帧，跟踪丢失、异常、输入丢帧及 SDK IMU 覆盖告警均为 0；最大位移约 0.270 m，轮速最高 0.466 m/s；惯性初始化仍未完成 |
| `run-20260930-164753-22274` | 180 s：5390 帧，跟踪丢失、异常、输入丢帧均为 0；最大位移约 0.327 m；惯性初始化仍未完成。录制的 8977 条 `/cmd_vel` 中 `angular.z` 均为 0，需要核对操作窗口与手柄转向链路 |

120 s 测试的 cuVSLAM 进程：平均 CPU 35.97%（100% 为一个 CPU 核），最大 RSS 510.59 MiB；Track 耗时 P50 8.71 ms、P95 20.96 ms；输出图像时间戳年龄 P95 47.05 ms。
这些数值只对应测试里程计进程，不是整套导航资源占用。
最后 180 s 测试：CPU 36.31% 单核、RSS 最大 529.28 MiB，Track P95 22.96 ms，输出年龄 P95 47.11 ms。

当前结论：200 Hz 输入及稳定视觉跟踪已验证；**真正完成惯性初始化的 VIO 尚未验证通过**。测试节点均已退出，操作员自行启动的手柄与底盘节点保留。

SDK 当前初始化状态机要求 25 s 滑窗内至少 20 个关键帧、跨度至少 2 s。
静止或一次短直行不保证满足条件。该门槛未被降低；是否完成惯性初始化必须结合重力输出、SDK 日志及初始化后的运动稳定性判断。

## 复用手柄进行采集

手柄、底盘已经启动，原始话题为 `/imu/data_raw` 和 `/odom` 时：

```bash
cd ~/ros2_ws
./r680_vio_test.sh --start-camera --use-running-chassis \
  --imu-topic /imu/data_raw --wheel-odom-topic /odom \
  --teleop --record --seconds 180
```

已有 `/r680/d455` 相机驱动时省略 `--start-camera`。
看到终端 `VALIDATION ONLY` 后先静止 5 s，再在可活动范围内低速往返、小幅左右转向，最后停稳。
脚本不发布电机命令；复用现有底盘时不会打开串口，也不会关闭操作员的手柄或底盘节点。
相机和 IMU 外参标定可复用，但 VIO 每次进程重启仍须初始化启动时的重力、速度、偏置状态。

```text
run-<ID>/
├── run.json               # 真实输入话题、模式、命令和进程
├── validation.yaml        # 配置快照
├── camera_original.json   # 临时相机设置恢复依据
├── tracker.log            # 包括 SDK 初始化与跟踪日志
├── bag.log
├── statistics.json
├── resources.json
└── sensors/               # 原始左右目、内参、IMU、轮速、TF、cmd_vel
```

原始传感器数据保留，不通过修改原始时间戳或删除失败结果来宣称测试通过。
独立回放命令使用 ROS_DOMAIN_ID=74，不启动底盘或相机硬件：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/.runtime/cuvslam_validation/ros_install/setup.bash
ros2 run wla_cuvslam_validation replay_validation.py \
  ~/nav_run/cuvslam_validation/run-<ID>/sensors --seconds 12
```

参考：[NVIDIA cuVSLAM 排障说明](https://github.com/nvidia-isaac/cuVSLAM/blob/v17.0.0/TROUBLESHOOTING.md)。
