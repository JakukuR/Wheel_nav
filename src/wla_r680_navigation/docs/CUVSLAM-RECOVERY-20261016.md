# cuVSLAM 恢复逻辑修复与当前链路

基于 Orin `/home/orin/ros2_ws` 的 `fd60143` 修复。官方对照：Isaac ROS `9058506efd0b967ac894a717bdbe98df017637e5`，运行 SDK `57f42cc92d93eef47577d726789852a350b2a369`（17.0）。本工程仍是 SDK17 的 ROS 适配，不宣称替换成配套 SDK15 的完整官方 ROS 节点。

## 本轮改动

- 删除自定义两秒健康窗口、重力模长/bias幅度阈值，以及速度有效性对 VIO 初始化的门控。保留 SDK gravity/IMU state 的可用性与有限值检查，作为本车导航初始化证据；这个 Bool 适配接口不是官方 ROS 原生接口。
- 速度改为官方 `PoseCache::GetVelocity()`；删除相邻帧差分、低通、200 ms 速度门控及其旧状态。空位姿后清 PoseCache，恢复第一帧速度为零，不再跨丢失段用旧状态计算。
- IMU 注册异常按样本捕获；Track、状态查询异常按帧处理并继续。无效位姿跳过。真正不可恢复的 worker 异常退出节点并返回失败，避免“进程活着但计算线程已死”；launch 保留对实际进程退出的 respawn。
- 关闭配置中的初始化强制关键帧，交由 SDK 选择关键帧。
- cuVSLAM 健康 false/超时只关闭导航准入，不再因3条false或1.5秒超时发送SIGTERM。输入恢复后，用同一估计器、同一odom坐标系重新检查地图证据。
- cuVSLAM 位姿跳变、无效位姿、轮速持续分歧仍关闭导航，复用已有地图恢复流程，但不重启前端。RGB-D前端的原重启路径保持不变。
- Web把“位姿TF”“里程计数据”“导航就绪”分别显示；数据收到不等于定位通过。

## `--cuvslam --no-degraded-odom` 的顺序

1. D455驱动提供双目红外图像、CameraInfo和光学TF；底盘提供 `/wheel/imu/data_raw`。相机仍只有一个owner。
2. 适配层检查消息格式、有限值、时间顺序；双目1 ms配对；IMU按图像时间送给SDK。现有图像年龄0.5 s与IMU gap 0.15 s仍保留；这是工程时序检查，不是官方初始化等待时间。
3. SDK执行 `RegisterImuMeasurement()` / `Track()`。单样本/单帧异常不结束估计器；空位姿不发布该帧。SDK正常输出惯性传播位姿时，这个命令分支允许继续发布。
4. 有效且新鲜位姿进入官方PoseCache；由它计算速度和pose covariance，VelocityCache计算twist covariance。SDK内部异常协方差只记录诊断，不复制成odom协方差，也不据此杀进程。
5. 发布 `/d455_slam/odom` 和 `d455_floor_odom → r680_mapping_floor`。SDK已有可用且有限的gravity/IMU state时发布初始化健康，不附加两秒或速度样本等待。
6. RTAB-Map结合RGB、对齐深度、odom和已有地图，发布 `map → d455_floor_odom`、定位位姿与匹配证据。
7. watchdog检查odom/状态新鲜度、地图证据与连续性。短时健康false/超时保持估计器运行；不满足条件时导航准入为false。跳变等异常进入原地图恢复流程，要求新的匹配和一致位姿，原有地图恢复稳定时间仍为2 s。这与已删除的前端2 s健康窗口不同。
8. interface_monitor汇总传感器、VIO、watchdog/地图状态；command_guard继续要求新鲜速度、综合就绪和任务运动许可，满足后才输出导航速度，否则输出零。不会因此销毁VIO。

关闭降级时，不启动 `degraded_odometry`，所以该节点的IMU100 ms/轮速150 ms门限不参与此链路。启用降级时，原有视觉筛选、辅助传感器时效、时间/距离/角度预算仍有效；本轮没有扩大这些预算。

`--auto-vio-init` 仍是显式可选分支，运动0.20 m/s、0.50 rad/s，最长2 s，0.25 m/0.35 rad预算不变。默认命令不会自行执行激励动作。

## 故障行为

| 情况 | 估计器 | 导航与恢复 |
|---|---|---|
| SDK初始化未完成 | 继续处理输入 | 等待SDK初始化和地图就绪 |
| 单个IMU样本注册异常 | 警告，继续后续样本 | 仍按有效输出检查 |
| Track/状态查询异常或空位姿 | 跳过该帧，后续继续 | 当前健康false，后续有效结果可恢复 |
| 无消息或输出超时 | 保持进程，不发送SIGTERM | 关闭准入；消息及地图证据恢复后重新判定 |
| 有限但不连续的位姿/轮速持续分歧 | 保持同一odom源 | 锁定导航，等待地图重新验证 |
| 不可恢复的worker错误 | 明确退出失败 | launch可重启；不伪装健康 |
| RTAB-Map退出/定位证据过期 | VIO继续运行 | 地图准入失效，导航不能因odom有数据就通过 |

## 2026-10-27 更新：RTAB-Map 矩阵崩溃的根因与修复

10-16 四次崩溃均为 `Link.cpp:137::setInfMatrix() ... Linear information X should not be null! Value=0.000000`，
三次车辆自启动起位移不超过 2 mm（静止），一次在行驶 4.2 m 后刚停下；四次崩溃前 SDK 都输出了逐帧完全相同的位姿
（例：`nav-20261016-150101-635372` 连续 `t=[-0.0004,0.0002,0.0003]`）。

根因：`/d455_slam/odom` 的位姿协方差取自上游 `PoseCache::GetCovariance()`，它是最近若干帧位姿的样本方差
（E[xx]−E[x]E[x]），描述的是“最近动了多少”，不是不确定度。静止且姿态逐帧不变时转角方差正好为 0，矩阵奇异；
OpenCV `Mat::inv()` 对奇异矩阵返回全零，RTAB-Map 得到信息 X=0 而断言退出。同样的公式在静止抖动时给出约 0.07 mm
的“精度”、直线行驶时方差随速度增大，即使不崩权重也不对。原回归测试曾把静止时全零协方差写成期望值。

修复：前端改为发布固定名义协方差（`odom_linear_variance` 0.001 m²、`odom_angular_variance` 0.001 rad²，
twist 0.01；SDK 仅惯性递推时乘 `inertial_only_variance_scale`=100），矩阵始终对角正定、可逆；历史样本方差只写入
健康诊断的 `history_covariance_diag`。实现见 `wla_cuvslam_navigation/src/odometry_covariance.hpp`。
`degraded_odometry` 的视觉可信判断仍读取该协方差，本次未修改（实车链路使用 `--no-degraded-odom`）。

## 尚未解决的边界

（10-16 原文）RTAB-Map 0.22.1的矩阵断言当时尚无故障当帧完整输入矩阵/调用栈；根因与修复见上节。底盘IMU外参/噪声/时差仍是待完整标定的配置。

SDK libs相对官方有5处修改、2个新增文件，主要为 `IsLastPoseInertialOnly()` 和测试，并非原封不动；PoseCache/VelocityCache原文件保持官方字节一致。完整对照证据见本次审计归档。

## 验证结果（2026-10-16）

Orin 上前端与导航包均编译、安装完成。前端6项CTest通过，导航10项相关CTest通过（其中初始化授权集含10个场景），另有启动参数与Web状态语义2个专项脚本通过。

- 前端：feature_overlay、feature_preview_ros、upstream_frame_recovery、visual_pose_evidence、health_evidence、initialization_sampling。
- 导航：map_localization_evidence、frontend_map_watchdog、cuvslam_transient_recovery、cuvslam_startup_match、visual_odom_sensor_independence、cuvslam_startup_timeout、frontend_interface、recovery_evidence、relocalization_prior、cuvslam_initialization_authority。
- 新增恢复回归运行真实watchdog、注入合成ROS输入：连续健康false、超时恢复不重启；跳变后需要新的地图匹配才能恢复，估计器进程保持不变。
- 自动初始化与降级节点已兼容新状态名 `waiting_for_inertial_initialization`。初始化测试校验实际授权撤销和输出归零，移除已过时的“退出必须切回MANUAL”断言；底盘NAV/手柄共用输出的现有策略没有修改。
- 测试域为198且限制本机发现，没有启动底盘驱动、真实相机或真实导航运动。测试结束后无残留初始化/command_guard测试进程。
- SDK运行来源记录已纠正为 `source_modified=true`；运行时文件不纳入源码提交。

软件测试不等于实车验收；本轮未复现并解决RTAB-Map矩阵崩溃。`addons/wla_cuvslam_navigation` 为COLCON_IGNORE下的旧分发副本，实际编译源为同级 `src/wla_cuvslam_navigation`，本轮没有覆盖该归档。
