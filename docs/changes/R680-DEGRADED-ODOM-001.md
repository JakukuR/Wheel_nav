# 短时视觉退化：轮速＋车身 IMU 连续里程计试验

适用：Orin，ROS 2 Jazzy，`r680_nav.sh --cuvslam`。RGB-D 回退与手动建图的定位链不改动。此版本的软件验证不等同于实车退化场景已通过。

## 链路

```text
cuVSLAM（双目＋车身 IMU）→ /r680_nav/vio_raw_odom（不广播 TF）
                                          ↓
/wheel/odom 的实际 linear.x ──→ degraded_odometry（C++）
/wheel/imu/data_raw 的 gyro ──→         ↓
                              /d455_slam/odom ＋ d455_floor_odom → r680_mapping_floor
                                          ↓
                                   RTAB-Map / MPC / Nav2
```

RTAB-Map 仍独立发布 `map → d455_floor_odom`。降级节点是导航模式下唯一的 odom→车体 TF 发布者，不新增相机订阅、不启动另一个 EKF。轮速和 gyro 在降级时进行平面运动积分，不对加速度二次积分；这是紧耦合 VIO 外部的短时补位，而不是把轮速加入 cuVSLAM 内部优化。

## 状态与门控

1. 启动：必须先完成原 VIO 初始化、视觉观测和真实地图匹配。不能用轮速补位绕过出生重定位或自动初始化。
   为避免costmap等待TF、初始化等待安全节点激活的相互等待，初始化期间仍转发SDK已有的有效原始位姿/TF，但健康为false；仅原有受限运动初始化监督器可使用其测量，不开放普通导航运动。
2. 视觉正常：转发 VIO 位姿、体坐标速度，保留连续 odom 坐标系。
3. 低纹理退化：仅在故障前地图门控健康、轮速/IMU 新鲜、已取得静止 gyro 零偏时允许；不立即杀前端。
4. 降级：最高命令 `0.15 m/s / 0.30 rad/s`，统一缩放线/角速度以保留请求曲率。只接受当前碰撞检查后的命令；零速、取消、命令超时、运动授权撤销和障碍数据失效仍有效。
5. 返回：至少 3 个不同图像时间戳的视觉位姿，有有效惯性证据和当前帧视觉观测，且与轮速/gyro预测一致。保留 odom gauge，避免局部 TF 跳变；后端再进行地图校正。
6. 失败：超预算、轮速/IMU过期、SDK重力/零偏异常、无效位姿、恢复位姿不一致等立即停止健康输出，现有看门狗接管停车、重启、地图验证。停止后必须车身静止且原 VIO 连续健康 2 秒，补位节点才重新提供里程计；它不能单独解开看门狗的地图定位锁。

正常返回的 3 帧是短时局部连续性验证，不等同于之前分级全局重定位的 RGB-D 几何匹配。RTAB-Map 的频率和分级恢复参数未修改。

## 参数与依据

配置：`/home/orin/ros2_ws/src/wla_r680_navigation/config/degraded_odometry.yaml`。

| 参数 | 初值 | 作用／依据 |
|---|---:|---|
| `enabled` | true | 本版 cuVSLAM 导航启用；false 禁用补位 |
| `max_degraded_s` | 2.0 | 短时退化试验预算 |
| `max_degraded_travel_m` | 0.30 | 实际轮速积分的累计绝对路程，倒车不会重置 |
| `max_degraded_turn_rad` | 0.80 | gyro积分累计绝对角度，反向转向不会重置 |
| `imu_timeout_s` / `wheel_timeout_s` | 0.10 / 0.15 | 基于当前200Hz反馈的试验新鲜度限制 |
| `raw_timeout_s` | 0.15 | 原始位姿／健康证据新鲜度 |
| `visual_return_translation_m` / `visual_return_yaw_rad` | 0.12 / 0.15 | 返回视觉与独立预测的一致性限制，含图像延迟补偿 |
| `visual_return_frames` | 3 | 连续不同时间戳的返回位姿，重复/旧帧不计数 |
| `max_position_sigma_m` / `max_yaw_sigma_rad` | 0.10 / 0.15 | 保守增长模型上限；未声称是真实误差测量 |

位置／角度不确定度模型初值0.02，叠加退化前位姿协方差，再随时间与累计运动增长；这是试验启发式，轮胎打滑、地面和标定误差需要实测，不能当作有标定依据的概率保证。原始位姿协方差不能在补位时降低。预算包含最近可信图像到检测退化之间的时间间隔。任一预算达到就锁定，视觉重现不会在验证前重置预算。

IMU仍沿用“车体中心、同轴刚性安装”的候选外参，`base_from_imu_rotation` 与 `cuvslam.yaml` 的 `base_from_imu` 旋转必须保持一致。静止至少1秒、100个以上有效样本，gyro z均值绝对值≤0.25rad/s、标准差≤0.03rad/s后估计零偏；行驶时不学习零偏。尚未取得零偏则允许正常VIO导航，但不允许降级运动。

SDK新增：`/r680_nav/vio_inertial_valid`、`/r680_nav/vio_visual_observed`、`/r680_nav/vio_health_reason`。视觉观测初值20（`config/cuvslam.yaml: minimum_visual_observations`），要求当前 SDK 状态时间戳与位姿时间戳相同、非warming-up。观测数不是几何内点数。SDK仅靠IMU仍输出位姿时，不能将其当作恢复视觉并无限重置预算。原两秒初始化健康判据保留。

地图最近匹配的新鲜度最多在有效降级窗口内增加2秒；不更新“最近可信地图位姿”锚点，不能把补位预测当成新的视觉地图证据。原始位姿依然保留供初始化与诊断。

## 使用与回退

```bash
cd ~/ros2_ws
./r680_nav.sh --map map-2026-10-08-1 --test-recovery --cuvslam --auto-vio-init --enable-motion

# 关闭补位；恢复视觉失效即停车
./r680_nav.sh --map map-2026-10-08-1 --test-recovery --cuvslam --auto-vio-init --enable-motion --no-degraded-odom
```

每次启动将实际补位配置复制到 `~/nav_run/nav-<ID>/degraded_odometry.yaml`，并从该快照启动。修改源码配置后需要重新构建安装；本次运行快照不随源码编辑变化。`--rgbd` 完整回退入口继续可用。

观察：

```bash
ros2 topic echo /r680_nav/continuous_odom_status
ros2 topic echo /r680_nav/odom_degraded
ros2 topic echo /r680_nav/vo_watchdog_status
```

日志关键字：`visual degeneration`、`visual tracking returned`、`continuous odom blocked`。状态为 `visual`、`wheel_imu_degraded`、`blocked`、`waiting_initialization`，同时带累计时间、路程、转角与零偏就绪情况。

## 验证

- Release编译新C++节点及SDK接口；静态检查launch、shell脚本。
- 平面直行/弧线/倒车积分，时间/路程/转角预算，协方差增长，候选一致性、保持曲率的限速测试。
- ROS_DOMAIN_ID=174隔离图：假传感器，真实补位节点＋看门狗＋接口监控＋命令门控，硬件输出关闭；16个场景验证正常启动/预热TF、低纹理连续输出、三帧恢复、碰撞零速、重复帧不恢复、超预算停车、IMU/轮速过期、恢复不一致、前端硬错误、SDK纯IMU输出不重置预算、惯性证据异常、禁用补位、缺地图锚点、缺零偏、初始不确定度过大、补位节点退出后的零速，以及停车后的地图锁保留。
- 原恢复与初始化回归。
- 未启动本次实车导航或发送电机命令；需由操作者后续验证真实白墙转向、返回视觉和停止距离。预算不保证能跨越整个长白墙或完成90°转向。
