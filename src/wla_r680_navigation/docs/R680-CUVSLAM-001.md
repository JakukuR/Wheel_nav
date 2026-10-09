# R680-CUVSLAM-001：cuVSLAM 可选导航前端

日期：2026-09-30。原导航基线：`e52d517`。默认前端保持 RGB-D VO + EKF。
保留原有未提交的 nav2.yaml、r680_mapping.rviz、web/app.js、web/style.css。

## 改动

新增独立 `wla_cuvslam_navigation` C++ 包，使用固定 cuVSLAM v17 SDK。
`r680_nav.sh --cuvslam` 选择该前端，`--rgbd` 回退，未指定时仍用原路线。
不修改 Agent、底盘驱动或地图原数据库，不改变 MPC 和局部 Scan 配置。

- 开启左右红外图像和车身原始 IMU；不运行旧 VO、EKF、IMU conditioner/Madgwick。
- 唯一前端输出 `/d455_slam/odom` 和 `d455_floor_odom -> r680_mapping_floor`。
- 车体坐标系速度由相邻位姿估计，不把未知速度假设成零速测量。
- 惯性初始化、跟踪、重力/偏置稳定窗口与 RTAB 地图匹配共同决定运动健康。
- 看门狗不伪造 RTAB OdomInfo，只重启匹配到的当前前端进程。
- 保存前端选型、配置快照和统计；运行失败后退出，原路线可独立重新启动。
- 启动时检查独立底盘驱动，避免串口被两个驱动打开。

## 已执行验证

1. 两个 ROS 包 Release 编译通过。
2. CTest：车体坐标系速度、角速度符号、非法时间间隔、连续初始化窗口通过。
3. 隔离 ROS_DOMAIN_ID=74、禁止进程重启的看门狗回归通过：
   旧 OdomInfo 路线、未初始化锁定、未匹配地图锁定、有效匹配放行、跟踪失效关闭。
4. 真实输入预览（复用已运行底盘反馈，硬件输出关闭，无导航目标）：
   `/home/orin/nav_run/cuvslam-nav-preview-20260930-175114`。
   双目/odom 约 29.61 Hz，车身 IMU 200.74 Hz，局部 Scan 8.60 Hz。
   1272 跟踪帧，丢失、无效位姿、IMU gap、图像队列丢帧均为 0。
   Track P50 3.74 ms、P95 8.99 ms；输出年龄 P95 35.55 ms。
   map/odom 两段 TF 连通，controller/planner/bt/smoother/collision 全部 active。
   车辆静止，gravity_frames=0，因此前端和运动健康始终 false：未验证惯性初始化成功。
5. 实际回退预览：`/home/orin/nav_run/rgbd-nav-preview-20260930-175320`。
   原 VO + EKF 路线输出约 29.54 Hz，TF 连通，五个 Nav2 节点全部 active，接口健康 true。
6. 上述测试没有发布电机命令，原有手柄与底盘会话保持运行。

首轮试验发现 RTAB launch 不能接受 `imu:=` 空重映射，已改为有效但无发布者的
`/r680_nav/disabled_rtabmap_imu`，RTAB 不再接收重复 IMU。首轮日志保留在
`cuvslam-nav-preview-20260930-174845`，不能作为完整导航栈通过的证据。
两条路线均存在既有 Smac 非圆形足迹/膨胀半径性能提示，本次不调整地图配置。

## 启动

先停止独立底盘/手柄会话，再运行（先不开放运动）：

```bash
cd ~/ros2_ws
./r680_nav.sh --map map-2026-09-29-1 --test-scan-only --cuvslam
```

完成初始化与地图定位核验后，运动测试命令：

```bash
./r680_nav.sh --map map-2026-09-29-1 --test-scan-only --cuvslam --enable-motion
```

当前尚未通过惯性初始化和实车导航，因此正式默认仍保持原路线。
回退必须先 Ctrl+C 停止试验，再运行：

```bash
./r680_nav.sh --map map-2026-09-29-1 --test-scan-only --rgbd --enable-motion
```

## 初始化与复用

当前 SDK 自动尝试惯性初始化；内部要求至少 20 个关键帧、覆盖至少 2 秒，
并使用最长 25 秒历史。20 个关键帧不等于 20 张图像，完全静止可能长期无法满足。
当前导出的公开 Odometry API 没有惯性状态保存/恢复或外部初始重力/偏置注入接口。
`Slam::SaveMap` 保存地图，不保存可直接恢复的当前 VIO 动态状态。

可复用：内参、相机/IMU 外参、时间偏移、噪声参数。重启后仍需重新估计当前
速度、重力方向和零偏，不能复制上次 initialized 标志来放行。
若希望一次初始化后切换建图/导航任务不重来，应后续将定位前端作为长期运行服务，
让任务启动/退出只管理规划控制；掉线重置或前端崩溃后仍需重新初始化。
本次没有新增后台自启动服务，也没有通过自动驱动车辆来获取初始化激励。
