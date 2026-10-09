# 分阶段视觉重定位恢复

2026-10-09，导航包基线 2213c2d。只调整实车 ROS 2 恢复链，未修改 Agent、MPC 或地图文件。

## 链路

```text
故障 → 运动锁定 → 重启前端 → VIO 稳定且轮速确认停车
    → 最近可信 map 位姿＋短时轮式位移提示
    → 1 m 附近空间匹配（8 s）
    → 3 m 扩大空间匹配（12 s）
    → RTAB-Map 原生 BoW 全局地点检索（30 s）
    → RGB-D 局部特征 3D→3D 几何配准通过
    → 至少 3 个不同时间戳图像的匹配位姿一致
    → 恢复原 RTAB 参数，连续稳定 2 s → 解锁
```

使用已安装 RTAB-Map 0.22.1：空间阶段调整 `RGBD/LocalRadius`、`RGBD/ProximityBySpace`、方向范围及路径比较数量；发送 `/d455_slam/initialpose` 作为中心提示。这个消息的协方差不会让 RTAB 自动扩大搜索，真正的半径由参数控制。

空间阶段把 `Rtabmap/LoopThr` 和 `RGBD/AggressiveLoopThr` 暂设为 1，优先空间配准，并在看门狗中检查结果落在当前搜索半径内。RTAB 的 BoW 计算仍存在，这不是关闭检索计算。全局阶段恢复原词袋接受阈值、关闭空间匹配，使用数据库中的视觉词检索候选。没有新增 HF-Net、云端模型或额外相机订阅。

恢复期间暂用 `Reg/Strategy=0`、`Vis/EstimationType=0`，即深度三维特征配准；保留原最小内点数、内点距离和图优化拒绝条件。候选的后验分数不算匹配证据，只有 RTAB 发布的已接受 loop/proximity 匹配与同期有效 localization_pose 才计数。匹配、位姿时间戳差不超过 0.25 s，同一时间戳不重复计数；停车时连续结果变化不超过 0.08 m、0.10 rad。地图 TF、协方差、轮速及前端新鲜度检查继续生效。

全局阶段允许经重复几何验证的地图位姿修正超过旧的 0.40 m 锚点一致性门槛，否则 VPR 即使找对地点也永远无法解锁；它仍要求故障期间的轮式位移在 0.50 m、0.80 rad 短时范围内且车辆停稳。扩大范围不是仅靠增加初值协方差，也不是直接把候选位置当真实定位。

阶段切换清除旧匹配证据；参数服务失败、地图结果不稳定或搜索超时保持停车。所有阶段失败后恢复原参数并要求人工定位，在锚点有效、轮式位移仍满足短时上限时，可向 `/d455_slam/initialpose` 给初值重新尝试验证；没有有效锚点或车辆已被搬动时需要重新启动导航并人工定位。结果验证后原参数恢复成功才允许解锁。进程识别额外检查 ROS_DOMAIN_ID，隔离测试不会给另一 ROS 域的前端发送信号。

## 配置与查看

源码：`src/vo_watchdog.cpp`；计数与稳定判定：`include/wla_r680_navigation/recovery_evidence.hpp`。
参数：`config/vo_watchdog.yaml`，新增 `staged_relocalization_enabled`、`recovery_*`。
诊断：`/r680_nav/relocalization_status`，日志关键字 `relocalization stage`、`VPR candidates`、`geometric match`、`verified_frames`、`motion health restored`。

启动命令不变，需退出旧导航后重新启动才能加载更新的看门狗。例如：

```bash
cd ~/ros2_ws
./r680_nav.sh --map map-2026-10-08-1 --test-recovery --cuvslam --auto-vio-init --enable-motion
```

回退阶段控制：把 `staged_relocalization_enabled` 改为 false，重新启动导航，恢复原来一次初值提示与锚点一致性验证。完整回退用 git revert 本次提交并重新编译导航包。

## 验证边界

软件测试使用隔离 ROS 域与模拟前端/地图输入，不发送电机命令；真实 RTAB 参数 API 验证使用临时数据库，没有驱动实车。没有声称已通过实际环境的 VPR 重定位或错误地点排除测试。真实遮挡与场景重复仍需要实车验证。

实际验证结果：导航包 Release 编译成功且无编译警告；recovery_evidence、relocalization_prior 两项 CTest 通过；隔离域中的成功恢复、VPR 全部失败保持锁定、人工初值重新验证测试通过；原一次初值恢复及 RGB-D/cuVSLAM 健康门控回归通过；安装版 RTAB 0.22.1 的真实参数服务和内部 applyParameters 更新验证通过（临时数据库，无硬件输入）。源地图数据库只读检查包含 560 个节点、156180 个视觉词，具备原生 BoW 检索数据。未进行实车遮挡恢复测试。
