# 全局动态障碍与 MPC 制动测试入口

日期：2026-10-08。

## 启动

```bash
cd ~/ros2_ws
./r680_nav.sh --map map-2026-10-08-1 \
  --test-recovery --cuvslam --auto-vio-init --enable-motion
```

`--test-recovery` 加载 `config/nav2_recovery_test.yaml`，使用 MPC、局部 Scan 和全局限时动态层。它不能与 `--test-scan-only`、`--mppi` 同时使用。相机、Scan 转换、速度和局部地图尺寸沿用现有测试配置；全局规划仍是 SmacPlanner2D。

## 全局动态层

`TimedObstacleLayer` 接收 `/r680_nav/d455/points`，只接受经过分割的 `r680_mapping_floor` 点云，使用观测时间戳的 TF 转至 map。高度范围 0.04～0.50 m，水平范围不超过 4.5 m；消息过期 0.5 秒即拒绝。

同一地图格需要两帧观测确认。**最后一次观测后保留 2 秒**，不是发布一份永久标记的点云；过期后重建受影响区域，保留静态地图中的障碍。全局代价地图 2 Hz 更新，过期生效最多再等约 0.5 秒。定位健康变为 false 时丢弃旧定位代次的动态障碍。局部层继续使用 Scan 标记和深度射线清除，其清除机制不等同于这个全局 TTL。

参数路径：`global_costmap.global_costmap.ros__parameters.dynamic_layer`，包括 `persistence_s: 2.0`、`confirmation_hits: 2`、`max_observation_gap_s: 0.3`、`source_timeout_s: 0.5`。

这个入口关闭旧 `dynamic_obstacle_memory` 进程，避免两套全局记忆重叠。保持碰撞监控和运动门控。

## MPC 配合

MPC 改动见 `wla_diff_mpc/CHANGES-20261008.md`：按障碍距离和响应时间提前限速，连续检查非线性预测轨迹及整个足迹，必要时在周期预算内降低参考速度再解一次 QP。

这仍是路径跟踪 MPC，绕行路线由全局规划器生成；它没有加入完整的障碍物优化约束。危险或无法在预算内求解时继续输出零速并请求重规划。更新 MPC 动态库后，这部分逻辑也作用于其他使用该插件的配置；切换 YAML 不能回退动态库。

## 验证与回退

插件测试验证两帧确认、单帧不标记、过期删除、静态障碍保留、过期输入拒绝和定位故障清除。

`test/check_global_obstacle_replan.py` 在 ROS_DOMAIN_ID=74 只启动地图服务、规划器和生命周期管理器。真实 SmacPlanner2D 对合成动态墙生成横向偏移约 0.794 m 的绕行路径；停止输入并超过 2 秒 TTL 后恢复直线路径。没有控制器、底盘或电机指令。

关闭全局动态测试：使用原 `--test-scan-only`。撤销自动初值：关闭 `relocalization_prior_enabled`。完整代码回退应分别 revert 导航包及 MPC 包对应提交，再在 `~/ros2_ws` 编译两个包；不要 reset 当前未提交的用户文件。

未做实车运动验证；首次实车测试需要同时观察全局代价地图、规划路径、MPC 制动日志与最终底盘速度。
