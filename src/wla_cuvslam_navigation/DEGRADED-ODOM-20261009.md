# 导航短时视觉退化的当前帧证据接口

配合 `wla_r680_navigation/degraded_odometry`，补充三种ROS诊断数据；保留原VIO初始化、重力/零偏检查、时间戳/有限值检查和健康判据。本文与导航包的 `docs/changes/R680-DEGRADED-ODOM-001.md` 对应。

- `/r680_nav/vio_inertial_valid`：当前位姿具有有效重力、IMU零偏与体坐标速度估计；无输出／错误状态为false。
- `/r680_nav/vio_visual_observed`：SDK当前帧观测数≥`minimum_visual_observations`（默认20），状态时间戳与位姿时间戳一致且非warming-up。观测不是几何内点证书，也不能代替地图匹配。
- `/r680_nav/vio_health_reason`：原健康诊断原因，区分低纹理、速度差分区间、重力/零偏异常。

SDK继续每帧处理图像与车身IMU。启用观测导出；现有默认 `initialization_diagnostics=true` 原本已经启用导出和GetState，因此默认运行不会重复GetState。诊断关闭时仍需导出视觉证据，不能静默把纯IMU位姿当作视觉恢复。

导航启动由上层launch覆盖输出到 `/r680_nav/vio_raw_odom` 并关闭本节点TF，由连续里程计节点唯一发布 `/d455_slam/odom` 和odom→车体TF。建图仍沿用原输出/TF路径，不启动短时补位。部署后须重新启动本次导航才加载新可执行文件。

验证：ROS2 Jazzy/aarch64 Release构建，原3个CTest；导航隔离域174软件图验证短时退化、持续SDK纯IMU输出不能刷新退化预算、视觉三帧返回、SDK惯性异常锁定等。没有发电机命令，没有声称实车白墙场景已验证。
