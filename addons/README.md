# 导航配套源码

根目录是 `wla_r680_navigation` ROS 包；`addons/COLCON_IGNORE` 避免 colcon 重复发现嵌套包。配套包需要复制到 ROS 工作区 `src/` 下作为导航包的同级包构建。

- `wla_diff_mpc/`：差速 OSQP MPC Nav2 插件，包含停车后对齐、终点切换及隔离控制器回归。
- `wla_cuvslam_navigation/`：cuVSLAM 双目＋底盘 IMU 前端，包含可信视觉来源区分及前端规则测试。
- `cuvslam-sdk/`：前端所需的 cuVSLAM v17 API 补丁与原始文件校验清单。

将两个配套包放入工作区后，按 [MPC 说明](wla_diff_mpc/README.md) 配置 Eigen3、OSQP、OsqpEigen 并构建插件。cuVSLAM 模式需先按 [SDK 补丁说明](cuvslam-sdk/README.md) 修补、重建 SDK，再按 [前端说明](wla_cuvslam_navigation/README.md) 构建前端与导航包。

这些目录归档的是本次 Orin 使用的源码，不包含 build/install、SDK 二进制或实机运行产物。软件回归通过不等同于实车运动验收。
