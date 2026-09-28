# 独立 Nav2 控制器包

`wla_diff_mpc/` 是差速车 OSQP MPC 插件。仓库根目录本身是
`wla_r680_navigation` ROS 包，因此 `addons/` 带有 `COLCON_IGNORE`，避免
colcon 把嵌套包当成根包的一部分重复发现。

部署到现有 Orin 工作区时，把插件作为导航包的**同级包**放置：

```bash
cp -a addons/wla_diff_mpc ~/ros2_ws/src/wla_diff_mpc
```

先按 [插件说明](wla_diff_mpc/README.md) 安装 Eigen3、OSQP 和 OsqpEigen，
然后构建 `wla_diff_mpc`。`config/nav2_mpc.yaml` 选择该插件；
`./r680_nav.sh --mpc` 才会使用它，默认导航仍使用 MPPI。
