# explore_lite

2026-11-09 按当前源码整理。

目录 explore 的实际 ROS 包名为 explore_lite。使用 Nav2 代价地图搜索前沿、发送 NavigateToPose，发布 explore_lite_msgs 状态；mapping_mission 编排探索、返航和保存。它与 wla_explore_wander 不是同一入口。

框架见 [工作区总览](../../docs/ros2_ws_architecture.md)，入口/参数/构建与检查见 [操作手册](../../docs/ros2_ws_operations.md)。
