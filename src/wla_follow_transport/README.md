# wla_follow_transport

2026-11-09 按当前源码整理。

C++ 节点 follow_transport（节点名 person_follow）负责 ROS 收发、轮速历史、采集时刻 TF、帧配对、IPC 和最终输出门。它启动 wla_person_follow.hybrid_worker；无内置模型推理或独立 MPC。输出仅支持 mux / 系统时间。

框架见 [工作区总览](../../docs/ros2_ws_architecture.md)，入口/参数/构建与检查见 [操作手册](../../docs/ros2_ws_operations.md)。
