# wla_behavior_interfaces

2026-11-09 按当前源码整理。

公共 ROS 接口：BehaviorTask.action、AgentSemanticCommand.srv、PersonCommand.srv、ResolveTarget.srv、TaskLease.msg。PersonCommand.mode 的语义由跟随后端 task_modes.py 定义（0–4）；不要将客户端只暴露 0–2 当作消息类型限制。

框架见 [工作区总览](../../docs/ros2_ws_architecture.md)，入口/参数/构建与检查见 [操作手册](../../docs/ros2_ws_operations.md)。
