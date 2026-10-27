# 当前部署入口

当前包在主工作区 `/home/orin/ros2_ws/src/wla_person_follow`，共用 `FollowEngine` 算法。
默认 `transport:=cpp` 启动 C++ ROS 节点和独立 Python 计算进程；`transport:=python`
使用原 `SimulationFollow` 适配器。`sim` 是历史模块名，不生成仿真真值。
旧 legacy 后端、独立物体包和旧 QP 服务已移除。完整接口见 [README](README.md)。

配置 `config/sim_follow.yaml`；启动 `r680_person_follow.sh run`，默认关闭运动。
入口只 source 主工作区 install。数字模式在任务命令选择，不需重启节点切换模式。
使用相同 request_id 改模式将被拒绝；新 request_id 原子替换任务并丢弃旧 worker 结果。
新版本未自动重启，历史 SIM_CORE_PROVENANCE.json 只说明初始算法来源，不代表当前源码哈希。

C++ 入口需要同时构建 `wla_follow_transport` 与 `wla_person_follow`。
混合入口当前只支持 `output_mode:=mux` 和系统时间；直接速度输出或仿真时钟请显式
使用兼容适配器，错误配置不会静默降级。见 [拆分协议与验证](HYBRID_TRANSPORT.md)。
