# 当前部署入口

当前包在主工作区 `/home/orin/ros2_ws/src/wla_person_follow`，仅保留 `sim_node` 实现。
`sim` 是历史模块名；它接收真实观测，不生成仿真真值。
旧 legacy 后端、独立物体包和旧 QP 服务已移除。完整接口见 [README](README.md)。

配置 `config/sim_follow.yaml`；启动 `r680_person_follow.sh run`，默认关闭运动。
入口只 source 主工作区 install。数字模式在任务命令选择，不需重启节点切换模式。
使用相同 request_id 改模式将被拒绝；新 request_id 原子替换任务并丢弃旧 worker 结果。
新版本未自动重启，历史 SIM_CORE_PROVENANCE.json 只说明初始算法来源，不代表当前源码哈希。
