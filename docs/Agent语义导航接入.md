> 2026-10-13 维护说明：本文中的 `.maintenance` 路径是历史记录。该目录无运行依赖，按用户要求清理；保留的原始测试报告、日志和哈希清单迁入 `/home/orin/ros2_ws/docs/validation/maintenance-history/`，按原分组名查找，完整列表见该目录 `manifest.json`。源码/安装备份压缩包和依赖缓存已清理，不能再按旧路径恢复。历史结果未在本次清理中重跑。

> 当前人物链已于2026-10-12改为相对靠近/跟随：不使用map/odom/Nav2，2026-10-13已移除Agent检测开关，NAV感知进程收到新鲜图像即检测，Agent仅管理人物执行，Agent反馈为detected/what/id/yaw/distance。实时B不再有map坐标；家具A存图和已存语义导航保留。详见 [当前接口与入口](Agent共享检测与YOLO模型接入.md)。下文先前人物Nav2/MPC、实时map反馈和旧启动参数仅为历史记录。新代码已构建不等于常驻底盘服务已重启或完成实车验收。

> 2026-10-12 当前感知链更新：YOLO/BoT-SORT 与实时深度/TF投影由 ros2_ws 的 nav_yolo_detector 承担；Agent 仅消费 nav 结果并按 B 白名单适配 observations/context。家具 A 与人物跟随共用 nav raw，导航意图/控制权机制保持既有接线。详见 [nav 统一检测与 Agent 反馈](Agent共享检测与YOLO模型接入.md)。后文中 Agent 作为默认 YOLO 发布者的描述是迁移前记录。

# ros2_ws 正式集成入口

2026-10-12，按用户要求将本轮语义意图、双白名单和网关接入现有 `/home/orin/ros2_ws`。本说明取代此前独立候选目录的启动说明；旧试验目录只作归档，不再是运行依赖。

## 代码位置

- `/home/orin/ros2_ws/src/wla_behavior_interfaces`：BehaviorTask、AgentSemanticCommand 等类型。
- `/home/orin/ros2_ws/src/wla_behavior_bt`：C++ 行为树执行器。
- `/home/orin/ros2_ws/src/wla_behavior_bridge`：Agent API、任务网关、最终速度门、launch 与 B 白名单。
- `/home/orin/ros2_ws/src/wla_r680_navigation`：A 家具白名单、地图采集与导航接线。
- `/home/orin/Wheel_Legged_Agent`：原本的 Agent 工程，保留动作注册表与感知代码归属，补入语义核心、退出处理和 CLI。本次没有再复制一套 Agent 运行源码。

在 `ros2_ws` 构建后，`source /home/orin/ros2_ws/install/setup.bash` 自动加载 Agent 核心路径与实时白名单路径。桥接包的 `WLA_AGENT_SOURCE_DIR` 构建变量指向正式 Agent 仓库，默认 `/home/orin/Wheel_Legged_Agent`。不再 source 独立试验工作区。

配置源：

| 配置 | 文件 |
|---|---|
| A 家具检测四类 | `src/wla_r680_navigation/config/yolo_mapping_whitelist.json` |
| B 实时检测原有17类 | `src/wla_behavior_bridge/config/yolo_realtime_whitelist.json` |
| 地图动作绑定与接近策略 | `config/agent_semantic/actual-map-preview-policy.json` |
| 三个基本查询/导航请求 | `config/agent_semantic/list-map.json`、`list-visible.json`、`navigate-table-preview.json` |

ROS运行使用安装后的白名单。修改源码配置后重建对应包；`WLA_REALTIME_WHITELIST` 环境钩子使已有 Agent 检测入口也使用 B 配置。显式 `--whitelist`/`--classes` 仍兼容。Agent 仓库中的原配置只作未加载ROS环境时的兼容回退。

## 直接测试

第一个终端：

```bash
cd /home/orin/ros2_ws
./r680_agent.sh preview
```

第二个终端：

```bash
cd /home/orin/ros2_ws
./r680_agent.sh query map
./r680_agent.sh query visible
./r680_agent.sh query navigate
```

默认沿用正式系统 ROS domain 73，尊重已设置的 `ROS_DOMAIN_ID`。preview 不派发Nav2任务，navigate查询返回绑定结果。没有实时检测源时 visible 是空数组。`WLA_AGENT_MAP_DIRECTORY` 和 `WLA_AGENT_POLICY` 可切换匹配的地图与策略；不能只换地图不换策略。

## 导航接线

原 `r680_nav.sh` 的源码入口和工作区根入口都新增 `--agent-control`。在该模式下，原 RViz/Web 目标桥关闭，command_guard 输出改到 `/r680_nav/task_guard_input`，由任务速度门形成最后一跳。未加该参数时保留原有导航接线，已有手柄、OpenVINS、底盘常驻服务等改动保留。

统一入口：

```bash
./r680_agent.sh navigation --map map-2026-10-08-1 --no-rviz
```

此命令等价于正式导航入口加 `--agent-control`，原本的 `--enable-motion` 仍默认关闭。网关另用本次启动生成的 selected_map.json：

```bash
./r680_agent.sh bridge <本次导航运行的selected_map.json>
```

bridge 默认输出仍为 `/r680_nav/task_safe_preview`。完整实车接线需核对实际地图、足迹与定位、按主部署文档确认唯一输出并在停车时 claim 控制权；`output_topic:=/r680_nav/chassis_cmd_vel` 和原导航 `--enable-motion` 是显式部署参数。本次合入没有启动这些实车动作。

`./r680_agent.sh execute` 可启动允许已登记意图交接的 API；还需已有BT/网关、正确控制权和调用方持续心跳。不能同时启动 preview 与 execute 两个API，也不能同时启用 Engine 内嵌 ROS 执行器。`query REQUEST.json --follow-seconds N` 支持有界接口试验。意图格式、超时/取消语义及地图坐标有效性规则仍见《Agent语义意图接口与双白名单》。

## 构建与核验

```bash
cd /home/orin/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --executor sequential --packages-select \
  wla_behavior_interfaces wla_behavior_bt wla_behavior_bridge wla_r680_navigation
```

四包已在该工作区实际构建。验证只加载正式 `ros2_ws/install`，核对四包prefix、Agent导入来源、三个桥接可执行文件与安装文件、导航脚本/launch/白名单的字节一致性。实际评估 launch 的普通与Agent两种接线条件，不只做AST检查。

验证期间曾发现 `AllSubstitution` 列表应以 `container=` 传入；已修复并重建，保留失败记录。DDS链路与真实存图预览在独立ROS域199测试，源代码与安装仍来自正式工作区；该域隔离只是测试网络隔离，不是另建工作区。合成传感器/规划器测试不能当成实车验收。

备份与证据位于 `/home/orin/ros2_ws/.maintenance/agent_semantic_integration_20261012`。`.maintenance/COLCON_IGNORE` 阻止备份被递归构建。持续建图/探索切换仍为设计；本次没有修改服务或开启底盘输出。

## 2026-10-12 后续：共享模型接入完成，验证边界更新

旧模型路径缺失是前一阶段状态。现已接入用户提供的YOLO11n权重，默认由Agent推理一次，家具A、实时B、人物跟随分别筛选；跟随不再要求独立torch/YOLO。主导航地图身份与健康接口已增加显式绑定模式。

参见 [Agent共享检测与YOLO模型接入](Agent共享检测与YOLO模型接入.md)：包含当前路径、启动参数、27/99项回归、真实模型/DDS/map坐标回放证据、尚未达标的时延与GPU架构警告，以及无预存图时的坐标方案。真实相机与人物跟随运动仍未验收，前文历史日志/数字不改写。
