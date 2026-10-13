> 2026-10-13 维护说明：本文中的 `.maintenance` 路径是历史记录。该目录无运行依赖，按用户要求清理；保留的原始测试报告、日志和哈希清单迁入 `/home/orin/ros2_ws/docs/validation/maintenance-history/`，按原分组名查找，完整列表见该目录 `manifest.json`。源码/安装备份压缩包和依赖缓存已清理，不能再按旧路径恢复。历史结果未在本次清理中重跑。

# 人物相对执行与 Agent 接口（2026-10-13）

人物功能只有 `follow`（持续跟随）和 `approach_interact`（靠近交互）。人物和实时B反馈不再转换到map，不订阅odom或定位健康，不调用Nav2，也不预测盲走或自动换人。家具A的地图语义存储及基于存图的导航仍独立保留。

```text
D455 RGB → nav_yolo_detector（单一YOLO11n/BoT-SORT，随新鲜图像持续检测）
   ├─ raw detections → 家具A筛选 → 原有map投影/聚合 → semantic.geojson
   └─ 同帧对齐深度 + camera→robot外参 → relative_objects
        ├─ B白名单 → Agent visible_objects / observations / context
        └─ person执行节点 ← Agent PersonCommand（模式、目标ID、租约）
             + 本地深度障碍门 → 预览Twist + 带任务/时间戳的运动候选
             → 唯一command_mux（NAV/PERSON互斥、手柄优先）→ 底盘
```

## 持续检测与人物执行

| Agent端点 | 类型 | 作用 |
|---|---|---|
| `/wla_agent/person_command` | `wla_behavior_interfaces/srv/PersonCommand` | `start/heartbeat/stop/status`人物执行 |
| `/wla_agent/visible_objects` | `std_msgs/msg/String` | B白名单的相对检测JSON |

人物执行内部对应 `/r680/person_follow/command`。NAV 感知进程启动后收到新鲜 RGB 即执行检测，无图像时工作线程等待；不需要 Agent 触发。人物 start/heartbeat/stop 只管理人物任务，不控制模型推理，也不阻断家具 A 或实时 B 消费者。

2026-10-13 已移除 Agent 的 detection on/off CLI、RosPersonTransport 检测开关、Agent/NAV 两个 SetBool 检测服务及 `--start-enabled` 参数。检测生命周期由 `r680_perception.sh run` 所启动的进程管理，退出该进程即停止检测。单帧槽、源时效、推理限频、断流/时钟回退/模型异常的 session/epoch 保护继续生效，停止进程时在途结果不能发布。

服务请求字段为 `command, request_id, mode, target_id, lease_duration_s`。回复为 `accepted, reason, request_id, state, mode, target_id, enabled`。`start`原子绑定模式和ID；同request_id同参数幂等，不续租；同request_id不同参数拒绝，已结束请求不能复活。`heartbeat`只续匹配的活跃请求，租约0.1–2秒；`stop`可指定请求，空request_id表示显式全停。accepted表示任务服务受理，不代表底盘已经授权运动。facade超时返回unknown，不能当成已停止；任务自身仍受租约限制。

## Agent检测业务字段

```json
{"detected":true,"what":"person","id":"detectorSession:2:7","yaw":-0.21,"distance":1.8}
```

每个对象只有上述五字段。协议外壳保留源采集时间、session/epoch、enabled、fresh、base_frame与TTL，用于拒绝迟到包，未重新盖时间戳。enabled 保留为只读兼容字段，运行中的检测器发布 true，消费者仍按状态和观测时效判断可用性；它不再是可设置开关，true 也不表示相机新鲜或任务可运动。没有目标时整体`detected=false, objects=[]`；识别到但深度/外参不可用时，类别和ID仍有值，yaw/distance为null，不能启动人物运动。

默认base为`r680_mapping_floor`，是随机器人运动的底盘安装坐标系：x前、y左。`yaw=atan2(y,x)`，弧度、左正；`distance=hypot(x,y)`，米。名称含mapping不表示世界map坐标。只需要相机到机器人刚体外参，可在无图时使用；仍需真实RGB-D与外参有效。depth取可见表面，不是人体质心或身份认证。ID是检测session、epoch与BoT-SORT整数ID的复合键，Agent必须原样回传。

内层 `/r680/perception/relative_objects` 使用`label`，Agent外层使用`what`。B配置在`src/wla_behavior_bridge/config/yolo_realtime_whitelist.json`；A配置在`src/wla_r680_navigation/config/yolo_mapping_whitelist.json`。唯一模型仍为`/home/orin/Wheel_Legged_Agent/models/perception/yolo/yolo11n.pt`，SHA256 `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`，由nav进程加载。

## 相对动作行为

- follow默认保持1.40m；approach_interact默认0.95m，并连续三个新观测达到距离/朝向阈值后锁存arrived，保持停止。
- 速度上限0.20m/s、0.45rad/s；偏角较大先转向；距离太近不倒车。目标丢失、源数据超过300ms、深度不可靠或前方有障碍时立即零候选；同ID重新可见且租约仍有效时可继续。不会换绑另一个ID。
- 检测进程退出或观测过期时停止运动候选，检测代际变化取消执行。租约到期停住，迟到heartbeat不能复活。模式切换先零输出、清控制状态。
- 障碍安全仅用当前深度和camera→robot外参，保留人物作为障碍；没有绕障规划、拐角世界系预测、全景安全感知。

## 无图接口测试入口

相机与唯一的相机安装TF应由已有传感器会话提供。这两个命令不启动相机、不加载地图：

```bash
cd /home/orin/ros2_ws
./r680_perception.sh run     # 检测+Agent反馈+人物任务代理，收到新鲜图像即检测
./r680_person_follow.sh run # 另一终端，默认运动预览
```

第三终端：

```bash
cd /home/orin/ros2_ws
./r680_agent.sh person visible
./r680_agent.sh person start --mode follow --target-id '从visible原样复制ID' --request-id demo --seconds 10
./r680_agent.sh person start --mode approach_interact --target-id '从visible原样复制ID' --request-id approach-demo --seconds 10
./r680_agent.sh person stop
```

CLI的`--seconds`是显式有界续租，结束后stop；不传时只发一次短租约。`r680_agent.sh person-api`可单独运行代理，不能与默认感知入口中的同域代理重复启动。`run --no-agent`仅起nav检测器。旧`--nav-map-directory/--nav-selected-map-record/--mapping-session`参数已从实时检测入口移除。

Agent工程提供非阻塞`RosPersonTransport`，调用方维护request_id，并只在当前决策仍有效时tick续租。原`r680_behavior_shadow.py`仍为观测决策，不自行开底盘；新人物执行不借用需要map_id/map_revision的语义导航Action。

## 底盘输出与部署状态

人物节点默认`enable_motion=false`。Twist预览主题`/r680/person_follow/cmd_vel_candidate`只供观察；仲裁只读`/r680/person_follow/command_candidate`，带源时间、follow_session、request_id。在预览模式此结构候选始终为零。

新command_mux有`/r680_base/set_person_source`（SetBool，true=PERSON/false=NAV）。切换需手柄在线中立和新鲜轮速停车证据，切换后处于MANUAL；还需独立`/r680_base/set_automatic`授权AUTO。人物节点只有`enable_motion=true`且收到新鲜PERSON/AUTO状态才发运动候选。仲裁再次校验请求、进程会话、源年龄和限幅；手动接管/断连使人物任务撤销。NAV/PERSON只可能有一个进入底盘，切换请求会取消现有Nav2目标。

本次只更新代码、构建及隔离测试，没有重启常驻`r680-manual-base.service`，因此正在运行的旧服务尚无新source接口；现场启用新仲裁前需要停车后重启该服务并分级实测。当前部署不是已完成实车跟随验收。

## 2026-10-13 移除检测开关的验证

本次变更及可恢复备份位于 `/home/orin/ros2_ws/.maintenance/remove_agent_yolo_gate_20261013`。Orin ROS domain 198 的 9 个相关测试模块 **103 passed、0 failed、0 skipped（4.63 秒）**，包含无检测开关的推理、人物 stop 与检测独立、代次/退出清理、相对几何与人物节点 DDS 预览回归。使用模拟推理和合成图像/深度，未实测真实相机、模型质量或实车行驶。结果见 `validation.json`、`gate-removal-tests.xml` 与最终 `final-audit.json`；早期测试入口错误和未显式开启隔离 ROS 测试的记录保留。源码与安装入口已同步，底盘服务 PID 2217 未变。未启动正式域感知或底盘运动，人物执行租约与碰撞约束保持。

## 2026-10-12 历史验证证据

下列当时的检测开关测试和默认关闭描述只适用于 2026-10-12 版本；该开关已于 2026-10-13 移除。

本轮日志、备份、源哈希及测试报告存于`/home/orin/ros2_ws/.maintenance/relative_person_20261012`。精确结果以`relative-tests.xml`、`real-relative-replay.json`及交付说明为准。运动相关验证仅ROS domain 198；模型回放使用真实用户权重、样例RGB和合成depth/TF，未提供真实相机、真人ID稳定性、整机负载、盲区或制动距离验收。旧map实时反馈与MPC人物入口的历史测试不证明当前新链。

本轮Orin最终5包构建成功；跟随/协议/几何/门控/Agent客户端/显式Engine/原语义桥与影子层回归 **239 passed、0 failed、0 skipped（4.98秒）**。独立DDS仲裁脚本通过NAV/PERSON互斥、显式AUTO、任务/进程绑定、传输时效、空闲后新任务、手动撤权、热插拔和Nav2取消；该脚本只运行隔离mux，不运行底盘驱动。真实用户YOLO模型回放通过检测默认关闭、关闭期间不发布新检测、两门控独立、指定ID/模式切换、stop、租约到期拒绝复活、检测重开旧ID拒绝、无深度类别保留、断流清空。15批有效检测源采样到raw回调P50=162.12ms、P95=211.59ms、最大220.79ms；一帧约1465ms的冷启动结果按300ms源时效丢弃。真实模型回放的底盘发布者为0、正式运动候选全零，预览候选可非零。人物/mapless链没有map/odom/定位健康订阅。全部测试ROS domain 198；样例RGB、合成深度/相机外参，未进行真人跟随、真实相机、全机负载、制动/碰撞验收。

最终源/安装字节核验通过，删除29个旧源码/配置/测试文件，并清理33个已安装/构建遗留文件，逐文件前后hash与可恢复备份保存。常驻`r680-manual-base.service`仍active、PID2215未改变，未重启和启用真实运动；正在运行的服务尚未加载新PERSON选择接口。备份及JUnit/真实模型回放/仲裁日志/最终审计在 `/home/orin/ros2_ws/.maintenance/relative_person_20261012`，其中`relative-tests.xml`、`real-relative-replay.json`、`mux-ros.log`、`final-audit.json`为本轮结果。早期fixture回调积压失败及测试过早调用尚未加载完成的检测后端记录保留；已修复夹具发布频率/就绪等待，不改变生产安全门限。

## 2026-10-13 检测目标频率调整

按用户要求，`nav_yolo_detector` 的默认 `--hz` 从 5 改为 30，即目标检测频率 30 FPS。`r680_perception.sh run` 未显式传入 `--hz` 时使用此默认值；仍可用 `--hz` 指定 0.1–30 的范围。安装检测入口为源码符号链接，本次修改自动同步。

处理仍采用单工作线程和最新帧槽，来不及处理的旧帧被替换；实际有效 FPS 由输入、推理/跟踪耗时和整机负载决定，不保证设置为 30 就实测达到 30。人物源观测 300 ms 时效、现有跟踪参数及家具语义消费者 1 Hz 均未修改。更新跟踪配置中的旧 5 Hz 注释，未宣称完成新频率下的 ID 稳定性验收。

本轮核验 Python 语法、实际参数解析的默认 30 与显式覆盖、源/安装哈希一致及差异范围。未启动检测进程、未重启服务、未改变已有静态 IMU 录制；未测量实际 30 FPS 吞吐或实车效果。
