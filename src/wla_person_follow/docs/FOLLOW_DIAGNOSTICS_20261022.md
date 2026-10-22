# 10月21日部署代码核对与诊断日志补充

日期：2026-10-22；车辆地址：orin@192.168.9.111。

本次仅增加诊断字段、只读日志订阅进程和对应测试，没有改变阈值、控制决策、候选生成、身份接受条件、恢复状态机或速度接口。没有启动生产车辆程序。以下 A 的行号均指**修改前副本**，对应 ros2_ws 提交 `1391388`，不是补日志后移动了行号的文件。

从车端取回源码后，核验 sim_node、sim_runtime、reid_identity、local_memory、blocked_recovery、rolling_depth_map、tracking、hybrid_controller 共8个关键模块与 10月21日 installed overlay SHA256 一致。保留原始 baseline 和候选 work 目录，避免引用行号随修改漂移。

## A1. 初始 front_blocked 的来源、范围与阈值

- [sim_core/hybrid_controller.py:49](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/hybrid_controller.py:49)，`HybridFollower.command`：先计算外侧宽弯参考，随后调用恢复状态机；传入的是宽弯参考（存在时）或目标历史。不是先筛完MPC候选再决定 front_blocked。
- [sim_core/blocked_recovery.py:114](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/blocked_recovery.py:114)，`BlockedRecovery._path_samples`：从当前车位置连到历史/绕行参考的最近投影，沿其后续路径取最多 **1.4m弧长**，约每 **0.05m** 采样，并保留范围内路径顶点和末端。这不是前方半径1.4m扇区，也不是相机深度范围。没有参考时用朝向目标的直线，最多1.4m。这里**没有在目标前减去期望跟随距离**，路径可以延伸到目标位置附近。
- [sim_core/blocked_recovery.py:143](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/blocked_recovery.py:143)，`forward_blocked`：先查地图；任一采样点的 `mapping.query <= radius+margin+0.026` 即返回 true。当前 **0.19+0.10+0.026=0.316m**。
- [sim_core/rolling_depth_map.py:136](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/rolling_depth_map.py:136)，`rebuild`：有效期内 log>=0 的格为occupied；障碍距离场还把地图边界padding设为阻挡。[sim_core/rolling_depth_map.py:154](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/rolling_depth_map.py:154)，`query`：查询值是保守距离，不是“相机到障碍的裸距离”；Python参考会从格中心EDT距离减去 res/√2 和查询点到格中心距离，越界返回0。实际车端走与参考测试一致的native_grid_query。因此0.316m不能直接当成现实中量尺测得的净间距，边界也能触发，不保证每次都是某个实体障碍格。
- 地图未触发时，再检查传入路人轨迹：`p+v*t` 预测，沿每段计算最近相对距离；阈值为 `o.r+0.19+0.10+0.025*t_segment_end`。行人预测时间用路径累计长度除以速度，速度钳在0.15–0.65m/s（同一函数155–161行）。先触发地图就不再执行这个原始路人分支。
- 阻挡格输入：[sim_runtime.py:82](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_runtime.py:82) `SimulationEngine.step` 更新地图；障碍端点来自深度前端，并由 [sim_core/rolling_depth_map.py:85](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/rolling_depth_map.py:85) `RollingDepthMap.update` 的118–125行写入。地图只存占据证据/时间，不存人物ID，不能从旧格反推它是否来自目标。
- 路人输入：[sim_runtime.py:47](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_runtime.py:47) `measurements_from_relative` 将除当前确认目标之外的人物相对观测转到局部参考；[sim_core/tracking.py:35](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/tracking.py:35) `PeopleTracks.update` 维护位置速度，半径为0.28+0.035+0.15×观测年龄，超过1.5秒删除。[sim_runtime.py:93](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_runtime.py:93) 有确认目标时会移除该ID在路人集合中的旧轨迹。
- [sim_core/blocked_recovery.py:183](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/blocked_recovery.py:183) `update`：有目标且未lost；有跟随需求（中心距离>desired_gap+0.25≈1.12m，或目标速度>0.15m/s）；blocked为true；实测|v|<0.06m/s，才走wait触发分支（196–200、256–265行）。锚点未建立或平移超过0.08m会重新建立等待锚点。

日志第4–5行的 `front_blocked` 证实恢复判断进入等待，**没有记录足够数据确认哪个格或哪个路人触发**。

`forward_total=0` 是后果：[sim_core/hybrid_controller.py:109](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/hybrid_controller.py:109) `command` 115行在wait/hold状态把期望线速度置零，随后才计算安全性；统计中forward_total只数第一步v>0.01的候选，safe数整个预测通过硬约束的候选。325条安全停车/转向候选不等于325条向前候选被碰撞拒绝。

## A2. resolved_target_id为空时，目标点云与历史地图

- [sim_node.py:349](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_node.py:349) `SimulationFollow.compute` 355行在resolved为空时向检测器请求空target_id；362–364行也给 `filter_target_depth` 传入None。
- [wla_r680_navigation_py/target_masks.py:29](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_r680_navigation/wla_r680_navigation_py/target_masks.py:29) `filter_target_depth`：31行在无目标ID/掩码时直接返回原深度。所以目标自身有效点**可能被保留**，经过C++高度/地面/小簇过滤后，符合障碍条件的点会写进占据图。不是所有人体像素都必定成为障碍。
- 即便身份tracking，若掩码身份/任务/时间/尺寸不匹配或深度不足，也不会剔除；只删除轮廓内、中值深度±0.35m的点（同函数32–54行）。`MaskDepthPairer.take` 约120ms无配对后允许未剔除深度继续处理，不无限等待掩码。
- [sim_core/rolling_depth_map.py:85](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/rolling_depth_map.py:85) `update`：115、117行有效清空射线/地面端点使log每次减0.8，命中125行每次加2、最高3。同一帧先清空后命中，因此仍观测到障碍可重新标占据。射线只使用符合高度条件的真实端点，不清空被人物遮挡的后方。
- [sim_core/rolling_depth_map.py:136](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/rolling_depth_map.py:136) `rebuild`：证据年龄超过12秒变未知，不直接变空闲；重新观测会更新原始证据时间，因此12秒不是“从锁定时刻固定倒计时”。环形地图滑出区域会被替换；轮速参考重置会重建地图。[sim_runtime.py:169](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_runtime.py:169) `process_step`：同一参考下换任务仍保留地图。
- 没有按目标ID删除历史点的逻辑，也没有地图格的语义归属。本日志没有掩码状态/点云/阻挡格，**无法确认初始front_blocked是目标残影造成**。

## A3. low_confidence是累计失败还是忽略

- [wla_r680_navigation_py/identity_features.py:9](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_r680_navigation/wla_r680_navigation_py/identity_features.py:9) `feature_quality`：confidence<0.5返回low_confidence。这是**检测框置信度**，不是OSNet相似度。
- [scripts/nav_yolo_detector:288](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_r680_navigation/scripts/nav_yolo_detector:288) `drain`：291–298行跳过该描述符，只将质量原因放到diagnostics；不会把低质量描述符送给TargetIdentity打分。
- [reid_identity.py:59](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/reid_identity.py:59) `TargetIdentity.consume`：没有失败次数累加器。feature_frames仍加1，低质量候选不进入valid/ranked；last_seen不更新。
- 若已完成登记且该批没有其他合格匹配，110–112行**立即**设为searching，并清空待确认计数。不是“连续失败若干次后才searching”。若尚在初次登记，则保持登记阶段并记录缺特征原因。
- [reid_identity.py:126](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/reid_identity.py:126) `select`：state不为tracking直接返回None，即使BoT-SORT仍有同ID的有效位置。这里的0.8秒是其他时效约束，不会覆盖state!=tracking这项。
- 因此准确表述是：**特征层跳过此帧，但任务状态并非完全忽略这一批；空的合格集合可能切入searching。**

## A4. expired后的候选与冻结数值

- [reid_identity.py:21](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/reid_identity.py:21) `_expire`：登记完成后按last_seen计时，超过max_lost=8秒设expired；24行一旦expired就始终返回true。
- [reid_identity.py:59](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/reid_identity.py:59) `consume`：61行先更新时间和feature_frames，64行再检查_expire并提前返回。所以expired后收到新批次，帧计数可以继续涨，但**不再执行候选归一化、相似度和空间匹配**。
- [reid_identity.py:143](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/reid_identity.py:143) `candidate_ids` 145行直接返回空；[reid_identity.py:126](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/reid_identity.py:126) `select` 128行返回None。没有自动解除expired的逻辑。
- 旧similarity/position_error_m只在接受或选出待确认候选时更新：`_accept`42–57行、直接接受109行、待确认120行；ranked为空时没有覆盖这些数值。它们不是“最后一个失败候选”的可靠实时分数。snapshot只是返回保存值（[reid_identity.py:166](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/reid_identity.py:166)）。
- 丢失约2.5秒时为searching、还没有expired；此阶段若后续没有合格候选进入待确认分支，数值就保持不变。8秒后的提前返回又使它们继续保留。日志证实未找回，但**不能从冻结数值断定每个新帧究竟因外观还是位置被拒绝**。

## A5. 内部OSNet与任务身份审核

- [wla_r680_navigation_py/boxmot_runtime.py:13](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_r680_navigation/wla_r680_navigation_py/boxmot_runtime.py:13) `TrackingRuntime.__init__` 创建原生BotSort并启用embeddings、SOF CMC；`associate`45–60行调用tracker.update并缓存其同帧特征。
- [wla_r680_navigation_py/boxmot_reid.py:59](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_r680_navigation/wla_r680_navigation_py/boxmot_reid.py:59) `CachedBoxMOTReID.get_features` 调用BoxMOT后端，保存last_features。[scripts/nav_yolo_detector:177](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_r680_navigation/scripts/nav_yolo_detector:177) `infer`复用runtime.features，[scripts/nav_yolo_detector:288](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_r680_navigation/scripts/nav_yolo_detector:288) `drain`把特征和RGB-D几何一起发布。
- TargetIdentity仅对收到的512维向量做归一化、点积、空间与时间筛选，不重新裁图或执行OSNet。因此**没有重复神经网络推理，但有第二套身份决策**。
- 两套判定可以不一致：BoT-SORT可通过运动/几何关联维持ID，而任务层因confidence<0.5、模板外观差异、空间门限或时效拒绝它。任务层不修改/重置BoT-SORT内部轨迹；直接影响的是下游目标接收、掩码请求与执行。
- 此日志不足以证明BoT-SORT连续两分钟始终保持同一人物轨迹；locked_id只是任务记忆。网页根据原始观测继续移动是用户观察到的现象，不能仅凭冻结locked_id推断全程跟踪正确。

## A6. retreat触发与目标自身阻挡

- [sim_core/blocked_recovery.py:183](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/blocked_recovery.py:183) `update`：满足A1的demand+blocked+stalled，在锚点未重置的等待达到2秒后调用rear_clear。通过则建立retreat_start/heading、进入retreat，原因persistent_front_block（256–265行）；不通过则hold，rear_unknown_or_blocked。
- [sim_core/blocked_recovery.py:164](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/blocked_recovery.py:164) `rear_clear`：从0.35m到0.06m试16个倒退距离，每个检查到末端再多0.05m。静态query必须>=0.30m，已知空闲query必须>=0.21m；路人按0–4秒、17个预测时刻检查，距离不能小于o.r+0.19+0.10+0.025*t。通过的实际退距可能小于0.35m。
- [sim_core/hybrid_controller.py:109](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_core/hybrid_controller.py:109) `command`：retreat使用7×9=63条候选；默认倒车速度上限0.15m/s，候选仍需MPC扫掠和动态碰撞检查。不是前方阻挡就无条件倒车。
- 地图是匿名占据，因此目标自己的残留点也可能触发前方阻挡；该函数没有“这是目标所以忽略”的分支。
- resolved有效时：[sim_runtime.py:47](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_runtime.py:47) `measurements_from_relative`把目标单列；[sim_runtime.py:93](/home/orin/ros2_ws/.staging/follow-diagnostics-20261022/baseline/src/wla_person_follow/wla_person_follow/sim_runtime.py:93)移除其路人旧轨迹，所以当次动态路人集合通常不含确认目标。但深度残影仍可能在地图。
- resolved为空时：sim_node传入空字符串，`measurements_from_relative`把所有检测到的人都归入people，原目标可能成为路人阻挡源；短暂coast阶段仍可参与恢复判断。目标已lost时update188–195行会提前转入丢失处理。
- 日志第6行身份tracking时已输出倒车，能排除“那一次倒车必然由身份丢失触发”，**不能排除地图里的目标自身点，也不能确认具体来源**。

## B. 新增字段与保存方式

只增加数据和记录进程。原有字段保留，包括旧similarity等保留值；新字段另外明确当前帧、是否评估、源时间和失效。

| 路径 | 新增内容 |
|---|---|
| target_observation | 原锁定ID在relative_objects中的检测状态、表面距离m、方位rad、原始时间和年龄；和控制确认位置区分 |
| diagnostics.resolved_target_observation | 被任务层确认ID的对应测量；不存在时明确null |
| diagnostics.resolved_target_source | 是否经TargetIdentity.select、返回ID、拒绝原因；若无观测位姿而没调用select明确标注 |
| diagnostics.front_blockage | 是否本周期执行前方检查、恢复时钟/控制时间、需求与低速条件、查询阈值、最小地图查询距离和净空、实际短路触发来源 |
| …front_blockage.source | 占据格坐标、中心、年龄、log值、边界/越界或路人ID、位置、速度、半径、预测距离和判定阈值 |
| …front_blockage.all_sources_scan | 额外只读扫描下各来源的路径距离，给出最近阻挡来源；与原代码首先触发的分支区分，不反馈控制 |
| diagnostics.target_mask | 是否应用、移除像素、失败原因、中值深度；源深度时间在diagnostics.depth_stamp_ns |
| diagnostics.reid.candidate_frame | 本批候选的相似度、位置误差、门限、质量、是否实际评估、拒绝/接受原因、确认计数 |
| …reid.consumed_candidate_frames | 同一次计算消费多个特征批次时保留各批记录，不只保留最后一批 |
| …reid.candidate_request | 当前空间候选筛选结果；过期则明确未评估，不偷偷继续评分 |
| measured_motion | 最新轮速消息的实测vx/vy/ω及消息pose/时间、是否被局部积分接受；另给LocalMotion积分位姿、参考代际 |
| actual_published_command | 跟随节点实际发布到其配置输出通道的v/ω，区别于候选；不是底盘实测执行证明 |
| diagnostic_sequence | 状态发布序号，配合独立记录器的sequence_gap识别漏收 |
| diagnostics.age_s / planner_result_fresh | 这份诊断属于哪次计算、年龄和结果是否仍有效，避免缓存被当成新鲜数据 |

说明：

1. 低质量、被上游过滤、模型不匹配或expired帧**没有实际参与匹配**时，分数为null并标记原因。为了“补数值”重新评估expired候选会改变原执行路径，本次没有这样做。
2. 地图占据格不含语义ID；新日志能定位查询所对应的代表性最近EDT源格（含并列数、边界），不能回溯旧格的真实物体身份。目标点来源仍需结合掩码/观测/记录判断。
3. 最近阻挡距离定义为**测试路径上的距离**，不是前方相机原始测距；路人使用段内最近相对位置。实际触发来源遵循原代码“先地图，再按顺序路人”的短路规则，只读汇总另行计算所有输入来源，不影响判定。
4. 原始wheel消息pose仅记录不参与控制。控制位姿仍是轮速twist积分的局部参考，不声称全局定位真值。坐标和测量源时间均有记录。
5. 每条状态都有target_observation/measured_motion；尚无计算、参考已换或计算失败时diagnostics显式为空/不可用，不伪造阻挡或匹配数据。

### 频率与文件

- 原状态发布频率保持原样：20Hz定时检查，加上新规划结果就绪唤醒；实际频率可能高于20Hz，按diagnostic_sequence及stamp_ns判断。规划启动间隔仍至少0.1秒，身份帧来自感知节拍。
- trial终端仍按原约1Hz打印，不改变心跳0.4秒节拍和租约1.5秒；只扩展打印字段。
- 用户下次手动运行原来的 `./r680_person_follow.sh run …` 时，launch会同时启动独立的**只读**status_recorder；本次没有启动生产程序。
- 默认文件：`/home/orin/ros2_ws/.runtime/follow-diagnostics/follow-status-YYYYMMDD-HHMMSS-PID.jsonl`。
- 独立进程按收到的每条status落一行JSON，包括任务ID、源时间、接收时间、序号和新增字段；不是每秒抽样。best-effort、深度100的有界接收队列，不反压控制器；如漏收记录sequence_gap，不承诺网络/过载时绝不丢样。
- 每文件32MiB轮转，保留8个备份，加当前文件共9个；每次启动新文件名。记录器日志会打印路径。
- 文件写入与轮转在独立进程，降低nice优先级；不增加任何速度发布者或控制服务调用。磁盘耗时不会同步阻塞MPC，但诊断计算/JSON仍有有限CPU开销。
- 可用启动参数 `diagnostic_log_directory:=/所需路径` 更改目录；仅控制日志目录。
- 旧/tmp/r680-follow-trial123.log不会被回填不存在的历史观测；本次新字段从下次手动启动新版本后才产生。

## 验证与范围

- 17组身份时序、300帧完整数值核心回放：原版与新版本速度命令逐值相同，目标状态、恢复状态、候选统计、地图log/时间均一致，覆盖follow/wait/retreat/hold。
- Orin该回放中位计算约21.32ms→21.48ms；约0.16ms增量，只代表该负载，不是实车最坏时延或调度保证。
- 77项组件测试通过，含新增诊断、原身份/地图/几何/客户端测试；10项隔离ROS测试通过，包含独立订阅落盘。ROS测试使用domain198、仅回环和模拟输出话题，没有生产域运动测试。
- 本机原生几何相关测试因没有Orin的C++扩展而不能通过；同组在Orin实际扩展环境重新执行通过，没有用跳过测试掩盖失败。
- 启动与身份/恢复参数不变，生产代码只准备供下次启动；未验收人物遮挡或真实阻挡来源，未修复现有行为。

