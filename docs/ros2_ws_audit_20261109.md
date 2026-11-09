# ros2_ws 文档与源码框架核对记录（2026-11-09）

## 范围与基线

Orin `orin@192.168.9.129:/home/orin/ros2_ws`。读取起点为外层 Git `22230b1`，不是旧本地离线副本。
实际主机 Ubuntu 24.04 / aarch64、ROS 2 Jazzy、Python 3.12.3；colcon 发现 21 个包。
常驻 r680-manual-base.service 为 active/running；仅查询，未重启。
根启动脚本默认域 73、CycloneDDS/回环发现；直接 source 后域与 RMW 未设置，操作手册给出每终端环境。

读取开始时已有两处未提交源码改动，原样保留且不纳入本轮提交：

- `src/wla_behavior_bridge/scripts/navigation_task_gateway`
- `src/wla_person_follow/wla_person_follow/follow_session.py`

外部 `~/Wheel_Legged_Agent`、底盘工作区、运行时 SDK/venv 是依赖，未将它们计入本轮 ros2_ws 项目源码覆盖。
不遍历 build/install/log 的重复生成代码、缓存、模型/地图二进制和历史原始测试日志；不是遗漏这些目录的存在。

## 交付入口

- [代码框架与数据流](ros2_ws_architecture.md)：21 包职责、收发/计算分层、传感器/TF/速度所有权、三种里程计、两类探索。
- [启动和常用参数](ros2_ws_operations.md)：SSH、环境、相机/感知/跟随/闲逛/导航/建图/Agent、录制回放、参数来源与查询。
- [完整源码目录树](ros2_ws_directory_tree.md)：逐文件展开项目代码、配置和文档。
- [全部 launch 参数](ros2_ws_launch_reference.md)：14 份 launch，含循环声明展开、必填项和源码行号。
- [读取清单 TSV](ros2_ws_source_inventory.tsv)：每个初始文本文件的类别、行数、字节数和 SHA256。

## 源码覆盖的准确含义

全量静态读取/索引覆盖 **499 个代码文件（82,144 行）**、**161 个配置/接口文件（9,525 行）**，
以及原有 **97 份项目 Markdown（7,495 行）**；另外 27 个许可/说明类文本也纳入逐文件清单。
Python 使用 AST 解析并提取定义/参数，shell 用 Bash 语法检查，配置按其实际格式解析，Markdown 做逐文件文本/链接/入口与参数核对。

**全文件静态扫描不等于逐行人工语义精读，也不等于证明所有算法正确。**
本轮重点通读/核对的是根启动脚本、全部 launch、最终 mux/跟随收发与任务账本、感知/模型适配、跟随运行编排以及常用配置；
OpenVINS 上游估计器和全量 C++ 数值实现/测试主要纳入静态索引与接口/依赖检查。
因此不把本轮结果写成“82,144 行均完成逐行语义审计”或“所有代码均正确”；用户要求的全仓逐行语义通读仍未全部完成。
各文件 TSV 的 coverage 为 full_file_static_scan，不能解读为人工逐行审查已通过。

## 已修正文档事实

1. 当前车址为 192.168.9.129；历史测量中的旧 IP 作为当时记录保留，不伪造历史。
2. 导航默认 RGB-D+MPC；cuVSLAM 连续后端默认 odom_fusion，不是旧 degraded_odometry。
3. 区分默认 3 s 滑窗与旧后端 2 s/0.30 m/0.80 rad 预算，区分 motion_ready 与旧地图匹配。
4. 根 nav/mapping/teleop 脚本是实现；包内同名脚本只是转发。删除“反向复制覆盖根脚本”的旧构建命令。
5. build_frontend.sh 仅构建 cuVSLAM 前端，不再同时构建导航包/同步根入口。
6. 相机为独立持有者；导航/建图复用，D455 IMU 需相应修复 overlay。
7. 建图默认可输出、OpenVINS 建图强制 dry-run；常驻底盘下 cuVSLAM 建图加 auto-vio-init 会被拒绝。
8. mux 采用新鲜非零输入优先级，输入一路过期不必然整车输出零；无需额外 AUTO/PERSON 切换。
9. 后端支持 0–4，试验客户端仅 0–2；旧英文 mode、独立物体包/legacy/release overlay 均不作当前入口。
10. 区分 transport 与 numeric_backend；C++ 数值后端只覆盖地图、跟踪和曲线，采样 MPC 仍在 Python。
11. 更新跟随 1.2 m/s 算法上限与 mux 最终 1.0 m/s 限幅、0.40/0.45/0.20/0.35 s 时效、8 m/0.05 m 地图。
12. 贴边观测按当前已有源码的距离新息条件描述；倒车已改为边退边转；物体路线不能称全程已知自由空间。
13. 录制 --camera 可能启动 JPEG 发布者；回放隔离域和显示白名单写明。
14. 移除 36 条已删除 .staging 基线绝对链接，改为 Git 1391388 的文件/行号证据；保留原测量结论。
15. 对历史设计/验收资料增加版本适用说明，补充缺少 README 的接口/桥接/探索包。

## 验证结果与已知问题

- 在 Orin 对 13 个已安装 launch 执行 --show-args，全部退出 0：跟随、bringup、相机、闲逛、OpenVINS、三个 Agent 入口、manual base、terrain、gamepad、explore_lite、官方 ov_msckf。
- 独立 cuVSLAM validation 的 launch 已静态解析，但未在本轮另切验证 overlay 做安装入口检查；不计入上述 13 项。
- 206 个 Python 文件/脚本的 AST 语法解析通过；不 import/启动生产代码。
- 项目 shell 共 30 份：23 份原文件 Bash 语法通过；以下 7 份 OpenVINS 上游旧实验批处理因为 CRLF 换行失败。
  内存中仅转换为 LF 的对照可通过，但本轮未改这些源码，也没有运行它们：

```text
src/open_vins/ov_msckf/scripts/run_ros_eth.sh
src/open_vins/ov_msckf/scripts/run_ros_kaist.sh
src/open_vins/ov_msckf/scripts/run_ros_tumvi.sh
src/open_vins/ov_msckf/scripts/run_ros_uzhfpv.sh
src/open_vins/ov_msckf/scripts/run_sim_calib.sh
src/open_vins/ov_msckf/scripts/run_sim_cams.sh
src/open_vins/ov_msckf/scripts/run_sim_featrep.sh
```

这七项不是“全仓检查通过”。它们是保留的 ROS 1/数据集实验脚本，不在当前 r680/OpenVINS ROS 2 正式启动链内。
OpenVINS 的 `%YAML:1.0` 配置使用 OpenCV FileStorage 检查；普通 ROS YAML 使用 PyYAML，未修改配置来迎合解析器。
新操作手册和速记中的 Bash 代码块做语法检查；所有项目 Markdown 相对链接逐一校验。
最终文档/配置验证：108 份 Markdown 的 319 个相对引用全部可解析；17 个新手册 Bash 代码块语法通过；51 份普通 YAML、52 份 OpenCV YAML、27 份 XML、17 份 JSON 均通过各自解析器。
源码哈希比对确认代码未被本轮改变。同步到 Orin 的 91 个文件（修订原文档 79 份、新增 Markdown 11 份、TSV 清单 1 份）逐文件 SHA256 验证一致，git diff --check 通过。
原文档备份在 `/home/orin/.codex-backups/ros2_ws-docs-20261109-lurdv6h8`；修改来源和初始哈希另存于该目录 manifest.json。
最终本地 Git 提交只包含本轮文档/索引；用户原有两处源码改动继续保留为未提交状态。

另发现 `PersonCommand.srv` 顶部注释仍只列 0/1/2，但实际后端 task_modes.py 接受 0–4。
本轮用接口 README 说明该差异，未修改消息源码或重生成接口。

没有执行 colcon 编译、算法/ROS 集成回归、模型推理、相机启动、导航/跟随任务、运动或制动测试；
文档中的历史测试数与性能数据保留原日期，不能算作本轮重测结果。
只修改文档/索引，无程序行为或阈值变更，也未推送远端。

## 原有 97 份 Markdown 的逐份处理

下表覆盖来源文件，而不是只列被修改的 README。历史记录追加版本边界，原结果和失效原因保留。
“原文保留”表示纳入静态文本/引用检查，未发现本轮需要改变的入口说明；不表示对其外部链接、历史测量或算法正确性重新验收。

| 原有文档 | 本轮处理 |
| --- | --- |
| [AGENTS.md](../AGENTS.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [FOLLOW_TRIAL.md](../FOLLOW_TRIAL.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [README.md](../README.md) | 更新当前调试地址 |
| [docs/Agent共享检测与YOLO模型接入.md](Agent共享检测与YOLO模型接入.md) | 增加当前共享感知/Agent 接口说明，旧接入内容明确归档 |
| [docs/Agent语义导航接入.md](Agent语义导航接入.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/Agent附近探索执行映射.md](Agent附近探索执行映射.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [docs/VIO健康与地图定位确认_20261013.md](VIO健康与地图定位确认_20261013.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/package-history.md](package-history.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [docs/validation/20261015-nav-manual-coexist.md](validation/20261015-nav-manual-coexist.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/validation/continuity-restart-removal-20261015.md](validation/continuity-restart-removal-20261015.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/validation/cuvslam-feature-preview-20261016.md](validation/cuvslam-feature-preview-20261016.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/validation/cuvslam-initialization-authority-20261015.md](validation/cuvslam-initialization-authority-20261015.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/validation/cuvslam-initialization-speed-20261015.md](validation/cuvslam-initialization-speed-20261015.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/validation/cuvslam-official-covariance-20261016.md](validation/cuvslam-official-covariance-20261016.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/validation/cuvslam-startup-match-20261015.md](validation/cuvslam-startup-match-20261015.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/validation/maintenance-history/README.md](validation/maintenance-history/README.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [docs/validation/modeless-nav-20261015.md](validation/modeless-nav-20261015.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/validation/object-follow-20261013.md](validation/object-follow-20261013.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/validation/openvins-official-20261015.md](validation/openvins-official-20261015.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/validation/openvins-zupt-20261014.md](validation/openvins-zupt-20261014.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [docs/人物跟随合入与系统链路核验.md](人物跟随合入与系统链路核验.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/gamepad_contorl/README.md](../src/gamepad_contorl/README.md) | 区分常驻车载配置与通用独立包默认值 |
| [src/wla_cuvslam_navigation/COVARIANCE-DIAGNOSTICS-20261009.md](../src/wla_cuvslam_navigation/COVARIANCE-DIAGNOSTICS-20261009.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_cuvslam_navigation/DEGRADED-ODOM-20261009.md](../src/wla_cuvslam_navigation/DEGRADED-ODOM-20261009.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_cuvslam_navigation/DIAGNOSTICS-20261008.md](../src/wla_cuvslam_navigation/DIAGNOSTICS-20261008.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_cuvslam_navigation/POSE-OUTPUT-20261009.md](../src/wla_cuvslam_navigation/POSE-OUTPUT-20261009.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_cuvslam_navigation/README.md](../src/wla_cuvslam_navigation/README.md) | 区分默认连续链与禁接管直接输出 |
| [src/wla_cuvslam_navigation/third_party/isaac_ros_cuvslam/UPSTREAM.md](../src/wla_cuvslam_navigation/third_party/isaac_ros_cuvslam/UPSTREAM.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_cuvslam_validation/README.md](../src/wla_cuvslam_validation/README.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_cuvslam_validation/VALIDATION_200HZ_RESULTS.md](../src/wla_cuvslam_validation/VALIDATION_200HZ_RESULTS.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_cuvslam_validation/VALIDATION_RESULTS.md](../src/wla_cuvslam_validation/VALIDATION_RESULTS.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_detection_bridge/README.md](../src/wla_detection_bridge/README.md) | 区分协议期限与当前跟随后端期限 |
| [src/wla_diff_mpc/CHANGES-20261008.md](../src/wla_diff_mpc/CHANGES-20261008.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_diff_mpc/CHANGES-TERMINAL-20261008.md](../src/wla_diff_mpc/CHANGES-TERMINAL-20261008.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_diff_mpc/README.md](../src/wla_diff_mpc/README.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_explore_wander/README.md](../src/wla_explore_wander/README.md) | 去掉当前不需要的 AUTO/PERSON 操作 |
| [src/wla_follow_local/README.md](../src/wla_follow_local/README.md) | 修正 tracker 所有权措辞 |
| [src/wla_manual_control/README.md](../src/wla_manual_control/README.md) | 修正过期的跟随最高速度 |
| [src/wla_openvins_navigation/README.md](../src/wla_openvins_navigation/README.md) | 补齐独立相机和当前 OpenVINS 启动入口 |
| [src/wla_openvins_navigation/patches/README.realsense-imu.md](../src/wla_openvins_navigation/patches/README.realsense-imu.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_person_follow/FULL_CHAIN_TESTS.md](../src/wla_person_follow/FULL_CHAIN_TESTS.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/HYBRID_TRANSPORT.md](../src/wla_person_follow/HYBRID_TRANSPORT.md) | 澄清历史默认与当前 YAML 覆盖 |
| [src/wla_person_follow/LATENCY_OPTIMIZATION.md](../src/wla_person_follow/LATENCY_OPTIMIZATION.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/PY_CPP_ARCHITECTURE.md](../src/wla_person_follow/PY_CPP_ARCHITECTURE.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_person_follow/README.md](../src/wla_person_follow/README.md) | 增加数值后端和当前总文档入口 |
| [src/wla_person_follow/REID_ENROLLMENT_FIX.md](../src/wla_person_follow/REID_ENROLLMENT_FIX.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/SIM_DEPLOYMENT.md](../src/wla_person_follow/SIM_DEPLOYMENT.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_person_follow/docs/2026-10-22_几何绑定与ReID职责拆分设计_v2.md](../src/wla_person_follow/docs/2026-10-22_几何绑定与ReID职责拆分设计_v2.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/docs/FOLLOW_DIAGNOSTICS_20261022.md](../src/wla_person_follow/docs/FOLLOW_DIAGNOSTICS_20261022.md) | 移除 36 条失效 staging 绝对链接，改为可追溯 Git 基线与行号 |
| [src/wla_person_follow/docs/TRIAL_TARGET_PICKER.md](../src/wla_person_follow/docs/TRIAL_TARGET_PICKER.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/docs/continuous_follow_client.md](../src/wla_person_follow/docs/continuous_follow_client.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/docs/continuous_local_free.md](../src/wla_person_follow/docs/continuous_local_free.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/docs/follow_scenarios_and_known_issues.md](../src/wla_person_follow/docs/follow_scenarios_and_known_issues.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_person_follow/docs/reid_recovery.md](../src/wla_person_follow/docs/reid_recovery.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/docs/shared_depth_filter.md](../src/wla_person_follow/docs/shared_depth_filter.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/docs/stall2_runtime_fix.md](../src/wla_person_follow/docs/stall2_runtime_fix.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/docs/startup_free_prior.md](../src/wla_person_follow/docs/startup_free_prior.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/docs/tracker_follow.md](../src/wla_person_follow/docs/tracker_follow.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_person_follow/neupan_bridge/README.md](../src/wla_person_follow/neupan_bridge/README.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_r680_navigation/README.md](../src/wla_r680_navigation/README.md) | 更新导航 Web 地址 |
| [src/wla_r680_navigation/addons/README.md](../src/wla_r680_navigation/addons/README.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_r680_navigation/addons/cuvslam-sdk/README.md](../src/wla_r680_navigation/addons/cuvslam-sdk/README.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_r680_navigation/docs/AGENT_OBSERVATION_WINDOW.md](../src/wla_r680_navigation/docs/AGENT_OBSERVATION_WINDOW.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_r680_navigation/docs/BOXMOT_MIGRATION.md](../src/wla_r680_navigation/docs/BOXMOT_MIGRATION.md) | 同步人物/物体不同的深度配对规则 |
| [src/wla_r680_navigation/docs/CUVSLAM-RECOVERY-20261016.md](../src/wla_r680_navigation/docs/CUVSLAM-RECOVERY-20261016.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/GMC_RECOVERY.md](../src/wla_r680_navigation/docs/GMC_RECOVERY.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-CUVSLAM-001.md](../src/wla_r680_navigation/docs/R680-CUVSLAM-001.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-CUVSLAM-002.md](../src/wla_r680_navigation/docs/R680-CUVSLAM-002.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-CUVSLAM-003.md](../src/wla_r680_navigation/docs/R680-CUVSLAM-003.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-CUVSLAM-004.md](../src/wla_r680_navigation/docs/R680-CUVSLAM-004.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-CUVSLAM-005.md](../src/wla_r680_navigation/docs/R680-CUVSLAM-005.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-CUVSLAM-006.md](../src/wla_r680_navigation/docs/R680-CUVSLAM-006.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-MAP-EDIT-001.md](../src/wla_r680_navigation/docs/R680-MAP-EDIT-001.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-MAP-EDIT-002.md](../src/wla_r680_navigation/docs/R680-MAP-EDIT-002.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-NAV-007-mapless.md](../src/wla_r680_navigation/docs/R680-NAV-007-mapless.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-NAV-008-odom-fusion.md](../src/wla_r680_navigation/docs/R680-NAV-008-odom-fusion.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-NAV-009-lost-recovery.md](../src/wla_r680_navigation/docs/R680-NAV-009-lost-recovery.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680-NAV-010-emitter-alternate.md](../src/wla_r680_navigation/docs/R680-NAV-010-emitter-alternate.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/R680导航建图启动链路与代码目录树.md](../src/wla_r680_navigation/docs/R680导航建图启动链路与代码目录树.md) | 为旧导航目录树提供准确替代入口并保留历史 |
| [src/wla_r680_navigation/docs/changes/R680-DEGRADED-ODOM-001.md](../src/wla_r680_navigation/docs/changes/R680-DEGRADED-ODOM-001.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/changes/R680-DEGRADED-ODOM-002.md](../src/wla_r680_navigation/docs/changes/R680-DEGRADED-ODOM-002.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/changes/R680-FIXES-20261009.md](../src/wla_r680_navigation/docs/changes/R680-FIXES-20261009.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/changes/R680-MPC-STATIC-GLOBAL-001.md](../src/wla_r680_navigation/docs/changes/R680-MPC-STATIC-GLOBAL-001.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/changes/R680-OBSTACLE-001.md](../src/wla_r680_navigation/docs/changes/R680-OBSTACLE-001.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/changes/R680-RECOVERY-001.md](../src/wla_r680_navigation/docs/changes/R680-RECOVERY-001.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/changes/R680-RECOVERY-002.md](../src/wla_r680_navigation/docs/changes/R680-RECOVERY-002.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/changes/R680-TERMINAL-001.md](../src/wla_r680_navigation/docs/changes/R680-TERMINAL-001.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/changes/R680-VIO-OUTPUT-001.md](../src/wla_r680_navigation/docs/changes/R680-VIO-OUTPUT-001.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/debug_web_smoothing.md](../src/wla_r680_navigation/docs/debug_web_smoothing.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_r680_navigation/docs/openvins_20261013.md](../src/wla_r680_navigation/docs/openvins_20261013.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/openvins_motion_init.md](../src/wla_r680_navigation/docs/openvins_motion_init.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/openvins_navigation_motion.md](../src/wla_r680_navigation/docs/openvins_navigation_motion.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/openvins_visual_health_20261014.md](../src/wla_r680_navigation/docs/openvins_visual_health_20261014.md) | 标注版本记录与当前文档的适用边界，保留历史证据 |
| [src/wla_r680_navigation/docs/perception_debug.md](../src/wla_r680_navigation/docs/perception_debug.md) | 原文保留；已纳入全文件静态文本与引用检查 |
| [src/wla_r680_navigation/docs/planning_topdown.md](../src/wla_r680_navigation/docs/planning_topdown.md) | 修正过期地址和不存在的独立规划页路由 |
| [src/wla_terrain_analysis/README.md](../src/wla_terrain_analysis/README.md) | 补齐脚本必需的 run 子命令 |
| [start_nav.md](../start_nav.md) | 替换失效旧地图和默认直接运动的孤立命令 |

## 按包的初始静态读取规模

| 包目录 | 代码文件 | 代码行 | 配置/接口文件 |
| --- | ---: | ---: | ---: |
| `src/explore` | 12 | 2208 | 3 |
| `src/explore_lite_msgs` | 1 | 12 | 2 |
| `src/gamepad_contorl` | 8 | 1350 | 2 |
| `src/open_vins` | 130 | 32472 | 52 |
| `src/wla_behavior_bridge` | 11 | 1229 | 3 |
| `src/wla_behavior_bt` | 2 | 204 | 2 |
| `src/wla_behavior_interfaces` | 1 | 11 | 6 |
| `src/wla_cuvslam_navigation` | 20 | 2053 | 2 |
| `src/wla_cuvslam_validation` | 8 | 1091 | 3 |
| `src/wla_detection_bridge` | 5 | 224 | 1 |
| `src/wla_diff_mpc` | 16 | 1513 | 3 |
| `src/wla_explore_wander` | 9 | 1314 | 1 |
| `src/wla_follow_local` | 14 | 3209 | 1 |
| `src/wla_follow_transport` | 5 | 880 | 1 |
| `src/wla_manual_control` | 19 | 2509 | 5 |
| `src/wla_openvins_navigation` | 16 | 840 | 23 |
| `src/wla_person_follow` | 63 | 9288 | 5 |
| `src/wla_r680_navigation` | 138 | 18549 | 39 |
| `src/wla_terrain_analysis` | 9 | 1197 | 2 |

以上数字来自读取快照；新增/修订的 Markdown 自然不包含在初始源码行数与哈希中。
