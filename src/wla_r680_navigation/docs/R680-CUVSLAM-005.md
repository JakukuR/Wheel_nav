# cuVSLAM 建图惯性初始化修正

2026-10-08。失败运行：`/home/orin/nav_run/run-20261008-120343`。

## 这次失败是什么

近场障碍检查已通过，初始化请求已发出，轮式反馈累计移动约 0.129 m。相机跟踪无丢失，车身 IMU 约 200 Hz，无 IMU 间隙。SDK 处理 875 帧，但重力返回次数和惯性健康帧数均为 0，所以未向 RTAB-Map 放行建图里程计。

`waiting_localization` 是初始化监督状态机的停止后等待状态；它同时检查惯性健康及地图就绪，不能据此推断旧地图重定位有问题。这轮首先没有完成 SDK 惯性初始化。

## 确定的问题及修改

1. cuVSLAM 节点位于 `/d455_vio/cuvslam_odometry`，原 YAML 的节点键未套上命名空间，配置没有生效。原统计实际写在 `/tmp/cuvslam_validation_statistics.json`，而不是本次运行目录。现在 `RewrittenYaml` 加入 `root_key='d455_vio'`，实际加载配置及路径覆盖均验证通过。
2. 当前固定 SDK 的重力初始化至少需要 20 个关键帧、覆盖至少 2 秒，默认关键帧最长间隔为 60 秒。短程、低速、纹理持续可跟踪时，不一定生成足够关键帧。[SDK 状态机](https://github.com/nvidia-isaac/cuVSLAM/blob/main/libs/pipelines/tracker_state_machine.h)
3. 新增 `initialization_keyframe_period_s: 0.25`：仅在 SDK 尚未给出重力时，每约 0.25 秒请求一个真实图像关键帧；其它帧保持自动选择。SDK 给出重力后永久停止该会话的特殊采样。设置为 `0.0` 可关闭采样策略。没有修改 SDK 源码，也没有降低 SDK 的关键帧数量、时间跨度、求解成功条件或外层健康阈值。
4. 新增实际关键帧数量、近期数量及初始化请求数量，统计每秒原子写入本次 `cuvslam_statistics.json`。`initialization_diagnostics: true` 启用 SDK 状态导出，用于当前接入验证。
5. 超时拆分为 `inertial_initialization_timeout` 和 `mapping_or_localization_timeout`。`vio_init.json` 新增两个就绪布尔值，启动终端显示实际缺失项。

采样策略使用固定 SDK v17 的内部 per-frame 参数接口，是试验接入；后续升级 SDK 应重新核对接口和采样行为。自动运动仍为最高 0.06 m/s、0.20 rad/s；距离、偏角、超时、碰撞检查、连续健康窗口均保持原值。

## 实际传感器对比

两轮都关闭硬件输出，未输出底盘速度，没有手推或电机运动。

| 项目 | 原自动关键帧选择 | 初始化采样 0.25 秒 |
|---|---:|---:|
| 记录目录 | `20261008-121348` | `20261008-121500` |
| 关键帧总数 | 1 | 28 |
| SDK 重力初始化 | 未完成 | 完成 |
| 初始化监督状态 | 等待惯性 | succeeded |
| 建图就绪 | false | true |
| 实际底盘速度消息数 | 0 | 0 |

目录均在 `/home/orin/ros2_ws/.runtime/mapping_cuvslam_validation/` 下。

第二轮 SDK 日志记录 `IMU INIT DONE ... num_kfs=20`、重力模长 9.81 m/s²。惯性健康经过原连续 2 秒检查后，建图门控开放，RTAB-Map 输出地图及 `map -> r680_mapping_floor`，手柄交接状态达到 succeeded。IMU 约 200.33 Hz、原始里程计约 29.23 Hz；SDK 未报告跟踪丢失、无效位姿、IMU 间隙或输入丢弃。跟踪计算耗时 P95 约 19.76 ms，输出年龄 P95 约 46.58 ms。

退出后二维地图、数据库及导航索引成功写入测试目录的 `maps/map-2026-10-08-1`，数据库完整性检查通过；没有写入用户正式 maps。退出后无本次启动进程残留。

这是静止实传感器启动与保存验证，尚未验证移动建图精度、退化场景恢复或导航表现。原失败运行没有关键帧统计，不能把复采的关键帧数量当作原运行测量值。

## 软件验证与启动

Release 编译通过；前端运动输出契约、初始化采样周期/关闭/初始化后退出、原初始化监督与故障门控、建图交接回归、实际 launch 命名空间参数展开均通过。测试不通过串口发送运动。

命令保持不变，关闭当前独立底盘和手柄后，由操作者在空旷起步区启动：

```bash
cd ~/ros2_ws
./r680_mapping.sh --cuvslam --auto-vio-init
```

关闭电机输出的启动验证使用 `--dry-run`。结束按 Ctrl+C，地图保存逻辑保持原有流程。

代码：`/home/orin/ros2_ws/src/wla_r680_navigation` 与 `/home/orin/ros2_ws/src/wla_cuvslam_navigation`。原用户未提交的 Nav2、RViz、Web 修改保持不动。
