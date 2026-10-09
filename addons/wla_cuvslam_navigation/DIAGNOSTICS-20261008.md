# VIO 健康诊断日志（2026-10-08）

本次只增加诊断，不修改健康阈值、初始化窗口、看门狗重启或底盘放行逻辑。

每个 cuvslam_odometry 进程在 statistics_path 所在目录追加写入
`vio_health-<PID>.jsonl`，重启后使用新文件，保留上次故障证据。
可通过 `health_diagnostics_path` 指定日志路径。状态变化立即记录，重复状态最多 1 Hz。
就绪状态的周期证据只写文件；异常状态同时进入 ROS 日志。

记录内容：ROS 时间、图像/IMU 调用时间、进程号、已处理帧和 IMU 数、
SDK 观测数量（开启 initialization_diagnostics 时有效；否则为 -1）、warming_up。
观测数量不是健康门限，不能据此断言 VIO 一定有效或失效。

健康检查记录重力/IMU state 是否存在、重力模长、陀螺与加速度偏置范数和阈值、
速度估计时间间隔、稳定窗口的时间间隔、估计数据年龄。
位姿越界记录 dt、平移和旋转变化、速度限制，以及上一帧/当前帧位置。
IMU 断流记录最近注册的 IMU 时间；估计过期记录实际数据年龄。
statistics JSON 增加日志路径、最近诊断原因、事件数。

上一轮第二次故障只留下 `cuVSLAM inertial tracking unhealthy`，没有上述细项，
无法事后确定是重力、偏置、时间间隔还是状态对象失效；新日志用于下一轮确认。

TF 所有权：cuVSLAM Odometry 输出 odom→车体；RTAB-Map 输出 map→odom。
RTAB-Map 重定位不会回写本节点的局部位姿。cuVSLAM 内部优化调整需通过越界记录分析。

验证：编译实际 SDK 链接的节点，并运行 health_evidence、odometry_contract、
initialization_sampling 回归测试。没有发送运动指令；实车故障日志需下次运行确认。
