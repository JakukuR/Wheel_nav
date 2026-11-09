# cuVSLAM initialization speed request

> 2026-11-09 核对：本文保留所述版本的设计/变更与验证证据；其中旧参数、包名、IP、启动/权限步骤不自动代表当前部署。当前框架见 [工作区总览](../ros2_ws_architecture.md)，操作见 [启动与参数](../ros2_ws_operations.md)。历史实测结果不因本轮文档核对而重新获得验收。

Operator requested peak forward speed 0.20 m/s, peak angular speed 0.50 rad/s,
and at most 2 seconds of motion. The existing waveform is retained: forward
0.10--0.20 m/s and angular 0.50*sin(pi*t/2) rad/s during the moving phase.

The operator explicitly retained the original budgets: target wheel travel
0.16 m, maximum travel/radius 0.25 m and maximum absolute turn 0.35 rad.
Those conditions or earlier VIO readiness can stop motion before 2 seconds.
The existing 20-second initialization completion deadline remains: 2 seconds
limits excitation, not the time allowed for inertial/localization readiness.

A separate cuVSLAM initializer config sets these requests. OpenVINS retains
its original initializer config and guard limits. The cuVSLAM guard caps are
0.20 m/s and 0.50 rad/s. Measured overspeed thresholds are 0.30 m/s and
0.75 rad/s, so the old 0.12/0.35 thresholds do not reject the new requests.
Emergency stop, takeover, obstacle stopping and authorization still apply.

No tests or physical-motion runs were performed, per the operator's instruction.
Compilation status is reported separately at delivery.
