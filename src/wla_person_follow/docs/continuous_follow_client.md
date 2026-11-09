# Continuous explicit-ID follow client

> 2026-11-09 核对：本文保留所述版本的设计/变更与验证证据；其中旧参数、包名、IP、启动/权限步骤不自动代表当前部署。当前框架见 [工作区总览](../../../docs/ros2_ws_architecture.md)，操作见 [启动与参数](../../../docs/ros2_ws_operations.md)。历史实测结果不因本轮文档核对而重新获得验收。

From `/home/orin/ros2_ws`:

```bash
./r680_follow_trial.sh list
./r680_follow_trial.sh follow --target-id 'FULL_PERSON_ID' --continuous --allow-motion
```

`--continuous` removes the client-side duration deadline only. It is mutually
exclusive with `--seconds`. Without either option, the default remains 30 s;
explicit timed runs still accept 1–120 s. Continuous mode is valid only for follow.

Ctrl+C (SIGINT) and SIGTERM end the loop and request stop. Each command retains
its 1.5 s backend lease and the client renews approximately every 0.4 s; this is
not an infinite backend lease. Stale status, rejected/failed heartbeat and a
backend-ended task terminate the client and attempt stop. Existing identity,
target-loss, obstacle and sensor checks are unchanged. There is no automatic
restart or selection of a replacement person.

This script issues task service requests, not velocity messages. Stop response
acknowledgement is not a measurement of physical braking. Seven fake-client
unit tests cover continuous operation beyond 120 s, both signals, normal 30 s
expiry, heartbeat rejection, stale status, backend stop and argument conflict.
No ROS graph or physical motion is used by these tests. Deployment requires no
node restart, because the wrapper executes this Python file on each invocation.
