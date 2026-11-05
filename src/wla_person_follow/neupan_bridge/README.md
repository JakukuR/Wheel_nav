# NeuPAN 进程

跟随 `local_planner:=neupan` 时调用的独立进程。代码是未修改的
[neupan_cpp](https://github.com/zhangkaiyuan007/neupan_cpp) `074718d`，这里只有一个调用它的 `bridge.cpp` 和配置 `r_la.yaml`。

- neupan_cpp 是 GPL-3.0，链接它的 `neupan_bridge` 同样受约束，所以放在独立进程里。对外分发前要确认。
- 产物在 `~/ros2_ws/.runtime/neupan/`，不进 Git，换机器要重新构建：

```bash
src/wla_person_follow/neupan_bridge/build.sh <neupan_cpp 074718d 目录> <OSQP v1.0.0 与 osqp-eigen v0.11.2 的 prefix> <car_50x40.bin>
```

系统自带的 OSQP 太旧，不能用。
