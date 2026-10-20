# Python/C++ 分层与 pySLAM 参考

2026-10-20 核对 pySLAM revision `96019cfafcfc099ac9866884d7143a9ed1451a0d`。

pySLAM 的可借鉴点是：Python/C++ 核心保持对应接口，由统一入口选择；通过
pybind11 暴露 C++ 对象，并把主要数据和生命周期集中在核心中。官方材料：
[核心说明](https://github.com/luigifreda/pyslam/blob/96019cfafcfc099ac9866884d7143a9ed1451a0d/pyslam/slam/cpp/README.md)、
[后端选择](https://github.com/luigifreda/pyslam/blob/96019cfafcfc099ac9866884d7143a9ed1451a0d/pyslam/slam/__init__.py)、
[绑定入口](https://github.com/luigifreda/pyslam/blob/96019cfafcfc099ac9866884d7143a9ed1451a0d/pyslam/slam/cpp/py_module/cpp_core_module.cpp)。
这里只参考架构，未复制其实现，也未安装 SLAM 依赖。

## 本版已经落地

- `sim_node.py`：ROS 消息、TF、任务服务、发布权限、时效和停止处理。
- `sim_runtime.py`：输入适配，以及 `process_step(job)` 这一整帧计算边界；
  独立进程持有地图/滤波/规划状态，最多一个在途任务，不逐点跨进程调用。
- `sim_core/`：冻结 Python 数值参考，保留原仿真策略与常量；可直接用于离线回归。
- `wla_follow_local/src/sim_geometry.cpp`：实际深度反投影/三维变换，pybind11
  批量数组接口，计算释放 GIL，输出内存由返回数组持有。
- status 明确报告 `numeric_backend=python_reference` 和
  `native_frontend=cpp_deproject`，不能把当前版本称为全 C++ MPC。

ROS 回调与 Python 规划放在同一进程的两个线程时，隔离测试出现约 173 ms
计算耗时，超过 150 ms 结果预算。数值计算改为独立进程后，相同 ROS 测试通过，
没有放宽源龄或求解结果有效期。此结果不代表与 YOLOE 并发的最坏延迟已验证。

## 下一步 C++ 核心按相同接口替换

拟定义 `reset(reference_key, pose, time)`、`update_observations(batch)`、
`plan(snapshot)`、`diagnostics()` 四个粗粒度操作。Python 参考与 C++ 实现遵守
相同输入/输出类型、坐标、时间戳、失败原因和参数；后端选择独立于
`output_mode=direct|mux`，不能改变运动授权。

C++ 核心应一次拥有滚动地图、距离场、目标历史、动态轨迹和候选缓冲；
MPC 在 C++ 内直接查询地图并做完整碰撞检查。避免每步调用 Python map.query，
也避免 Python/C++ 各维护一份可能不一致的地图。切换数值后端前必须停止任务，
重置参考状态，禁止把旧后端正在求解的结果带入新后端。

数组尽量批量、连续、有明确只读生命周期；可零拷贝时也必须保证所有者存活，
异步工作不得引用已经回收的 ROS 图像缓冲。当前 `_wla_sim_geometry` 对不兼容
数组允许显式转换/复制，不宣称零拷贝。进程间传输当前也存在复制，计入延迟。

迁移顺序：地图更新/查询和轨迹碰撞 → 采样 MPC → 路径搜索与外侧参考。
每一步先用同一输入对比 Python 参考，再测 Orin 耗时；可行性不能因浮点差异
变宽松。当前两组 65 s 走廊的 2602 帧零误差结果作为算法基线，真实适配层的
缺数据停车、无解候选否决和不伪造空闲起步区保留独立测试。

C++ 原生数值测试、pybind11 数组/内存测试、ROS 集成和归档仿真一致性分别验证。
尚未实现的 C++ 后端不能静默回退并仍报告 cpp；本版只交付实际存在的参考后端。
