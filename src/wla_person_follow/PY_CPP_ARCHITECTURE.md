# Python/C++ 分层与 pySLAM 参考

2026-10-26 当前架构：C++ `follow_transport` 持有 ROS 收发、TF、缓存、配对、
轮速积分和独立输出门禁；Python `hybrid_worker` 持有原任务、跟踪及地图/MPC。
两进程通过有界共享内存及 UNIX 消息通道交换数据，数值 worker 不再另起进程。
兼容适配器 `transport:=python` 仍使用下述原 ROS + ProcessPool 结构。
完整当前说明与验证边界见 [HYBRID_TRANSPORT.md](HYBRID_TRANSPORT.md)。

以下 2026-10-20 描述及延迟数字保留为历史架构证据。

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

## C++ 核心编码与构建规范（2026-11-09）

适用于此后新写的 C++ 数值核心（`FollowCore` 及其地图、跟踪、规划、控制部件）。
已有的 `sim_geometry.cpp` 不为改风格而改动，被迁移触及时再按本节整理。
规范参考 [pans](https://gitee.com/huajianke2026/pans)（2026-11-07 提交 `2337dcc`）的
命名与工程组织；只借鉴约定，未复制其代码。

### 命名

| 对象 | 规则 | 例 |
|---|---|---|
| 常量 | 全大写，下划线分隔 | `ROBOT_RADIUS_M` |
| 类、结构体、枚举类型 | 首字母大写驼峰 | `RollingMap`、`TrackState` |
| 非静态成员函数 | 首字母小写驼峰 | `update()`、`queryClearance()` |
| 静态成员函数、自由函数 | 首字母大写驼峰 | `FromConfig()`、`WrapAngle()` |
| 非静态成员变量 | `m_` 前缀，小写驼峰 | `m_resolution` |
| 静态成员变量 | `s_` 前缀，小写驼峰 | `s_instanceCount` |
| 函数参数、局部变量 | 小写，下划线分隔 | `robot_pose`、`cell_count` |

命名空间 `wla::follow`。头文件保护宏按路径：`WLA_FOLLOW_<目录>_<文件>_H`。

### 接口与对象

- 对外头文件放 `include/wla_follow/`，实现放 `src/`。`FollowCore` 头文件只声明
  `reset / updateObservations / plan / diagnostics` 等粗粒度操作，内部状态
  （滚动地图、距离场、目标历史、动态轨迹、候选缓冲）放在
  `struct Impl; std::unique_ptr<Impl> m_impl;` 中，pybind11 层只依赖这层窄接口。
- 持有状态的对象禁止拷贝与移动（五个特殊成员显式 `= delete`），只能由一个所有者
  持有；需要比较 Python 参考时由测试显式构造两份，而不是复制。
- 返回结果、不改状态的查询加 `[[nodiscard]]`，不会失败的加 `noexcept`；状态和失败
  原因用 `enum class`，原因字符串与 Python 参考一致，供 status 原样报告。
- 参数在构造或 `reset` 时由 Python/ROS 传入；不使用全局配置注册表或单例，以保证
  切换后端前能完整清空状态，测试之间互不影响。

### 失败处理

- 运动规划失败即停车：非法输入、非有限数、越界等一律抛异常（pybind11 转成 Python
  异常），本帧不产生命令。禁止"Release 下只记日志然后继续"的断言。
- `assert` 只用于内部不变量的调试检查，不能替代上面的输入检查。
- C++ 后端不可用时直接报错，不静默回退到 Python 参考并仍报告 cpp（见上节）。

### 构建

- C++20（Orin：g++ 13.3、CMake 3.28）。不在 CMake 中写死编译器路径，由 colcon 决定。
- 一个 INTERFACE 目标 `wla_follow_options` 统一选项：`-Wall -Wextra -Wpedantic`、
  `-ffp-contract=off`（与 Python 参考逐位比对的前提，不得移除）、
  Release/RelWithDebInfo 下 `-O2 -fno-omit-frame-pointer`（保留帧指针，便于在 Orin
  上 perf 剖析），Debug 下 `-O0 -g3`；可选覆盖率开关默认关闭。核心库另加 `-Werror`。
- 日志、断言中的 `__FILE__` 使用相对仓库根的路径，不泄露构建机绝对路径。
- 不引入新的大型第三方依赖（Boost、protobuf、abseil 等）；已有的 pybind11、Eigen
  足够。

### 测试

- C++ 数值单测：`test/cpp/*.cpp` 每个文件一个可执行文件、一个 `add_test`，不依赖
  gtest，由 colcon test / ctest 运行。
- 跨语言一致性（同一输入下 C++ 与 Python 参考逐元素比较）、pybind11 数组生命周期、
  ROS 集成和 A/B 回放仍在 pytest 中，验收标准见上节迁移顺序。

## 迁移进度

### 阶段 0+1：对比基线与 C++ 滚动地图（2026-11-09，a1ead05）

- C++ 所有者 `_wla_follow_core.RollingMap`（`wla_follow_local/include/wla_follow/rolling_map.h`、
  `src/rolling_map.cc`）逐语句对应 `sim_core/rolling_depth_map.py`：环形状态、滑窗、射线/地面/
  命中更新、车体足迹、启动先验、人物轨迹清除、`drop_established`，以及两张距离场的精确 EDT
  （整数平方距离的 Felzenszwalb 变换，结果与 scipy 逐位相等）。
- Python 侧 `native_map.NativeRollingDepthMap` 提供与参考实现相同的属性和方法：环形状态为只读
  视图，派生场每次重建返回新数组；`query`/`snapshot` 共用参考实现。
- 开关 `numeric_backend:=python|cpp`（launch 参数与节点参数，默认 python）。cpp 不可用时报错，
  不回退；切换后端从空地图开始；status 报告 `cpp_map_python_mpc`。
- `_wla_sim_geometry.ray_slots` 的射线采样移入 `src/grid_kernels.h`，两处共用，输出逐字节不变。
- 阶段 0 工具 `tools/map_replay/`：`rec_main.py` 在 domain 199 回放中记录全部地图调用，
  `compare.py` 把记录同时喂给 Python 参考与 C++，每次调用后比较全部状态与返回值。

验证：
- `test/test_native_rolling_map.py`：8 组随机 60 步场景，每次调用后所有数组与返回值完全相等。
- 2026-11-06 A* 包 60 s 回放记录约 6000 次地图调用，逐次完全相等。单次耗时 python → cpp：
  update 7.8 → 2.6 ms，rebuild 6.2 → 1.45 ms，slide 0.75 → 0.09 ms。
- 170 s 回放 A/B（各 2 次，跟随帧）：compute_ms p50 49 → 41 ms，p95 约 58 → 52 ms；
  depth_to_result p50 约 150 → 135 ms；状态、阻塞原因和命令分布不变。
- 测试：默认 python 251 项全过；以 cpp 为默认时地图相关测试全过。

### 阶段 2：跟踪与参考曲线（2026-11-09，b1ae353）

- C++ 部件（`wla_follow_local/include/wla_follow/tracking.h`、`src/tracking.cc`、
  `src/reference_path.cc`）逐语句对应参考实现：`TargetHistory`（固定时间栅格重采样）、
  `VisualTrack`（匀速 Kalman、新息门限、滑行衰减、预测）、`PeopleTracks`（其他人作动态障碍，
  保持首次出现顺序）、`BezierReference`、`BsplineCorners`。持有状态的对象不可拷贝；预测只拷贝
  滤波状态值 `KalmanState`。
- 逐位相等的前提：numpy 在 Orin 上走系统参考 BLAS/LAPACK（numpy 1.26.4，LAPACK 3.12），
  先测定再移植，写在 `src/numpy_blas.h`：
  - `matmul`/`dot`/一维 `norm` 按顺序以融合乘加累加；
  - `norm(axis=...)` 与逐元素运算不融合；
  - `solve` 为 dgesv：主元行交换、乘倒数、融合的 Schur 更新与三角回代；
  - `einsum` 按顺序求和，只有一段三次曲线时 numpy 去掉长度 1 的轴，改走融合乘加。
- Python 门面 `native_tracking.py`：`NativeVisualTrack`、`NativeTargetHistory`、`NativePeopleTracks`
  （任意 Python 键映射为 int64 id）、`reference_functions(backend)`。`numeric_backend:=cpp`
  同时切换 `FollowTracker` 的目标航迹、引擎的 `PeopleTracks` 和 `HybridFollower` 的两条参考
  曲线（构造参数 `references`）；status 报告 `cpp_core_python_mpc`。cpp 不可用时报错，不回退。
- `PeopleTracks.drop/keys` 取代直接操作字典；空路径、非有限输入在 C++ 中抛异常（参考实现同样
  抛异常）。
- 工具：`tools/map_replay/rec_tracking.py`（`WLA_TRACK_RECORD` 设置时由 `rec_main.py` 加载）在
  domain 199 回放中记录跟踪调用，`compare_tracking.py` 把记录同时喂给两种实现并逐次比较。

验证：
- `test/test_native_tracking.py`：随机测量流（门限拒绝、滑行、丢失、重复时间戳，5 组配置）、
  随机人群、各 3000 条随机历史与折线，返回值与全部状态完全相等；另做 6 万条曲线抽样，0 处不等。
- 2026-11-06 A* 包 170 s 回放：2614 次 people.update、2961 次 predict、2927 次 Bezier、3271 次
  B 样条，以及按目标航迹参数重放的 9139 次更新，逐次完全相等。单次耗时 python → cpp：
  B 样条 2.66 ms → 85 µs，Bezier 460 → 22 µs，people.update 173 → 41 µs，predict 94 → 15 µs。
- 170 s 回放 A/B（全 python 对比 cpp 地图+跟踪，跟随帧）：compute_ms p50 48.4 → 37.8 ms，
  p95 58.1 → 48.5 ms；depth_to_result p50 139.9 → 135.9 ms；状态、绑定、阻塞原因不变。
- 测试：默认 python 274 项全过；以 cpp 为默认（全部默认值改为 cpp 的副本）274 项全过；
  C++ ctest 2 项全过。

下一阶段：规划与控制（采样 MPC `HybridFollower.command`、`DetourPlanner` 含 Dijkstra、
`ApproachGoal`），验收为可行性判定一致、命令差 ≤1e-9。

### 阶段 3：规划与控制（2026-11-09，6c34417）

- C++ 部件（`wla_follow_local/include/wla_follow/planning.h`、`src/planning.cc`、
  `src/approach_goal.cc`、`src/csgraph.cc`）逐语句对应参考实现：
  - `HybridFollower`：采样 MPC `command`（13 档速度 × 25 档转向偏置、24 步滚动、底盘一阶滞后与
    偏航死区模型、已知空间/静态/行人三类安全判据、视场约束与代价各项），以及 `recover`、
    `_backup`（沿走过轨迹倒车，含停滞超时）、`_round_corners`、`_shortcut`、`_remember`；
  - `DetourPlanner`：边缘点绕行（两侧候选、保持侧、退出滞回）、网格 Dijkstra 路线、可选 A*
    细化、看侧翼；
  - `ApproachGoal.select`：物体表面去重、0.1 m 格点连通图、虚拟起点、矩形车体间隙、
    路线复用；
  - `ChassisYaw`、`ApproachTangent`。
- 逐位相等的前提（先测定再移植，`src/numpy_ops.h`）：
  - `np.argsort` 默认为 numpy 1.26.4 的 introsort（16 个以下插入排序，深度耗尽转堆排序），
    逐行移植，相等分数的次序也相同；
  - scipy 1.11.4 `dijkstra` 的 Fibonacci 堆逐操作移植，等代价路径的前驱相同；
    `coo_matrix.tocsr()` 按行列排序并合并重复边；
  - `np.interp` 不融合；`np.linspace` 两端精确，向量端点有一维步长为 0 时整体改为先除后乘；
  - cKDTree 最近距离为 `sqrt(fma(dy,dy,dx*dx))`，只与阈值比较，C++ 用精确的暴力最小值；
  - 2×2 块均值按内存顺序累加；`np.round` 为偶数舍入；Python 内建 `min/max` 的并列取前者。
- `_wla_sim_geometry` 的 `grid_query`、`grid_astar` 移入 `src/grid_kernels.h`，规划器与
  原模块共用，输出逐字节不变（随机 80 组比对）。
- Python 门面 `native_planning.py`：`NativeHybridFollower`（`c`、`last_*` 仍为普通属性，
  引擎 mode 4 照常覆盖；`detour._grid_route` 供 mode 3 点目标）、`NativeApproachGoal`。
  `TrackingPolicy` 的两个判据是纯函数，在 `policy_time` 求值一次后传入 C++。C++ 规划器只读
  C++ 地图（`NativeRollingDepthMap._core`），所以 `numeric_backend:=cpp` 时地图、跟踪、参考
  曲线、MPC、绕行和接近目标全部在 C++ 内；status 报告 `cpp_core_cpp_mpc`。cpp 不可用或传入
  Python 地图时报错，不回退。
- 工具：`tools/map_replay/rec_planning.py`（`WLA_PLAN_RECORD` 设置时由 `rec_main.py` 加载）把
  规划调用记录进地图记录流，与地图调用交错；`compare_planning.py` 逐次重建两种地图，再把
  每次规划调用分别交给两种实现比较（策略对象由 `recorded_policy.py` 记录）。
- 手工建图注入引擎的测试（`test_goal_modes`、`test_sim_port`）按引擎后端建图：C++ 规划器
  不接受 Python 地图。

验证：
- `test/test_native_planning.py`（55 项）：随机场景（横穿直线路线的墙、已知地面、未见侧翼、
  行人、各任务模式的参数、搜索策略、恢复、参考路径与朝向点）中连续 30 步命令、全部可行性
  计数、绕行报告、参考路径、卡住计数与倒车距离完全相等；覆盖边缘绕行、保持、Dijkstra 与
  A* 网格路线、看侧翼、倒车与恢复；接近目标的规划、复用与回退；网格路线、`chassis_yaw`、
  `approach_tangent`、采样点直接比较。
- C++ ctest 3 项：排序与 Dijkstra 的性质、端点、死区、无距离场时拒绝计算、恢复不前进。
- 2026-11-06 A* 包 170 s 回放记录 2878 次 MPC 调用与 14 次 reset（其中无可行解 113 次、
  倒车 25 次、恢复 9 次，含边缘绕行、看侧翼和保持），离线逐次重放到两种实现：命令与全部诊断
  完全相等。单次 `command` python → cpp：28.2 → 4.0 ms（含门面转换）。该包只有 mode 0，网格
  路线与接近目标由随机测试覆盖。
- 170 s 回放 A/B（各 2 次，跟随帧，numeric_backend python 对比 cpp）：
  compute_ms p50 49.2 / 49.8 → 13.2 / 15.8 ms，p95 59.0 / 61.7 → 22.8 / 23.5 ms；状态与
  原因分布在两次运行间的波动范围内。
- 测试：默认 python 329 项全过；以 cpp 为默认 329 项全过。

下一阶段（阶段 4）：单一 `FollowCore` 所有者，`process_step` 成为一次不持 GIL 的调用。
