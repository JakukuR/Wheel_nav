# 当前深度与地图查询数值扩展

构建两个 Python 扩展：

- `_wla_sim_geometry`：深度反投影、地面/孤立簇过滤、射线槽及距离场查询。
- `_wla_follow_core`：C++ 跟随核心，目前是滚动深度地图 `RollingMap`（阶段 1）和跟踪部件
  `VisualTrack` / `PeopleTracks` / `TargetHistory` / 参考曲线（阶段 2，见
  wla_person_follow/PY_CPP_ARCHITECTURE.md），由 `numeric_backend:=cpp` 启用。
  `src/numpy_blas.h` 记录与 numpy 逐位相等所需的舍入行为。

C++ 单测在 `test/cpp/`，每个文件一个 ctest。
当前 tracker 与 MPC 位于 wla_person_follow；已删除旧 `_wla_follow_local` 的局部规划器、
TargetKalman 和 object_close，避免并存两套跟随控制链。
