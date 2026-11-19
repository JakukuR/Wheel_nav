# 探索复用Follow规划，2026-11-19

mode3点目标与mode0网格绕障统一调用`route_to_point`（Python）/`routeToPoint`（C++）。共享原有Dijkstra可达端点、净空代价及可选A*优化，保持原Dijkstra接口可用。正式`sim_follow.yaml`及launch默认改为`astar`，显式`grid_route:=dijkstra`仍可用。MPC、动态障碍、深度时效、租约和最终停车包络不变。

能力包增加数值字段`grid_route_lookahead_m`。外部Kalo v2.2的`Runtime/vehicle_b.py`以共享前视距离加两倍观察区域半径作为探索最短候选段；当前0.66m，原0.16m。候选不足时沿用现有观察/无点处理，不缩短该距离。候选BFS只用于上层选点，Follow仍逐周期拥有实际路线和控制。

构建：`colcon build --packages-select wla_follow_local wla_person_follow --executor sequential`，2包完成；pybind/GCC ABI提示与LTO串行警告，未见编译错误。

软件检查：`test_goal_modes.py`、`test_native_planning.py`、`test_follow_core.py`合计99项通过；Kalo的subject_vehicle_motion/formal_exploration/vehicle_clearance/vehicle_l1_motion合计59项通过；`build/wla_follow_local`下ctest 4组通过。涵盖绕箱A*、完整引擎逐值一致性和点目标观察/停车交接。新增测试的前两轮轨迹比较断言不适用于MPC加密/圆滑，修正为几何判据；失败记录保留。

证据和源码备份在`.maintenance/follow-shared-planning-20261119/`。Kalo不是Git仓库，其增量补丁存于Agent仓库`deploy/kalo/v2.2-shared-route-lookahead.patch`，根证据报告同步更新。本轮保留原有未提交的到点间隙、L4转向保护等其他改动。

install已更新；没有启动Agent或发送运动目标，核查时B服务inactive、无跟随后端。下次原入口加载新代码。没有昨日原始地图和候选评分，不能据此声称实车已消除单侧循环或已完成运动验收。
