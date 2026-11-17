# 原跟随点云前端接入验证（2026-11-17）

用户要求接入原跟随点云处理链路，不再另写地面算法。本次撤除独立 ground_filter C++ 实现、启动脚本、测试、网页面板和白名单。原自建地面地图/栅格此前已撤除。旧验证记录仅保留历史，不能视为当前运行方案。

## 复用范围

直接调用原 `wla_person_follow/sim_runtime.py::packet_from_depth` 使用的 `_wla_sim_geometry.deproject` 和 `filter_points`。实现仍为 `src/wla_follow_local/src/sim_geometry.cpp`，上述原代码均无改动。采用原深度范围 0.15–5 m、步长 4；原局部地面高度估计、5 cm 障碍高度判断和孤立簇过滤不改参数，不添加 RANSAC。

新增 `wla_person_follow/frontend_preview.py` 仅适配 LAS2 深度/内参、源时间戳、现有机身几何和 ROS 输出；由 `robot_follow_cloud_preview.sh` 启动。以 10 Hz 发布既有 `/r680/person_follow/depth_obstacles`，复用主调试网页现有“障碍体素”图层和原 PointCloud2 渲染。发现同话题有其他发布者即退出，避免与完整跟随节点竞争。此退出保护未另做实机冲突测试。

当前地面参考来自既有 `/robot_state/geometry` 的关节/IMU 支撑几何，依赖操作者确认双轮接触平地；不是新增视觉平面拟合，也不是接触传感器证明。相机机身安装使用现有 CAD 数据及光学轴约定，尚未做机身到光心的实测外参标定。输入需满足深度、内参、几何的时间有效性，失败发布空障碍点云并标明不可用。

输出坐标系 `follow_support_floor_preview` 为随支撑几何变化的预览坐标系，发布其到 `base_Link` 的真实坐标转换，绝不冒充全局 `r680_mapping_floor` 或补造轮式里程计。原深度时间戳保留。有效空深度产生空障碍/清除点集，不能解释为已观测自由空间。

## 当前边界

只读点云前端已接通。未启动人物识别/目标剔除、滚动地图、ESDF、规划或运动控制。当前未收到真实 `/wheel/odom_50hz`，完整跟随所需图像时刻位姿及导航 TF 尚未具备，因此没有宣称完整跟随链路已运行。原设备电控自启动进程未由本次操作启动或修改。

## 验证

- `colcon build --packages-select wla_person_follow --symlink-install` 成功。
- 原原生投影/高度过滤测试 2 项通过；原“地面不重标记、空深度保持未知”回归测试 1 项通过。
- 隔离 ROS_DOMAIN_ID=197 测试 10 项通过：纯平面、障碍坐标高度、过期几何、缺源时间、错误 frame、截断数据、深度停止、几何停止、有效空深度、恢复。
- 初次合成障碍块过小，被原孤立簇规则正确过滤；扩大测试目标后复验。另一初版测试误把空深度当作输入格式错误，后按原前端空集语义修正断言，并运行空深度地图回归；原算法未为测试修改。
- 30 s 实机采样：深度 19.975 Hz，点云 10.000 Hz；277 条前端状态均有效；该适配进程 CPU 占一个核 10.17%，并非全机总占用。
- 前端处理耗时中位 2.799 ms、P95 3.334 ms（包含适配/点云发布路径，不是纯 C++ 算法耗时）。障碍点数中位 2290。
- 抽查 12 帧，使用对应源深度、实际输出 TF、CAD 安装与校正 TF 再调用原 C++ 函数，与发布数组在 2e-6 m 容差内全部一致，时间戳均完全一致。
- 障碍话题发布者仅 1 个，旧自建滤除话题已消失。浏览器已实看原“障碍体素”10 Hz 图层正常绘制；RGB 人物识别未启动。

原始证据：`.runtime/follow_cloud/live_result.json`、`native_test.log`、`empty_depth_native_test.log`、`ros_contract_final2.log`。采样程序为 `.runtime/follow_cloud/measure_native.py`。当前进程记录 `.runtime/follow_cloud/processes.json`；当前深度记录 `.runtime/las2_384_20hz/depth_process.json`。进程号只代表本次验证时刻，不能跨重启复用。

主页面：http://192.168.50.1:18081/ 。查看下方俯视图“障碍体素”。深度有效尺寸 384×216，网络填充到 384×288，深度目标 20 Hz，网页预览与本次点云输出 10 Hz。
