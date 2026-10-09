# 导航使用人工编辑的二维地图（2026-10-09）

## 问题与行为

原导航虽然为 Web 指定了编辑后的 `map.yaml/map.pgm`，Nav2 静态层仍订阅 RTAB-Map 根据数据库生成的地图。因此 Web 擦掉的障碍可能仍阻挡规划。

现在 `r680_nav.sh --map <地图名>` 将选中的 `map.yaml` 同时传给导航静态地图和 Web。`map_server` 是 `/r680/d455/map` 的发布源，RTAB-Map 的数据库栅格改为发布 `/d455_slam/localization_map`，仅供调试。RTAB-Map 仍使用选中地图的数据库工作副本执行旧地图匹配和 `map→odom` 修正；删除 PGM 像素不会改变数据库的视觉定位内容。

建图模式不启动静态 map_server，仍由 RTAB-Map 向 `/r680/d455/map` 发布实时地图，保存流程沿用原链路。全局动态层、局部避障和碰撞保护继续使用实时 D455 数据。

## 参数与启动

```bash
cd ~/ros2_ws
./r680_nav.sh --map map-2026-10-08-1 \
  --test-recovery --cuvslam --auto-vio-init --enable-motion
```

原启动命令不变，重新启动后加载编辑结果。继续使用 `accept_navigation_map_edit` 接受人工编辑并更新清单校验，参见 `R680-MAP-EDIT-001.md`。

直接使用 launch 时，新参数是 `navigation_map_yaml:=/绝对路径/map.yaml`；为兼容已有调用，默认沿用 `web_map_yaml`。导航模式中 Web 优先使用 `navigation_map_yaml`，即使另传了不同的 Web 文件，也不会与规划底图分离。正常定位导航没有指定二维地图时会明确拒绝启动，不再静默回退为数据库地图；关闭状态估计的独立图测试仍可自行发布地图。

`navigation.json` 新增 `static_map_source`、`static_map_topic` 和 `rtabmap_debug_map_topic`，便于核对本轮来源。

## 文件

- `launch/bringup.launch.py`：静态地图节点、独立生命周期管理、导航与建图条件、RTAB-Map 话题隔离。
- `scripts/r680_nav.sh`：传入选中地图和运行元数据；工作区根目录脚本同步更新。
- `package.xml`：声明 `nav2_map_server` 运行依赖。
- `test/test_navigation_static_map.py`：启动配置回归和无硬件 ROS 验证。
- `CMakeLists.txt`：注册配置回归测试。

## 验证与边界

在隔离的 ROS Domain 174 中，不启动底盘、相机或运动控制；使用实际 launch 中的 map_server、生命周期管理器和 RTAB-Map Include，数据库只打开临时副本。

检查选中编辑地图 `map-2026-10-08-1` 的全部 275×299 栅格、分辨率及原点与 map_server 发布结果一致；确认 map_server 已激活，导航地图只有它一个发布者；真实 RTAB-Map 数据库地图在独立调试话题，向调试话题发布冲突栅格不能覆盖导航静态地图。

配置测试覆盖建图模式保留实时地图、导航缺失地图的明确拒绝、Web 与导航使用相同文件以及含单引号路径。原 cuVSLAM 参数重写、地图存储和建图运行记录回归同时核验。

本修改不修复 VIO 位姿突跳、旧地图匹配失败，也没有验证实际行驶。实时传感器再次检测到的障碍仍会进入动态/局部障碍层。
