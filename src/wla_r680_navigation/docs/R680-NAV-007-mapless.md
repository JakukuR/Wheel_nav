# R680-NAV-007 无图导航与单次匹配放行

## 问题

- 运动门控绑定 `localization_ready`（需要 watchdog 确认地图匹配），不在已建图区域时车永远不能动。
- 全局代价地图静态层直接使用 RTAB-Map 地图；未定位时 map→odom 是数据库残留的旧值，地图障碍位置错误。
- 首次放行需要两个视角一致（R680-CUVSLAM-006），开机转身时间内常凑不齐。

## 改动

| 位置 | 内容 |
|---|---|
| `vo_watchdog` | 新发 `/r680_nav/vo_odometry_healthy`（`!fault ∧ !timeout ∧ odometryReady`，不要求地图）；`startup_view_consistency: false`，单次验证匹配即放行 |
| `interface_monitor` | 新发 `/r680_nav/motion_ready`：与 `localization_ready` 相同输入，但 watchdog 项换成 `vo_odometry_healthy`。`localization_ready` 含义不变 |
| `command_guard`（launch） | `health_topic: /r680_nav/motion_ready` |
| `navigation_motion_session.py` | `--health-topic`，默认 `/r680_nav/motion_ready` |
| `navigation_map_mode`（新） | 全局静态层改读 `/r680_nav/navigation_map`：已定位时转发真实地图，否则发布以车为中心 40 m 空白栅格；接近边缘、未确认的 map→odom 移动或 odom_epoch 变化时重建（触发代价地图 resize，清空全局障碍层）。丢失需持续 3 s 才回到无图；建图模式透传。模式发布在 `/r680_nav/navigation_map_mode` |
| `nav2*.yaml` | 静态层 `map_topic: /r680_nav/navigation_map` |
| `navigation_goal_bridge` | 无图时拒绝 `map` 系目标；机器人系目标在到达时转换到 odom 系；odom_epoch 变化时取消本桥发出的 odom 目标 |
| `web_gateway` / `app.js` | 显示"无图模式·定位中"；无图时网页地图点选被拒绝（`goal_status=rejected_mapless`） |
| `r680_nav.sh` | 就绪改为里程计健康（`motion_ready`），打印导航模式 |

## 状态

| 状态 | 条件 | 运动 | 全局地图 |
|---|---|---|---|
| 未就绪 | 传感器或里程计不健康 | 停 | 保持上一模式 |
| 无图 | `motion_ready ∧ ¬localization_ready`（从地图模式回落需持续 3 s） | 允许 | 空白栅格 + 实时障碍层 |
| 地图 | `localization_ready`（持续 0.5 s） | 允许 | 真实地图 |

无图期间 odom 系目标经规划器每次重规划重新变换到 map，重定位导致的 map→odom 跳变不会改变其世界位置。

## 验证

- `mapless_navigation`（新）：启动无图且栅格以车为中心；map→odom 跳变重建；map 目标被拒、机器人系目标转换为 odom；epoch 变化取消 odom 目标；匹配后转发真实地图并接受 map 目标；短时丢失保持、持续丢失回无图；建图透传。
- `frontend_interface`：`motion_ready` 与 `localization_ready` 分离，watchdog 话题过期时两者都为 false。
- 全部 30 项 `wla_r680_navigation` 测试通过。

## 限制

- 未实车验证。Agent `navigation_task_gateway`（未提交 WIP）发送 map 系目标，无图时会被拒绝。
- 单次匹配认错时没有自动撤销，依赖局部避障兜底。
- `nav2_recovery_test.yaml` 的 TimedObstacleLayer 仍按 `localization_ready` 清空。
