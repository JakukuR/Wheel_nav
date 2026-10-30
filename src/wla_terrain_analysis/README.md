# wla_terrain_analysis

坡面 / 台阶 / 跌落分析，供 Nav2 与人物跟随使用。输入为前向 D455 深度，姿态来自轮速 + 车身 IMU。

## 来源与许可

核心思路移植自 CMU `terrain_analysis`（Ji Zhang，Carnegie Mellon University；
`autonomous_exploration_development_environment` jazzy 分支，`package.xml` 声明 BSD，
仓库本身没有 LICENSE 文件）：在重力对齐的里程计坐标系里累积多帧点云、按时间衰减，
用每格高度的低分位数估计地面，输出每个点“高出局部地面多少”。代码为重写，未直接复制。

针对本车（前向深度相机，坡 ≤20°、台阶 ≤5 cm）的改动：

| | CMU | 本包 |
| --- | --- | --- |
| 地面 | 0.2 m 格 + 3×3 邻域分位数 | 0.1 m 格自身分位数 + 稳健局部平面（以本格高度为种子，2 cm 离群） |
| 坡度 | 不显式计算 | 平面倾角，> max_slope_deg + slope_margin_deg 且四周有数据才判陡 |
| 台阶 | ext 版相邻格高差 < 0.5 m | 相距 0.3 m 的两格各自只用本侧格子拟合平面，两平面在线段上不相交且间距 > max_step 即台阶 |
| 跌落 / 下沉 | considerDrop 取绝对高差（Diablo 实车关闭） | 默认关闭（`consider_drop: false`）：不判下沉、跌落和下台阶；打开时地面格后面一段未观测再接到低于 max_step 的地面即跌落边 |
| 动态 / 残影 | 全部按 decayTime 衰减 | 本帧看到的格子只保留最近 replace_age 内的点 |
| 噪声 | minBlockPointNum 10 + 3×3 邻域汇总；规划器 pointPerPathThre 2 | 障碍点需 ≥3 点且占本格 30%；凸起格需 ≥8 点（`min_residual_points`）；只靠地面判阻挡的格（陡坡/台阶/凸起）周围 8 格里至少 2 格也阻挡（`min_blocked_neighbours`）；均需连续 min_confirm 帧 |
| 远处 | 同样处理 | terrain_range（3.5 m）外不判坡/台阶/凸起（D455 深度误差超过 5 cm） |

## 节点

- `terrain_odometry`：`/wheel/odom_50hz` + `/wheel/imu/data_raw` → `/r680_nav/terrain_odom`
  （`terrain_odom` → `r680_mapping_floor`，6 自由度）。横滚/俯仰为陀螺积分加加速度计重力修正
  （前 2 s 静止估计陀螺零偏）；轮速沿车体 x 轴三维积分，上坡时 z 增加。不发布 TF。
- `terrain_analysis`：D455 对齐深度 + CameraInfo + 相机安装 TF + 上述里程计 →
  - `/r680_nav/d455/points`、`points_safety`：障碍点，基座坐标系水平偏移，**z = 高出局部地面的高度**；
    不可通行的陡坡/台阶/跌落格以 z = 0.2 m 的格心点表示；
  - `/r680_nav/d455/clearing_points`：参与分析的地形点（z = 离地高度，截断到 −0.05…0.45 m）；
  - `/r680_nav/terrain_map`：里程计坐标系 XYZI，intensity = 离地高度（CMU terrain_map 格式）。

话题名与 `wla_r680_navigation/depth_to_points` 相同，Nav2 配置不用改；两者不能同时运行。

## 验证

- `test_terrain_core`（无 ROS，模拟 D455 光线投射，噪声 σ = 0.005·距离²）：平地无阻挡；10/15/20° 坡可通行，
  25/30° 被拦；3/4 cm 台阶通过，6/8 cm 上台阶被拦，下台阶默认不拦（consider_drop 打开时 10 cm 跌落被拦）；
  平地上 25 簇 ±8 cm 离群点不产生只靠地面判定的阻挡格；箱子单独成障碍；
  单帧噪声块不发布；物体移走后 0.4 s 内清除；车已在 20° 坡上时前方坡面仍可通行。
- `test/test_terrain_node_ros.py`：渲染的 20° 坡 + 箱子 + 车上实际相机安装外参，障碍云只含箱子
  （及其相邻 0.25 m 内的边缘格）。
- `test/test_terrain_odometry_ros.py`：模拟 20° 上坡，俯仰 −20.0°，z/x = tan20°。

实车静止抽查（2026-10-29，平地办公室，原地不动）：单帧约 7 ms（最大 12.5 ms），9 Hz；
障碍格与 depth_to_points 重合 99 格，阻挡格在彩色图上落在墙根、柜子底板和柱脚处
（`figures/live_overlay_*.png`：before 为步长全跨度标记，after 为只标中间格）。
还没有在真实坡道 / 台阶上开过车；`terrain_range` 与噪声相关阈值需要用坡道数据复核。

## 启用

```bash
./r680_nav.sh --map <地图> --terrain
```

或 `ros2 launch wla_r680_navigation bringup.launch.py terrain_analysis:=true …`；默认关闭，
关闭时仍启动原 depth_to_points。诊断话题 `/r680_nav/terrain_blocked_cells`
（odom 系格心，intensity = 10×类别 + 原因：1 下沉、2 凸起、3 两侧平面台阶、4 阴影后跌落、5 点高出地面）。

## 人物跟随

`ros2 launch wla_person_follow follow.launch.py terrain:=true`（或 `./r680_person_follow.sh terrain:=true`）：
跟随另起一对地形节点（话题 `/r680/person_follow/terrain/*`），跟随地图的障碍改用地形分析结果，
清除射线和目标处理仍来自深度，见 `wla_person_follow/HYBRID_TRANSPORT.md`。
