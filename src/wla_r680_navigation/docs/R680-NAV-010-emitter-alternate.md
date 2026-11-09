# R680-NAV-010 红外投射器逐帧交替

> 2026-11-09 核对：本文保留所述版本的设计/变更与验证证据；其中旧参数、包名、IP、启动/权限步骤不自动代表当前部署。当前框架见 [工作区总览](../../../docs/ros2_ws_architecture.md)，操作见 [启动与参数](../../../docs/ros2_ws_operations.md)。历史实测结果不因本轮文档核对而重新获得验收。

## 问题

`d455_cuvslam.yaml` 打开投射器（`emitter_enabled: 1`）。散斑是相机自己投出的，随相机一起运动；cuVSLAM 在红外图上跟踪这些点会污染运动估计。白墙这类弱纹理处散斑几乎是唯一纹理，误导最严重。关掉投射器则深度在弱纹理处出现空洞，避障变差。

## 方案

- 相机 `r680_camera.sh run --infra --emitter alternate`：`depth_module.emitter_on_off: true`，投射器逐帧交替。
- `emitter_frame_splitter`（随相机启动）：按 `/r680/d455/infra1/metadata` 的 `frame_emitter_mode`（缺失时用 `frame_laser_power_mode`）判断每帧投射器状态，只把投射器关闭的 infra1/infra2 帧转发到 `/r680/d455/infra{1,2}_dark/image_rect_raw`。元数据与图像按时间戳配对，先后到达均可；无元数据的帧最多缓存 8 帧后丢弃。
- 导航 `r680_nav.sh --dark-infra`：cuVSLAM 的 `left_topic/right_topic` 改读 `*_dark`（launch 参数 `dark_infra`）。

键名已在运行中的 D455 上确认（2026-10-29 的深度元数据含 `frame_emitter_mode: 1`、`frame_laser_power_mode: 1`）。

## 验证

- `emitter_frame_splitter`（新）：真实元数据解析；只转发投射器关闭的成对帧，图像先到或元数据先到都正确；无元数据帧被淘汰。

## 限制与待办

- 未在相机上验证：需要用 `--emitter alternate` 重启相机（会中断其他人的链路，需批准）。
- 交替后暗帧约 15 Hz，cuVSLAM 帧率减半；需实测跟踪质量。
- 避障 `depth_to_points` 仍使用对齐到彩色的全部深度帧，其中投射器关闭帧在弱纹理处较差；改用亮帧留作后续。
- 转发在 Python 中进行（2 路 640×480 单通道约 15 Hz），CPU 占用需实测，必要时改 C++。

## 红外建图（RTAB-Map `slam_image`）

- 实测（2026-11-02，相机 alternate）：`depth/image_rect_raw` 与 infra1 时间戳逐帧相同，内参相同（640×480，fx=389.34），深度元数据逐帧 0/1 交替。D455 深度本就在 infra1 坐标系计算，infra1 + 原始深度天然逐像素对齐，无需对齐到彩色。
- splitter 增加第三路：投射器开启的深度帧转发到 `/r680/d455/depth_lit/image_rect_raw`（复用 infra1 元数据按时间戳路由）。
- launch 参数 `slam_image:=color|infra`（默认 color）决定 RTAB-Map 的 `rgb/depth/camera_info`：

  | slam_image | dark_infra | 图像 | 深度 | 同步窗口 |
  |---|---|---|---|---|
  | color | 任意 | color | aligned_depth_to_color | 0.025 s |
  | infra | false | infra1 | depth | 0.025 s |
  | infra | true | infra1_dark | depth_lit | 0.045 s |

  暗帧图像与亮帧深度相差一帧（33 ms），运动中深度与特征有轻微错位（0.3 m/s 约 1 cm，0.5 rad/s 约 6 px），对关键帧特征深度和 5 cm 栅格可接受。
- `r680_mapping.sh --slam-image infra --dark-infra`：写入 `run.json`，保存时进入地图 `map_info.json` 的 `slam_image`（受 manifest 校验；旧地图缺省为 color）。`r680_nav.sh` 从所选地图读取并传给 bringup，保证定位与建图同源；`--dark-infra` 时相机检查要求 `--emitter alternate` 和三路 splitter 输出。
- 彩色流保持开启，避障点云、跟随、语义不变。
