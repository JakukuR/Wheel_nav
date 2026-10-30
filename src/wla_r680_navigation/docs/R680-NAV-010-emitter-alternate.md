# R680-NAV-010 红外投射器逐帧交替

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
- 深度仍使用全部帧，其中投射器关闭帧的深度在弱纹理处较差；`depth_to_points` 按深度元数据只用亮帧留作后续。
- 转发在 Python 中进行（2 路 640×480 单通道约 15 Hz），CPU 占用需实测，必要时改 C++。
