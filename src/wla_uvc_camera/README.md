# 新机 UVC 双目 / RGB 采集与深度

目标：Sunplus 2560×720 拼接双目（左右各 1280×720），DICOTA 1920×1080 RGB，ROS 2 Jazzy。完整流程见工作区 `docs/新机器人三相机与IMU标定方案.md`。

- `uvc_camera` 采集 MJPEG，连续读取并保留最新帧；V4L2 使用 3 个驱动缓冲，避免单缓冲造成隔帧丢失。左右来自同一帧，保留同一主机接收时间。
- `uvc_depth` 将 RGB 中央 1440×1080 裁到 640×480；有真实 rig 标定时先去畸变，并更新内参。未标定时 `CameraInfo.k[0]==0`，仅输出二维图像。
- 有标定时，以 640×360 校正双目，经 StereoSGBM（可选 BM）得到三维点，变换到 RGB 光学坐标，再投影到裁切图。多点同像素使用最近 Z，输出 `16UC1` 毫米深度，无效值为 0。当前过滤范围 0.15–5 m 是工程初值，须经实测确认。
- 没有默认内参、基线或外参；文件分辨率、单位、SO(3) 和基线均校验。模板不含可冒充标定的数值。
- RGB 和双目的曝光不同步；25 ms 为主机接收时间的近似配对初值。分别保留原始时间戳，不用重打时间戳掩盖差异。深度计算超过新鲜度窗口则丢弃。
- 不广播机身 TF，不启动 IMU、电控、导航或 Agent 运动任务。新机 URDF 和标定尚待提供/采集。

```bash
cd /home/onana/ros2_ws
./robot_camera.sh probe
./robot_camera.sh run both
# 另一个终端，无标定时先做二维裁切。
./robot_camera.sh pipeline
# 有实际标定后替代以上 pipeline：
./robot_camera.sh pipeline --calibration config/uvc_rig/rig.yaml --algorithm sgbm
./robot_perception.sh check
./robot_perception.sh run
```

新入口默认 `ROS_DOMAIN_ID=0`、Fast DDS，与观察到的真实电控默认配置一致；可通过环境变量覆盖。ROS 域一致不等于 IMU 数据已恢复或接口已验收。

原始图像使用 `/camera/{sunplus/left,sunplus/right,dicota/color}/image_raw`，对应 CameraInfo 同目录。处理后为 `/camera/processed/color/image_raw`、`/camera/processed/color/camera_info`、`/camera/processed/aligned_depth_to_color/image_raw`。深度只在有效标定和配对图像可用时发布，二维检测保留现有 `/r680/perception/*` 协议话题。

软件验证：`ROS_DOMAIN_ID=197 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest src/wla_uvc_camera/test`，目标机 21 项通过，包含拆帧、校验失败、裁切内参、Z-buffer，以及 BM/SGBM 合成 2 m 平面、ROS 深度/RGB 保留各自时间戳和错时拒绝测试；它们不构成实机深度精度验收。

标定文件导出：`PYTHONPATH=src/wla_uvc_camera python3 -m wla_uvc_camera.export cameras-camchain.yaml --output config/uvc_rig/calibrated_v1`。导出目录必须不存在，cam0/1/2 顺序和话题固定为本机左/右/RGB；通过独立验收后设置 `WLA_UVC_CALIBRATION_DIR`，采集器和 pipeline 会加载同一目录的真实标定。

大图像跨进程传输使用 `config/uvc_rig/fastdds_uvc.xml`（128 MiB SHM / 8 MiB 消息）。新脚本只为自己设置 FASTRTPS_DEFAULT_PROFILES_FILE，保留显式覆盖；不改现有电控。默认配置实测曾有右目丢帧，调整后正式入口三路及裁切约 30 Hz。
