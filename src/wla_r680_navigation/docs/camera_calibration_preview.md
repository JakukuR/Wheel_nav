# 三相机标定预览与采集

网页：<http://192.168.50.1:18081/calibration.html>，也可从 Kalo 调试台顶栏进入。

左目与右目来自同一 Sunplus 拼接帧，USB 左半幅对应物理右目、顺时针旋转 90°；USB 右半幅对应物理左目、逆时针旋转 90°。转正后各为 720×1280；DICOTA RGB 为 1920×1080。三路原始图像配置发布 30 fps。网页单独使用 C++ `kalo_jpeg` 生成完整分辨率 JPEG，限速 10 fps、质量 80；它只供摆板预览，不用于标定计算。三画面独立更新，网页不保证三相机曝光同步。停止更新超过 1.5 秒时会覆盖停帧提示。

## 启动

在 `/home/onana/ros2_ws`，分别在三个终端执行（已运行的进程不要重复启动）：

```bash
./robot_camera.sh run both
./robot_debug_web.sh run
./robot_calibration_view.sh
```

关闭网页后 JPEG 编码器自动退订原始图像；原始相机发布仍继续。预览脚本退出会回收自己启动的三个编码进程。此流程不启动检测、导航或运动。

日志位于 `.runtime/robot_cameras.log`、`.runtime/robot_debug_web.log` 和 `.runtime/robot_calibration_preview.log`。

## 标定板

用户于 2026-11-16 确认：6×6，完整标签黑色外框边长 32.5 mm，标签间隙 9.75 mm。配置已保存到 `config/uvc_rig/aprilgrid_measured_6x6_32p5mm.yaml`：`tagSize: 0.0325`，`tagSpacing: 0.3`。PDF 文件名标称的 90 mm 不适用于这张缩印实物。

## 录制原始数据

页面不会自动开始录制。采集时固定相机焦距、分辨率和安装位置，慢速移动平整标定板，覆盖画面中央、四角、近远距离和不同倾角；用于三相机外参的帧需要共享板的观测。不要用网页 JPEG 或 YOLO 的缩放图做本次标定。

在车上新的终端执行：

```bash
cd /home/onana/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/onana/ros2_ws/config/uvc_rig/fastdds_uvc.xml
mkdir -p /home/onana/calibration_data
ros2 bag record --storage mcap \
  -o "/home/onana/calibration_data/three_camera_$(date +%Y%m%d_%H%M%S)" \
  --topics /camera/sunplus/left/image_raw /camera/sunplus/right/image_raw \
  /camera/dicota/color/image_raw \
  /camera/sunplus/left/camera_info /camera/sunplus/right/camera_info \
  /camera/dicota/color/camera_info
```

按 Ctrl+C 正常结束录制。相机未经标定时 CameraInfo 内参为占位信息，录制它不代表内参已求得。三路未压缩 BGR8 在 30 fps 时约 353 MB/s（约 21 GB/分钟），实际频率与磁盘吞吐应在录制时复查，先短录检查，再正式采集。

## 本轮验证（2026-11-16）

在三路网页预览开启时进行了 8 秒只读 ROS 观测（包括发现时间）：转正前左/右原始话题均约 30.01 Hz，RGB 约 28.26 Hz，三路 JPEG 约 9.95–10.00 Hz；图像时间戳均递增。实际浏览器确认三幅完整画面可见、刷新正常，并使用 NVJPG 编码。未开始正式 bag 录制，未验证磁盘持续录制吞吐。

2026-11-16 修正物理左右目映射及安装方向：旋转在采集端完成，`image_raw`、CameraInfo、预览和新标定共同使用 720×1280，保留完整像素与双目原时间戳。旧的 1280×720 标定不能直接用于此模式；若已录制旧方向的数据，须与新模式分开处理。标定导出和深度节点同时支持旧横幅/新竖幅并严格按标定分辨率校验。

转正后复测：左右目各 30.02 Hz、RGB 29.88 Hz；观察到 164 对左右完全相同 header 时间戳，三路各只有一个发布者，CameraInfo 尺寸/帧名与图像一致，未发布伪造内参或深度。网页显示两路正立人像及 RGB。若物理调整镜头以得到完整横屏视野，必须重新核对旋转选项与成像模式，再开始正式标定。
