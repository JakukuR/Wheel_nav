# 三相机标定预览与采集

网页：<http://192.168.50.1:18081/calibration.html>，也可从 Kalo 调试台顶栏进入。

左目与右目来自同一 Sunplus 拼接帧：USB 右半幅为物理左目，USB 左半幅为物理右目；用户调整安装后两路各旋转 180° 转正，当前输出均为 1280×720 横屏；DICOTA RGB 为 1920×1080。三路原始图像配置发布 30 fps。网页单独使用 C++ `kalo_jpeg` 生成完整分辨率 JPEG，限速 10 fps、质量 80；它只供摆板预览，不用于标定计算。三画面独立更新，网页不保证三相机曝光同步。停止更新超过 1.5 秒时会覆盖停帧提示。

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

## 当前模式与验证（2026-11-16）

早前安装曾使用两路相反的 90° 软件旋转，发布 720×1280 竖幅。用户物理调整镜头后，去掉 90° 旋转的实测画面为上下倒置横幅，因此最终采用 `--swap-eyes --left-rotation 180 --right-rotation 180`，得到正立的 1280×720。每帧像素及原捕获时间戳完整保留，CameraInfo 使用同一输出尺寸和帧名，预览没有独立的额外旋转。

标定使用当前横屏模式与最终刚性安装。旧竖幅数据、旧安装标定不能直接混入；即使旧图和当前图都为 1280×720，也须核对实际方向、镜头对应和安装状态。当前内参仍未标定（fx=0），没有发布深度。网页只是预览，不会自动录包。

最新实测见 `.runtime/landscape_check/report.json`；检查包含三路频率、图像/CameraInfo 尺寸、唯一发布者、时间戳递增和双目同戳计数，网页另行确认人物与场景正立。

横屏复测：双目各约 30 Hz，RGB 在三路原始图像和 JPEG 同时观测时为 23.67 Hz，单独观测为 28.50 Hz（采集申请 30 fps，不将配置值当作实测值）。预览约 9.7–10 Hz。CameraInfo 与图像的尺寸和帧名一致，三路各只有一个发布者，时间戳递增，6 秒观测得到 179 对双目同戳。RGB 单独复测后又检测到整机重启，已重新启动相机与网页；进程不是开机自启服务。
