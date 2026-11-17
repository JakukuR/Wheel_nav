# 2026-11-17 关闭预览的温度对照

用户要求：只输出深度图和无色点云，检验debug图像是否造成高温。

已停止原始预览组7683和深度预览组7684（核对每个组员命令后发送SIGINT），共6个kalo_jpeg。保留HTTP调试台/Foxglove，页面不再有实时JPEG。只启动Sunplus及Light ESS FP16；DICOTA、RGB去畸变、RGB染色均不运行。

```bash
# 车上独立终端运行；先停止已有同类采集/网络和预览进程，避免重复占用。
cd /home/onana/ros2_ws
./robot_camera.sh run sunplus --publish-fps 10
# 另一终端
./robot_stereo_depth.sh --model light_ess --max-rate 5 --no-color
```

深度：`/camera/stereo/depth/image_raw`，16UC1/mm；内参：`/camera/stereo/depth/camera_info`。
点云：`/camera/stereo/points`，原始左目光学坐标，xyz米。点云按订阅需求计算/发布；订阅后出数据。预览、投影和彩色点云话题仍被节点注册，但没有订阅时不计算/发布，话题名存在不表示仍在做图像处理。

本轮5分钟监测器实际订阅深度和点云，确保测速包含点云生成。没有改变模型、5Hz深度上限、10Hz双目ROS发布、GPU时钟/风扇或电控。相机物理60FPS MJPEG解码仍进行，ROS限频不等于限解码；GPU原有min=max1173MHz。

与上一轮同时移除了预览、RGB采集及染色，且启动温度不同，因此本对照只能判断整条附加图像链路的负担，不能单独量化JPEG编码贡献。没有据此认定散热已修好。

原始证据：车上`/home/onana/ros2_ws/.runtime/stereo_no_debug_20261117/`；本机同名outputs目录，含进程登记、每5s温度/负载、ROS订阅关系和最终结果。

## 5分钟结果

- 300s监测收到1470帧深度、1469帧点云，二者约4.948Hz（主动限5Hz）。末帧点云53061点，xyz全部有限。
- 最后一分钟平均CPU88.03°C、GPU85.00°C；整个窗口最高88.812/85.312°C。末段温度趋稳，没有97°C暂停。
- 采样中的处理耗时中位34.14ms；末帧GPU7.86ms。双目采集CPU约78%单核，网络/几何节点约16–18%单核。
- 相比上一轮完整图像/RGB链路CPU最高95.406°C、处理末帧64.37ms，精简后负担明显下降。不同起始温度、同时关闭多个环节，不能将约7°C差全部归给预览编码。
- 结束时保留双目采集PID22850、Light ESS PID22851运行，监测订阅者退出；点云在下游订阅时生成。网页服务保留，JPEG预览关闭。原有97°C锁定暂停继续有效，不是长期热稳定验收。
