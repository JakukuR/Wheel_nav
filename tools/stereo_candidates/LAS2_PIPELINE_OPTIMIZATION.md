# LAS2 采集、预处理与点云优化实测（2026-11-17）

已实现单进程 V4L2→灰度 JPEG 解码→CUDA 融合预处理→TensorRT→GPU 反投影→ROS 深度/点云。LAS2-S 在真实相机上短测达到约20Hz，但热稳定性未通过，不能当作持续导航20Hz的验收结果。

## 参考方案与实际实现

参考 [SpacemiT perception](https://github.com/spacemit-robotics/perception/tree/4d6abfa85f7f313be63a9027e62992010b8e46b1) 的外部输入、融合校正/张量转换及缓冲复用思路。K3 SDK 的 NV12 DMA-BUF/OpenCL/RVV 路径不能直接移植到 Jetson。本轮独立实现 CUDA 后端，未复制其专有推理库。K3 公布的 LAS2-M 384×288 推理吞吐与本轮 LAS2-S 640×384 全链路数据不构成同条件对比。

- V4L2 直接读取2560×720 MJPEG，申请4个MMAP缓冲；相机60FPS排空，待处理槽只保留最新压缩帧。只按算法目标频率解码，避免将每帧展开为两路大幅ROS图像后再接收。
- 黑白相机使用CPU灰度JPEG解码。真实JPEG彩色解码与灰度复制三通道的最大差为0。本路径针对现有黑白双目；不能据此保证任意彩色相机的模型效果等价。
- CUDA一个预处理kernel完成物理左右交换、180°旋转、校正、上下各补12行、RGB CHW；复用设备缓冲，输出使用固定页内存。OpenCV remap 的1/32像素插值取整也保持一致。
- GPU完成Q反投影及R1转置坐标还原；CPU仍负责PointCloud2打包和ROS发布。导出stride2，最多57600点，坐标单位m，frame为sunplus_left_optical_frame。
- 不是硬件JPEG解码或端到端零拷贝。当前环境nvjpegCreateSimple返回6，未启用nvJPEG；未替换SDK或驱动。输入Mat仍需H2D，ROS消息仍有拷贝。
- direct-camera目前只支持现有2560×720黑白拼接相机和LAS2-S/M 640×384。CMake CUDA目标为Orin sm87。默认ROS输入路径和默认Light ESS模型不变。

## 实测条件和结果

设备为实机识别的Orin NX16GB Super（不是Orin Nano），JetPack7.2/CUDA13.2/TensorRT10.16.2.10；真实标定config/uvc_rig/calibrated_20261116/rig.yaml，基线约0.036803m，输出640×360。LAS2-S FP16、no-color、点云订阅开启，网页左右JPEG另限约2Hz；板上其他用户进程保持运行。

| 指标 | 原GPU频率1173MHz，30秒 | 临时GPU上限816MHz，计划120秒 |
|---|---:|---:|
| 实际测试时长 | 30.00s | 34.82s后热暂停 |
| 深度频率（收到首末帧间） | 19.86Hz | 19.46Hz |
| 点云频率 | 19.82Hz | 19.37Hz |
| 本节点CPU单核等效均值 | 25.80% | 25.99% |
| 推理GPU耗时P50 | 12.95ms | 17.33ms |
| 节点处理耗时P50 / P95 | 21.39 / 22.30ms | 26.04 / 27.19ms |
| JPEG解码P50 | 4.55ms | 4.59ms |
| 点云接收时源龄P50 / P95 | 47.73 / 56.05ms | 51.72 / 59.79ms |
| CPU/GPU最高观测温度 | 96.72°C | 97.56°C |
| 结果 | 短测通过 | 热稳定性失败 |

CPU均值包括同进程采集、解码、推理等待、后处理及发布，不包括网页服务、JPEG编码器、测量订阅器或板上其他进程；不是峰值保证。处理耗时包含解码到ROS发布调用，不包含等待下一帧或订阅者执行。源龄来自V4L2 monotonic时间映射到ROS时钟，尚未验证是曝光时刻，不能当作曝光到导航延迟。分位数来自0.5s状态采样（不是每帧trace）。测试时间包含启动；Hz基于首末收到帧，不等于总帧数除总时长。

另一次未及时挂接观测器的20Hz试运行也触发97°C暂停，不作为性能通过证据。临时频率测试已恢复原min=max1173000000；风扇PWM255保持不变。没有停止/修改电机、LLM等其他用户服务，没有启用导航或运动。

## 验证

- CTest: geometry_matching_projection、stereo_input_contract、stereo_gpu_invalid_geometry，3项通过。
- packed_pipeline_test：真实彩色解码、真实灰度解码、随机彩色三组；CUDA与CPU输入张量最大差0、视差最大差0、毫米深度最大差0、有效点mask差0，xyz最大误差<1e-6m。
- GPU异常输出测试覆盖NaN/Inf/负/零视差：输出视差和深度为0，点无效；有效视差的米制深度保持。
- 点云最大57600点且xyz有限；这不证明米制精度或弱纹理/遮挡场景无漏检。尚未验收实距误差、机身外参、地面剔除与Nav2障碍层。
- HTTP200，ROS左右预览JPEG有输出并检查图像。浏览器自动化不可用，本轮不声称浏览器实时目视验收。

## 构建与复现

```bash
cd /home/onana/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select wla_uvc_depth --parallel-workers 1 --cmake-args \
  -DWLA_BUILD_STEREO_TRT=ON -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
ctest --test-dir build/wla_uvc_depth --output-on-failure
# 先停止已有Sunplus采集和深度实例；wrapper与原相机共用独占锁。
./robot_stereo_depth.sh --model las2_s --max-rate 20 --no-color \
  --direct-camera /dev/v4l/by-id/usb-SunplusIT_Inc_USB_Global_Camera-video-index0
```

这是实验命令，不承诺持续20Hz。温度≥97°C锁定暂停，必须降温并重启；没有自动反复重启或提高温度门槛。LAS2-M可选择但本轮未做M的全链路吞吐测试。

在相机无人占用时运行真实输入等价性探针：

```bash
build/wla_uvc_depth/packed_pipeline_test models/stereo_candidates/las2_s_fp16.engine \
  config/uvc_rig/calibrated_20261116/rig.yaml \
  /dev/v4l/by-id/usb-SunplusIT_Inc_USB_Global_Camera-video-index0 \
  .runtime/las2_pipeline_opt_20261117/contract
```

左目预览改订阅/camera/stereo/left_rect/image_raw（bgr8），与深度同640×360校正网格。JPEG编码器仍输出/r680/debug/calibration/left/compressed，depth-lite.html显示“校正左目”。节点预览和JPEG编码器各有限频，实测左目约1.3Hz、深度预览约2Hz；均不代表实际深度处理频率。

完整机器日志、观测脚本、逐次状态和图像保存在.runtime/las2_pipeline_opt_20261117/；紧凑测试结果见las2_validation/pipeline_results_20261117.json。没有将模型、构建产物或运行缓存入库。

## 最终低负载档位

恢复原GPU频率后，最终运行上述direct-camera命令的`--max-rate 10`版本。60秒实机检查通过：深度9.952Hz、点云9.899Hz，CPU单核等效均值14.31%，处理P50/P95为21.63/22.55ms，最高94.25°C，113/113状态有效。左右JPEG各约2Hz。节点PID95498仅供本轮溯源，不应作为以后停止进程的依据。

点云订阅回调的源龄P50/P95/P99为46.36/57.14/344.98ms；尾部出现超过300ms的接收延迟。节点发布前的300ms源龄检查不保证DDS传输与接收队列延迟；导航消费者必须另外验证时间戳、丢弃过期帧，并验证有负载条件下端到端时效。当前保留10Hz观察档位及97°C保护，一分钟通过不代表长期热稳定认证。
