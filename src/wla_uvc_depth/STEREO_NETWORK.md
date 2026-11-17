# 同步黑白双目网络实验（2026-11-16）

入口：`/home/onana/ros2_ws/robot_stereo_depth.sh`；节点为原生 C++20
`wla_uvc_depth/uvc_stereo_trt`，TensorRT 10.16，CUDA 13.2，ROS Jazzy/domain 0。
不启动跟随或运动，不接入控制命令，不修改电控模型。

## 图像与坐标

下方 Sunplus 原始左右目必须同时间戳，1280×720、bgr8，沿用已核对的
swap-eyes 与双路 180° 安装补偿。使用 `config/uvc_rig/calibrated_20261116/rig.yaml`
真实标定（基线 0.03680308236134797 m）。先校正到 640×360，再缩小到
320×180，上下各复制边界 6 行，送入 320×192 双目网络。网络只使用这两张图。
RGB/灰度三通道按 ImageNet 均值/标准差转 CHW；去掉输出填充后把视差
插值回 640×360，乘水平缩放倍率 2，再用同分辨率 Q 反投影。
640×360 输出是插值后的输出尺寸，实际匹配分辨率为 320×180。

深度 `16UC1` 单位 mm、0 无效，预览/点云导出范围 0.15–5 m；不填洞。
`/camera/stereo/depth/image_raw` 为校正左目光学 Z，配套 CameraInfo 和
`sunplus_left_rect_optical_frame`。仅发布内部 raw-left→rect-left 静态旋转 TF。
点云由 R1 转置还原到 `sunplus_left_optical_frame`，xyz 单位 m，步长 2 导出；
深度图保留全输出网格。

RGB 来自独立的 `--rgb-only --rgb-mode full` 处理，完整去畸变 640×360，
没有中央裁切。染色必须有匹配时间戳的有效 CameraInfo，且 K/D 与当前标定
全幅缩放一致。RGB 按主机接收时间 ±25 ms 近似配对，**没有硬件曝光同步**。
原始左目三维点经 `T_rgb_left` 投影到去畸变 RGB 像素，采用 Z-buffer 可见性
检查（10 mm 容差，工程初值）；无颜色/遮挡/视野外点保留灰色，不删除几何点。
没有可配对 RGB 时，无色点云和双目深度仍输出。

| 话题 | 用途 |
| --- | --- |
| `/camera/stereo/depth/image_raw` | 校正左目深度，16UC1/mm |
| `/camera/stereo/depth/camera_info` | 校正左目投影内参 |
| `/camera/stereo/disparity/image_raw` | 输出网格像素单位视差，32FC1 |
| `/camera/stereo/points` | 无色几何点云，不依赖 RGB |
| `/camera/stereo/points_color` | RGB 可配对时的彩色点云 |
| `/camera/stereo/aligned_depth_to_color/image_raw` | 静态标定投影的 RGB 网格深度 |
| `/camera/stereo/status` | 源龄、耗时、温度、停用原因 |

输入/输出发布前源龄不得超过 300 ms，未来时间容忍 50 ms；左右目必须严格
相同时间戳，不按 RGB 是否存在决定能否测深。后台仅保留一组最新待处理帧，
不积压推理。节点观察 CPU/GPU 温度，达到 97°C 则锁定暂停推理；这是实验
保守预算，低于本机 99°C 的软件降频点。降温后须人工重启，禁止静默恢复。
未触发时的 stream 状态不代表机器人下游控制的时效门禁已通过。

## 模型与构建

[NVIDIA C-Fast-FoundationStereo](https://huggingface.co/nvidia/c-fast-foundationstereo)，
14.6M 参数；公开动态 ONNX：
`onnx/c_fast_foundationstereo_bp2_dynbatch_dynhw.onnx`。
SHA256：`2c79bbb274dc0aef687a8732b987077e9e3aa26ff9dd653141cda48c57dad5d3`。
原始模型约 99 MiB，受 NVIDIA Open Model Agreement 管理；不随 Git 提交权重。
配置来自同模型 cfg.yaml，8 次迭代，max disparity 416。

本机引擎：`models/depth_comparison/c_fast_stereo_192x320_fp32.engine`，
FP32 I/O，TensorRT 默认允许 TF32，未启用 FP16。模型仍是学习得到的双目视差，
双目基线提供米制尺度；稠密输出不能替代实测距离/障碍漏检验收。

```bash
cd /home/onana/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wla_uvc_depth --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DWLA_BUILD_STEREO_TRT=ON
ctest --test-dir build/wla_uvc_depth --output-on-failure
# 模型下载到 models/depth_comparison/c_fast_foundationstereo_dynamic.onnx 后：
python3 tools/stereo_depth/build_guarded.py
```

构建脚本固定 1×3×192×320 profile、1 GiB 工作区，并记录温度/可用内存；
CPU/GPU 任一≥97°C、可用内存<1500000 KiB、或超过 600 秒则终止该构建。
开始时温度须≤90°C；阈值是本次实验预算，不是硬件规格。
工作区上限不等于推理占用。本机首次编译约 281 秒，生成约 102 MiB 引擎。

640×360 匹配的 FP16 构建在 KTM/CASK 着色器处报错；FP32 的 512 MiB
工作区不足，4 GiB 工作区尝试触发 97°C 保护。没有把这些尝试记为成功。
本机 TensorRT 还打印设备型号兼容性警告；引擎为同一 Orin 本机生成，保留
构建/推理原日志，不能据此宣称跨设备引擎兼容。

## 启动和日志

以下为各独立终端前台命令，Ctrl-C 停止；不写开机自启。

```bash
cd /home/onana/ros2_ws
export WLA_UVC_CALIBRATION_DIR="$PWD/config/uvc_rig/calibrated_20261116"
./robot_camera.sh run both --publish-fps 20
# 另一终端：
./robot_camera.sh pipeline --rgb-mode full --rgb-only --max-rate 30
# 另一终端：
./robot_stereo_depth.sh --max-rate 2
# 只检查无 RGB 深度时可加 --no-color，不会使用单目预测补洞。
# 以下每条命令分别在独立终端运行（网页预览）：
./robot_stereo_depth_view.sh
./robot_calibration_view.sh
./robot_debug_web.sh run
```

网页 `http://192.168.50.1:18081/depth.html`，显示左右原图、双目深度、完整 RGB
与投影到 RGB 的深度。预览为只读；网页启动不是运动授权。
原始传感器固定 MJPEG 60/30 fps，`--publish-fps 20` 只减少 ROS 发布频率。

本次受管进程及 PID 记录 `.runtime/stereo_run_processes.json`。
日志 `.runtime/stereo_{cameras,rgb,network,previews,web}.log`；
实测 `.runtime/stereo_live/result.json`；独立域契约测试
`.runtime/stereo_ros_test.json`；离线基准 `.runtime/stereo_reference/result.json`。
进程应以登记的 PID/进程组核对命令后停止，不用宽泛 pkill 停电控。

`tools/stereo_depth/fan_boost_diagnostic.py` 为限时 900 秒风扇诊断，结束会恢复
原来的 nvfancontrol 服务，不改配置；`gpu_build_limit.py` 为限时 900 秒 GPU
306–612 MHz 测试约束，结束恢复保存的 min/max。两者需要 root，不能当开机
服务使用；它们不修改 CPU 或电控。提高风扇 PWM 不能替代散热接触检查。

## 已验证与边界

- C++ 几何与输入契约 2 项通过：合成 2 m 平面、0/NaN 视差拒绝、禁用传统
  matcher、图像尺寸/通道/归一化/边界填充检查。
- 隔离域 197 真模型测试：没有 RGB 时有深度与点云；有效 RGB 可染色；
  不同左右时间戳及过期输入不发布新深度。测试只覆盖 ROS 数据契约。
- 限频 612 MHz 离线基准，10 次 GPU 耗时约 219 ms、含预处理/拷贝约 221 ms。
- 实时限 2 Hz，25 秒收到深度、无色点云、彩色点云各 50 帧，约 2.02 Hz；
  每帧导出 57600 点，均为有限 xyz。末帧完整处理约 258 ms。
- 上述连续测试后曾触发温度锁定暂停，不能据短时采集宣称可持续稳定运行。
- 没有做卷尺距离/左右遮挡/弱纹理漏检验收，没有接入机身地面/障碍高度筛选、
  跟随、Nav2 或 SLAM。SGBM、Depth Anything V2、DepthART 均未运行。
- 本机浏览器访问被客户端设置禁用；使用 HTTP 与 ROS/JPEG 数据验收，
  未声称本轮已在浏览器目视检查页面。

最终状态：在降低 ROS 发布至 20 Hz、临时风扇最大 PWM 和 GPU 612 MHz 约束下，
第二轮同样触发温度暂停。`preview_check.py` 收到左右目/RGB 原图，因网络已暂停
未收到新的深度 JPEG，故本轮实时网页深度预览未验收成功。页面会显示
`thermal_pause_restart_after_cooling`，不把旧图当实时深度。首次 25 秒成功数据另存
`.runtime/stereo_live/first_25s_before_thermal_pause.json`；最新 `result.json` 可表示
暂停期间的零帧结果，不能覆盖/混称第一次成功记录。11 月 16 日结束测试时已停止相机/RGB/网络采集进程，并恢复临时风扇/GPU约束；
未修改开机自启。不能把该轮暂时出图当作持续稳定性验收。

## 2026-11-17 复测

重启后先做 5 秒独占原始 UVC 采集（仅拆分左右半幅并旋转 180°，无校正/
缩放/网络），两路均完整收图，没有采集错误。用户反馈一侧半幅发糊，目前
人物/白墙原图不足以判断局部失焦或硬件损坏，等待同距离高对比标定板对照；
不得因为网络输出稠密就认为相机完好。源图和读数在
`.runtime/stereo_health_20261117/`。未调整镜头焦距或标定参数。

默认 GPU 频率 1.173 GHz 下，同一原图对 10 次离线推理约 126 ms GPU、
128 ms 含预处理/拷贝。隔离域测试新增网络深度→JPEG 验证，收到 10 帧
深度 JPEG，同时无 RGB 测深、RGB 染色、错配/过期拒绝均通过。

恢复真实相机后，25 秒订阅窗口（发现后）收到深度、无色/彩色点云各 44 帧，
有效订阅段约 2.00 Hz；末帧 GPU 127.8 ms、完整处理 164.1 ms。
8 秒 JPEG 数据检查五路均到达：左目 77、右目 52、RGB 77、深度 15、
RGB 投影深度 15 帧。HTTP 200，未使用浏览器目视验收。
末帧 CPU/GPU 94.1/91.2°C，仍保留 97°C 暂停保护，不能据此承诺长时稳定。

今日进程记录 `.runtime/stereo_health_20261117/run_processes.json`；当前两路原图
与实验深度运行，RGB 仅染色。语音服务重启后曾自启，已按此前要求再次停止，
没有禁用它们的开机启动。半幅模糊尚未定位，米制误差、地面过滤与避障未验收。
