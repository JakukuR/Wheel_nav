# 原生C++同步双目深度（2026-11-17）

当前默认是 **Light ESS FP16**，替换此前的C-Fast-FoundationStereo FP32实验。四类候选的实测、失败日志、依赖和来源见 [候选实验记录](../../tools/stereo_candidates/README.md)。

节点 `wla_uvc_depth/uvc_stereo_trt` 使用C++20、TensorRT10.16、CUDA13.2、ROS Jazzy/domain0。没有启用跟随、SLAM、YOLO或运动；没有修改电控模型。VPI SGM目前是离线对照，LAS2/LightStereo的本机FP16构建失败，没有自动切换为FP32。

## 图像、几何与过滤

下方Sunplus原始1280×720 bgr8，严格同左右时间戳；保持USB左右半幅交换与双路180°安装补偿。`config/uvc_rig/calibrated_20261116/rig.yaml`为真实标定，基线0.03680308236134797m。左右先校正为640×360，再按模型预处理：

| `--model` | 网络输入 | 归一化 | 输出还原 |
|---|---|---|---|
| `light_ess`（默认） | 完整视野resize480×288 | RGB，(pixel−127.5)/127.5，CHW | 置信度≥0.4；视差resize640×360并乘640/480 |
| `ess` | 完整视野resize960×576 | 同上 | 同置信度；视差缩回640×360并乘640/960 |
| `las2_s` | 640×360上下各补12行→640×384 | RGB0–255，内部归一化 | 去填充；尚无可用FP16 engine |
| `lightstereo_s` | 同上 | RGB/255与ImageNet mean/std | 去填充；尚无可用FP16 engine |
| `cfast`（历史显式选项） | 320×180上下各补6行→320×192 | ImageNet mean/std | 去填充、放大视差×2；不会默认启动 |

ESS依赖官方同版本aarch64插件，必须先加载插件再反序列化引擎。所有模型接口仍为float32，内部半精度由引擎决定；不把接口dtype当内部精度。模型名、实际匹配尺寸、填充、置信度、来源、温度与时效均进入`/camera/stereo/status`。置信度空洞缩放时要求所有参与插值邻点都有效，绝不补成深度。

视差单位为640×360校正网格像素，用相同Q反投影。深度16UC1/mm、0无效、范围0.15–5m；`sunplus_left_rect_optical_frame`光轴Z。只有内部raw-left→rect-left静态旋转TF。三维点用R1转置还原到`sunplus_left_optical_frame`，xyz单位m；点云导出步长2，深度网格保留640×360。不发布未经确认的机身安装TF。

RGB独立去畸变全幅1920×1080→640×360，含像素中心缩放修正，无中央裁切。必须显式加载相同标定，CameraInfo的K/D、尺寸、frame与图像时间戳全部校验。三维点经`T_rgb_left`投影，用Z-buffer可见性检查（10mm容差，工程初值）染色；遮挡/视野外为灰色。没有可配对RGB时，深度和无色点云照常输出。RGB **不参与深度推理**。

RGB按主机接收时间±25ms近似配对，不能称作曝光硬件同步。输入与发布前源龄≤300ms、未来容忍50ms，左右必须完全同时间戳；只保留最新待处理帧，不积压。达到97°C锁定暂停，降温后人工重启；这是实验预算，低于观察到的99°C软件降频点，没有为出图放宽门禁。

## 启动

以下每条在独立终端前台运行，Ctrl-C停止；不要同时运行另一个`run both`或另一个深度后端。不写开机自启。先确认散热和已有话题发布者。

```bash
cd /home/onana/ros2_ws
./robot_camera.sh run sunplus --publish-fps 10
# 另一终端
./robot_camera.sh run dicota --publish-fps 30
# 另一终端：标定必须显式给出；环境变量也可，但不会跨独立终端自动继承。
./robot_camera.sh pipeline \
 --calibration /home/onana/ros2_ws/config/uvc_rig/calibrated_20261116/rig.yaml \
 --rgb-mode full --rgb-only --max-rate 30
# 另一终端
./robot_stereo_depth.sh --model light_ess --max-rate 5
# 停止上一个网络后可对比Full ESS；不会并行输出。
./robot_stereo_depth.sh --model ess --max-rate 5
```

`--no-color`可独立检查无RGB深度；`--engine`和`--plugin`可显式覆盖路径，但必须与模型/本机SDK匹配。缺失或空引擎直接失败，不回退FP32。wrapper用flock避免本入口重复发布。模型切换需要停止再启动；`--max-rate`允许0–15Hz（不含0），是主动上限，不是性能承诺。

相机物理采集仍是60/30fps MJPEG，ROS限频不等同降低传感器曝光帧率。最终测试分别设双目10Hz与RGB30Hz；两者同降10Hz曾使近邻相差43ms而无染色，因此不要以放宽25ms门禁解决。

网页预览各入口若未运行，在独立终端启动：

```bash
./robot_stereo_depth_view.sh
./robot_calibration_view.sh
./robot_debug_web.sh run
```

[深度调试页](http://192.168.50.1:18081/depth.html)显示左右原图、深度、全幅RGB、RGB网格投影深度；模型和当前有效性以实时状态栏为准。色标近红远蓝，黑色无有效深度，不填洞。页面只读。

## 话题

| 话题 | 内容 |
|---|---|
| `/camera/stereo/depth/image_raw`、`depth/camera_info` | 校正左目16UC1毫米深度及相同内参 |
| `/camera/stereo/disparity/image_raw` | 640×360，32FC1像素视差 |
| `/camera/stereo/points` | 无色几何点云，不依赖RGB |
| `/camera/stereo/points_color` | 有效RGB可配对时的彩色点云 |
| `/camera/stereo/aligned_depth_to_color/image_raw` | 标定投影的RGB网格深度；无RGB也可计算几何投影 |
| `/camera/stereo/depth/preview`、`aligned_depth_to_color/preview` | bgr8可视化，非测量 |
| `/camera/stereo/status` | 模型、配置、耗时、深度有效比例、颜色可配对、温度与停用原因 |

## 验证与当前状态

- CTest几何和输入/输出契约2项通过，新增四类模型归一化、水平视差倍率、无效置信度和空洞保留检查。
- 隔离域197，Light/Full ESS各20帧JPEG；无RGB有深度/点云，有RGB可染色，错左右时间戳/过期帧被拒绝。
- Light ESS真实相机25秒最终测试：深度、无色/彩色点云各116帧，均约5Hz；末帧GPU8.12ms、完整处理64.37ms，xyz均有限。五路网页JPEG全到达，HTTP200；没有使用被客户端禁用的浏览器控制，未声称浏览器目视验收。
- 此前连续数分钟CPU曾升到96°C；末轮最高95.406/89.937°C。最终短测通过后已停止本轮相机、RGB和网络，保留调试网页和预览服务，没有修改时钟、风扇或开机自启。长时运行需先解决散热，不能把5Hz短测当长期稳定性保证。
- 用户已反馈半幅模糊现象当前恢复；未更改镜头或标定，不据此认证硬件完好。
- 尚未做卷尺米制误差、弱纹理/遮挡漏检验收，未接入机身地面剔除、障碍高度筛选或避障控制。低于0.15m、超过5m与无效置信度点已剔除。

车上日志`.runtime/stereo_candidate_live/{sample_final,preview_final}.log`与`final_guard.json`为最终真实相机证据；`.runtime/stereo_live/{result,preview_check}.json`为该轮数据副本；`.runtime/stereo_ros_test_{light_ess,ess}.json`为独立域契约测试。离线原始结果和构建失败见`tools/stereo_candidates/README.md`。不要用旧轮PID或旧缓存图判定仍在运行。

历史FFS的FP32与失败FP16日志留在`.runtime/stereo_reference/`、`.runtime/stereo_health_20261117/`及本机`outputs/fast_stereo_20261116`、`outputs/stereo_quantization_20261117`；它不再是默认活动网络。语音服务曾随重启自启，本轮已按此前要求停止，没有禁用开机启动。
