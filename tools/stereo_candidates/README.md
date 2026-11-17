# 轻量双目候选实测（2026-11-17）

本机是 **Orin NX 16 GB**，不是 Nano。JetPack 7.2-b187 / L4T39.2、CUDA13.2、TensorRT10.16.2.10、VPI4.1.3；GPU 原有锁频1173MHz。没有套用其他硬件的公开FPS。模型来源、版本和SHA256见 `provenance.json`。不在Git中存权重、引擎或运行缓存。

## 结论与同输入离线对比

同一组实际相机图先按现有标定校正为640×360，逐个运行，每个3次预热、20次测量。下表取中位数；输入转换/CPU归一化、H2D、推理、结果D2H计入“主机耗时”，不含相机采集、校正、米制投影、ROS/Web。GPU时间仅围住TensorRT enqueue；VPI不冒充单独GPU计时。各模型推理尺寸不同，见表。范围0.15–5m，置信度门禁不同，**有效比例不是精度**。

| 候选 | 模型输入 | GPU ms | 主机 ms | 有效米制深度比例 | 结果 |
|---|---|---:|---:|---:|---|
| Light ESS，FP16 | 480×288 | 7.83 | 14.66 | 73.42% | 已接入C++ ROS；默认候选 |
| Full ESS，FP16 | 960×576 | 21.55 | 48.73 | 46.98% | 引擎与ROS契约测试通过，可手动切换 |
| VPI CUDA SGM，默认置信度 | 640×360/maxDisp128 | — | 11.09 | 92.92% | 离线跑通；白墙散点明显 |
| VPI CUDA SGM，严格对照 | 同上 | — | 10.45 | 53.83% | LR置信度65023、uniqueness0.95；空洞增加 |
| VPI OFA SGM | 同上 | — | 9.69 | 96.68% | 离线跑通；不提供置信度图，弱纹理误匹配明显 |
| VPI OFA＋PVA＋VIC | 同上 | — | 15.33 | 67.32% | 离线跑通；INFERENCE置信度；仍有散点 |
| LAS2-S | 640×384，上下各补12行 | — | — | — | ONNX已生成；本机FP16构建失败 |
| LightStereo-S General | 同上 | — | — | — | ONNX已生成；本机FP16构建失败 |

Light ESS引擎42.56MiB，context需求35.81MiB，离线进程峰值RSS615.8MiB；Full ESS引擎33.49MiB、context142.69MiB、进程峰值RSS742.8MiB。引擎磁盘大小不能直接推算算力或总RAM；RSS也不等同系统总CUDA占用。Light ESS引擎检查169层、170个层输出中168为Half、2为Float，包含50个CaskConvolution和5个PluginV2，确实使用半精度；I/O接口仍为float32。

LAS2和LightStereo都不是“运行太慢退出”，而是在TensorRT构建阶段报 `KTM assertion failure ... convolutionTimingModel.cpp:65 shader != nullptr`，此前FFS也有同类CASK linkable/base shader错误。没有有效引擎，不能报Orin FPS，也没有静默改成FP32。两者已在本机x86 CPU ONNX Runtime上跑过同一对校正图，仅用于观察输出，不并入Orin测速。LAS2原始PyTorch与ONNX实图最大差0.000169px；LightStereo随机输入导出最大差0.000488px。下一步是隔离/修复本机TensorRT工具链兼容性，再测FP16；不要覆盖系统SDK、强行随机剪枝或用无关图伪造INT8校准。

## 数据定义

- 输入仅下方同步黑白双目，1280×720原图，USB右半=物理左，双路180°安装补偿。
- 现有真实标定基线0.03680308236134797m，640×360校正焦距469.7587575069932px，`bf=17.288570242494362 m·px`。
- ESS官方预处理：BGR→RGB，完整视野直接resize，`(pixel-127.5)/127.5`，CHW；置信度≥0.4才保留。输出视差乘`640/model_width`，再用同分辨率Q反投影。无上下填充。
- LAS2-S官方输入为RGB原始0–255、归一化在网络内部；LightStereo为RGB/255与ImageNet mean/std。两者640×360上下复制边界各12行到640×384，输出去掉填充，不改变水平视差比例。
- VPI输出为S16 **Q10.5，除32**，不是OpenCV的除16。CUDA/OFA+PVA+VIC显式应用confidenceMap；OFA单后端没有置信度图。BL输出直接按官方示例锁HOST_PITCH_LINEAR，不能调用不支持的S16_BL→S16格式转换。
- 调回640×360时，参与插值的置信度邻点必须全有效，禁止把无效区插成假深度。没有填洞、时域平滑或实机地面剔除。
- 黑色表示无有效深度。ESS的空洞和传统SGM的高覆盖噪声都需独立的实距/漏检验证；单张室内标定板场景不能替代白墙、地面、遮挡、近远距离测试。

## 实机连续采样与停止状态

Light ESS实际相机连续25s：发现后的有效采样116帧深度、116帧无色点云、116帧彩色点云，均约5Hz（主动限5Hz，不是推理极限）。末帧GPU8.12ms、包含点云/染色等处理64.37ms；xyz均有限，末帧点云54672点。RGB只染色；没有启用YOLO/跟随/SLAM/控制。无RGB也能输出深度，严格同左右时间戳与300ms源龄限制均通过独立域197测试，Light/Full ESS各收到20帧深度JPEG。

末轮8s网页数据检查：左/右原图69/69帧，RGB66帧，深度/投影深度30/30帧；HTTP200。客户端关闭了浏览器控制，因此只验证HTTP及实际ROS/JPEG数据，未声称浏览器目视验收。

此前较长的一轮CPU升至96°C，故主动停止。降温后的最终25s采样最高CPU95.406°C/GPU89.937°C；保留97°C暂停门禁，结束后**已停止本轮相机、RGB、网络，保留网页和预览服务**。没有修改风扇/时钟/电控模型/开机启动。短测通过不表示可持续运行；应先解决散热接触/风道问题，再做长测。

RGB染色要求有效标定CameraInfo。没有传`--calibration`时RGB仅二维、K=0，会被染色拒绝。RGB和双目都降到10Hz时最近帧相差约43ms，被25ms门禁拒绝；**最终使用双目ROS10Hz、RGB ROS30Hz、RGB处理30Hz**恢复染色，没有放宽配对门禁。相机曝光仍没有硬件同步。

## 复现离线实验

在Orin、工作区根运行；以下离线程序不启动相机或运动。不要与实时网络并行测速。

```bash
cd /home/onana/ros2_ws
cmake -S tools/stereo_candidates -B .runtime/stereo_candidate_build/build -DCMAKE_BUILD_TYPE=Release
cmake --build .runtime/stereo_candidate_build/build -j2
# 同一组640×360校正输入（历史样本，非当前画面）
probe=.runtime/stereo_candidate_build/build
pair=.runtime/stereo_reference
"$probe/model_probe" tools/stereo_candidates/profiles/light_ess.json "$pair/left_rect.png" "$pair/right_rect.png" .runtime/stereo_candidates/light_ess_repeat
"$probe/model_probe" tools/stereo_candidates/profiles/ess.json "$pair/left_rect.png" "$pair/right_rect.png" .runtime/stereo_candidates/ess_repeat
"$probe/vpi_probe" cuda "$pair/left_rect.png" "$pair/right_rect.png" .runtime/stereo_candidates/vpi_cuda_repeat strict
"$probe/vpi_probe" ofa "$pair/left_rect.png" "$pair/right_rect.png" .runtime/stereo_candidates/vpi_ofa_repeat
"$probe/vpi_probe" ofa-pva-vic "$pair/left_rect.png" "$pair/right_rect.png" .runtime/stereo_candidates/vpi_hybrid_repeat
```

`model_probe`输出原始float32视差、毫米深度PNG、统一色标JPG、结果JSON；`vpi_probe`输出原始float32视差、视差JPG与耗时JSON，其valid_fraction是正视差比例，不能与范围过滤后的valid_depth_fraction混用。离线配置路径按工作区根解析。VPI目前保留离线基线，没有冒充ROS避障已接入。

```bash
# 使用与TRT10.16匹配的官方ESS包和aarch64插件；不通用复用其他设备的engine。
python3 tools/stereo_candidates/build_engine.py \
 --onnx models/dnn_stereo_disparity_v4.1.0_onnx_trt10.16/light_ess.onnx \
 --engine models/dnn_stereo_disparity_v4.1.0_onnx_trt10.16/light_ess_new_fp16.engine \
 --plugin models/dnn_stereo_disparity_v4.1.0_onnx_trt10.16/plugins/aarch64/ess_plugins.so \
 --report .runtime/stereo_candidates/light_ess_new_build.json
```

构建限时600s、开始≤90°C、任一温度≥97°C或MemAvailable<1500000KiB即停止；只停止自己创建的子进程，不改变电源模式。1GiB workspace不是推理内存上限。输出已存在会拒绝覆盖，失败不把空engine当成功。

LightStereo导出辅助器只绕开训练包的全量导入，强制严格加载完整权重，未变更模型算子。CPU隔离环境使用PyTorch2.6、torchvision0.21、ONNX1.19.1、ONNXRuntime1.23.2、onnxsim0.4.36；LAS2使用timm1.0.31，LightStereo使用其官方要求的timm0.4.12（`pip --no-deps --target`到独立目录，进程局部导入）。原始工作文件和锁定依赖记录在本机`outputs/stereo_candidates_20261117`。调用：

```bash
python export_lightstereo.py --repo /path/to/OpenStereo --checkpoint /path/to/LightStereo_S_General.pth --timm-dir /path/to/timm_0_4_12 --output /path/to/lightstereo_s_640x384.onnx
# LAS2直接用上游export_onnx.py，version=las2/model_size=s/max_disp=192/width=640/height=384/simplify。
```

模型与部署依据均为原始来源：

- [NVIDIA Isaac ROS ESS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth)，[官方ESS参数和性能](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/isaac_ros_ess/index.html)。
- [ESS TRT10.16包](https://api.ngc.nvidia.com/v2/models/nvidia/isaac/dnn_stereo_disparity/versions/4.1.0_onnx_trt10.16/files/dnn_stereo_disparity_v4.1.0_onnx_trt10.16.tar.gz)。
- [LiteAnyStereo](https://github.com/TomTomTommi/LiteAnyStereo)，[LAS2-S权重](https://huggingface.co/tomtomtommi/LiteAnyStereoV2/blob/main/LAS2_S.pth)。
- [OpenStereo v2](https://github.com/XiandaGuo/OpenStereo/tree/v2)，[LightStereo部署](https://github.com/XiandaGuo/OpenStereo/blob/v2/deploy/README.md)，[General权重](https://huggingface.co/XiandaGuo/OpenStereo/blob/main/checkpoint/LightStereo/LightStereo-S-SceneFlow-General.pth)。
- [VPI算法](https://docs.nvidia.com/vpi/algo_stereo_disparity.html)，本机4.1头文件和`/opt/nvidia/vpi4/samples/02-stereo_disparity/main.cpp`为版本匹配依据。

原始证据：车上`.runtime/stereo_candidates/`、`.runtime/{light_ess,ess,las2_s,lightstereo_s}_fp16_build.{log,json}`、`.runtime/stereo_candidate_live/`、`.runtime/stereo_live/`与`.runtime/stereo_ros_test_{light_ess,ess}.json`。本机副本`outputs/stereo_candidates_20261117/evidence/`。该目录多轮实验并存，最终连续采样以`stereo_candidate_live/final_guard.json`及`sample_final.log`为准；旧10Hz未染色结果不是最终结果。
