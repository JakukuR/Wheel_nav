# C++ 双目深度与 RGB 对齐（2026-11-16）

当前已在 Orin 运行原生 C++20 `wla_uvc_depth/uvc_depth_cpp`：双目校正 640×360 → BM/SGBM → 米制三维点 → 原始左目 optical → RGB optical → 去畸变裁切后的 RGB 网格。本轮无电机、跟随任务、导航或 TF 发布。标定是初版，实际距离精度尚未用尺测量验收。

## 启动与停止

```bash
cd /home/onana/ros2_ws
export WLA_UVC_CALIBRATION_DIR=$PWD/config/uvc_rig/calibrated_20261116
# 相机已经运行时不要重复启动；仅在没有采集进程时执行：
./robot_camera.sh run both
# 另一个终端同样 export 上述环境变量：
./robot_camera.sh pipeline --rgb-mode crop --algorithm sgbm \
  --disparities 128 --block-size 5 --max-skew-ms 25 --max-rate 10
# Ctrl+C 正常结束 pipeline；对比 BM 时用 --algorithm bm 替换，不并行运行。
# 网页服务仅在没有运行时启动 ./robot_debug_web.sh run；另一个终端：
./robot_depth_view.sh
```

页面：`http://192.168.50.1:18081/depth.html`。左为 RGB，中为校正左目视角的视差，右为 RGB 视角的深度。深度预览近红远蓝，黑色无效，不填洞。原始相机预览仍为 `/calibration.html`。公共 pipeline 默认仍为 full（640×360）；本次显式 crop（640×480），未改变其他调用者的默认视野。

原始采集由现有 Python UVC 节点完成；新增的 ROS 缓冲/配对、图像处理、匹配、三维转换、投影、状态和发布全部由 C++ 节点持有。Python `stereo.py` 与旧 `pipeline.py` 保留为参考，主入口不自动回退。CPU OpenCV 原生库为 4.8；Python cv2 为 4.6。

## 标定与坐标

测量配置：`config/uvc_rig/calibrated_20261116`，源自两组实际录制联合求解，基线 36.803 mm。`provenance.json` 保留源 camchain 哈希。左右输入各 1280×720，必须继续使用当前左右交换和各旋转 180° 的采集映射。RGB 输入 1920×1080、连续自动对焦关闭；重新上电后需核对焦点、映射和系统时间，不能把焦点控制值一致当作独立光学验收。

裁切矩阵 A 对应 (240,0,1440,1080) → 640×480，sx=sy=4/9。使用 `cx'=sx*(cx-x+0.5)-0.5`，cy 同理。`initUndistortRectifyMap(K,D,I,A*K,640×480)` 融合去畸变、裁切、缩放，不把畸变后的原图直接当作理想相机。实际处理后 K：

```text
744.235051029  0              314.495980292
0              745.552445480 245.190691335
0              0              1
```

CameraInfo 为相同 640×480、D 全零、R=I、P=[K|0]，未沿用完整 RGB 的 K。

`reprojectImageTo3D` 的点在校正左目坐标中，先乘 `R1.T` 回到原始左目 optical，再乘 `T_rgb_left`，最后用裁切 K 投影。旋转后的中间点保持 float64；Z-buffer 存 float32，毫米转换也在 float32 乘 1000 后做 ties-to-even 舍入，与 Python 数值参考一致。多个三维点投到同一像素时保留最近 RGB optical Z。过滤范围为 0.15–5 m，输出 16UC1 毫米，0 无效；这是光轴 Z，非欧氏距离。

## 话题、时效和配对

| 话题（均位于 /camera/processed） | 内容 |
|---|---|
| `/color/image_raw`、`/color/camera_info` | 同一去畸变裁切网格，RGB 原时间戳 |
| `/aligned_depth_to_color/image_raw` | 640×480、16UC1，RGB optical frame，双目原时间戳 |
| `/disparity/image_raw` | 640×360、32FC1，单位视差像素（已除 16） |
| `/left/image_rect`、`/right/image_rect` | 可选订阅的校正 mono8 图 |
| `/disparity/preview`、`/aligned_depth_to_color/preview` | bgr8 可视化，不能当深度测量 |
| `/status` | backend=cpp_opencv、实际配置、计数、计算耗时、配对差和有效比例 |

图像使用 SensorDataQoS，有界最新帧缓存和工作项，忙时替换积压。左右 header 必须完全一致；RGB 仅在主机时间差 ≤25 ms 时配对。最大允许配置为 35 ms，来自既有配对预算，不为提高帧率放宽。输入源龄及发布前源龄均须 ≤300 ms（未来偏差最多 50 ms），不重打时间戳。RGB 与双目不是曝光硬件同步；下游仍须检查各自 header。

没有传标定时只输出原始 RGB 中央裁切和 K[0]=0 的 CameraInfo，绝不发布深度测量；显式传入损坏或不存在的标定直接失败，不静默伪装为已标定。原始左右缺一路、配对超时或过期时不产生该帧深度。参数为只读启动参数，改变算法需要停止再启动。wrapper 的 flock 保证同一入口不重复占用处理后话题。

## 本轮验证

1. C++ 数值检查通过：像素中心修正、RGB 外参方向、0.15–5 m 拒绝、NaN、最近 Z、毫米编码和正视差左右检查；BM/SGBM 合成 2 m 平面中位均 2.007 m，满足测试预设 ±50 mm。合成结果不代表实机精度。
2. 域 198、LOCALHOST 的隔离 ROS 检查通过：未标定收到 17 帧 RGB、零深度、fx=0；损坏配置拒绝、无双目/100 ms 错时/1 s 过期不发布深度；真实标定时尺寸、D/R/P、RGB 与深度分别保留原时间戳全部通过。错时用 220 ms 发送周期，避免后续 RGB 时间戳刚好与上一帧双目相遇而变成合法配对。
3. 录制像素对比中，C++/Python RGB 裁切逐像素一致。OpenCV 4.8/4.6 校正图平均灰度差约 0.0016、分别 357/347 像素不一致，匹配器输出不能宣称 bit-identical。冻结 native 视差、R1、Q 后，BM 与 SGBM 的后续几何投影均与 Python 参考逐像素相同（深度差 0 mm）。没有为通过测试冒称跨版本匹配完全一致。
4. 20 s SGBM 实机采样：对齐深度约 5.31 Hz，compute_ms 中位 174.7、P95 199.1；回调处源龄中位 223.8 ms、P95 256.5 ms；当前场景有效深度中位 9.81%。只有一个原生深度发布者，读取运行节点参数确认 crop/sgbm/128/5/25 ms/0.15–5 m 与实际候选标定路径。
5. 相邻 15 s BM 试验约 9.99 Hz，计算中位 67.1 ms，但当前场景有效比例中位 0.92%；随后恢复 SGBM。两段不是严格相同静态场景的精度对照，不能凭覆盖率判定距离正确。

`compute_ms` 计入深度处理和预览封装/发布，不含前置 RGB 处理和等待；最大 10 Hz 是上限，不是性能保证。当前白墙/低纹理区域和投影空洞保留零值。仍需有纹理物体、卷尺距离、动态错位和长期稳定性验收；本次没有启动跟随/SLAM，也没有修改电控模型。

复现：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ctest --test-dir build/wla_uvc_depth --output-on-failure
python3 src/wla_uvc_depth/test/test_ros_pipeline.py --workspace "$PWD" --output .runtime/uvc_depth_validation/ros
python3 src/wla_uvc_depth/test/compare_python.py --workspace "$PWD" --output .runtime/uvc_depth_validation/reference
```

原始证据保存在 `.runtime/uvc_depth_validation/{ros,reference,live_sgbm_final,live_bm}/` 以及 `.runtime/uvc_depth_*log`；本机备份目录为 `outputs/uvc_depth_cpp_20261116`。测试不构成机器人运动安全验收。

## 双目网络实验

原生 TensorRT 双目网络、RGB 仅染色、启动命令和实测边界见 [STEREO_NETWORK.md](STEREO_NETWORK.md)。
