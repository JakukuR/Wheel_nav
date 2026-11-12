# 新机器人三相机与 IMU 标定方案

日期：2026-11-12。目标机：`onana@192.168.50.1`，工作区 `/home/onana/ros2_ws`。
目前没有这套硬件的标定数据。本文件是采集与验收方案，模板中的数值不能当作标定结果。

## 1. 固定硬件与工作模式

| 相机 | 采集模式 | 标定使用的图像 | 用途 |
|---|---|---|---|
| Sunplus USB Global Camera | 2560×720 拼接 MJPEG；驱动申请 60 fps，默认发布 30 fps | 左右各 1280×720，来自同一拼接帧 | 双目、SLAM、深度 |
| DICOTA 4K | 1920×1080 MJPEG，申请 30 fps | 完整 1920×1080 RGB | 彩色检测、与双目深度对齐 |

相机型号和设备格式来自目标机枚举；实际交付帧率应由 ROS 消息时间戳、计数和丢帧统计实测，不能用申请值代替。左右在同一 USB 帧内拆分，共用接收时间戳；左右曝光同步依据用户的硬件说明，仍应通过动态画面检查。RGB 与双目是两台 USB 设备，目前只有各自的主机接收时间，不能认定三路硬件同步。

把两台相机与机身 IMU 固定在最终安装位。记录设备序列号、左右顺序、安装照片、USB 拓扑、焦距/焦点、曝光/增益、固件。调整分辨率、镜头焦点、缩放或安装位后，需要重新检查相应标定。先读 `v4l2-ctl --list-ctrls`，有自动对焦时固定焦点；不盲目写不存在的控制项。标定与运行使用相同成像模式。

新入口为 Fast DDS 2.14.6 配置独立的 128 MiB SHM 段和 8 MiB 消息上限（`config/uvc_rig/fastdds_uvc.xml`），适配超过 6 MB 的原始 1080p BGR 消息。仅对新入口的子进程设置 `FASTRTPS_DEFAULT_PROFILES_FILE`，保留用户显式配置，未改系统内核参数或现有电控环境。正式多进程入口实测三路与裁切约 30 Hz；默认传输对比中右目曾降至约 14 Hz，因此标定录制前也要核查实际收到的帧率。[Fast DDS SHM 说明](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/transport/shared_memory/shared_memory.html)

## 2. 准备一块可共同看见的标定板

推荐 Kalibr AprilGrid：**5 列 × 6 行，黑色 tag 外边长 30 mm，tag 间隙 9 mm（比例 0.3）**。图案宽 186 mm、高 225 mm，四周至少留 39 mm 白边，可以放在 A3 上。建议哑光打印并贴在平整硬板上，100% 比例打印，禁用“适应页面”。用尺或卡尺在不同位置测量多个 tag 和整行尺寸，把实测值写入 `config/uvc_rig/aprilgrid_proposed.yaml` 的副本，再作为正式 `target.yaml`。不能把白边或包含间隙的尺寸当成 tagSize。

Kalibr 官方建议 AprilGrid，允许部分图案可见，且能消除对称棋盘带来的姿态翻转；官方也明确要求平整、实测打印尺寸和白边。[官方标定板说明](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)

已生成可直接打印的 [A3 AprilGrid PDF](calibration/new_orin_aprilgrid_5x6_30mm_A3.pdf)，使用本机已有的官方 Kalibr 容器（源码提交 `1f602274`），图案未缩放，外层仅排版到 297×420 mm。PDF 矢量 tag 边长、间距和页面尺寸已检查，渲染后由官方检测器识别全部 30 个 tag / 120 个角点。附带 100 mm 核尺线；这不能代替打印后的实测。生成与检测证据见同目录 JSON。

本机现有 `kalibr-d455-rsusb:2.58.1-rs2.3.2-headers` Docker 镜像包含官方 Kalibr 求解器，可以离线复用其工具；只用标定程序，不启动它的旧 D455 驱动。目标 Orin 不需要为此安装旧 ROS 1 相机驱动。在容器内生成图案的命令如下（ROS 包脚本通过 `rosrun` 调用）：

```bash
rosrun kalibr kalibr_create_target_pdf --h
rosrun kalibr kalibr_create_target_pdf --type apriltag --nx 5 --ny 6 --tsize 0.030 --tspace 0.3
```

生成工具的页面尺寸需要检查；超出打印机可打印区域时换更大纸张，不缩放图案。现有旧板也可复用，但必须使用它自己的行列数、图案 ID/排布和实测尺寸，不能套用本例配置。

## 3. 第一轮：三路相机几何标定，机器人保持静止

固定机器人，移动标定板。采集约 **40–60 个不同静止姿态**：近、中、远距离；图像中央、四边、四角；不同俯仰、偏航与滚转。每个姿态停留约 1–2 秒，避免模糊和反光。确保存在足够多三路同时看到板的姿态，并让板覆盖整个视场；不能只拍正对中央的小板。具体距离由实际视场和 tag 清晰度确定，先试 0.4–1.5 m，不能把建议距离当成相机硬件量程。

这里让板停稳，是为了减小独立 RGB 相机与双目不同步对外参的影响。留出另一天/另一次采集的约 20% 姿态做独立验收，不混入拟合。

目标机三个终端分别执行。默认 ROS 域为 **0**，与当前真实电控进程的默认域一致；不要套用旧机的 73 或仿真专用的 76。

```bash
# 终端 1：原始左右图。4 Hz 仅降低发布频率，保持原采集模式。
cd /home/onana/ros2_ws
./robot_camera.sh run sunplus --publish-fps 4

# 终端 2：原始 RGB。标定完整图，不启动 YOLO 裁切节点。
cd /home/onana/ros2_ws
./robot_camera.sh run dicota --publish-fps 4

# 终端 3：先确认三路真的有新帧，再录包；Ctrl+C 正常结束。
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic hz /camera/sunplus/left/image_raw
# 退出 hz 后执行；另两路也应分别检查。
mkdir -p /home/onana/ros2_ws/calibration_data
ros2 bag record --storage mcap --start-paused --max-bag-size 2000000000 \
  -o /home/onana/ros2_ws/calibration_data/cameras_$(date +%Y%m%d_%H%M%S) \
  --topics /camera/sunplus/left/image_raw /camera/sunplus/right/image_raw \
  /camera/dicota/color/image_raw /camera/sunplus/status /camera/dicota/status
```

录包从暂停状态开始。将板摆好、停稳约 0.5 秒后，在 recorder 终端按空格开始录制 1–2 秒；再次按空格暂停，再移动板到下一姿态。只记录停稳段，最后 Ctrl+C 结束。4 Hz 三路发布相位独立，几何标定命令允许 0.15 s 近似配对；这仅适用于板完全静止的采集段，不能用于动态 IMU 标定或运行深度同步，也不修改原时间戳。若录入了移动段，先剔除后再拟合。

原始 BGR 三路在 4 Hz 时约 47 MB/s，即约 2.8 GB/分钟，另加封装开销。录制前检查 `df -h`，采集后用 `ros2 bag info <目录>` 检查消息数和时长，并从包里检查清晰度、左右顺序与时间差。分包大小限制不等于总时长或总大小限制。

离线用 Kalibr 联合求三路 K、D、左右 R/T、RGB 与左目的 R/T。第一版选择 `pinhole-radtan`；若残差显示广角模型不适配，应换模型并重新验证，不能靠放宽误差掩盖。当前运行节点支持 5/8 项径向切向畸变，其他模型需先扩展运行端。

```bash
# 在已配置的 Kalibr 环境中运行，输入须是该版本支持的 bag。
rosrun kalibr kalibr_calibrate_cameras --bag cameras.bag \
  --topics /camera/sunplus/left/image_raw /camera/sunplus/right/image_raw /camera/dicota/color/image_raw \
  --models pinhole-radtan pinhole-radtan pinhole-radtan --target target.yaml --approx-sync 0.15
```

ROS 2 MCAP 目录不能直接改扩展名交给传统 ROS 1 Kalibr。需要保留原 `header.stamp` 和图像内容的格式转换，或使用兼容 ROS 2 的 Kalibr 工作流；转换后的首末时间、每路消息数、图像尺寸和 IMU 单位必须复核。已确认本机既有 Kalibr 容器可执行官方脚本并识别新图案；目标 Orin 尚未安装/验证 Kalibr 求解环境。上述求解命令尚无新机实采数据可以运行，不能视为已经完成标定。[官方多相机标定流程](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration)

离线格式转换可使用本机已有的 `rosbags-convert` 0.11.0。已在明确标记的 Jazzy MCAP 测试数据上验证 Image/Imu 字段、图像字节、header 与记录时间一致；这只是格式验证，不是实机标定：

```bash
rosbags-convert --src /path/to/cameras_ros2_bag_directory --dst /path/to/cameras.bag
python3 /path/to/ros2_ws/tools/uvc_calibration/check_bag_conversion.py \
  /path/to/cameras_ros2_bag_directory /path/to/cameras.bag --output conversion-check.json
# 要求 matched=true；新文件需不存在，保留原 MCAP 目录。
```

独立离线环境可以安装 `rosbags==0.11.0`；无需在相机采集进程里混装 ROS 1。转换工具用法见 [Rosbags 官方说明](https://ternaris.gitlab.io/rosbags/topics/convert.html)。

保存原 bag、正式 target.yaml、camchain YAML、拟合报告、保留验证集及其 SHA-256。导出运行文件时明确：`T_right_left` 将原始左目光学坐标变到原始右目光学坐标，`T_rgb_left` 将原始左目光学坐标变到原始 RGB 光学坐标；平移单位米。不要直接猜测转置或变换方向。模板为 `config/uvc_rig/rig.template.yaml`，空值必须全部换成求解结果，原始分辨率必须匹配。

相机求解和保留集检查完成后，使用导出工具生成运行文件，避免手工猜测变换方向：

```bash
# 在有 numpy / OpenCV / PyYAML 的离线环境执行；不需要启动 ROS 节点。
PYTHONPATH=/path/to/ros2_ws/src/wla_uvc_camera python3 -m wla_uvc_camera.export \
  cameras-camchain.yaml --output /path/to/calibrated_20261112_v1
```

要求 cam0/1/2 依次为左目、右目、RGB，且 rostopic 与本方案一致；其他顺序拒绝导出。工具按 `T_rgb_left = cam2.T_cn_cnm1 @ cam1.T_cn_cnm1` 组合变换，将 Kalibr `[k1,k2,p1,p2]` 变成 OpenCV `[k1,k2,p1,p2,0]`（原模型固定 k3=0），生成 rig.yaml、三路 ROS CameraInfo YAML、用于 IMU 标定的 stereo-camchain.yaml 及来源哈希。已存在的输出目录拒绝覆盖。它只转换求解结果，不替代独立验收。

将整个导出目录复制到新机，例如 `/home/onana/ros2_ws/config/uvc_rig/calibrated_20261112_v1`。通过验收后，在采集与处理终端统一设置：

```bash
export WLA_UVC_CALIBRATION_DIR=/home/onana/ros2_ws/config/uvc_rig/calibrated_20261112_v1
./robot_camera.sh run both
# 另一个终端同样 export 后运行：
./robot_camera.sh pipeline
```

导出的原始左右 CameraInfo 带双目 R/P，原始图像本身不会因为加载 YAML 变成校正图；SLAM 输入是否需要校正图应另按前端配置核查。RGB 裁切后的内参由 pipeline 生成，不能用原始 RGB YAML 直接替代。

## 4. 深度生成、RGB 裁切与对齐

顺序为：原始左右图 → 双目校正 → BM/SGBM 视差 → 米制三维点 → RGB 光学坐标 → 去畸变并裁切的 RGB 像素。第一版匹配分辨率 640×360，以 SGBM 为默认，BM 可用于对比。没有真实标定文件时，节点只输出 RGB 裁切与零内参 CameraInfo，不发布深度测量。

RGB 先按完整 1920×1080 标定；运行时中央取 `(x=240,y=0,w=1440,h=1080)`，缩到 640×480。处理后 `CameraInfo` 用相同去畸变、裁切和缩放更新；不能沿用完整 RGB 的 K。实现包含 resize 像素中心修正：`cx' = sx*(cx-x+0.5)-0.5`，cy 同理。[OpenCV 投影与标定说明](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)

```bash
# 常规采集：左右/RGB 默认各发布 30 Hz；实际性能需测量。
./robot_camera.sh run both

# 另一个终端，无标定：可先接 2D YOLO，不能获得距离。
./robot_camera.sh pipeline

# 有真实 rig.yaml 后，用此命令替代上一个 pipeline。
./robot_camera.sh pipeline --calibration /home/onana/ros2_ws/config/uvc_rig/rig.yaml \
  --algorithm sgbm --disparities 128 --block-size 5 --max-skew-ms 25
# 对比算法：--algorithm bm

# 第三个终端，单独启动检测，不启动底盘或 Agent 运动任务。
./robot_perception.sh check
./robot_perception.sh run
```

输出 `/camera/processed/color/image_raw`、`color/camera_info` 和经标定后才有的 `aligned_depth_to_color/image_raw`。深度 `16UC1`，单位毫米、0 为无效，图像 640×480，坐标轴为 RGB optical；多点投到同一像素取最近 Z，空洞不伪造测量。RGB 保留自身接收时刻，深度保留双目接收时刻。25 ms 是当前主机时间近似配对的工程初值，35 ms 是现有检测器的配对上限；这不能证明曝光同步，移动物体仍可能错位。若动态验收不通过，应接硬触发或可靠曝光时间戳，而非修改 stamp 假装同步。

SGBM 的视差数 128、窗口 5、深度过滤 0.15–5 m 是初始软件参数，不是这套硬件的已验收性能或量程。根据实际 fx、基线、最近距离和实测耗时再调整。远距误差近似随 `Z²/(fx*B)` 增长，不应承诺未测的远距精度或深度 30 fps。无纹理、反光、遮挡和 RGB 超出双目共同视场的区域应保留无效深度。

## 5. 第二轮：IMU、相机与机身坐标/时间标定

先解决现有电控的采样阻塞。2026-11-12 观察到真实控制进程没有设置 `ROS_DOMAIN_ID`，因此域为 0；代码发布 `/imu/data`（SensorDataQoS，frame 为 `base_Link`），但现有监督器反复报告“未找到唯一达妙设备”，暂未取得稳定 IMU 流。未停止/重启/使能该电控。域 76 仅存在于它的仿真分支。

还需核查原始采样语义：当前代码在“完整 IMU 样本”回调使用节点当前时间发布，独立陀螺包虽然更新内部状态，却没有单独发布；陀螺、加速度和姿态可能不是同一采样时刻。因此看见 `/imu/data` 话题或 frame 名，不能直接认为数据足以做高质量 VIO 标定。

需要取得新鲜原始角速度（rad/s）、线加速度（m/s²）、实际各自采样时间及其时钟来源，检查单调性、dt 分布、批量到达、静止模长、转轴正负和零偏。确认 IMU 驱动已经按真实安装方向旋转数据，才能使用 `base_Link`；单改 frame 字符串没有这个效果。不得复用旧机的 IMU 噪声、DLPF 延迟或时间偏置。

URDF 到位后，核对 `base_Link`、IMU 安装 frame、三个相机 optical frame。机身通常 x 前/y 左/z 上，光学坐标 x 右/y 下/z 前；按 [REP-103](https://www.ros.org/reps/rep-0103.html) 对照实物。URDF 先确认相机与 IMU 是否属于同一刚体；活动云台或不同连杆不能贯穿所有姿态复用常量外参，需要固定标定/运行姿态或另行设计关节状态与动态 TF 的处理。URDF 提供安装几何和初值，不能替代镜头内参、精确光学外参或时间偏置。不要同时由 URDF 与独立 static TF 节点重复广播同一 child。

IMU 数据稳定后，做静止噪声记录和相机—IMU 联合标定。先记录 5–10 分钟用于检查噪声/零偏/丢包；正式随机游走参数需足够长、温度稳定的 Allan 数据，不能把短记录当成完整噪声标定。IMU 量程、滤波与输出频率应固定，并记录其标度/轴对准校正状态。

第二轮优先以**同步双目 + IMU**建立 VIO 标定，使用导出的 `stereo-camchain.yaml`。RGB 的几何关系用第一轮外参组合，独立评估 RGB 时间偏置；不要把 USB RGB 当作与双目曝光同步。

动态采集时固定标定板，由操作者在电机不参与的条件下，平缓移动**整个相机—IMU 刚性组件**，覆盖三轴旋转和平移约 60–120 秒；不要只动相机支架。相机约 20 Hz、IMU 尽量达到稳定约 200 Hz，具体以硬件能力为准，不能插值造采样率。照明充足、短曝光、无碰撞。轮足整机若不能手持平缓运动，需要安排支撑/专用夹具后再采集，当前没有执行此物理实验。

```bash
# 仅在原始 IMU 数据与离线 Kalibr 环境都准备好后执行。
rosrun kalibr kalibr_calibrate_imu_camera --bag cameras_imu.bag \
  --cam stereo-camchain.yaml --imu imu.yaml --target target.yaml
```

采集命令在原始 IMU 语义验证后使用，不能用目前未稳定的流替代：

```bash
# 采集终端：保持原分辨率，降低发布率到 20 Hz。
cd /home/onana/ros2_ws
./robot_camera.sh run sunplus --publish-fps 20
# 录包终端：动态段连续记录，不用上一轮静态板的 0.15 s 配对设置。
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic info -v /imu/data
ros2 bag record --storage mcap --storage-preset-profile zstd_fast --max-bag-size 2000000000 \
  -o /home/onana/ros2_ws/calibration_data/stereo_imu_$(date +%Y%m%d_%H%M%S) \
  --topics /camera/sunplus/left/image_raw /camera/sunplus/right/image_raw /imu/data
```

若可靠原始 IMU 改用其他话题，录包与 `imu.yaml` 必须一起修改。ROS2 bag 同样先转换并检查再交给 Kalibr。符号统一为 `T_目标_来源`：若 Kalibr 给出 `T_left_imu`，则 `T_rgb_imu = T_rgb_left @ T_left_imu`；若 URDF 给出 `T_base_imu`，则 `T_base_left = T_base_imu @ inverse(T_left_imu)`。应核对实际输出字段/坐标轴后再落 TF，而非对矩阵只做转置。

保留时间偏置估计，检查多段结果是否一致，并以独立运动数据验证。RGB 与双目之间的独立 USB 延迟也要评估。Kalibr 默认估计相机—IMU 时间关系，但不能修复任意抖动和丢帧。[官方 IMU 标定要求](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)

## 6. 独立验收与交付

1. 保留集上检查重投影残差及其空间分布，复核校正后对应点的垂直误差。可先以重投影 RMS <0.5 px、垂直误差 P95 <1 px 作为项目初始筛查目标；它们是待评审的工程目标，不是官方通用合格线。若系统性偏差集中在边缘，应检查模型、焦点与板平整度。
2. 用卷尺测量约 0.5、1、2、3 m 的有纹理平面，报告深度中位误差、P95、有效像素比例及连续静止噪声。距离基准应换算到左目/RGB光学中心，并考虑光轴方向；不同位置独立测试。量程外不强求输出。标定板尺寸误差会直接影响米制尺度。
3. 对静态/移动物体分别叠加 RGB 与深度边缘，检查近远处、左右边缘、遮挡和空洞；移动物体错位须单独归因于曝光、滚动快门和时差，不能只调外参“凑齐”。
4. 测量两台相机同时运行时的丢帧、时间差分布、CPU/GPU、深度耗时、端到端延迟和十分钟稳定性。SGBM CPU 输出帧率可能低于相机 30 Hz，先保留新鲜帧丢弃积压，之后根据实测决定 C++/GPU 优化。
5. 交付真实 `rig.yaml`、CameraInfo、IMU 噪声/时间外参、URDF/TF关系、采集模式清单、原始数据哈希、拟合报告、独立验收报告。更换镜头/安装位/驱动时间戳策略后重新核验。

尚缺标定板实物数据、三路采集集、可靠 IMU 数据和新机 URDF；在这些完成前，不宣称已经替代 D455 的完整 RGB-D/VIO 功能，也不启动旧底盘导航链路。
