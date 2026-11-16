# 新机器人三相机与 IMU 标定方案

日期：2026-11-12。目标机：`onana@192.168.50.1`，工作区 `/home/onana/ros2_ws`。
目前没有这套硬件的标定数据。本文件是采集与验收方案，模板中的数值不能当作标定结果。

## 1. 固定硬件与工作模式

| 相机 | 采集模式 | 标定使用的图像 | 用途 |
|---|---|---|---|
| Sunplus USB Global Camera | 2560×720 拼接 MJPEG；驱动申请 60 fps，默认发布 30 fps | USB 每半幅 1280×720；物理左右交换、两路各补 180° 转正，输出正立横屏 1280×720，来自同一拼接帧 | 双目、SLAM、深度 |
| DICOTA 4K | 1920×1080 MJPEG，申请 30 fps | 完整 1920×1080 RGB | 彩色检测、与双目深度对齐 |

相机型号和设备格式来自目标机枚举；实际交付帧率应由 ROS 消息时间戳、计数和丢帧统计实测，不能用申请值代替。左右在同一 USB 帧内拆分，共用接收时间戳；左右曝光同步依据用户的硬件说明，仍应通过动态画面检查。RGB 与双目是两台 USB 设备，目前只有各自的主机接收时间，不能认定三路硬件同步。

把两台相机与机身 IMU 固定在最终安装位。记录设备序列号、左右顺序、安装照片、USB 拓扑、焦距/焦点、曝光/增益、固件。调整分辨率、镜头焦点、缩放或安装位后，需要重新检查相应标定。先读 `v4l2-ctl --list-ctrls`，有自动对焦时固定焦点；不盲目写不存在的控制项。标定与运行使用相同成像模式。

新入口为 Fast DDS 2.14.6 配置独立的 128 MiB SHM 段和 8 MiB 消息上限（`config/uvc_rig/fastdds_uvc.xml`），适配超过 6 MB 的原始 1080p BGR 消息。仅对新入口的子进程设置 `FASTRTPS_DEFAULT_PROFILES_FILE`，保留用户显式配置，未改系统内核参数或现有电控环境。正式多进程入口实测三路与裁切约 30 Hz；默认传输对比中右目曾降至约 14 Hz，因此标定录制前也要核查实际收到的帧率。[Fast DDS SHM 说明](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/transport/shared_memory/shared_memory.html)

## 2. 准备一块可共同看见的标定板

当前实际标定板已由用户于 2026-11-16 确认：**6 列 × 6 行，完整黑色 tag 外边长 32.5 mm，tag 间隙 9.75 mm（比例 0.3）**。正式目标配置使用 `config/uvc_rig/aprilgrid_measured_6x6_32p5mm.yaml`；用于 Kalibr 时可复制为 `target.yaml`。图案从最外侧标签黑框边缘到另一侧共 243.75 mm。不要使用文件名里的 90 mm，也不要混用旧 5×6 板的参数。标定板须平整固定，测量 tagSize 时包含黑色外框，不包含白边或标签间隙。

Kalibr 官方建议 AprilGrid，允许部分图案可见，且能消除对称棋盘带来的姿态翻转；官方也明确要求平整、实测打印尺寸和白边。[官方标定板说明](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)

历史备用板（不是本次实测板）：已生成可打印的 [A3 AprilGrid PDF](calibration/new_orin_aprilgrid_5x6_30mm_A3.pdf)，使用本机已有的官方 Kalibr 容器（源码提交 `1f602274`），图案未缩放，外层仅排版到 297×420 mm。PDF 矢量 tag 边长、间距和页面尺寸已检查，渲染后由官方检测器识别全部 30 个 tag / 120 个角点。附带 100 mm 核尺线；这不能代替打印后的实测。生成与检测证据见同目录 JSON。

本机现有 `kalibr-d455-rsusb:2.58.1-rs2.3.2-headers` Docker 镜像包含官方 Kalibr 求解器，可以离线复用其工具；只用标定程序，不启动它的旧 D455 驱动。目标 Orin 不需要为此安装旧 ROS 1 相机驱动。以下是历史 5×6 备用板的生成命令（不是当前 6×6 实物配置）（ROS 包脚本通过 `rosrun` 调用）：

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

RGB 先按完整 1920×1080 标定；运行时默认 `--rgb-mode full`，完整视野等比例缩到 640×360。对照旧方案可用 `--rgb-mode crop`，中央取 `(x=240,y=0,w=1440,h=1080)`，缩到 640×480。处理后 `CameraInfo` 用相同去畸变、裁切和缩放更新，深度按所选模式投影到同一网格；不能沿用完整 RGB 的 K。实现包含 resize 像素中心修正：`cx' = sx*(cx-x+0.5)-0.5`，cy 同理。全幅模式有 x=y=0、sx=sy=1/3。YOLOE 的内部补边不写回 ROS 图像或 CameraInfo。[OpenCV 投影与标定说明](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)

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

输出 `/camera/processed/color/image_raw`、`color/camera_info` 和经标定后才有的 `aligned_depth_to_color/image_raw`。深度 `16UC1`，单位毫米、0 为无效，图像默认 640×360（crop 模式 640×480），坐标轴为 RGB optical；多点投到同一像素取最近 Z，空洞不伪造测量。RGB 保留自身接收时刻，深度保留双目接收时刻。25 ms 是当前主机时间近似配对的工程初值，35 ms 是现有检测器的配对上限；这不能证明曝光同步，移动物体仍可能错位。若动态验收不通过，应接硬触发或可靠曝光时间戳，而非修改 stamp 假装同步。

SGBM 的视差数 128、窗口 5、深度过滤 0.15–5 m 是初始软件参数，不是这套硬件的已验收性能或量程。根据实际 fx、基线、最近距离和实测耗时再调整。远距误差近似随 `Z²/(fx*B)` 增长，不应承诺未测的远距精度或深度 30 fps。无纹理、反光、遮挡和 RGB 超出双目共同视场的区域应保留无效深度。

## 5. 第二轮：IMU、相机与机身坐标/时间标定

2026-11-12 早些时候电控曾因“未找到唯一达妙设备”而无法发布 IMU；后续复测已恢复：域 0 上 `/motor35_powered` 以 SensorDataQoS 发布 `/imu/data`，12 秒收到 2399 条、约 200 Hz，时间戳单调且四元数归一正常，frame 仍为 `base_Link`。本次地面估计只读运行中也继续收到约 200 Hz IMU。数据链路恢复不等于采样语义、轴向或相机外参已标定。没有停止/重启/使能该电控；域 76 仅存在于其仿真分支。

已核对当前主机融合路径：启用 zju2026 时，SerialImuParser 先用 WIT 四元数初始化 Mahony，之后每份独立陀螺包都会经包装器生成主机融合姿态样本并发布 `/imu/data`，实测约 200 Hz。陀螺、加速度和姿态仍可能来自不同采样时刻：主机融合允许最近加速度有最多 20 ms 的间隔，ROS header 为主机发布时刻，不能等同硬件统一采样时间。不能仅凭话题存在或频率认定高质量 VIO 标定条件已满足。

需要取得新鲜原始角速度（rad/s）、线加速度（m/s²）、实际各自采样时间及其时钟来源，检查单调性、dt 分布、批量到达、静止模长、转轴正负和零偏。确认 IMU 驱动已经按真实安装方向旋转数据，才能使用 `base_Link`；单改 frame 字符串没有这个效果。不得复用旧机的 IMU 噪声、DLPF 延迟或时间偏置。

2026-11-12 已导入用户提供的 CAD URDF/CSV，详见 [新机器人传感器坐标关系](新机器人传感器坐标关系.md)。CSV 确认 URDF 根 `base_link` 使用 CAD 的 `base_point`，用户根据四个 ToF 中间且较低的位置确认该基准为 IMU；IMU 原始测量轴仍待核对。用户确认 DICOTA RGB 对应上方 `upper_camera_R`（上左目未安装），Sunplus 同步黑白双目对应下方 `bottom_camera_L/R`，其安装点间距为 37 mm、安装 pitch 约 11°，另有前后左右四个 ToF；七个传感器均以 fixed joint 直接连接机身，不经过活动轮腿。安装点间距不是已标定光学基线，暂未据此开启深度或广播 optical TF。2026-11-16 用户确认原半幅左右名称颠倒；后续用户物理调整安装后，当前将 USB 右半幅作为物理左目、USB 左半幅作为物理右目，两路均补 180° 旋转，输出正立的 1280×720 横屏。实际光心/机身外参仍待标定，不能从画面转正推断已标定 optical TF。

电控已有的 `base_Link` 与 CAD 的 `base_link` 大小写不同，需在数据轴向明确后统一，不能直接加 identity TF 假定对齐。机身通常 x 前/y 左/z 上，光学坐标 x 右/y 下/z 前；按 [REP-103](https://www.ros.org/reps/rep-0103.html) 对照实物。若实物存在 CAD 未描述的活动云台或不同连杆，不能贯穿所有姿态复用常量外参，需要固定标定/运行姿态或另行设计关节状态与动态 TF 的处理。URDF 提供安装几何和初值，不能替代镜头内参、精确光学外参或时间偏置。不要同时由 URDF 与独立 static TF 节点重复广播同一 child。

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

## 6. 复用电控标准状态接口

用户要求不保留不必要的自建估计。原独立地面估计与位置积分代码已移除，改为只读 C++20 包 `wla_robot_state`：直接复用电控 `/imu/data`，将实测 `q_rad` 转成 `/joint_states`，提供启动航向参考的四元数和 RPY；不改电控模型、不推断 xyz/高度。详见 [新机器人标准状态接口](新机器人标准状态接口.md)。后续由局部跟随/SLAM 使用这些标准输入，并继续完成相机与 IMU 的几何和时间标定。

2026-11-16 标定成像模式更新：当前 `robot_camera.sh` 默认物理左右交换并转正；用户物理调整安装后取消早前相反的 90° 旋转，改为两路各补 180°；标定 `image_raw`、CameraInfo 与网页预览统一为左右 1280×720、RGB 1920×1080。转正为无插值像素旋转，保留原捕获时间戳。请使用实测 6×6、32.5 mm 标签、9.75 mm 间隔的 `config/uvc_rig/aprilgrid_measured_6x6_32p5mm.yaml`，不要沿用旧版板尺寸、竖幅内参或安装调整前的标定。
