# ESP32 四路 ToF：C++ 只读接收与点云预览

`wla_tof/tof_serial` 是 C++20 / rclcpp 节点，直接打开 ESP32 USB 串口，接收四路 VL53L8CX 的 TF4 协议。运行不依赖 Python，不经过电控，不发送串口命令或主动切换 DTR/RTS，不刷固件，不发布电机命令。

协议核对来源：工作站 `/home/jakukur/esp/vl53l8cx-four-test/main/main.c`、`tools/capture.py`、`tools/geometry.py`。支持 444 字节 v1 和 460 字节 v2；校验版本、长度、逻辑 ID 和标准 CRC32。坏帧丢弃并重新寻找帧头，序列号重复/倒序和无效 MCU 时间字段拒绝；boot_id 改变允许新序列。缓冲有界，每次回调只发布各通道最新收到的有效帧。输出时间戳是主机完整帧接收处理时间，不是曝光时间；原 MCU 时刻独立保留，未实施时钟同步。

## 运行

```bash
cd /home/onana/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run wla_tof tof_serial --ros-args --params-file config/tof/new_orin.yaml
```

串口用 `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_28:84:85:9E:C4:68-if00`，设置 raw 921600、8N1。节点使用 advisory flock 和 TIOCEXCL；同一串口不能同时开其他采集器。异常断开时在 status 中报告 serial_error、停止使用旧文件描述符，每秒尝试按持久设备名重新打开；不自动复位板卡，也未配置开机自启。

```bash
ros2 topic echo /r680/tof/status
ros2 topic hz /r680/tof/front_right/points_preview
ros2 topic echo /r680/tof/front_right/raw --once
tail -f /tmp/wla-tof-serial-20261112.log
```

本轮临时启动 PID 文件：`/tmp/wla-tof-serial-20261112.pid`。日志/PID 文件不保证跨重启保留。

## 数据与预览几何

| sensor_id | 固件逻辑名 | 原始数据 | 点云预览 |
|---:|---|---|---|
| 0 | front_left | `/r680/tof/front_left/raw` | `/r680/tof/front_left/points_preview` |
| 1 | front_right | `/r680/tof/front_right/raw` | `/r680/tof/front_right/points_preview` |
| 2 | rear_left | `/r680/tof/rear_left/raw` | `/r680/tof/rear_left/points_preview` |
| 3 | rear_right | `/r680/tof/rear_right/raw` | `/r680/tof/rear_right/points_preview` |

逻辑编号来自固件 LPn GPIO4/5/6/7，不等于已验证的实物安装对应。

`raw` 为 `std_msgs/String` JSON，含 64 区距离（mm）、sigma（mm）、状态、目标数，以及 boot/seq/flags/错误/MCU 时间与主机时间。所有原始区域均保留，无效值不替换为 0。

`points_preview` 为 `sensor_msgs/PointCloud2`，frame_id 为 `tof_<逻辑名>_preview_optical_frame`，坐标 x 右、y 下、z 前，单位米。只取 status=5、targets>0、20–4000 mm 的区域，最多 64 点；有有效帧但无有效区域时输出空云，没有帧时不补空云或零距离。

点格式为 x/y/z/distance_m（FLOAT32），sigma_mm/zone（UINT16），point_step=20。与原工程一致，采用默认 45°×45° 区域中心等角度近似：ax/ay 由原始列/行号得到，`X=Z*tan(ax), Y=Z*tan(ay)`。原 ST ULD 开启径向到垂直距离修正，距离按光学 Z 使用。该投影和区域排列未标定，sigma 保留供诊断，没有另加未验证的阈值。

`/r680/tof/status` 5 Hz，包含 backend=cpp、各路帧数/新鲜度/最近有效点数/序号缺口/重启数、CRC 和帧头错误、固件 HEALTH 文本、固件累计 I²C 错误和发送覆盖数。`firmware_diagnostics_last` 保留最新 32 条 `# ` 开头的非 HEALTH 启动/诊断行，每行最多 256 字符。超过 stale_after_s=0.30 无有效帧即不新鲜，接收程序正常运行不能代替四路硬件在线。

## 与人物跟随的关系

现有 C++ `wla_follow_transport/follow_transport` 预留的正式输入是 `/r680/tof/{front_left,front_right,rear_left,rear_right}/points`，要求帧龄 ≤150 ms、≤256 点、数据 ≤65536 字节，并按消息时刻取得到 base_frame 的 TF。其配置 `tof_enabled` 当前为 false；本节点不会把未标定预览直接发布到这些正式输入。

正式接入还需确认四个逻辑 ID 的实物对应、每区方向/射线、安装光心与 CAD 外参以及主机时间模型。已有 CAD 安装记录在 `config/uvc_rig/cad_20261112/`，不是光学标定，也没有自动广播运行 TF。不能因为点云的 frame 名看起来正确就补 identity TF；不能将缺失/无效区域当清障射线或安全空地。当前不启动跟随任务或改动其运动配置。

## 构建和验证

```bash
colcon build --packages-select wla_tof --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
ctest --test-dir build/wla_tof --output-on-failure
ROS_DOMAIN_ID=211 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 src/wla_tof/test/test_ros.py \
  --executable /home/onana/ros2_ws/install/wla_tof/lib/wla_tof/tof_serial
```

纯 C++ 测试覆盖标准 CRC 向量、v1/v2、四个 ID、逐字节拆包、坏帧重同步、Health 文本、单位/投影及无效区域拒绝。ROS 测试使用隔离域 211 和伪终端，运行真实 C++ 接收器，核对 PointCloud2 字段和原始数据配对、CRC/重复帧拒绝、MCU 重启、离线超时不补数据；不接触真实设备。

USB 重连回归通过替换伪终端设备链接验证：旧设备断开被标记 serial_error，新设备出现后重新打开，新的 boot_id 数据正常输出；诊断队列有 32 行上限。主机只重开串口文件，不主动复位 MCU。

新机编译和上述测试通过。最初 6 秒真实串口样本：Python 原工程与 C++ 解码器都得到 ID1 的 85 帧、CRC 错误 5、帧头错误 0，其余 ID 无帧；有错误的传输不能视为四路验收通过。固件 HEALTH 显示 ID1 在线，其余三路离线。用户确认四路均已接线，仍需排查传感器初始化、供电/I²C；没有修改固件或电控模型。原始采样和实测记录位于目标机 `.runtime/esp32_tof_probe_20261112/`，以及工作站 `outputs/orin_migration_20261112/tof_cpp/evidence/`。

随后真实 C++ 驱动连续观察 15 s，四路点云和 raw 均为 0 帧；串口累计字节和 HEALTH 文本持续更新、serial_error 为空，四路均报 online=0。前右固件错误数从最初的 12 增至 16，固件停读；其最后一帧年龄已超过 400 s。用户随后对传感器组重新上电，USB 枚举从 ttyACM1 改为 ttyACM2，持久 by-id 不变；再次观察 15 s 仍为四路 online=0、0 测距帧，表明此次启动也未恢复测距。该状态是传感器端采集故障，不能以“ROS 话题已创建”宣称点云已出数据。固件在连续三次 I²C 错误后置该路 offline，不在主循环中自动重新初始化；要恢复，需进一步排查初始化日志、供电与 I²C。本轮主机没有刷机或发出板卡复位指令。

2026-11-13 重启后复测 15.03 s：四路均 0 帧，前右 HEALTH 的 checks=4、errors=3、online=0，其余三路 online=0。主机串口接收字节从 484 增至 3424，serial_error 为空；没有测量二进制帧，因此此窗口的 CRC=0 不能用于证明测量数据质量。证据：工作站 `evidence/live-20261113.json`，目标机 `.runtime/esp32_tof_probe_20261113/live.json`。当日临时 PID/日志为 `/tmp/wla-tof-serial-20261113.pid` 和 `/tmp/wla-tof-serial-20261113.log`。
