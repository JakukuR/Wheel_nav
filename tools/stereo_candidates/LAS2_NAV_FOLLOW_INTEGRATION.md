# LAS2 接入原导航与人物跟随（2026-11-17）

本轮接入的是感知数据接口，不更换定位/里程计，不启动导航运动或人物跟随命令。机上原导航和跟随进程当前未运行。用户确认三相机仍固定在CAD位置；这不等于CAD base_link、控制base_Link及旧导航r680_mapping_floor三个原点已完成对接。

## 选择的最少转换路径

一份LAS2-S FP16推理提供两种深度网格：

| 消费者 | 深度输入 | 配套内参 | 配置入口 |
|---|---|---|---|
| 导航depth_to_points / terrain_analysis | /camera/stereo/depth/image_raw | /camera/stereo/depth/camera_info | bringup.launch.py obstacle_depth_source:=las2 |
| 人物检测/目标定位 | /camera/stereo/aligned_depth_to_color/image_raw | /camera/stereo/aligned_depth_to_color/camera_info | robot_perception.sh默认las2 |
| 人物跟随障碍/人体剔除 | /camera/stereo/depth/image_raw | /camera/stereo/depth/camera_info | follow.launch.py depth_source:=las2 |
| 检测RGB | /camera/processed/color/image_raw | 同RGB校正网格 | 现有DICOTA完整视野640×360 |

原生深度是640×360、16UC1毫米、sunplus_left_rect_optical_frame；RGB对齐深度是640×360、16UC1毫米、dicota_color_optical_frame。零深度无效，范围0.15–5m，保留遮挡/视野外空洞。不能把原生左目深度与RGB人物框直接混用。

导航复用现有C++地面/孤立点过滤，再输出原Nav2已订阅的/r680_nav/d455/points、points_safety、clearing_points；这些旧名称是兼容接口，不说明传感器仍是D455。默认stride4在640×360上最多采样14400像素。普通导航过滤与坡道terrain后端仍二选一。跟随复用原身份跟踪、三维目标位置、局部地图和运动门控，不通过PointCloud2→深度图往返转换。las2人物跟随使用原生双目视场，RGB mask的frame不匹配时拒绝像素剔除，使用已有tracked_position_cylinder从原生深度生成的障碍/清障点中排除被选中人物；不是将所有人抹去。旧地图目标残留仍按原逻辑退回unknown，不当作已测量的自由空间。

参考K3的共享数据、按需输出、不同频率消费思路，不额外引入TSDF/ESDF：原导航/跟随已有局部障碍表示，本轮新增完整三维地图反而会重复计算。导航原生深度不承担RGB投影开销；人物检测与跟随共用同一次投影。

## 本轮降低的开销

- 无点云/对齐消费者时，不从GPU下载XYZ；无视差消费者时，不下载视差；左目预览仅在请求且到达限频周期时下载。原始深度结果与完整输出路径保持一致。
- RGB对齐按订阅需求计算。对齐和可选点云染色共用同一个z-buffer，避免重复ProjectDepth。
- 空间对齐只依赖真实三相机标定，不等待RGB到达。只有可选彩色点云消费者需要RGB像素及25ms近邻配对。检测端按原有35ms时间门限配对RGB和对齐深度，不修改源时间戳。
- 导航深度先发布，RGB网格深度保持同一个双目采集时间戳；没有硬件曝光同步声明。中间35ms等待方案实测仅47/99帧成功，最终已移除：空间配准和图像时间配对分别归生产者与检测消费者负责。5Hz接入测试不能作为20Hz整链路验证。
- 现有RGB采集/校正仍是独立生产者，可被检测、跟随等共享。未实现RGB硬件JPEG解码，也没有宣称整链路零拷贝或全板CPU低于60%。

## 启动与检查

所有旧相机/深度生产者必须先退出；相机与输出话题有flock独占锁。下面这个入口只启动共享相机和深度，不启动控制、检测或导航：

```bash
cd /home/onana/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch wla_uvc_depth las2_inputs.launch.py depth_rate:=5
```

只读核验（退出码2表示接口缺失，JSON逐项指出原因）：

```bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/onana/ros2_ws/config/uvc_rig/fastdds_uvc.xml
python3 tools/stereo_candidates/check_las2_consumers.py
```

原导航启动命令追加`obstacle_depth_source:=las2`即可切换障碍/terrain输入；D455仍是该启动文件默认选项，定位图像源与定位后端不随此参数改变。不要在已经运行导航消费者时再启动同输出话题的另一个depth_to_points。

跟随预览入口：

```bash
ros2 launch wla_person_follow follow.launch.py depth_source:=las2 enable_motion:=false
```

该入口使用已有配置的r680_mapping_floor和/wheel/odom_50hz。启用terrain:=true时，terrain同样读取原生深度，按同一深度时间戳对应。las2配置针对人物模式；需逐像素物体表面分割的object approach应显式使用depth_source:=las2_rgb，否则其原有frame/mask门控会保持等待，不以RGB mask套用左目图。

检测入口：

```bash
WLA_DEPTH_SOURCE=las2 WLA_PERCEPTION_BASE_FRAME=r680_mapping_floor ./robot_perception.sh run
```

WLA_DEPTH_SOURCE=classical可恢复原processed对齐深度入口。检测、跟随与导航必须使用已建立真实TF关系的同一个机身参考，不能仅修改frame字符串。检测模型、ReID、跟随命令和控制服务没有在本轮实机启动。

## 验证与边界

- 四个修改包编译成功；几何、模型输入、GPU无效视差及terrain核心CTest共4项通过。
- 已存真实JPEG与随机图等价性检查：融合输入和视差最大差0，深度差0mm；仅下载深度与完整下载结果一致。
- 域194实际depth_to_points节点：有效输入15帧出点云；过期/错CameraInfo frame/截断数据/缺TF各0帧；恢复后15帧出点云。fixture的初次测试误将1m高的合成平面排除在默认0.5m clearing band之外，修正fixture高度范围后通过，未放宽生产过滤参数。
- 域194原terrain场景回归：20度坡道不误判障碍，箱体障碍正确。
- 域198跟随C++/Python实际IPC回归test_actual_compute_cancel_and_late_start通过；只在隔离域中使用合成传感器/命令，不连接真车。
- 两个原启动文件的--show-args验证通过。可选彩色点云仍校验RGB内参与已加载rig.yaml一致；空间配准深度不依赖RGB是否在线，检测端缺少匹配RGB/深度/TF时不产生可信目标位置。
- 导航消费者新增输入源龄、内参frame/时效、图像布局检查，TF按图像时间查询；缺数据/TF停止相应输出，不制造空闲空间。此行为不是独立硬件急停。
- 实机最终结果见本文后续实测段和.runtime/las2_nav_follow_integration/。点云、深度都不是已通过米制精度/细小障碍物验收的导航安全证据。

仍缺真实r680_mapping_floor到三相机光学frame的完整TF，以及新机器人可用的/wheel/odom_50hz（含同一child_frame）或经验证的替代里程计。现有IMU/关节状态不能直接伪装成平面Odometry。没有新建identity mount、虚构地面高度或把standby默认速度当真实速度。CAD位置确认和三相机相互标定已具备，后续机身坐标接入应与电控现有坐标/支撑平面统一。

97°C应用层暂停按用户要求保持移除；Jetson系统热策略和满速风扇不变。

## 原生深度人物剔除回归

域198新增test_rgb_mask_is_not_applied_to_native_depth通过：注入其他光学frame的RGB mask，不发生像素掩膜应用；已有tracked_position_cylinder根据已绑定人物的三维位置移除目标障碍点，其他障碍点仍保留，停止命令生效。该用例是合成场景，不代替真机人体边界精度验收。

## 换电池后最终真实输入检查

移除不必要的RGB等待后，20秒观测收到原生深度100帧、RGB网格深度99帧，约5Hz，彩色图约20Hz。两路均为640×360/16UC1，各自frame与CameraInfo一致；深度节点CPU单核等效均值13.09%，最后一帧处理32.35ms（包括一次CPU空间投影，不代表全部RGB采集开销）。先前47/99配对率是中间等待方案的失败证据，不作为最终结果。

用户进一步确认只有三相机间标定，没有相机—IMU/机身外参。真实检查仍无相机到r680_mapping_floor的TF、无/wheel/odom_50hz发布者。感知消费接口与隔离跟随剔除验证完成，实际导航/跟随就绪仍受这些接口限制。RGB采集/校正与LAS2输入、只读网页已恢复；没有启动YOLO/人物跟随任务或导航控制。
