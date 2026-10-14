# OpenVINS 接入第一版

本包将固定版本 OpenVINS 接入 Wheel_nav，定位为**影子验证和禁止硬件输出的导航联调版本**。既有 cuVSLAM、RGB-D 分支仍可使用。实际标定、实物遮挡搬运、独立真值精度和完整导航恢复尚须验收；合成测试不能代替这些验收。

## 实现范围

- C++ 前端直接调用 OpenVINS VioManager / Propagator；单线程串行访问估计器，关闭后台初始化并发、在线标定、运行期 ZUPT 和 SLAM 特征状态。MSCKF 的视觉更新仍正常启用。
- 双目输入检查时间戳、帧名、尺寸、编码和同步偏差，并比对 CameraInfo.P 与固定内参；队列有界。IMU 检查时间顺序、间隔、有限值及量程配置。异常 IMU 将故障锁存，需显式 reset。
- 每个有效 IMU 采样驱动快速传播，保留 XYZ、完整姿态和三轴速度。图像持续到达但无有效特征时，由正常图像回调推进主状态，缺少视觉纠正仍可惯性输出；快速输出预测不额外推进主状态。完全停流时主状态不再周期性推进，IMU 历史可能增长，暂不承诺长期停流的有界缓存；没有伪造图像、没有第二套 ESKF。
- IMU 原点转为机身原点，包括杆臂旋转速度及协方差雅可比；位置姿态协方差使用全局扰动，twist 使用 child frame。输出仍继承上游快速协方差中角速度/偏置相关项的近似，并非完整联合误差模型。
- `EstimatorStatus` 的视觉证据来自通过三角化、卡方筛选、压缩后真正执行的 MSCKF EKF 更新。图像到达、特征被检测到和高频 odom 输出都不会直接赋予健康状态。
- 接触状态 UNKNOWN/CARRIED 时不允许地面运动。缺失真实接触接口时保持 UNKNOWN，不根据轮速为零自动生成 GROUNDED。
- 纯遮挡不会触发重启；输出超过 10 秒仍继续，质量单独报告。显式 reset 增加 generation，并更换 odom frame；质量门锁存该变化，避免拼接新旧地图。
- HF-Net 尚未包含在本次实现中，仍作为独立 RTAB-Map 增强模块。

## 构建

在独立 ROS 2 Jazzy 工作区导入 `openvins.repos`，必须使用指定提交，再应用补丁：

也可直接使用源码中的 `scripts/build_isolated.sh /absolute/path/to/new-workspace`，它校验上游提交、应用补丁并构建本包；不会启动机器人。

```bash
vcs import src < src/Wheel_nav/src/wla_openvins_navigation/openvins.repos
git -C src/open_vins apply ../Wheel_nav/src/wla_openvins_navigation/patches/openvins-v1.patch
source /opt/ros/jazzy/setup.bash
colcon build --executor sequential --packages-up-to wla_openvins_navigation \
  --cmake-args -DOV_BUILD_ROS_VISUALIZER=OFF -DENABLE_ARUCO_TAGS=OFF \
  -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG -g0'
source install/setup.bash
colcon test --packages-select wla_openvins_navigation --event-handlers console_direct+
colcon test-result --verbose
```

补丁包括：实际视觉更新计数和序号、快速传播请求的 IMU 时间处理、局部速度协方差的姿态雅可比，以及 Jazzy 的 cv_bridge 头文件。补丁来自固定上游提交，不对任意其他版本静默应用。

本机系统 cv_bridge 链接 OpenCV 4.6，而开发库为 OpenCV 4.8。推荐构建关闭上游 ROS 可视化程序，仅保留估计核心；本包直接复制已检查的 mono8 图像，不依赖 cv_bridge。必须检查最终程序没有混合加载两套 OpenCV；系统软件包不在本次修改范围内。

## 实机标定配置

复制 `config/calibration_manifest.example.yaml` 到自己的标定目录，替换其中全部示例数值，填写标定来源后才能设置 `calibration_validated: true`。

- `T_imu_base` 把机身坐标变换到 IMU 坐标。
- `T_imu_camera_rectified` 把每个**校正后的**相机坐标变换到 IMU 坐标。若标定的是原始 optical frame，先使用 CameraInfo.R 转换：`R_imu_rect = R_imu_raw * R_rect_raw.transpose()`。
- 内参来自校正图像的 CameraInfo.P 左上 3×3；左右物理基线保留在外参平移里，不再次把 P 的平移加到坐标中。
- `timeshift_cam_imu` 约定 `t_imu = t_camera + offset`。帧名必须与实际传感器消息一致。
- 图像帧与 CameraInfo 帧分别校验。实测 D455 右图像为 `d455_infra2_optical_frame`，右 CameraInfo 为 `d455_infra1_optical_frame`；可在相机 manifest 中单独填写 `camera_info_frame_id`，不能为了通过检查改写图像帧。
- IMU 输入必须保留重力，单位为 rad/s、m/s²，已经统一采样时钟；本包不使用消息中的姿态四元数。
- 噪声参数需来源明确。原工程中的“底盘 IMU 在机身中心且轴对齐”暂定 TF 不能代替标定。
- `T_imu_base` 中的 base 必须与导航使用的 `r680_mapping_floor` 一致；校验既有相机安装 TF 与新标定的相机到机身变换一致，第一版不会自动重写硬件安装 TF。

```bash
ros2 run wla_openvins_navigation generate_config.py measured_manifest.yaml /absolute/path/openvins_calibration
ros2 launch wla_openvins_navigation openvins.launch.py config_dir:=/absolute/path/openvins_calibration
```

默认输出 `/openvins_trial/odom_imu`、`/openvins_trial/odom`、`/openvins_trial/estimator_status`，不发布 TF、不启动相机或底盘。没有通过初始化时不会输出 odom。静态初始化需要先静置，再由人工提供适当运动激励；本包不自动驱动机器人初始化。

2026-10-13 原工作区入口已接入完整 D455 候选配置，使用 `~/ros2_ws/r680_nav.sh --openvins --openvins-imu d455 --map MAP --test-scan-only`。当前默认就是 D455 内置 IMU。详细参数来源、脚本所有权、初始化预览和验证边界见 [接入说明](../wla_r680_navigation/docs/openvins_20261013.md)。导航层显式 `--auto-vio-init` 仅增加默认关闭的初始化运动预览与独立安全节点；OpenVINS 硬件输出仍被启动校验禁止。

尚无联合标定且仅做接口联调时，manifest 可明确设置 `kind: commissioning`、`calibration_validated: false`，记录现有参数的来源与假设。生成时必须添加 `--allow-commissioning`，启动时必须设置 `allow_commissioning:=true`；原导航入口使用 `openvins_allow_commissioning:=true`。这一配置始终关闭运动许可，不能将接口测试结果当作标定或精度验收。

初始化排查时可在生成的 `frontend.yaml` 参数中设置 `estimator_verbosity: INFO`，观察视觉位移与 IMU 激励检查。默认 `WARNING`；信息级日志仅用于排查，不能将调低初始化阈值等同于标定。

初始化前的 IMU 暂时未到达保持等待；若采样间隔中断则丢弃初始化窗口重新积累。初始化之后的 IMU 超时/采样间隙仍锁存故障，需显式恢复。所有阶段的非有限值、错误帧及时间倒退仍拒绝。

D455 启动时 global-time 可能尚未稳定。尚未初始化且未输出 odom 时，正时间戳超出既有新鲜度范围的样本丢弃，并清空已积累的初始化窗口，状态为 `waiting_for_imu_clock_stability`。这些样本不会送进滤波器，也不会触发初始化运动预览。初始化后的时间异常继续锁存；运行期时间限制保持原值。

## 接入现有导航进行无硬件输出联调

将附带的 `navigation-integration.patch` 应用到固定基线的 Wheel_nav 副本，再在隔离工作区编译修改后的导航包。只编译前端不会更新已有导航启动文件：

```bash
source /home/orin/ros2_ws/install/setup.bash
source /absolute/path/to/trial-workspace/install/setup.bash
cd /absolute/path/to/trial-workspace
colcon build --executor sequential --packages-select wla_r680_navigation
source install/setup.bash
```

```bash
ros2 launch wla_r680_navigation bringup.launch.py \
  odom_source:=openvins openvins_config_dir:=/absolute/path/openvins_calibration \
  enable_hardware_output:=false
```

该分支由本包独占 `/d455_slam/odom` 及 `d455_floor_odom -> r680_mapping_floor`，不启动 cuVSLAM、平面 degraded_odometry 或现有 EKF。RTAB-Map 使用门控后的 `/r680_nav/mapping_odom`，不再接收同一 IMU 作为额外姿态输入。

现有 `vo_watchdog` 的重启、平面轮速先验恢复逻辑不适合抱起场景，因此该分支使用独立质量门替代；不改变旧分支行为。定位模式同时要求 RTAB-Map `/d455_slam/info` 报告 `loop_closure_id>0` 或 `proximity_detection_id>0`，以及时间戳相差不超过 50 ms 的新鲜 `localization_pose`，两者都必须晚于最近的视觉失效；候选似然、持续位姿输出或 map→odom 广播不能单独确认恢复。确认后由持续定位位姿维持有效性，视觉再次失效或位姿超过 3 秒未更新会撤销确认。完整六自由度全局重定位一致性策略和可用接触证据仍需后续实车验收，第一版在启动校验中禁止 `enable_hardware_output:=true`。

`/r680_nav/contact_state` 的输入契约为 `std_msgs/String`，取值 `GROUNDED / CARRIED / UNKNOWN`，需由真实接触判断模块持续更新（有效期 0.3 s）。本包不生产该证据。建图只在接地且视觉健康时插入；定位模式允许在不接地时恢复视觉地图匹配，但继续禁止地面运动。

定位输入与运动许可使用不同门限：恢复期已有连续有效视觉更新时，允许送给 RTAB-Map 尝试重定位，即使局部位置不确定度仍较大；地面运动仍要求完整健康条件，避免“需要地图恢复、却被局部不确定度挡住地图输入”的闭环等待。

## 验证与记录

- `test_adapter`：非零杆臂、任意旋转、协方差有限差分及状态策略。
- `test_inertial`：已知初始状态下 20 秒 IMU 传播、非零正负时间偏移、黑图、无图像、10 秒视觉约束缺失后恢复。使用合成数据及测试专用真值初始化，不代表真实初始化或漂移精度。
- `test/test_ros_contract.py`：隔离 ROS 域内验证消息、故障锁存、reset 代次、TF 默认关闭和质量门。
- `record_metrics.py`：记录实际输出时间戳间隔、状态和不确定度，文件以独占方式创建。
- `stereo_fault_injector.py`：回放用黑图/停流注入，保留真实时间戳。默认读取 `/replay/infra{1,2}/image_rect_raw`，输出 `/openvins_fault/infra{1,2}/image_rect_raw`；用独立回放标定目录将 frontend.yaml 的左右输入指向该输出，原标定几何不变。`fault:=black/drop`、`start_after_s` 和 `duration_s` 通过 ROS 参数设置。不要对生产传感器话题进行同名重发布。

实际试验仍需完成：充足初始化 → 正常跟踪 → 1/3/5/10 秒遮挡与搬运 → 恢复；分别录制图像黑屏和相机停流，使用独立真值测量位姿误差。低延迟、低资源占用与真实精度均以本机测量为准。
