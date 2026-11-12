# 新机器人地面估计与 C++ 坐标变换

日期：2026-11-12。目标：`onana@192.168.50.1`，ROS 2 Jazzy。

## 1. 模块和当前边界

新增独立包 `src/wla_ground_estimation`。运行时 ROS 收发、URDF 轮心运动学、有限轮宽支撑距离、四元数时间插值、ToF 平面拟合和传感器平面变换都使用 C++20/Eigen；Python 只用于隔离 ROS 测试。遵循 [PY_CPP_ARCHITECTURE.md](../src/wla_person_follow/PY_CPP_ARCHITECTURE.md) 的核心/传输分层、不可复制状态所有者、异常输入检查、编译选项及独立 C++ 数值测试约定，不引入新的大型依赖。

用户明确要求不改电控模型，且说明两份模型的差异属于图纸坐标问题。本次没有改写 `/home/onana/wheel_leg_nav20261105_lqr10_cpp` 的 URDF、配置、零位或代码，没有调用电机服务。节点只订阅数据并发布估计，不发布运动命令、不自动启动电控、不广播 TF。

实际接口已观察到 `/imu/data` 约 200 Hz、`/motor35_powered/state` 含实测 `q_rad`；尚未观察到 `/joint_states` 和四个 ToF 的 ROS 数据。当前电控接地观测为 Unknown，不能将其控制侧默认的 `ground_weight=1` 当成已经验证接地。节点会发布带 `valid=false` 的平地候选，不能把候选高度称为实测地面高度。

## 2. 固定安装与实时姿态分别计算

记 `B` 为 CAD/IMU 位置基准 `base_link`，`M` 为电控模型根 `base_Link`，`S` 为某个安装 frame，`W` 为局部重力参考。

- `T_BS`：七个实际安装传感器的固定外参，来自 CAD 导入记录；上左目未安装。
- `R_WB(t)`：实时 IMU 姿态四元数对应的旋转；需先确认实际测量轴和世界坐标约定。
- `p_WB(t)`：本模块只估计机身相对局部地面的竖直高度；局部 x/y 原点随当前机身投影定义，不提供全局里程计。

点和姿态按下式组合：

```text
p_W = R_WB(t) [R_BS p_S + t_BS] + p_WB(t)
R_WS(t) = R_WB(t) R_BS
T_WS(t) = T_WB(t) T_BS
```

因此传感器与机身之间的 TF 不需要因整体 roll/pitch/z 改变而重新标定；改变的是整个机身相对局部参考的位姿。四元数只给旋转，不能单独恢复高度或地面坡度。

## 3. 轮腿高度：实测关节角、轮心和接地约束

只读加载电控现用 `assets/Motor35_WF.urdf`，保留它已有的电机方向和关节零位修正，用实测角度逐关节做 FK；不使用目标 `reference_height_m`，不使用目标 `reference_q_rad`。状态数组的固定顺序来自现用电控 `motor35::names`：

```text
abad_L, abad_R, hip_L, hip_R, knee_L, knee_R, wheel_L, wheel_R
```

两版髋基座位置的差给出图纸参考变换：`p_B = p_M + [0.020, 0, -0.0005] m`，旋转不变。这个换算只存在于本模块的 `body_from_model_xyz_m` 配置中，未写回电控模型。用户确认腿部结构相同、差异是根原点；图纸传感器安装仍需按实物复核，所以默认 `mount_geometry_verified=false`。

轮子读取 collision 圆柱的原点、轴向、半径和长度。现用模型轮径约 80 mm、宽约 32 mm。设轮心为 `c_i`、轮轴单位向量为 `a_i`、轮半径为 `r_i`、半轮宽为 `w_i`，平地假设下在机身系中的上法向为 `n_B=R_WB^T [0,0,1]`。有限圆柱的支撑距离和高度为：

```text
s_i = r_i sqrt(1 - (n_B dot a_i)^2) + w_i abs(n_B dot a_i)
h_i = s_i - n_B dot c_i
plane_B: n_B dot p_B + h = 0
```

这包含侧倾后轮宽对接地点的影响。已知接地的一侧可提供平地高度；两侧都接地时取平均，并检查两侧结果差。两侧都腾空时不产生轮腿地面估计。没有已知接地点时，只对非已知腾空的轮子计算明确标记的候选。仅两个轮子不能确定任意三维地面法向，因此此模式名为 `kinematic_flat`；地面坡度需要其他观测。

## 4. 四个 ToF 拟合局部地面

`ground_source=tof_plane` 时接收四路 `sensor_msgs/Range`。默认话题只是待对接的接口占位，当前没有证据表明电控已发布这些名字。驱动需要给出物理通道、量程/失效值、时间戳及 frame；实际测距轴必须映射到对应 Range frame 的 +X，确认后才能启用 ToF 路径。

```text
p_Bi = R_B_ToFi [range_i, 0, 0] + t_B_ToFi
```

至少三路新鲜、非共线且时间足够接近的点才拟合平面。法向按重力上方向定号，检查地面在机身下方、点残差和法向倾角。四路不一致时拒绝该组，不静默改用轮腿高度。ToF 可能打到台阶、障碍或边缘，几何平面拟合本身不证明这些点都属于同一地面。

目前只对近似同时的 ToF 端点组做拟合，没有对组内每个时刻做完整六自由度运动补偿。默认组内总跨度最多 10 ms；运动快或驱动延迟不确定时不能将其当作精确同步观测，需要后续曝光/采样时间和运动补偿验证。

机身系平面定义为 `n_B dot p_B+d_B=0`、上法向单位长度。转换到每个传感器安装系：

```text
n_S = R_BS^T n_B
d_S = d_B + n_B dot t_BS
```

坡面模式中 `d_B` 是法向距离，竖直高度为 `d_B / (R_WB n_B).z`，两者不能混用。

## 5. 时间、有效性和输出

IMU 历史有界缓存最多 600 个样本；按关节消息时刻寻找前后 IMU，使用归一化 SLERP，只在两端样本都存在且跨度不超过 20 ms 时插值；不外推、不补造采样。无姿态、四元数非法、时间倒退、关节缺失或超时均发无效结果。

优先使用有时间戳的标准 `/joint_states`。目前只有电控 JSON 状态时，在节点接收状态的时刻取姿态，标记 `controller_state_receipt_approximate`；该状态没有逐关节采样时间，不能伪称精确硬件对齐，且永远不会因为把参数设为 true 而把此接收时间近似升级为 `timing_verified=true`。

默认四个核对项均为 false，记录的是当前已知证据边界：

| 参数 | 需要核对的实际内容 |
|---|---|
| `imu_axes_verified` | 原始 IMU/ROS 输出的轴向、姿态旋转方向和重力约定 |
| `mount_geometry_verified` | 图纸到实物的原点、安装位、视轴/测距轴和根坐标换算 |
| `sample_times_verified` | 标准关节与 IMU、ToF 使用的实际采样时间来源和时钟关系 |
| `tof_axes_verified` | 四个通道实际对应关系和 Range frame 的 +X 测距方向 |

`valid=true` 需要估计成立、对应模式的接地或 ToF 条件成立，以及姿态/安装/采样时间已核对；不能为消除 invalid 而直接修改核对项。

| 输出 | 内容 |
|---|---|
| `/ground_estimation/plane` | 自定义 `GroundEstimate`；含时间/frame、has_estimate、valid、方法/原因、法向、平面偏移、竖直高度、左右轮候选、接地状态、时间依据及七个安装系的地面平面 |
| `/ground_estimation/status` | JSON 运行状态、真实输入计数、候选高度、失败原因；后端明确为 cpp |
| `/ground_estimation/sensor_poses` | 仅 valid 时发布 PoseArray；顺序同 plane.sensor_names，位姿在 `ground_estimation/local_reference` 中，仅为局部参考 |

本节点不发布 TF，不占用 `map→odom`、`odom→base_link` 或任何现有传感器 child。后续接入统一 TF 所有者时，再从有效结果计算动态机身位姿；相机安装 frame 与 optical frame 的校准关系仍单独处理。

## 6. 常用命令与参数

目标 Orin，域 0。入口只有估计节点：

```bash
cd /home/onana/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run wla_ground_estimation ground_estimator --ros-args \
  --params-file /home/onana/ros2_ws/config/ground_estimation/new_orin.yaml
```

另一个已 source 的终端查看：

```bash
ros2 topic echo /ground_estimation/status
ros2 topic echo /ground_estimation/plane
```

ToF 真实接口、方向和采样时间核对后，使用自己的核对配置切换 `ground_source:=tof_plane`；不要把占位话题或 false 核对项直接当作已接通硬件。

当前工程初值：数据最大源龄 100 ms、IMU 插值跨度 20 ms、ToF 组内总跨度 10 ms、左右轮高度差 30 mm、ToF 点最大平面残差 15 mm、地面倾角上限 0.61 rad。它们用于拒绝明显不一致结果，并非已测传感器精度；应由驱动采样延迟、实测噪声和接地对照数据进一步收紧或重估。

构建与 C++ 测试：

```bash
colcon build --packages-select wla_ground_estimation --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
ctest --test-dir build/wla_ground_estimation --output-on-failure
```

隔离 ROS 合成测试（只在非零独立域发布模拟数据）：

```bash
ROS_DOMAIN_ID=208 python3 src/wla_ground_estimation/test/test_ros.py \
  --executable /home/onana/ros2_ws/install/wla_ground_estimation/lib/wla_ground_estimation/ground_estimator \
  --params /home/onana/ros2_ws/config/ground_estimation/new_orin.yaml \
  --fixture /home/onana/ros2_ws/src/wla_ground_estimation/test/data/kinematic.urdf
```

两组 C++ 测试覆盖有限轮宽、接地/腾空/未知、roll/pitch、根平移、轮转动、FK、平面变换、异常输入，以及 SLERP、四元数反号、缺样本/间隔/时间回退。新 Orin 上 8 项隔离 ROS 场景已通过：实测角影响高度、七安装系平面、未知接地、腾空、IMU 中断、关节中断、四 ToF 平面、ToF 异常拒绝。软件测试不代表真机高度或图纸外参已标定。

## 7. 参考代码的实际借鉴点

- `sp_vision_25` 核对提交 `bd9f5e798fa3c6dd3b483ae6627796afb41c608d`：[solver.cpp](https://github.com/TongjiSuperPower/sp_vision_25/blob/bd9f5e798fa3c6dd3b483ae6627796afb41c608d/tasks/auto_aim/solver.cpp) 将固定相机外参与 IMU 旋转分层组合；[gimbal.cpp](https://github.com/TongjiSuperPower/sp_vision_25/blob/bd9f5e798fa3c6dd3b483ae6627796afb41c608d/io/gimbal/gimbal.cpp) 按时间使用四元数插值。
- `auto_aim_rps26` 核对提交 `f1ddedd65d93e4660fbda67520b9b7fc38bbf761`：[tracker_node.cpp](https://github.com/aiyo472/auto_aim_rps26/blob/f1ddedd65d93e4660fbda67520b9b7fc38bbf761/src/rm_auto_aim/armor_tracker/src/tracker_node.cpp) 保留观测 header 并通过 TF 转换；[rm_gimbal.urdf.xacro](https://github.com/aiyo472/auto_aim_rps26/blob/f1ddedd65d93e4660fbda67520b9b7fc38bbf761/src/rm_gimbal_description/urdf/rm_gimbal.urdf.xacro) 区分安装坐标与 optical frame。其串口代码的 pitch 取负属于该机坐标约定，不能照搬到本机。

只参考坐标与时间处理方式，未移植自瞄、控制或发射功能，也未复制参考仓库的模型、权重或控制参数。

## 8. 本次真机只读验证

域 0，12.02 秒共收到 2373 个 IMU 样本、239 个电控状态输入，发布 593 个地面结果，其中 590 个为候选、0 个为有效测量。末帧候选高度约 0.08942 m，左右候选分别约 0.09048/0.08836 m；这是当前关节/姿态在平地接触假设下的计算结果，不能当作已经量尺验证的真实离地高度。接地状态为 `[Unknown, Unknown]`，时间依据为 `controller_state_receipt_approximate`；节点如实保留无效原因，没有修改核对项获得 valid。

估计节点收到 SIGINT 后正常退出，未留下常驻任务。构建与接口/合成测试、真机候选输出分别验证；没有执行机器人运动、腾空或坡面物理试验。
