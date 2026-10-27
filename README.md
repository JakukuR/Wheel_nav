# R680 轮腿机器人工作区

Orin NX 上的 `~/ros2_ws`：ROS 2 Jazzy，CycloneDDS，默认 ROS 域 73。
包含底盘仲裁、D455 感知、目标跟随/靠近、Kalo 调试台、导航建图和 Agent 接口。

## 功能

| 功能 | 入口 | 主要包 |
| --- | --- | --- |
| 底盘、手柄与速度仲裁（常驻） | systemd `r680-manual-base` | `wla_manual_control`、`gamepad_contorl` |
| D455 相机（彩色＋对齐深度，可选红外/IMU，含安装 TF） | `r680_camera.sh` | `wla_r680_navigation` |
| 感知：YOLOE 分割＋BoT-SORT＋OSNet ReID，输出机器人相对位置 | `r680_perception.sh` | `wla_r680_navigation` |
| 人物跟随 / 人物靠近 / 物体靠近（模式 0/1/2） | `r680_person_follow.sh`、`r680_follow_trial.sh` | `wla_follow_transport`、`wla_person_follow` |
| Kalo 调试台（浏览器 WebGL，画面＋检测＋障碍体素） | `r680_perception.sh debug-camera` | `wla_r680_navigation` |
| 导航 / 建图（RTAB-Map＋Nav2 MPC，RGB-D、cuVSLAM 或 OpenVINS 里程计） | `r680_nav.sh`、`r680_mapping.sh` | `wla_r680_navigation`、`wla_diff_mpc` 等 |
| Agent 接口（语义导航、人物任务、观测） | `r680_agent.sh` | `wla_behavior_bridge` |

## 启动链路

**常驻（开机自启）**：底盘驱动 ＋ 手柄 ＋ C++ `command_mux`。mux 是 `/r680_base/cmd_vel`
唯一发布者，优先级 手柄 ＞ 跟随 ＞ 导航，输入过期或急停即输出零。

**跟随**（每行一个终端）：

```bash
cd ~/ros2_ws
bash r680_camera.sh run                                   # 相机 + 安装 TF
./r680_perception.sh run --base-frame r680_mapping_floor  # 检测器
./r680_person_follow.sh run enable_motion:=true           # 跟随后端；省略 enable_motion 为预览
./r680_follow_trial.sh follow --mode 0 --continuous --allow-motion   # 选目标 ID 下发任务
```

```text
D455 → 检测器 → /r680/perception/relative_objects → 跟随（C++ 收发 + Python 计算）
     → /r680/person_follow/command_candidate → command_mux → 底盘
```

调试页：`./r680_perception.sh debug-camera`，浏览器打开 `http://<robot-ip>:18081/`。

**导航 / 建图**（相机先启动；cuVSLAM/OpenVINS 需 `r680_camera.sh run --infra`）：

```bash
./r680_nav.sh --map map-2026-10-08-1 --cuvslam            # 定位、规划、预览
./r680_nav.sh --map map-2026-10-08-1 --cuvslam --enable-motion
./r680_mapping.sh --cuvslam --dry-run
```

**Agent**：`./r680_agent.sh preview|execute|person-api|person|observations|query|navigation`，
`./r680_agent.sh help` 查看参数。

## 安全默认

- 导航、建图预览和跟随默认不出速度；需显式 `--enable-motion` / `enable_motion:=true`，
  跟随还需操作者下发任务并持续续租。
- 急停：手柄急停键或 `/r680_base/set_estop`；复位需停稳并重新下发任务。
- 软件门禁不等于物理制动验收，实车运动前确认场地空旷。

## 目录

```text
ros2_ws/
├── r680_*.sh                 入口脚本（见上）
├── src/
│   ├── wla_manual_control/   C++ command_mux 与速度策略
│   ├── wla_follow_transport/ 跟随的 C++ ROS 收发与输出门禁
│   ├── wla_person_follow/    跟随计算：跟踪、身份恢复、局部地图、MPC
│   ├── wla_follow_local/     跟随用原生几何模块 _wla_sim_geometry
│   ├── wla_r680_navigation/  相机、感知、Kalo、导航建图
│   ├── wla_behavior_*        Agent 桥接、行为树、接口
│   ├── wla_detection_bridge/ 检测消息
│   ├── wla_diff_mpc/         Nav2 MPC 控制器
│   ├── wla_cuvslam_*/  wla_openvins_navigation/  open_vins/   视觉里程计
│   ├── explore/  explore_lite_msgs/   建图前沿探索
│   └── gamepad_contorl/      手柄（目录名保留原拼写）
└── maps/map-2026-10-08-1/    唯一提交的示例地图（Git LFS）
```

工作区外依赖：`~/r680_chassis_candidate_ws`（底盘驱动）、`~/Wheel_Legged_Agent`（模型与 Agent）。
运行数据：跟随日志在 `.runtime/follow-trials/`、`.runtime/follow-diagnostics/`，导航运行数据在 `~/nav_run`。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
source ~/r680_chassis_candidate_ws/install/setup.bash
export CMAKE_PREFIX_PATH="$HOME/.local/wla_mpc_deps:${CMAKE_PREFIX_PATH:-}"   # OSQP
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip wla_cuvslam_navigation wla_cuvslam_validation
```

cuVSLAM 两包不走上面的构建：前端需先按 [SDK 补丁说明](src/wla_r680_navigation/addons/cuvslam-sdk/README.md)
重建 SDK，再运行 `bash src/wla_cuvslam_navigation/build_frontend.sh`（安装到 `.runtime/cuvslam_navigation`）。

## 测试

隔离域 198，不连底盘：

```bash
export ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RUN_SIM_ROS=1
python3 -m pytest -q src/wla_person_follow/test
ctest --test-dir build/wla_follow_transport
```

完整命令见 [全链路测试](src/wla_person_follow/FULL_CHAIN_TESTS.md)。

## 克隆

```bash
git lfs install
git clone https://github.com/OnanaRobotics-MMAgent/Wheel_Legged_Nav.git ~/ros2_ws   # 或 JakukuR/Wheel_nav
cd ~/ros2_ws && git lfs pull
```

只要源码可加 `GIT_LFS_SKIP_SMUDGE=1`；示例地图数据库为 LFS 指针时不能重定位。

## 文档

- [导航与建图](src/wla_r680_navigation/README.md) · [Kalo 调试台](src/wla_r680_navigation/docs/perception_debug.md)
- [跟随](src/wla_person_follow/README.md) · [C++ 收发层](src/wla_person_follow/HYBRID_TRANSPORT.md) · [三模式操作](FOLLOW_TRIAL.md)
- [速度仲裁 mux](src/wla_manual_control/README.md) · [源码包历史](docs/package-history.md)
