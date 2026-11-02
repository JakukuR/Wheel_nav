# R680 轮腿机器人工作区

这是 Orin NX 上的 `~/ros2_ws`：ROS 2 Jazzy，CycloneDDS，默认 ROS 域 73。

工作区包含以下部分：底盘速度仲裁、D455 感知、目标跟随与靠近、Kalo 调试台、导航建图，以及 Agent 接口。

## 功能

| 功能 | 入口 | 主要包 |
| --- | --- | --- |
| 底盘、手柄与速度仲裁（常驻） | systemd `r680-manual-base` | `wla_manual_control`、`gamepad_contorl` |
| D455 相机（彩色 + 对齐深度，可选红外 / IMU，含安装 TF） | `r680_camera.sh` | `wla_r680_navigation` |
| 感知：YOLOE 分割 + BoT-SORT + OSNet ReID，输出相对机器人的位置 | `r680_perception.sh` | `wla_r680_navigation` |
| 人物跟随 / 人物靠近 / 物体靠近（mode 0/1/2） | `r680_person_follow.sh`、`r680_follow_trial.sh` | `wla_follow_transport`、`wla_person_follow` |
| Kalo 调试台（浏览器 WebGL：画面 + 检测 + 障碍体素） | `r680_perception.sh debug-camera` | `wla_r680_navigation` |
| 导航 / 建图（RTAB-Map + Nav2 MPC；里程计可选 RGB-D、cuVSLAM 或 OpenVINS） | `r680_nav.sh`、`r680_mapping.sh` | `wla_r680_navigation`、`wla_diff_mpc` 等 |
| Agent 接口（语义导航、人物任务、观测） | `r680_agent.sh` | `wla_behavior_bridge` |

## 常驻服务

开机自启三项：底盘驱动、手柄、C++ `command_mux`。

- `command_mux` 是 `/r680_base/cmd_vel` 的唯一发布者。
- 优先级：手柄 > 跟随 > 导航。
- 任一输入过期或急停时，输出为零。

## 跟随

在车上依次运行，每条命令一个终端：

```bash
cd ~/ros2_ws
./r680_camera.sh run
./r680_perception.sh run
./r680_person_follow.sh run transport:=cpp enable_motion:=true
./r680_follow_trial.sh follow --mode 0 --continuous --allow-motion
```

- 前三条分别启动相机、检测器、跟随后端。省略 `enable_motion` 时后端只预览，不出速度。
- 最后一条用来选目标 ID 并下发任务，Ctrl+C 停止。
- 从自己的电脑操作时，用 `ssh -t orin@<车 IP> '…'` 在车上执行。

数据流：

```text
D455 → 检测器 → /r680/perception/relative_objects → 跟随（C++ 收发 + Python 计算）
     → /r680/person_follow/command_candidate → command_mux → 底盘
```

注意事项：

- **同一时间只能运行一个跟随后端。** 不要和 Wheel_Legged_Agent 的 `tools/run_all_actions.py` 同时运行：它会再起一套后端，向同一个话题发布速度，导致跟随变慢。
- **改了 `wla_person_follow` 后，先执行 `colcon build --packages-select wla_person_follow`，再重启后端。** 它在 install 里是文件副本，不构建就重启仍会运行旧代码。
- **停止后端**：`pkill -INT -f follow.launch.py`。

后台启动、行为说明、参数和日志分析见 [跟随 README](src/wla_person_follow/README.md)。

调试页：运行 `./r680_perception.sh debug-camera`，然后在浏览器打开 `http://<车 IP>:18081/`。

## 导航 / 建图

相机要先启动。cuVSLAM 和 OpenVINS 需要红外图像，相机改用 `r680_camera.sh run --infra` 启动。

```bash
./r680_nav.sh --map map-2026-10-08-1 --cuvslam            # 定位、规划、预览
./r680_nav.sh --map map-2026-10-08-1 --cuvslam --enable-motion
./r680_mapping.sh --cuvslam --dry-run
```

## Agent

```bash
./r680_agent.sh preview|execute|person-api|person|observations|query|navigation
```

运行 `./r680_agent.sh help` 查看参数。

## 安全默认

- 导航、建图预览和跟随默认都不出速度，需要显式加 `--enable-motion` 或 `enable_motion:=true`。
- 跟随还需要操作者下发任务并持续续租。
- 急停：手柄急停键，或调用 `/r680_base/set_estop`。复位前必须停稳，并重新下发任务。
- 软件门禁不等于物理制动验收。实车运动前请确认场地空旷。

## 目录

```text
ros2_ws/
├── r680_*.sh                 入口脚本（见上）
├── src/
│   ├── wla_manual_control/   C++ command_mux 与速度策略
│   ├── wla_follow_transport/ 跟随的 C++ ROS 收发与输出门禁
│   ├── wla_person_follow/    跟随计算：跟踪、身份恢复、局部地图、MPC、避障
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

工作区外的依赖：

- `~/r680_chassis_candidate_ws`：底盘驱动。
- `~/Wheel_Legged_Agent`：模型与 Agent。

运行数据：

- 跟随：`.runtime/follow-backend-*.log`、`.runtime/follow-trials/`、`.runtime/follow-diagnostics/`。
- 导航：`~/nav_run`。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
source ~/r680_chassis_candidate_ws/install/setup.bash
export CMAKE_PREFIX_PATH="$HOME/.local/wla_mpc_deps:${CMAKE_PREFIX_PATH:-}"   # OSQP
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip wla_cuvslam_navigation wla_cuvslam_validation
```

cuVSLAM 的两个包不走上面的构建：

1. 先按 [SDK 补丁说明](src/wla_r680_navigation/addons/cuvslam-sdk/README.md) 重建 SDK。
2. 再运行 `bash src/wla_cuvslam_navigation/build_frontend.sh`，产物安装到 `.runtime/cuvslam_navigation`。

## 测试

在隔离域 198 运行，不连底盘：

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

- 只需要源码时，克隆前加 `GIT_LFS_SKIP_SMUDGE=1`。
- 这样拿到的示例地图数据库只是 LFS 指针，不能用来重定位。

## 文档

- [导航与建图](src/wla_r680_navigation/README.md) · [Kalo 调试台](src/wla_r680_navigation/docs/perception_debug.md)
- [跟随](src/wla_person_follow/README.md) · [C++ 收发层](src/wla_person_follow/HYBRID_TRANSPORT.md) · [三模式操作](FOLLOW_TRIAL.md)
- [速度仲裁 mux](src/wla_manual_control/README.md) · [源码包历史](docs/package-history.md)
