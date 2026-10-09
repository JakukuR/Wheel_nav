# Wheel Legged Navigation 工作区

本仓库归档 Orin 的 `~/ros2_ws` 源码工作区，包含入口脚本、8 个 ROS 包和一份示例地图。
导航链路为 **D455 双目＋车身 IMU -> cuVSLAM -> 有预算的连续里程计 -> RTAB-Map 重定位 -> Nav2 MPC -> 速度平滑／碰撞监控／最终指令门禁 -> 底盘**。

详细链路、话题、故障恢复和参数说明见 [导航包 README](src/wla_r680_navigation/README.md)。导航脚本默认控制器是 MPC、默认里程计前端仍为 RGB-D；当前 cuVSLAM 链路需显式指定 `--cuvslam`。

## 工作区内容

```text
ros2_ws/
├── src/
│   ├── wla_r680_navigation/      导航、建图、门控、Web 和地图管理
│   ├── wla_diff_mpc/             Nav2 MPC 控制器
│   ├── wla_cuvslam_navigation/   双目＋车身 IMU 前端
│   ├── wla_cuvslam_validation/   cuVSLAM 验证与 SDK 准备说明
│   ├── wla_person_follow/        人员跟随
│   ├── explore/                  前沿探索
│   ├── explore_lite_msgs/        探索消息接口
│   └── gamepad_contorl/          手柄控制（保留原目录拼写）
├── maps/map-2026-10-08-1/         唯一提交的示例地图
├── r680_nav.sh
├── r680_mapping.sh
├── r680_teleop.sh                指向导航包脚本的符号链接
├── r680_vio_test.sh
└── start_nav.md                  原工作区启动笔记
```

不提交 `build/`、`install/`、`log/`、`logs/`、`.runtime/`、`.gateway-backup-*`、`__pycache__`、`.pytest_cache` 及编译缓存。
源包内原有的 `.git` 不作为子模块提交；从 GitHub 克隆后使用工作区根目录统一管理。
其他地图保留在 Orin，本仓库默认忽略它们。

## 克隆与示例地图

示例地图包含二维栅格、RTAB-Map 数据库、语义数据、编辑记录及校验清单。
`rtabmap.db` 为 217,227,264 字节，通过 Git LFS 保存完整文件。

先安装 Git LFS，然后克隆一个远端：

```bash
git lfs install
git clone https://github.com/OnanaRobotics-MMAgent/Wheel_Legged_Nav.git ~/ros2_ws
cd ~/ros2_ws
git lfs pull
```

目标目录须为空；已有 Orin 工作区不要直接覆盖。另一远端为 `https://github.com/JakukuR/Wheel_nav.git`。
源码提交历史沿用原导航仓库的 `main`，导航包现在位于 `src/wla_r680_navigation`。
从旧的“仓库根目录就是导航包”结构更新时，应使用完整工作区布局，不能直接将整个新仓库放进旧包目录。

只获取源码而暂不下载示例数据库：

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone \
  https://github.com/OnanaRobotics-MMAgent/Wheel_Legged_Nav.git ~/ros2_ws
```

数据库仍为 LFS 指针时不能运行地图重定位；在工作区执行 `git lfs pull` 获取完整文件。

## 构建与外部依赖

现有 Orin 环境使用 ROS 2 Jazzy、`~/r680_chassis_candidate_ws` 底盘工作区，
以及 `~/Wheel_Legged_Agent/deploy/r680/cyclonedds-local.xml`。
底盘驱动、完整 cuVSLAM SDK、SDK 构建与运行缓存不包含在本仓库。
SDK 准备说明见 [验证包](src/wla_cuvslam_validation/README.md)。

**cuVSLAM 前端依赖已修补 SDK。** 先按 [SDK 补丁说明](src/wla_r680_navigation/addons/cuvslam-sdk/README.md)
应用视觉来源 API 补丁并重建 SDK，再构建前端。
OSQP / OsqpEigen 的现有前缀为 `~/.local/wla_mpc_deps`。

```bash
source /opt/ros/jazzy/setup.bash
source ~/r680_chassis_candidate_ws/install/setup.bash
export CMAKE_PREFIX_PATH="$HOME/.local/wla_mpc_deps:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$HOME/.local/wla_mpc_deps/lib:${LD_LIBRARY_PATH:-}"
cd ~/ros2_ws
colcon build --packages-select wla_diff_mpc wla_r680_navigation \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash src/wla_cuvslam_navigation/build_frontend.sh
```

`src/` 已包含配套源码包，无需再从 `addons/` 复制。
其他源码包按各自 README 和依赖构建；本次归档不代表所有功能都经过完整工作区构建或实车验收。

## 导航与建图

定位、规划和命令预览：

```bash
cd ~/ros2_ws
./r680_nav.sh --map map-2026-10-08-1 --cuvslam
```

操作者确认起步条件后启动受限 VIO 初始化与真实导航：

```bash
./r680_nav.sh --map map-2026-10-08-1 \
  --cuvslam --auto-vio-init --enable-motion
```

导航默认关闭底盘运动，只有 `--enable-motion` 开放输出。
建图脚本的默认行为不同，预览必须指定 `--dry-run`：

```bash
./r680_mapping.sh --cuvslam --dry-run
```

地图重定位接受一次有效几何匹配；连续里程计内部的三帧视觉返回确认是另一项检查。
轮速＋IMU 接管仍有 2 秒／0.30 米／0.80 弧度预算，故障或预算耗尽仍停车。
运行日志、数据库工作副本和参数快照写到 `~/nav_run`，不写回示例地图。

此工作区快照保留了原本尚未提交的 Nav2、RViz、Web、手柄配置及辅助源码。
本次只归档与发布，没有启动导航、建图或底盘运动；此前软件修复的实车验证边界仍见导航包说明。

## 提交历史

原导航历史及其他 6 个包的原提交已接入工作区 `main`，原提交号保留。路径对应关系和查询方法见 [源码包历史索引](docs/package-history.md)。
