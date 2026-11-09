# ros2_ws 启动命令与常用参数（2026-11-09）

当前 Orin：`192.168.9.129`，用户 `orin`，工作区 `/home/orin/ros2_ws`。SSH 密码不写入仓库。
本文命令供操作者按场景使用；本轮仅检查文档/源码/环境，没有启动硬件、发送任务或重启服务。

## 连接和每终端环境

```bash
ssh -t orin@192.168.9.129
cd /home/orin/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/r680_chassis_candidate_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/ros2_ws/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml"
```

根脚本一般自行加载环境；直接执行 `ros2` 查询、launch 或服务时先做上述设置。
DDS 配置绑定 `lo` 且关闭组播，电脑上的 ROS 节点不能仅靠同域发现机器人；通过 SSH 在车上操作。

网页：[Kalo 感知/跟随](http://192.168.9.129:18081/) · [导航/建图](http://192.168.9.129:8080/) ·
[bag 回放](http://192.168.9.129:18082/)。相应进程运行时页面才可用。

## 默认行为先查清

| 入口 | 默认行为 |
| --- | --- |
| `r680_camera.sh run` | 打开硬件相机：彩色+对齐深度 640×480@30，infra/IMU 关，emitter on，安装 TF 开 |
| `r680_perception.sh run` | 检测 + Agent 适配/人物 API + 可选观察窗口；不启动相机或跟随 |
| `r680_person_follow.sh run` | `transport=cpp`、`numeric_backend=python`、`local_planner=mpc`，运动关闭，等任务 |
| `r680_nav.sh` | RGB-D + MPC，定位/规划预览；`--enable-motion` 开放导航侧输出 |
| `r680_mapping.sh` | **默认允许建图链硬件输出**；`--dry-run` 才关闭；OpenVINS 分支强制 dry-run |
| `r680_vio_test.sh` | 独立验证，根脚本带相机准备和底盘启动选项；不能当成只读查询 |
| 常驻手柄 | 独立服务仍可人工驱动；关闭导航/跟随输出不等于关闭手柄 |

同域一个相机、一套跟随后端；导航与建图共用模式锁，不能并行。先在原启动终端 Ctrl+C 关闭要替换的会话，再启动新配置。

## 相机

每个 run 占一个终端，以下配置择一：

```bash
./r680_camera.sh run                                      # 跟随/RGB-D
./r680_camera.sh run --infra                              # cuVSLAM / 车身 IMU OpenVINS
./r680_camera.sh run --infra --imu                        # D455 IMU OpenVINS
./r680_camera.sh run --infra --emitter alternate          # 逐帧投射器交替
```

```bash
./r680_camera.sh status
./r680_camera.sh check --infra
./r680_camera.sh check --infra --imu
./r680_camera.sh check --infra --emitter alternate
```

`--emitter on|off|alternate` 默认 on；`--no-mount` 关闭本入口的安装 TF，需已有唯一 TF 发布者。
`--imu` 要求 `.runtime/realsense_openvins/install/local_setup.bash` 已构建。
check 退出码：0 就绪、3 本域无相机、4 缺流/交替设置、5 有驱动进程但本域没话题、6 多个彩色发布者。
这些检查主要看发布端与 TF/参数，不能代替逐帧新鲜度和图像质量验收。

## 感知和 Kalo

```bash
./r680_perception.sh check
./r680_perception.sh run                                  # 正式感知入口
./r680_perception.sh debug-camera                         # 另一终端；有检测器时复用
./r680_perception.sh window --check
```

run 可加 `--no-agent`（仅检测器，同时不启用观察窗口）、`--no-window`（不启窗口）、
`--debug`（一并启动 Kalo）、`--cache-only`（窗口缓存诊断）。debug-camera 支持 `--port 18081`、`--with-points`、`--check`。
run 的页面端口覆盖为 `--debug-port`；debug-camera 的页面端口覆盖为 `--port`。

| 参数/变量 | 默认 | 作用/来源 |
| --- | --- | --- |
| `--hz` / `--idle-hz` | 20 / 10 Hz | 活跃跟随请求/空闲的检测目标频率；非保证吞吐 |
| `--imgsz` | 416 | .pt 推理尺寸；可用 FP16 引擎固定为 320×416 |
| `--base-frame` | `r680_mapping_floor` | 相对位置输出参考系 |
| `R680_PERCEPTION_PYTHON` | `.venv-boxmot/bin/python` | 检测解释器 |
| `R680_PERCEPTION_DEVICE` | `0` | 检测设备 |
| `R680_YOLO_MODEL` | Agent 的 `models/perception/yoloe/yoloe-26s-seg.pt` | 权重 |
| `WLA_YOLOE_CLASSES` / `WLA_YOLOE_TEXT_ENCODER` | Agent 词表 / `mobileclip2_b.ts` | 修改后需匹配引擎 sidecar |
| `WLA_YOLOE_ENGINE` | 工作区 `models/perception/yoloe/yoloe-26s-seg-fp16.engine` | 置空选 .pt 路线 |
| `WLA_TRACKER_BACKEND` | `cpp` | 有界 native BoT-SORT；不可用记录错误并回 Python |
| `WLA_REID_ENGINE` | 工作区 `models/perception/reid/` | 置空选 OSNet FP32 graph |
| `WLA_PERCEPTION_NUM_THREADS` | `1` | BLAS/OMP 线程数 |
| `config/perception.botsort.yaml: track_buffer` | 17 帧 | 不是 17 秒；跟随 20 Hz 时约 0.85 s |

检测输出行只有 `detected,label,id,yaw,distance` 五字段，附加质量信息放消息顶层；完整 ID 包含 session/epoch，不要跨重启复用。
检测器/掩码仍有自己的 0.30 s 时效，跟随可用观测上限 0.40 s 不会延长检测发布期限。

## 人物跟随 / 人物靠近 / 物体靠近

相机、感知已在其他终端运行后：

```bash
./r680_person_follow.sh check
./r680_person_follow.sh run                              # 默认预览；保持运行
```

另开终端操作：

```bash
./r680_follow_trial.sh list
./r680_follow_trial.sh status
./r680_follow_trial.sh follow --mode 0 --continuous       # 交互选人，持续续租
./r680_follow_trial.sh follow --mode 1 --seconds 30        # 选人，靠近到达后结束
./r680_follow_trial.sh follow --mode 2 --target-id 123 --seconds 30
```

操作者明确要实车执行时，先退出预览后端，再启动：

```bash
./r680_person_follow.sh run enable_motion:=true
```

任务客户端相应加 `--allow-motion`，例如 `./r680_follow_trial.sh follow --mode 0 --continuous --allow-motion`。
这个客户端参数只允许连接已开放运动的后端，不会自行打开后端权限。Ctrl+C 停止当前任务并停止续租；后端退出在其自身终端 Ctrl+C。
不要与 `Wheel_Legged_Agent/tools/run_all_actions.py` 再起的跟随后端并行。

### 启动可调项

| launch 参数 | 默认 | 说明 |
| --- | --- | --- |
| `transport` | `cpp` | C++ ROS + Python worker；`python` 为兼容收发 |
| `numeric_backend` | `python` | `cpp`：C++ 地图/跟踪/曲线；MPC 仍为 Python |
| `local_planner` | `mpc` | `neupan` 仅替换 mode 0 部分阶段；需单独构建 |
| `grid_route` | `dijkstra` | 可选 `astar`，映射到节点参数 `grid_route_planner` |
| `person_filter` | `cylinder` | 可选 `cluster`，映射 `target_person_filter` |
| `depth_speckle` | `0` | 0 关闭；如 150，映射 `depth_speckle_max_px` |
| `terrain` | `false` | 独立跟随地形节点/话题 |
| `enable_reid` | `true` | 映射 `reid_enabled` |
| `enable_motion` | `false` | 后端运动门，任务仍须下发/续租 |
| `output_mode` | `mux` | C++ 收发仅支持 mux |
| `config` | 空 | 为空读取安装版 `config/sim_follow.yaml` |
| `python` | `WLA_FOLLOW_PYTHON` 或 `/usr/bin/python3` | worker/兼容节点解释器 |

例如：`./r680_person_follow.sh run numeric_backend:=cpp grid_route:=astar`。
launch 会用上述默认值覆盖 YAML 中对应字段；只改自定义 YAML 的 `enable_motion`、规划器等同名项不一定生效，需同时给 launch 参数。
未暴露为 launch 参数的时效项，在自定义完整 YAML 中修改后通过 `config:=/绝对路径/follow.yaml` 启动；节点没有通用热更新回调。

### 常用节点/算法参数

以下为配置/代码值，不是本轮实测最优值；单位不能混用。

| 参数 | 当前值 | 来源/含义 |
| --- | --- | --- |
| `max_sensor_age_s` | 0.40 s | YAML：观测可用年龄；启动校验上限 0.45 s |
| `map_max_age_s` | 0.45 s | YAML：控制补帧可用地图年龄；校验上限 0.50 s |
| `result_ttl_s` / `result_hold_s` | 0.20 / 0.35 s | YAML：结果接收预算 / 输出保留上限，仍受源龄约束 |
| `depth_stride` / `depth_min_m` / `depth_max_m` | 4 / 0.15 / 5.0 m | YAML：深度采样 |
| `wheel_topic` | `/wheel/odom_50hz` | YAML：局部轮速参考，超 0.20 s 不健康 |
| `surface_to_center_offset_m` | 0.28 m | YAML：人体表面转跟踪中心 |
| `local_assumed_free_radius_m` / `startup_clear_radius_m` | 0.50 / 0 m | YAML：持续近体自由空间先验 / 一次性启动先验；假设不是测量 |
| `target_prediction_s` | 2.0 s | YAML：短时丢失预测 |
| `target_history_window_s` / `target_history_sample_period_s` | 5.0 / 0.4 s | YAML：目标历史 |
| `reid_search_similarity` / `reid_alone_similarity` | 0.55 / 0.45 | YAML：多候选/唯一候选恢复阈值 |
| `reid_margin` / `reid_confirm_frames` | 0.08 / 3 | YAML：最佳与次佳差值 / 连续确认 |
| `desired_gap` / `target_stop_gap` | 1.20 / 1.16 m | `Tuning`：跟随中心距离 |
| `max_v` / `max_w` | 1.2 m/s / 0.9 rad/s | `Tuning`：采样 MPC 最高值；后端和 mux 还会限幅 |
| `radius` / `margin` | 0.19 / 0.10 m | `Tuning`：跟随圆形车体 / 代价余量 |
| `step` / `horizon` | 0.1 s / 24 步 | `Tuning`：模型预测 |
| `chassis_tau_v` / `chassis_tau_w` | 0.18 / 0.50 s | `Tuning`：底盘响应模型 |
| `search_speed` / `search_turn_rate` | 0.4 m/s / 0.6 rad/s | `Tuning`：开往消失点 / 扫视 |
| `backup_speed` / `backup_max_m` | 0.2 m/s / 0.6 m | `Tuning`：倒车脱困 |
| 地图分辨率/尺寸 | 0.05 m / 160×160（8×8 m） | `sim_runtime.py`，物体路线图约 0.1 m 格 |

`Tuning` 是源码 dataclass 常量，不是任意 `ros2 param set` 都能修改的 ROS 参数。
mode 1 到达参考为人体表面离机器人原点约 0.95 m；mode 2 请求外廓到目标表面 0.15 m，0.48 m 方形车体用外接圆扫掠，可能停得更远。
物体路线只以观测障碍判阻，远处未知格并非一概禁止；近场未知空间由 MPC 再检查，不能称整条路线均为已知自由空间。

## 闲逛探索（不用预存地图）

先启动同一相机、感知和跟随后端。下列服务会启动/暂停/恢复动作；是否真动仍受后端和 mux 门控：

```bash
ros2 launch wla_explore_wander explore.launch.py boundary_radius_m:=3.0 session_budget_s:=60.0
# 另一终端，同样先加载环境
ros2 service call /r680/explore/start std_srvs/srv/Trigger '{}'
ros2 service call /r680/explore/pause std_srvs/srv/Trigger '{}'
ros2 service call /r680/explore/resume std_srvs/srv/Trigger '{}'
ros2 service call /r680/explore/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /r680/explore/status --once
```

默认边界 4 m、会话预算 0（无限）、seed -1。底层用 mode 3 的 `target_id=dx,dy` 和 mode 4 的相对弧度；交互 follow trial 不支持这两个模式。

## 导航、建图与可选前端

先在独立终端按需求启动相机；以下命令彼此择一运行：

```bash
./r680_nav.sh --map map-2026-10-08-1 --rgbd --no-rviz
./r680_nav.sh --map map-2026-10-08-1 --cuvslam --no-rviz
./r680_nav.sh --map map-2026-10-08-1 --cuvslam --dark-infra --no-rviz
./r680_mapping.sh --cuvslam --dry-run --no-rviz
```

地图名必须在本机存在且 manifest 有效；不指定 `--map` 时 `storage.yaml: active_map=latest`。
cuVSLAM 相机需要 `--infra`，`--dark-infra` 还需相机 `--emitter alternate`。
OpenVINS D455 IMU 示例：相机先 `run --infra --imu`，再 `./r680_nav.sh --openvins --openvins-imu d455 --map map-2026-10-08-1 --no-rviz`。
`--mppi` 选择 `nav2.yaml`，当前 FollowPath 插件为 `nav2_mppi_controller::MPPIController`；默认 `--mpc` 选择 `nav2_mpc.yaml`。

| 导航选项 | 默认/意义 |
| --- | --- |
| `--enable-motion` | 默认不带；带上授权导航侧硬件输出 |
| `--cuvslam` / `--openvins` / `--rgbd` | 默认 rgbd；互为前端选项 |
| `--no-auto-vio-init` | cuVSLAM 默认 auto init 开；此项关闭；其他前端默认关 |
| `--auto-vio-init` | 仅 cuVSLAM/OpenVINS；是否执行运动还依赖分支授权 |
| `--no-fusion-backend` | cuVSLAM 导航选旧 degraded 后端，默认 fusion |
| `--no-degraded-odom` | 两种连续后端都不启动 |
| `--initial-pose "X Y Z R P Y"` | 六个有限数，米/弧度，只是 RTAB-Map 初值 |
| `--scan-obstacles` | 本地标记改用派生扫描，清障/碰撞链保留 |
| `--terrain` | 用地形分析替换 depth_to_points |
| `--test-scan-only` / `--test-recovery` | 独立测试 YAML；二者互斥且不能与 --mppi 混用 |
| `--agent-control` | Agent 接管目标/任务速度；关闭普通目标桥 |
| `--openvins-config DIR` | 完整成套配置目录 |
| `--openvins-motion-init` | 独立一次实车初始化动作；不能当成预览选项；与 auto-init/agent-control 互斥 |
| `--storage-config FILE` | 地图选择与运行目录配置 |

操作者需要实车导航时，在预览命令中显式加 `--enable-motion`；cuVSLAM 默认初始化包含自主激励。
**有常驻 `r680-manual-base` 时，cuVSLAM 建图不支持 `--auto-vio-init`，脚本会拒绝；用手柄初始化。**
建图开始前若只是预览必须带 `--dry-run`。OpenVINS 建图即使不带也被脚本强制 dry-run。
建图 `--slam-image color|infra` 默认 color，选 infra 的地图在定位时自动沿用；`--no-semantics` 关闭家具/出生点采集。
Ctrl+C 会在节点仍运行时尝试停稳和保存，检查终端的保存结果后再关闭相机。

### 主要导航配置位置

| 文件（均在 wla_r680_navigation/config） | 当前关键项 |
| --- | --- |
| `storage.yaml` | `run_root=~/nav_run`、`maps_root=~/ros2_ws/maps`、`active_map=latest` |
| `nav2_mpc.yaml` | 根导航脚本默认；FollowPath 插件 `wla_diff_mpc::DiffMpcController` |
| `odom_fusion.yaml` | `window_s=3`、`keyframe_s=0.1`、`keyframe_lag_s=0.4`、`visual_timeout_s=0.5`、`max_sigma_xy/yaw=0.25` |
| `degraded_odometry.yaml` | 旧后端预算 2 s / 0.30 m / 0.80 rad；不代表默认 fusion 预算 |
| `vio_initializer_cuvslam.yaml` | 静止 0.8 s、静态初始化等 10 s、前进目标 0.15 m、转角 0.52 rad、运动最长 6 s |
| `vo_watchdog.yaml` | 地图健康/恢复；`recovery_stable_s=2`、`startup_view_consistency=false` |
| `d455_points.yaml` | `stride=4`、`max_rate=10`、深度 0.15–5 m |
| `web_gateway.yaml` | 地图/相机/位姿 Web；launch 默认监听 0.0.0.0:8080 |

### 查看实际运行值

```bash
systemctl status r680-manual-base.service --no-pager
./r680_camera.sh status
ros2 param get /person_follow max_sensor_age_s
ros2 param get /person_follow local_planner
ros2 param get /person_follow numeric_backend
ros2 param get /r680_command_mux max_forward_mps
ros2 param get /r680_command_guard hardware_output_enabled
ros2 param get /controller_server FollowPath.plugin
ros2 topic echo /r680_base/control_state --once
ros2 topic echo /r680/person_follow/status --once
ros2 topic echo /r680_nav/perception/timing --once
ros2 topic echo /r680_nav/continuous_odom_status --once
ros2 topic echo /r680_nav/motion_ready --once
ros2 topic echo /r680_nav/localization_ready --once
ros2 lifecycle get /controller_server
```

查询某节点前须已启动对应链路；CLI 超时/找不到节点不是参数默认值的证据。
运动停止：优先当前任务 Ctrl+C/stop；底盘锁存急停为：

```bash
ros2 service call /r680_base/set_estop std_srvs/srv/SetBool '{data: true}'
```

该服务是软件停止请求；零命令/成功应答不等于测得了物理制动距离。

## 录制与显示回放

```bash
./r680_follow_bag.sh mpc --camera --depth                 # 当前应确实在用 mpc；Ctrl+C 正常关包
./r680_follow_bag.sh rla --camera --depth                 # neupan 对应标签 rla
./r680_follow_replay.sh                                  # 列表，不带文件不播放
./r680_follow_replay.sh 实际包目录名 --rate 0.5 --port 18082 --start 0
```

录制默认只订阅 ROS，`--camera` 在没有 kalo_jpeg 时会额外启动 JPEG 编码发布者；原始深度/彩色分别用 `--depth` / `--color`。
回放脚本固定域 199 且只播放显示话题，速度命令留在 bag 内。不要在真实车域 73 直接全话题回放。

## Agent 入口

```bash
./r680_agent.sh help
./r680_agent.sh preview
# 另一终端：
./r680_agent.sh query map
./r680_agent.sh query visible
./r680_agent.sh person visible
./r680_agent.sh person start --mode 0 --target-id '完整ID' --request-id demo-unique --seconds 10
./r680_agent.sh person stop --request-id demo-unique
```

person API 默认已随感知 run 启动；不要再起第二个 `person-api`。
`execute` 允许语义意图派发，还需 BT、正确地图/策略/租约和导航输出链；默认策略文件为 actual-map-preview-policy.json。
`navigation` 等价导航加 `--agent-control`；`bridge 本次selected_map.json` 的输出默认仍为预览话题。

## 构建与非运动检查

```bash
source /opt/ros/jazzy/setup.bash
source ~/r680_chassis_candidate_ws/install/setup.bash
cd ~/ros2_ws
export CMAKE_PREFIX_PATH="$HOME/.local/wla_mpc_deps:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$HOME/.local/wla_mpc_deps/lib:${LD_LIBRARY_PATH:-}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip wla_cuvslam_navigation wla_cuvslam_validation
source install/setup.bash
# cuVSLAM SDK 按 addons/cuvslam-sdk/README.md 准备后，单独构建前端：
bash src/wla_cuvslam_navigation/build_frontend.sh
```

修改跟随 Python 后，按当前 install 为副本的部署应重新 `colcon build --packages-select wla_person_follow`，再在原终端重启后端。
改 C++ 数值层要构建 `wla_follow_local`；改收发层构建 `wla_follow_transport`。
构建不更新已经运行的进程；先用 `ros2 pkg prefix`、`readlink -f` 和实际状态确定加载版本。

只看启动参数，不启动节点：

```bash
ros2 launch wla_person_follow follow.launch.py --show-args
ros2 launch wla_r680_navigation bringup.launch.py --show-args
ros2 launch wla_explore_wander explore.launch.py --show-args
```

隔离软件回归命令（不是本轮文档核验的实车验收）：

```bash
export ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RUN_SIM_ROS=1
python3 -m pytest -q src/wla_person_follow/test
ctest --test-dir build/wla_follow_local --output-on-failure
ctest --test-dir build/wla_follow_transport --output-on-failure
```

## 日志位置

| 内容 | 路径 |
| --- | --- |
| 后端输出 | 当前启动终端；后台运行时自行重定向的 `.runtime/follow-backend-*.log` |
| 交互客户端日志 | `.runtime/follow-trials/follow-*.log` |
| 跟随逐帧诊断 | `.runtime/follow-diagnostics/follow-status-*.jsonl*`，滚动覆盖 |
| 录制数据 | `.runtime/follow-bags/` + 同名 metadata JSON |
| 导航/建图完整运行 | `~/nav_run/`，含日志、工作数据库、配置快照 |
| 精简地图 | `~/ros2_ws/maps/map-*`，含 map.yaml/pgm、数据库、语义/索引/manifest |
