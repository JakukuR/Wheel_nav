# 人物跟随实车测试与双窗口

车端 `orin@192.168.9.118`，工作区 `/home/orin/ros2_ws`，ROS 域 73。
2026-10-20 按最新要求：默认跟随走原候选接口，mux 按新鲜速度自动仲裁，
不需要 PERSON 来源选择、AUTO 切换或跟随状态握手。

## 当前链路

```text
人物跟随 /r680/person_follow/command_candidate
物品跟随 /r680/object_follow/command_candidate
导航     /r680_nav/chassis_cmd_vel
手柄     /r680_manual/cmd_vel
                  ↓ r680_command_mux
         /r680_base/cmd_vel → 常驻底盘
```

固定优先级：有效非零手柄 > 人物 > 物品 > 导航。零速度释放该路优先级，
若其他路有新鲜非零速度则由下一路接管；没有有效非零输入就输出零。
每路速度独立验证，不相加。自动候选有效期 150 ms，人物/物品保留采集时间戳检查。
急停、非法数值检查、手柄就绪检查和底盘限幅保留。松杆后只接收交接后的新命令，
不重放手柄接管期间缓存的速度。手柄接管不再撤销跟随任务，松杆后新鲜候选可恢复输出。

mux 不再检查跟随状态/身份租约来决定是否执行速度；选人、任务租约、感知时效和
规划可行性由跟随节点自己处理，失败时发零。默认预览也只发零到实际候选接口。
原 source/AUTO 服务仅兼容旧调用；不再控制速度优先级。不要再把它们列为测试前置步骤。
原 `/r680/person_follow/cmd_vel_candidate` Twist 是调试预览，mux 实际消费 JSON
`command_candidate`。`control_state.velocity_passthrough=true` 标识新版已加载。

## 两个轻量窗口

- [检测：人物/物品框、ID、距离](http://192.168.9.118:18081/detections)
- [规划：深度障碍体素、局部路径与栅格](http://192.168.9.118:18081/planning)

共用一个只读 HTTP 服务，目标 5 Hz；不启动 RViz 或 Nav2。人物路径需要有效人物任务。
当前体素来自深度帧，人物栅格来自跟随器局部地图。RGB 投影仅做近同步匹配，不做遮挡消除。
本轮两窗口和预览节点已启动。重新启动时：

```bash
cd /home/orin/ros2_ws
./r680_perception.sh debug-camera
# 另一终端；禁止运动的默认预览
./r680_person_follow.sh run
```

## 先做人物预览

人站到相机正前方约 1–2 米，检测窗口出现 person 和有效距离后执行：

```bash
cd /home/orin/ros2_ws
./r680_follow_trial.sh list
./r680_follow_trial.sh follow --target-id '完整person id' --seconds 30
```

完整 ID 是 `session:epoch:track_id`，不是页面上的单个数字，不能用 chair ID。
客户端每 0.4 秒续租，租期 1.5 秒；到时或 Ctrl+C 发送停止。默认拒绝已启用运动的后端，
不自动选人、不发布底盘速度。左右走动、直线走动，检查 ID、距离和路径。
当前未实现 ReID，检测器重置或重新分配 ID 后需要重新选人。

## 实际跟随

完成预览、确认操作者可立即停止后，第一轮在空旷地面做短距离直行/停止，再测试椅子绕行。
核心仍保留仿真最高线速 0.65 m/s、角速 0.9 rad/s；尚未完成实车制动验收。

终端 A：结束本轮后台预览，等待其退出，再启用跟随输出。默认已经是 mux。

```bash
cd /home/orin/ros2_ws
kill -INT "$(cat .runtime/follow-preview-mux.pid)"
# 等待 .runtime/follow-preview-mux.log 中 process has finished cleanly
./r680_person_follow.sh run enable_motion:=true
```

终端 B：重新读取当前人物 ID，开始有时限的任务。**这条 follow 命令被接受后即可开动，无需再切 AUTO：**

```bash
cd /home/orin/ros2_ws
./r680_follow_trial.sh list
./r680_follow_trial.sh follow --target-id '完整person id' --seconds 15 --allow-motion
```

停止任务：终端 B 按 Ctrl+C。手柄非零输入优先，但松杆后仍可能恢复当前跟随任务，
所以结束测试应停止任务或使用急停，而不是仅短暂拨动手柄。

软件急停（已加载 ROS 环境的终端，或先执行下方环境设置）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/orin/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=73 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/orin/ros2_ws/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml
ros2 service call /r680_base/set_estop std_srvs/srv/SetBool '{data: true}'
```

本轮未发送真实人物任务或非零运动测试。ToF 尚未接入，未知区域可能使规划器保持停止。

## 验证和日志

- 6 项速度优先级单测：无门控、优先级、零速释放、过期/非法数值、手柄交接、急停与限幅。
- 真正 mux 进程的隔离 ROS 测试：无需来源/AUTO/状态消息，人物/物品/NAV/手柄输出与停止均检查。
- 跟随器与新 mux 联调、真实深度适配和数值单测共 17 项通过。
- 复用的原策略/手柄/物品基础代码回归 53 项通过。
- 测试客户端验证预览续租、结束停止、默认拒绝运动后端、启动拒绝撤销；隔离域内没有真实底盘。

日志：`.runtime/velocity-tests-final.log`、`.runtime/velocity-mux-ros.log`、
`.runtime/mux-shared-regressions.log`、`.runtime/test_trial_client.log`。
实际预览日志 `.runtime/follow-preview-mux.log`，窗口日志 `.runtime/follow-debug-camera.log`。

当前相机曾面对低纹理墙面，GMC 连续拒绝；调整后恢复匹配，观察到右侧 chair 距离约 2.51 m。
没有将椅子作为人物目标。可视化已显示真实 RGB 和障碍数据。
