# R680-MPC-STATIC-GLOBAL-001

默认 `r680_nav.sh --map ... --cuvslam --auto-vio-init --enable-motion` 使用
`config/nav2_mpc.yaml`。移除该配置全局 costmap 的动态点云标记/清除层，只保留
保存地图的 StaticLayer 与 InflationLayer。局部 costmap 的原始点云、清除射线、
观测缓存、MPC、碰撞监测、速度限制与定位恢复均保持原配置。

启动脚本根据实际选中的 YAML 判断全局动态障碍是否启用，以及是否需要
`dynamic_obstacle_memory` 节点；默认 MPC 不再启动该节点，运行记录中的
`global_dynamic_obstacles_enabled` 为 false。显式 MPPI、scan-only 与
recovery-test 各自沿用原有配置。

临时障碍仍由局部控制器避让，但不会更新全局路线的障碍信息；完全堵路时，
全局规划器可能继续规划原路线，需依靠局部停车/恢复或后续再启用全局动态层。

验证：源码与安装配置比对，默认 MPC 与原版逐项比较（仅全局障碍层移除），
四种参数文件的动态层/记忆节点开关校验，Bash 语法检查及安装更新。
未启动真实导航或发送运动指令。回退可对本次提交执行 git revert，再构建安装。
