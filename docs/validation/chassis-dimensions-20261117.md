# 车体宽 22 cm × 长 29 cm（2026-11-17）

用户确认网页与碰撞尺寸一起修改。轴向约定保持 x 前后、y 左右，沿用现有 base 原点居中的外形，不更改 CAD 传感器安装外参、关节或 URDF 运动学。

- 网页俯视图从固定像素图标改为宽 0.22 m、长 0.29 m 的按比例矩形，转向箭头在前端；机身状态的示意外壳同步宽长，原高度保留。
- Nav2 四份配置（nav2、nav2_mpc、nav2_test、nav2_recovery_test）的局部/全局 costmap 共 8 处 footprint 改为 x=±0.145、y=±0.11 m；原 inflation 和 padding 不改。
- 原跟随 Python/C++ 共用尺寸约定：既有圆形碰撞检查使用矩形外接圆半径 hypot(0.29,0.22)/2 ≈ 0.1820 m。原 margin=0.10 m 保留，依赖半径的人物中心距离随新半径计算，表面净距不改。
- 物体接近模式原有 0.48 m 方形也同步为新矩形；表面净距使用矩形，轨迹扫掠仍用外接圆。诊断文字同步更新。
- 未启用的独立 NeuPAN 实验后端使用专属 car_50x40.bin 训练模型，本次没有伪装成 22×29 cm 模型；其独立遗留实验配置不属于原跟随 Python/C++ 后端，需要模型匹配后另行迁移。

验证：

- colcon 构建 wla_follow_local、wla_person_follow 成功（33.8 s；LTO 串行警告，无编译错误）。
- test_chassis_dimensions.py：5 项通过，包含矩形旋转后的角点外接圆覆盖、不同朝向的前/侧表面距离、8 处 Nav2 footprint 一致性与余量。
- test_native_planning.py 中 approach_goal_matches_reference、random_commands_match_reference：34 项通过，34 项未选择。
- CTest test_follow_core、test_planning：2 项通过。
- node --check kalo.js、原网页机身状态测试、相机尺寸测试通过。
- 浏览器刷新后实看新车体矩形与“车体宽 22 × 长 29 cm”文字，深度预览仍 10 Hz。

运行边界：当前仅深度与原跟随点云前端预览，未运行 Nav2、完整跟随或 NeuPAN。未启动运动。配置安装目录链接到源码，Python/C++ 构建产物已更新，下一次启动相应规划节点时加载新碰撞尺寸。本次并非实机行驶避障验收。
