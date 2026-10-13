# 物体接近/跟随核验（2026-10-13）

范围：wla_object_follow 新包、ObjectCommand、共享相对执行壳的扩展点、C++ 表面距离/速度/扫掠内核、独立 OBJECT mux 通道、启动入口及文档。

用户最后确定：最低到 0.05 m/s 可以，靠近逐步收敛到 0；没有要求把 0.05 作为全程上限，也不采用 0.10 最低输出钳位。

目标间距为机器人前沿到可见目标表面 0.15 m，软件到达容差 ±0.02 m；实际轮速接近零且三个不同源帧确认才完成 approach_object。follow_object 保持租约并在物体远离时继续接近。默认关闭运动。

验证结果：

- 五个相关包完成构建（接口包先构建，其余随后构建）。
- 完整 pytest：126 passed，13.77 s，无跳过。包括原人物/Kalman/轮速参考系/局部绕行、manual policy、新物体控制、RGB-D ROS 对接及 OBJECT mux ROS 测试。
- 原 test_relative_mux_ros.py 独立运行通过：NAV/PERSON 独占、显式 AUTO、旧会话拒绝、心跳过期、手动接管、热插拔、急停及恢复、Nav2 取消与限幅。
- ./r680_object_follow.sh check、bash -n、launch --show-args 和 git diff --check 通过。
- 源码/安装 Python、YAML、launch、接口、mux 文件哈希一致；C++ build/install .so 哈希一致。
- 全部 ROS 测试在域 198。OBJECT mux 输出重映射到 /object_test/cmd；原 mux 测试在域 198 有唯一测试 mux 输出，无底盘驱动。测试进程干净退出。
- 生产驱动 PID 2746 与 mux PID 2748 启动时间仍为 2026-10-13 09:56:46，未重启。

边界：没有实车运动或停靠精度验收。0.24 m 前沿/半宽是现有 collision_monitor 外包络；0.7 s 延迟参考现有配置记录；0.3 m/s² 制动是待实测假设。相机近距盲区、透明反光物体、bbox 内其他物体和侧后盲区仍需现场验证。近距受阻停车，不保证狭窄空间绕行。最新深度参与检测帧间的障碍检查；未移除整块 bbox 障碍。

生产 mux 尚未加载 OBJECT 扩展，需要停车维护时重启加载；本提交不自动启用运动。Agent 侧 LLM 动作路由未修改，Nav 新提供 /wla_agent/object_command 服务。

仅提交本功能文件。其他 OpenVINS、导航启动和 Agent 目标相关改动由其他工作维护，不纳入本提交；不推送远端。
