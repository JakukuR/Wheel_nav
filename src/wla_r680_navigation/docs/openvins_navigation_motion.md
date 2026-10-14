# OpenVINS 初始化后导航

入口：

```bash
cd ~/ros2_ws
./r680_nav.sh --openvins --openvins-imu d455 \
  --map map-2026-10-08-1 --test-scan-only \
  --openvins-motion-init --enable-motion
```

`--openvins-motion-init` 授权一次有时限的初始化动作，`--enable-motion` 授权随后正常导航；不加后者仍是预览。两者不需要手柄在线。初始化完成、Nav2 和当前定位接口就绪后，导航会话先取消此前提交的导航点并等待取消完成，然后在零速度下获得 AUTO/NAV，最后开始发布任务许可。终端出现 `navigation_enabled` 后发送新的导航点。

本次取消的条件：OpenVINS 前端的硬编码硬件输出禁用、联调标定元数据导致的固定禁用，以及普通定位导航对 `/openvins/motion_ready` 的重复门控。该旧话题组合了接地检测、回环匹配和标定状态；它保留为原质量/建图链路接口，不再是普通定位导航的驱动许可。标定元数据没有改为“已验收”。

仍生效的是当前 VIO 健康与里程计输入、Nav2 碰撞检查、命令时效、底盘速度限制、急停及人工接管。`initialized=true` 是初始化历史；持续出现 `no_recent_accepted_visual_update` 仍会导致 VIO 健康为假并停车，这种真实跟踪问题需要修前端，不以永久授权掩盖。

导航会话不发布速度，只持有原 command_guard 输出开关、任务许可心跳和一次 AUTO 授权。交接前清空旧目标，输出开关或任务许可打开时清除缓存速度。退出、急停或人工接管会撤销许可、关闭输出，并只释放自己获得的 NAV 权限；不会抢回手柄，也不会修改另一个 PERSON/OBJECT 会话的权限。进程被强杀时，任务心跳超过原 0.5 秒时限后输出归零；强杀不保证收到 MANUAL 状态切换，不能把超时停车等同于控制权释放。

本轮使用隔离 ROS 域 198、真实 command_guard 与 command_mux、模拟里程计和 action 服务验证授权交接、旧目标清除、健康失效与恢复、手柄接管、急停和进程退出。没有进行本轮实车导航，不代表定位精度或地图匹配已验收。
