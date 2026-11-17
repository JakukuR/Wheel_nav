# 单路深度网页预览（2026-11-17）

精简模式关闭所有JPEG后，ROS仍输出深度，但原首页只读取旧RGB话题，用户看不到新双目链路。本次在首页加入depth-lite.html嵌入预览与独立入口，复用calibration.js的Foxglove/CDR解析和过期状态提示。仅订阅深度JPEG及网络状态；保留旧调试界面，不把左目光学坐标点云假装成车体坐标障碍物。

只启动一个kalo_jpeg：输入/camera/stereo/depth/preview、输出/r680/debug/stereo/depth/compressed、max_rate=2、quality=70。该编码器按网页订阅需求接收图像。其余五路JPEG、RGB采集/染色继续关闭。深度网络仍Light ESS FP16、5Hz、--no-color；网页帧率上限不改变深度节点速率。

入口：http://192.168.50.1:18081/ 或 http://192.168.50.1:18081/depth-lite.html。已打开首页需刷新加载新增入口。

验证HTTP200、首页引用、DOM必要ID/脚本引用、实际640×360JPEG解码及状态。客户端浏览器控制禁用，没有声称浏览器目视验收。第一次12s收18张JPEG，末尾有no_valid_stereo_depth状态，严格实时检查失败；保留日志，不把旧图冒充实时，继续复查。

进程与证据：/home/onana/ros2_ws/.runtime/stereo_minimal_preview_20261117/。本次没有测单路预览长时热稳定性。

复查12s收到15张可解码深度JPEG，发现后约1.67FPS（5Hz输入与2Hz编码节流的采样间隔）；末帧depth_fresh=true、有效深度83.94%、CPU88.875°C/GPU85.718°C，严格实时检查通过。期间未调整置信度或深度范围。
