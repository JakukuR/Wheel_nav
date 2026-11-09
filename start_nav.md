# 导航启动速记

2026-11-09 Orin：`ssh -t orin@192.168.9.129`，工作区 `~/ros2_ws`。
先在独立终端启动相机，以下均为预览，不开放导航硬件输出：

```bash
cd ~/ros2_ws
./r680_camera.sh run --infra
# 另一终端：
cd ~/ros2_ws
./r680_nav.sh --map map-2026-10-08-1 --cuvslam --no-rviz
```

地图名应替换为本机已校验地图，或省略 `--map` 使用 storage.yaml 的 latest。
实际导航由操作者在上述导航命令中显式加 `--enable-motion`；cuVSLAM 默认可触发初始化激励。
建图预览使用 `./r680_mapping.sh --cuvslam --dry-run`，建图默认行为与导航不同。
完整前端/输出选项和参数见 [操作手册](docs/ros2_ws_operations.md)，框架见 [总览](docs/ros2_ws_architecture.md)。
