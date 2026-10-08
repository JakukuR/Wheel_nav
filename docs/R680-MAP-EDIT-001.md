# 接受人工编辑的二维地图

导航启动仍校验地图清单，避免意外替换栅格、坐标信息或定位数据库。
使用 GIMP 编辑并保存 `map.pgm` 后，先关闭编辑器，再执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run wla_r680_navigation accept_navigation_map_edit \
  --map map-2026-10-08-1 --reason '人工清理确认不存在的障碍像素'
```

工具只接受尺寸不变的 8 位 P5 PGM；其余地图文件必须通过原清单校验，
RTAB-Map 数据库必须通过完整性检查，分辨率和原点必须与地图信息一致。
通过后原子更新清单中的 PGM 校验值，记录修改理由、前后哈希，并在地图目录
`map-edit-*/` 下保存旧清单和本次接受的 PGM。原始地图仍保留在源运行目录
`map_archive/map.pgm`，定位数据库不修改。

这个操作接受的是人工确认过的二维导航地图修改，不会清理 RTAB-Map 数据库中的
三维点云，也不能判断被删除的障碍是否真实存在。不要改变图像尺寸、旋转地图、
修改原点或分辨率。没有人工修改意图时应检查文件来源，不要直接刷新校验值。
