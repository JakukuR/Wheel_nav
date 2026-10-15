# R680 BICMap Web

2026-10-15：以本地托管 BICMap 0.0.9 替换原 Canvas2D 页面。网关程序名、包名和默认端口保持不变。

## 运行

原 `wla_r680_navigation` bringup 的 `start_web:=true` 继续启动 `web_gateway`，默认 `http://<Orin-IP>:8080`，使用该次启动选择的 map_yaml 和 semantic_path。不要同时在同一端口手动启动第二个网关。

仅看数据可单独运行（无需启动车体或 Nav2）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/orin/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/orin/ros2_ws/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml
ros2 run wla_r680_navigation web_gateway --ros-args -p port:=18080 -p allow_commands:=false
```

独立预览默认读取 `/r680/d455/map`；没有发布者时明确显示等待地图。可显式传 `-p map_yaml:=/absolute/selected/map.yaml` 查看已选择地图，但必须与机器人定位使用的地图一致。页面不会自动猜选历史地图。

## 显示与控制

- 二维栅格、2D/3D 点云切换、机器人方向、路径、出生点、家具、用户地图和速度调试。
- 点云来自 `/r680_nav/d455/points_viz` 和 `/r680_nav/d455/points_confirmed`，保留 xyz，默认每来源最多 12000 点、最高 4 Hz；无有效时间/坐标系或超过 2 秒的消息不更新；网页隐藏过期点云。
- 坐标采用以赤道为原点的局部 Mercator 平面，一 ROS 米对应一显示米。栅格四角考虑 origin_yaw，图像上边对应地图局部 +Y；3D z 保持米单位，不采用示例高度夸张系数。
- 位姿通过 TF 获取；失效后标灰并保留最后位置。位姿新鲜不等同于已地图重定位或底盘可运动。
- 相机使用 `sensor_msgs/Image` 的 rgb8/bgr8，`camera_topic` 默认 `/r680/d455/color/image_raw`。最多缓存 20 帧，按需编码至最大 960×720 JPEG；没有新鲜图像时返回 503，页面隐藏旧图。
- 检测框仅匹配 `/r680/perception/detections` 的相同 stamp_ns、frame_id、宽高；不匹配时只显示原图。列表和物体跟随状态是只读显示，不自动启动跟随。
- 点击“选择目标”或“重定位”，依次选择位置和朝向，再明确提交。目标继续发布 `/r680_nav/goal_request`；不会绕过原仲裁和租约。Agent 控制模式未运行该入口订阅者时，目标按钮禁用，应使用 Agent 原任务入口。
- HTTP 202 是 queued，不是 Nav2 已接受或到达；取消不是软件急停。请求超时显示未确认，不自动重试导航目标。
- `allow_commands:=false` 在服务器端拒绝所有 POST 控制；POST 校验 JSON、有限位姿、同源 Origin、ROS 接口可用性及队列上限。

## 依赖、更新与回滚

`web/vendor/bicmap-0.0.9.min.js` 是官方 npm 包中的自包含浏览器产物，随包本地托管，运行时无需 npm、CDN 或互联网。版本、tarball SHA512 和文件 SHA256 见 `web/vendor/provenance.json`；保留 BICMap 和第三方许可。此处使用真实 BICMap API，无官方示例地图或模拟运动。

本次只修改 Python/静态资源，没有重编译并发修改中的 OpenVINS/C++。原 CMake 已有 `install(DIRECTORY ... web ...)` 和网关脚本安装项，后续正常 colcon 构建会安装同样文件。本次以明确文件清单同步已安装 web/config，web 静态资源部署为实际文件，网关脚本仍为 source 的符号链接；已核验安装哈希。HTTP 处理器也兼容后续 colcon 再次生成的逐文件符号链接。

回滚使用本次本地 Git 提交的父版本恢复相应源码和 web/config，再按清单同步 install；不要整库 reset。部署前副本保留在 `~/.cache/r680-bicmap-20261015/backup/`。只读预览为临时用户 service `r680-bicmap-preview.service`，可用 `systemctl --user stop r680-bicmap-preview.service` 停止，不启用开机自启。

## 验证

```bash
# 坐标转换和旋转原点（Node 22+，无 npm install）
node --test src/wla_r680_navigation/test/coordinates.test.js
# 隔离 ROS 域，真实网关 HTTP 与消息回调；不发布运动指令
ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 src/wla_r680_navigation/test/test_bicmap_gateway.py
```

3 项坐标测试、8 项 Python/HTTP 测试通过（含 colcon 逐文件符号链接回归）；浏览器使用明确标注的 UI fixture 验证二维/三维显示，无控制提交。原 8080 网关启动期间已核验新首页、JS、真实 map.png 和 camera.jpg 返回 200；没有重启或停止外部启动的导航进程。随后其 8080 和相机数据源退出，独立只读预览继续运行在 18080，使用该次 8080 实际选择的 map-2026-10-08-1 地图，节点名为 r680_bicmap_preview，避免与正常网关重名。

真实地图和相机接口已有数据证据；当时没有有效机器人位姿或通过时效/TF 检查的三维点云，没有检测发布者，因此真实点云显示和相机识别框尚未现场验收。仍需在数据源正常时核对刷新、叠框及目标坐标，并另行授权实际导航测试。
