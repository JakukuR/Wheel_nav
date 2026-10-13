# wla_detection_bridge

无模型、无运动代码的共享检测协议包。默认发布者为 nav 的 `r680_nav_yolo_detector`；Agent 只消费 nav 空间结果，不负责默认 YOLO 推理。

`/r680/perception/detections` 是 String JSON `wla-detections/v1`，保持 session/epoch、源时间、尺寸、frame、track_id/label/confidence/bbox_xyxy 校验及有界配对。跟随300 ms、家具500 ms TTL不变；原 `agent` 后端拼写兼容新 `shared` 后端。

原始检测供家具 A 白名单独立完成地图投影与语义存储。实时 B/人物使用 `/r680/perception/relative_objects` 的 `wla-relative-detections/v1`，按机器人系反馈类别、复合 ID、yaw 和 distance；Agent 适配器按 B 白名单转换为 `detected/what/id/yaw/distance`，不进行 map 转换。无有效深度或安装外参时保留类别/ID，yaw 与 distance 为 null。源时间和接收时间约束观测新鲜度；检测进程持续推理，没有 Agent 检测启停服务。

详见 [统一入口、字段、两份白名单和验收边界](../../docs/Agent共享检测与YOLO模型接入.md)。模型环境仅在 ros2_ws/.venv-perception，本协议包不依赖模型运行时。
