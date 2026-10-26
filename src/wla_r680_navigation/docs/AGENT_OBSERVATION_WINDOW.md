# 同帧观察与单次冻结窗口

2026-10-26 实现；本页参数是可调起点，不能作为产品性能标准。

## 所有权和接线

RealSense 保持唯一采集者；`nav_yolo_detector` 保持唯一 YOLO/BoxMOT 工作者。
检测器先走既有 raw、relative、mask/ReID 发布，再把同一输入 Image 的引用交给有界后台出口。
出口按 **整数纳秒 stamp + optical frame** 连接 color metadata，无最近帧回退。
原始 RGB 图像在后台转换为不可变 bytes，SHA256 与帧身份绑定。

`agent_observation_window` 独立进程通过本机 Unix socket 接收二进制图像和小型 JSON 头；
不通过 DDS 重复订阅 RGB，不在该传输中 base64 图像，不加载第二个 YOLO。
接收端保存原始媒体、检测与追溯信息，在唯一截止选帧后才编码。
输出 `/wla_agent/perception/window` 是通过本地 Schema 校验的 PerceptionFrame 外壳；
该节点没有速度、导航目标或行为命令发布者。Agent 上层消费结果时应交给
`Engine.ingest_perception_frame`，不能把它重新送入 `ingest_multimodal_system1` 推理。
本次不改 GUI/行为服务的人工输入流程。

Nav 几何继续实时工作。Agent 复用相同 batch 的距离/方位和深度时间；没有几何则显式
`unavailable`。新窗口路径不调用额外定位，不控制共享相机/YOLO/VIO 的启停。

## 启动与配置

- `./r680_perception.sh run`：下次启动同时加载观察出口和窗口消费者，沿用现有 Agent 轻量接口。
- `./r680_perception.sh run --no-window`：仅关闭新增窗口出口/消费者。
- `./r680_perception.sh run --no-agent`：Nav 单独运行。
- `./r680_perception.sh run --cache-only`：明确的缓存诊断模式，不调用模型。
- `./r680_perception.sh window --check`：只检查配置和编码依赖，不启动 ROS/模型。
- `--window-config PATH`：给 run 指定 JSON 配置；window 子命令用 `--config PATH`。

这不是相机/底盘自动启动入口；相机仍由既有 bringup 提供。`debug-camera` 保持调试用途，
要使用完整链路应从正式 run 入口启动，不能重复启动第二个检测器。

安装配置为 `share/wla_r680_navigation/config/agent_observation_window.json`。
当前起点：pre=4s、post=2s、调用硬截止=触发后3s、封窗后准备预算至多1s、缓存8s/256条/256MiB、
选8帧、confidence=.50、stable=.30s、融合=.40s、静默8s+确认1s、结果期限20s、JPEG质量80、编码线程1。
包在接纳时必须新鲜；Nav 原有300ms运动数据门限没有放宽。历史缓存不重用该运动 TTL。

`t_cut=min(t0+post_roll_s,t0+call_deadline_s)` 固定不刷新。
实际模型提交还须早于 `min(t_cut+dispatch_budget_s,t0+call_deadline_s)`。
如果 deadline 截短后窗且不剩编码/网络发起时间，本次明确失败，不越过调用硬截止。
计时器延迟同样不补写时间、不引入截止之后的事实。

## 数据与限制

- metadata 包含 frame_number、原 clock_domain 与原始 timestamp 字段；原值保留。
- 相机采集会话、检测 session/epoch、相机序号共同构成 source_frame_id；optical frame 是坐标系。
- ROS stamp 经同主机 ROS/monotonic 采样对转换到单调时间轴，使用 Linux boot_id 标识。
  时钟跳变、帧号回退或明显流间断切换源代次，并退役窗口；跨主机音频不能直接使用另一台机器的 monotonic。
- 图像与检测对不上、时间倒退、重复源帧、hash 不一致均拒绝；空检测但真实图像有效的包仍可用于离开判定。
- 出现/离开/显著框变化有持续时间与重复抑制。静态高置信目标首次稳定出现后不会每帧触发。
  动作事件支持已有结构化来源；当前 YOLO 出口未增加姿态/动作模型，action_events=[] 不伪造动作。
- 一次只收集一个 activation。相同事件刷新活动时间，不改变 t0/t_cut；短融合窗内第二模态可合并。
  其他不相关事件记录 busy，当前模型忙时新窗口明确失败，不无界排队。
- 输入按源时间严格递增；封窗再稳定排序，保留端点、触发帧、事件峰值（置信度+归一化位移），再最远时间填充。
  强制锚点超出 selected_frames 则失败，不静默删除。全窗口观察事实和选中媒体分别保留。
- 图像和嵌套结构深层不可变；截止前未接纳的 ASR/检测/定位不能回填。
  post_roll_complete 同时检查截止和采样覆盖；pre_roll_complete 也显式记录。
- 活动状态 IDLE/TRIGGERED/ACTIVE_PERCEPTION/QUIET_PENDING 与 COLLECTING/FROZEN/SUBMITTED/DONE 请求状态分开。
  关闭退役旧代次；共享生产者继续工作。模型线程不能强制中断网络请求，但迟到结果不再发布。
- 未配置模型时继续接收/cache，并报告 provider_unavailable；触发请求失败，不落到 Mock 模型。
- OpenAI-compatible 路径一次 HTTP、真实 image_url JPEG 媒体、显式 ASR 文本，不修补模型输出、不自动重试。
  Jev 路径将已选帧组成短 AVI，video_frames 与实际选中帧数相同；音频文本明确标记工程 ASR。
  **当前音频入口是带来源的 ASR/DOA 事实，不提供原音频；raw_audio_loaded 永远为 false。**
  没有增加或启动 ASR/VAD/DOA 模型；真实采集器需按下述接口接入。
- 真正的 raw audio+video 联合媒体能力取决于模型服务；不把 ASR 文本宣称为原音频已理解。
- 幂等键与本地提交锁存保证本路径至多一次尝试。网络异常结果未知时不重试，不声称跨崩溃 exactly-once。

## 音频接入

话题 `/wla_agent/perception/audio_window_ingress`，`std_msgs/String` JSON。
必填：`speech_session_id, segment_id, revision, status(partial/final), text, confidence,
engine, engine_version, captured_start_at_s, captured_at_s, clock_domain,
timeline_start_s, timeline_at_s, timeline_clock_id`。
source 时间保留原域；timeline 为车端同一 boot_id 的 monotonic 秒。新会话 final 事件可携带 `trigger:true`。
异步回包携带 `activation_id,generation`；即使漏带 token，已绑定退休 speech_session 的结果也拒绝。
可选 DOA 必须带 angle_deg、uncertainty_deg、timeline_at_s，且位于片段区间内。
超窗片段不裁造 ASR；不完整/未就绪文本按配置失败或明确缺失。

上层也可用 `WakePerceptionIngress.ingest_observation(package,runtime)` 与
`ingest_audio_window(packet,runtime)` 接入同一窗口，不能同时再走旧 fact-only 触发入口。

## 资源预算与失败隔离

帧池限制时间、条数及序列化元数据+图像字节；元数据 pending 最多64条、图像出口队列2条/16MB。
断开接收端时跳过图像 hash/复制；慢接收时只淘汰出口旧待处理帧，不等待 Nav。
Socket 限同 UID，权限0600、单消费者锁；消息头<=256KiB、图像<=8MB 是协议防护上限。

一个在途快照最多另外固定一个缓存容量，raw+序列化事实预算上界为2×cache_bytes；
实际共享 bytes 引用通常更少。音频队列另有128条/2MiB限制。Python/ROS对象开销、codec、
模型请求编码和网络临时缓冲不等于这些字节数，不能把它们当进程 RSS 硬上限。
模型请求上限32MiB、响应上限2MiB。没有跨进程零拷贝承诺。

2026-10-26 Orin 合成基准（640×480 RGB8，单个低置信对象，160帧）：metadata 提取中位约0.035ms，
后台图像复制+hash约1.28ms，接收验证约0.91ms，缓存入场约0.018ms。
这四段合计平均 CPU 约2.29ms/帧，按15.5FPS折算约一个核的3.55%；不含实际IPC传输、
相机、截止编码/模型，也不是整机 A/B。没有 GPU 计算，不据此声称整机CPU下降。

## 验证与维护

隔离测试使用 domain198/LOCALHOST、假检测器/假 System 1 和合成图像；不启动硬件或运动。
覆盖精确配对/错误帧拒绝、hash、缓存淘汰、低置信不触发、固定截止、迟到证据、ASR修订、
融合、锚点预算、状态关闭、一次提交、结果Schema和命令拒绝、私有IPC、慢模型不阻塞Nav。
首次codec冷加载曾触发源流间隔检测；已移到消费者启动时预热，保留断流门限并复测。
旧 Jev 测试夹具补上现有 social_request 问题，不改变断言或模型决策规则。

新入口安装后须重新启动对应感知进程才加载；本次没有重启底盘、启动相机或进行真人/实机模型验收。


## 2026-10-26：授权重启后的真实相机验证

用户明确要求重启测试后，于车端 domain73 停止原 debug-camera 的旧感知进程树，启动正式 `r680_perception.sh run --base-frame r680_mapping_floor`，再启动 `debug-camera --port 18081` 复用其唯一检测器。相机、YOLO、窗口状态话题各一个发布者，调试界面18081继续可用。没有启动底盘/导航运动，也没有发送运动命令。新链路已实际加载，替代上一节“尚未重启”的部署状态；历史测试记录保留。

真实640×480 RGB/metadata已进入窗口。首个观察到的activation仅冻结一次，选8帧，post_roll_complete=true；随后明确报 system1_provider_unavailable。车端未配置System1提供方，因此没有真实模型提交或PerceptionFrame结果，不能记作模型端到端通过。后续活动正常退役，20秒稳态记录中累计冻结数保持3不再增长，IDLE期间缓存持续更新。采样末累计接纳1290包、接纳拒绝0，缓存114–117条、105681902–108464234字节（约101–103MiB），符合8秒/256条/256MiB上限。

20秒稳态检测约14.65Hz，最大相邻接收间隔约90ms，末次camera/result均fresh。独立20秒进程资源采样：窗口进程约15.32%单CPU核、RSS335008KiB（约327MiB），包含ROS、IPC、校验和对象开销；这不是早先四段合成基准的3.55%，也不是整机A/B或峰值保证。未新增GPU模型实例。相机重启时出现短暂libusb Resource temporarily unavailable警告，随后真实流和检测持续正常。

证据：`ros2_ws/.staging/observation-live-20261026/` 中的 restart-receipt.json、restart-observation.json、steady-observation.json、live-resources.json、live-validation.json。保持新感知链路运行；真实System1、原音频与实车运动仍未验证。
