# 三模式操作与日志

后端默认只预览，操作者选择 ID 后启动任务：

```bash
./r680_follow_trial.sh follow --mode 0 --continuous
./r680_follow_trial.sh follow --mode 1 --continuous
./r680_follow_trial.sh follow --mode 2 --continuous
```

0 人物跟随，1 人物靠近到约 0.95 m，2 物体靠近到外廓间距 0.15 m 或局部最近可达位置。
两种靠近到达后结束。后端已获运动授权时客户端加 `--allow-motion`。
可以传 `--target-id 短ID`，不传则交互选择。Ctrl+C 发送具名停止并停止续租。
终端输出保存在 `.runtime/follow-trials/`，逐帧状态在 `.runtime/follow-diagnostics/`。
源码和构建检查不证明实车停车精度；本轮未启动运动。
