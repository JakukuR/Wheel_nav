# Jetson 风扇常态满速

2026-11-17 按用户要求，将 Orin NX 的风扇固定为 PWM 255（100% 占空比），开机生效。实际机械转速由风扇决定，不能用 PWM 数值冒充 RPM。

使用 nvfancontrol.service 的 systemd drop-in 替换自动温控进程，保留原厂服务启动顺序和开机启用状态。oneshot 成功后 active (exited) 是预期状态。内核 tj-thermal 的 step_wise 策略也会覆盖 PWM，因此启动脚本确认该区域仅绑定 pwm-fan 后，切换为 user_space；其 mode 仍 enabled，critical trip 保留。CPU/GPU 等其他区域保持 enabled/step_wise，过热降频与保护未关闭。原策略保存到 /run/wla-fan-full-speed.policy，服务停止时自动恢复。原始 /etc/nvfancontrol.conf 保留不变；没有修改 GPU 频率或电源模式，没有启动相机、深度算法或运动服务。此设置覆盖此前 LAS2 测试结束时的自动风扇状态；历史测试条件仍按当时记录。

安装命令：

```sh
cd /home/onana/ros2_ws
sudo install -m 0755 tools/fan_full_speed/set-full-speed.sh /usr/local/sbin/wla-fan-full-speed
sudo mkdir -p /etc/systemd/system/nvfancontrol.service.d
sudo install -m 0644 tools/fan_full_speed/99-full-speed.conf /etc/systemd/system/nvfancontrol.service.d/99-full-speed.conf
sudo systemctl daemon-reload
sudo systemctl enable nvfancontrol
sudo systemctl restart nvfancontrol
```

验证见 verification_20261117.json。已验证服务重启生效及连续30秒 PWM 保持255；未重启整板验证。程序只等待目标 pwm-fan 设备，最长30秒，写入失败会使服务明确失败。

恢复自动调速：

```sh
sudo systemctl stop nvfancontrol
sudo rm /etc/systemd/system/nvfancontrol.service.d/99-full-speed.conf
sudo systemctl daemon-reload
sudo systemctl restart nvfancontrol
```
