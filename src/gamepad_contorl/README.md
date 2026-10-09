# Gamepad Teleop - 手柄遥控控制节点

基于 ROS2 Humble 的手柄遥控节点，专为 **高重心、大质量差速驱动底盘** 设计，内置多重安全控制逻辑。

---

## 目录

- [功能特性](#功能特性)
- [硬件要求](#硬件要求)
- [操控方式](#操控方式)
- [安装与编译](#安装与编译)
- [运行方式](#运行方式)
- [参数配置](#参数配置)
- [移植到其他包](#移植到其他包)
- [更换手柄设备](#更换手柄设备)
- [控制逻辑详解](#控制逻辑详解)
- [话题接口](#话题接口)
- [故障排除](#故障排除)

---

## 功能特性

| 特性 | 说明 |
|------|------|
| 死人开关 (Deadman Switch) | 必须按住 LT 才能运动，松开立即平滑停车 |
| 加速度限制 | 平滑加减速，防止高重心车辆侧翻/倾覆 |
| 指数响应曲线 | 低速精细控制 + 全速可达 |
| 低通滤波 | 消除摇杆抖动和突变输入 |
| 转弯自动减速 | 车速越快，角速度自动限制越严 |
| 紧急停止 | A 键一键切换急停锁定 |
| 三档速度切换 | 低速 30% / 中速 60% / 高速 100% |
| 原地旋转 | LB/RB 一键原地左右旋转 |
| RT 加速 | RT 扳机线性叠加额外速度 |
| 自动重连 | 手柄断开后自动尝试重连 |

---

## 硬件要求

- **手柄**：Flydigi Dune Fox（飞智沙漠之狐）或其他标准 Linux joystick 兼容手柄
- **连接方式**：USB 有线
- **ROS2 版本**：Humble

---

## 操控方式

```
┌─────────────────────────────────────────────┐
│                手柄按键映射                    │
├─────────────────────────────────────────────┤
│  LT (按住)     = 使能/死人开关（必须按住才能动）│
│  左摇杆 Y轴    = 前进 / 后退                  │
│  右摇杆 X轴    = 左转 / 右转                  │
│  RT            = 加速（线性叠加 0~30%）        │
│                                              │
│  A 按钮        = 紧急停止（切换开关）           │
│  X 按钮        = 低速档 (30%)                 │
│  Y 按钮        = 中速档 (60%)                 │
│  B 按钮        = 高速档 (100%)                │
│  LB            = 原地左转                     │
│  RB            = 原地右转                     │
└─────────────────────────────────────────────┘
```

---

## 安装与编译

### 方式一：独立编译（当前方式）

```bash
cd ~/gamepad_contorl
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 方式二：放入已有工作空间

```bash
# 将包复制/链接到工作空间的 src 目录
cp -r ~/gamepad_contorl /path/to/your_ws/src/gamepad_control
# 或使用软链接
ln -s ~/gamepad_contorl /path/to/your_ws/src/gamepad_control

cd /path/to/your_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gamepad_control
source install/setup.bash
```

---

## 运行方式

### 直接运行节点

```bash
source install/setup.bash
ros2 run gamepad_control gamepad_teleop
```

### 使用 launch 文件（加载 yaml 配置）

```bash
source install/setup.bash
ros2 launch gamepad_control gamepad_teleop.launch.py
```

### 运行时覆盖参数

```bash
ros2 run gamepad_control gamepad_teleop --ros-args \
  -p device:=/dev/input/js1 \
  -p max_linear_speed:=0.5 \
  -p cmd_vel_topic:=/robot/cmd_vel
```

---

## 参数配置

所有参数定义在 `config/gamepad_params.yaml` 中，也可通过命令行 `-p` 覆盖。

### 速度参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_linear_speed` | 1.0 | 最大前进线速度 (m/s) |
| `max_angular_speed` | 1.5 | 最大转弯角速度 (rad/s) |
| `min_linear_speed` | 0.05 | 最小有效线速度，低于此值归零 (m/s) |

### 加速度限制

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_linear_accel` | 0.5 | 线加速度上限 (m/s²) |
| `max_linear_decel` | 1.0 | 线减速度上限（刹车更快）(m/s²) |
| `max_angular_accel` | 1.5 | 角加速度上限 (rad/s²) |
| `max_angular_decel` | 3.0 | 角减速度上限 (rad/s²) |

### 响应曲线与滤波

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `expo_linear` | 2.0 | 线速度指数响应幂次（越大低速越精细） |
| `expo_angular` | 1.8 | 角速度指数响应幂次 |
| `deadzone` | 0.08 | 摇杆死区 (0~1) |
| `filter_alpha` | 0.3 | 低通滤波系数 (0~1，越小越平滑但延迟越大) |

### 安全参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `steering_speed_factor` | 0.6 | 满速行驶时角速度缩减到此比例 |
| `speed_profile_low` | 0.3 | 低速档比例 |
| `speed_profile_mid` | 0.6 | 中速档比例 |
| `speed_profile_high` | 1.0 | 高速档比例 |
| `spin_angular_speed` | 0.8 | 原地旋转角速度 (rad/s) |

### 轴和按钮映射

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `axis_linear` | 1 | 线速度轴编号（左摇杆 Y） |
| `axis_angular` | 2 | 角速度轴编号（右摇杆 X） |
| `axis_lt` | 5 | LT 扳机轴编号 |
| `axis_rt` | 4 | RT 扳机轴编号 |
| `btn_estop` | 0 | 紧急停止按钮编号 (A) |
| `btn_speed_low` | 2 | 低速档按钮编号 (X) |
| `btn_speed_mid` | 3 | 中速档按钮编号 (Y) |
| `btn_speed_high` | 1 | 高速档按钮编号 (B) |
| `btn_spin_left` | 4 | 原地左转按钮编号 (LB) |
| `btn_spin_right` | 5 | 原地右转按钮编号 (RB) |

---

## 移植到其他包

如果你要将这个手柄控制功能集成到别的 ROS2 包中，需要修改以下内容：

### 1. 复制核心文件

将以下文件复制到目标包中：

```
your_package/
├── your_package/
│   └── gamepad_teleop.py       ← 核心控制节点（从 gamepad_control/ 复制）
├── config/
│   └── gamepad_params.yaml     ← 参数配置文件
└── launch/
    └── gamepad_teleop.launch.py ← Launch 文件
```

### 2. 修改 `setup.py`

在目标包的 `setup.py` 中：

```python
# data_files 中添加 config 和 launch 目录
data_files=[
    # ... 原有内容 ...
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
],

# entry_points 中添加可执行文件
entry_points={
    'console_scripts': [
        # ... 原有内容 ...
        'gamepad_teleop = your_package.gamepad_teleop:main',
        #                  ^^^^^^^^^^^^
        #                  改为你的包名
    ],
},
```

### 3. 修改 `package.xml`

添加依赖：

```xml
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
```

### 4. 修改 `launch` 文件

修改 `gamepad_teleop.launch.py` 中的包名：

```python
pkg_dir = get_package_share_directory('your_package')  # ← 改为你的包名
# ...
gamepad_node = Node(
    package='your_package',     # ← 改为你的包名
    executable='gamepad_teleop',
    # ...
)
```

### 5. 重新编译

```bash
cd /path/to/your_ws
colcon build --symlink-install --packages-select your_package
source install/setup.bash
```

### 移植检查清单

- [ ] `gamepad_teleop.py` 已复制到 `your_package/your_package/` 目录
- [ ] `setup.py` 的 `entry_points` 中 `your_package.gamepad_teleop:main` 包名正确
- [ ] `setup.py` 的 `data_files` 包含 config 和 launch 目录
- [ ] `package.xml` 添加了 `geometry_msgs` 和 `std_msgs` 依赖
- [ ] `launch` 文件中的 `package` 参数改为你的包名
- [ ] `config/gamepad_params.yaml` 已复制
- [ ] 编译通过并 source 了 install/setup.bash

---

## 更换手柄设备

如果你更换了手柄（不同品牌/型号），需要做以下操作：

### 第一步：确认设备文件

```bash
# 插入手柄后查看设备
ls /dev/input/js*
# 通常是 /dev/input/js0

# 查看 USB 设备信息
lsusb | grep -i gamepad
# 或
lsusb
# 找到类似：Bus 001 Device 015: ID 04b5:2415 ROHM LSI Systems USA, LLC Flydigi Dune Fox
```

### 第二步：获取轴和按钮编号

安装 joystick 工具并测试：

```bash
# 安装
sudo apt install joystick

# 查看轴和按钮数量
jstest /dev/input/js0

# 交互式测试（推动摇杆/按按钮看编号变化）
jstest --event /dev/input/js0
```

或者用 Python 快速检测：

```bash
python3 -c "
import struct, os, fcntl, array

js = open('/dev/input/js0', 'rb')

buf = array.array('B', [0])
fcntl.ioctl(js, 0x80016a11, buf)
print(f'轴数量: {buf[0]}')

buf = array.array('B', [0])
fcntl.ioctl(js, 0x80016a12, buf)
print(f'按钮数量: {buf[0]}')

buf = array.array('B', [0]*64)
fcntl.ioctl(js, 0x80406a13, buf)
name = buf.tobytes().split(b'\x00')[0].decode()
print(f'设备名称: {name}')

js.close()
"
```

### 第三步：实时查看轴/按钮编号

运行以下脚本，推动摇杆和按按钮观察输出：

```bash
python3 -c "
import struct, os
JS_EVENT_FORMAT = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)

fd = os.open('/dev/input/js0', os.O_RDONLY)
print('按下按钮或推动摇杆，观察输出... (Ctrl+C 退出)')
try:
    while True:
        event = os.read(fd, JS_EVENT_SIZE)
        ts, value, etype, number = struct.unpack(JS_EVENT_FORMAT, event)
        etype_clean = etype & ~0x80
        if etype_clean == 1:
            print(f'[按钮] number={number}  value={value}')
        elif etype_clean == 2:
            print(f'[轴]   number={number}  value={value:+6d}  ({value/32767.0:+.3f})')
except KeyboardInterrupt:
    os.close(fd)
    print('退出')
"
```

### 第四步：修改配置

根据观察到的编号，修改 `config/gamepad_params.yaml`：

```yaml
gamepad_teleop:
  ros__parameters:
    device: "/dev/input/js0"        # 设备文件路径

    # 修改以下映射为你的手柄对应编号
    axis_linear: 1      # 前后摇杆的轴编号
    axis_angular: 2     # 左右摇杆的轴编号
    axis_lt: 5          # 左扳机轴编号
    axis_rt: 4          # 右扳机轴编号

    btn_estop: 0        # 紧急停止按钮编号
    btn_speed_low: 2    # 低速档按钮编号
    btn_speed_mid: 3    # 中速档按钮编号
    btn_speed_high: 1   # 高速档按钮编号
    btn_spin_left: 4    # 原地左转按钮编号
    btn_spin_right: 5   # 原地右转按钮编号
```

也可以运行时直接覆盖，无需修改文件：

```bash
ros2 run gamepad_control gamepad_teleop --ros-args \
  -p device:=/dev/input/js1 \
  -p axis_linear:=1 \
  -p axis_angular:=3 \
  -p axis_lt:=2 \
  -p axis_rt:=5
```

### 常见手柄轴映射参考

| 手柄 | 左摇杆Y | 右摇杆X | LT | RT |
|------|---------|---------|----|----|
| Flydigi Dune Fox (DInput) | 1 | 2 | 5 | 4 |
| Xbox Controller | 1 | 3 | 2 | 5 |
| PS4 DualShock | 1 | 3 | 2 | 5 |
| Logitech F710 (DInput) | 1 | 2 | 2 | 5 |
| Logitech F710 (XInput) | 1 | 3 | 2 | 5 |

> **注意**：不同手柄模式（DInput/XInput）下映射可能不同，务必用上面的检测脚本实际确认。

### 第五步：设置 udev 规则（可选）

为手柄创建固定设备名，避免 js0/js1 编号变化：

```bash
# 查看手柄 udev 信息
udevadm info -a -n /dev/input/js0 | grep -E "idVendor|idProduct|product"

# 创建规则文件
sudo tee /etc/udev/rules.d/99-gamepad.rules << 'EOF'
# Flydigi Dune Fox
SUBSYSTEM=="input", ATTRS{idVendor}=="04b5", ATTRS{idProduct}=="2415", SYMLINK+="input/gamepad", MODE="0666"
EOF

# 重载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 之后就可以用固定路径
# device: "/dev/input/gamepad"
```

---

## 控制逻辑详解

整个控制管线处理流程如下：

```
摇杆原始值 (-1~1)
    │
    ▼
[1] 死区过滤 ──────── 消除摇杆归中不精确导致的微小偏移
    │
    ▼
[2] 指数响应曲线 ──── 低速更精细，高速仍可达 (value^expo)
    │
    ▼
[3] 档位缩放 ──────── 根据当前低/中/高速档缩放
    │
    ▼
[4] RT 加速叠加 ───── 右扳机线性增加最多 30% 速度
    │
    ▼
[5] 转弯自动减速 ──── 线速度越大，角速度允许值越小
    │                  (防止高速急转弯侧翻)
    ▼
[6] 后退减速 ──────── 后退速度上限为前进的 60%
    │
    ▼
[7] 低通滤波 ──────── 平滑突变输入（alpha=0.3）
    │
    ▼
[8] 加速度限制 ────── 限制速度变化率
    │                  加速: 0.5 m/s²  减速: 1.0 m/s²
    ▼
[9] 安全限幅 ──────── 最终限制在最大速度范围内
    │
    ▼
  cmd_vel 发布
```

### 为什么这套逻辑适合高重心大质量底盘？

1. **加速度限制**：防止突然加速导致重心后移、急刹导致前倾
2. **转弯自动减速**：高速行驶时自动限制转弯幅度，防止离心力过大导致侧翻
3. **指数曲线**：低速精细操控便于狭窄空间调整，不会一碰摇杆就冲出去
4. **后退减速**：后退视野受限，降低最大后退速度增加安全余量
5. **死人开关**：松手即停，是工业机器人遥控的基本安全要求

---

## 话题接口

### 发布话题

| 话题名 | 类型 | 频率 | 说明 |
|--------|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 50 Hz | 速度指令 |
| `/gamepad_status` | `std_msgs/String` | 5 Hz | 手柄状态信息 |

### 监控速度输出

```bash
# 实时查看速度指令
ros2 topic echo /cmd_vel

# 查看手柄状态
ros2 topic echo /gamepad_status
```

---

## 文件结构

```
gamepad_contorl/
├── config/
│   └── gamepad_params.yaml      # 参数配置文件
├── gamepad_control/
│   ├── __init__.py
│   └── gamepad_teleop.py        # 核心控制节点
├── launch/
│   └── gamepad_teleop.launch.py # Launch 文件
├── resource/
│   └── gamepad_control          # ament 资源标记
├── package.xml                  # ROS2 包描述
├── setup.py                     # Python 包安装配置
├── setup.cfg                    # 安装路径配置
└── README.md                    # 本文件
```

---

## 故障排除

### 手柄无法识别

```bash
# 检查 USB 连接
lsusb

# 检查设备文件
ls /dev/input/js*

# 检查权限
sudo chmod 666 /dev/input/js0
# 或将用户加入 input 组
sudo usermod -aG input $USER
# 然后重新登录
```

### 摇杆方向反了

在 `gamepad_teleop.py` 中找到以下行，去掉或添加负号：

```python
raw_linear = -self.axes.get(self.axis_linear, 0.0)   # 前后方向
raw_angular = -self.axes.get(self.axis_angular, 0.0)  # 左右方向
```

### 节点启动但机器人不动

1. 检查是否按住了 **LT 扳机**（死人开关）
2. 检查是否误触了 **A 按钮**（紧急停止），再按一次解除
3. 确认 `cmd_vel` 话题名与底盘驱动订阅的一致：
   ```bash
   ros2 topic list | grep cmd_vel
   ```

### 延迟太大 / 反应太慢

调小滤波系数（更快响应但可能抖动）：

```yaml
filter_alpha: 0.5      # 默认 0.3，调大则响应更快
max_linear_accel: 1.0   # 默认 0.5，调大则加速更猛
```

### "sequence size exceeds remaining buffer" 警告

这是 ROS2 DDS 序列化的非致命警告，不影响功能。可以忽略，或切换 DDS 实现：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
