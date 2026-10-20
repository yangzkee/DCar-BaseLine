# DCar ROS 2 客户端

版本 **0.3.1**。电脑通过串口直接连接车端DFLink口，默认 **队列v6**。
两个包分别是 `dcaron_interfaces`（Motion、Trajectory与MotionSegment定义）和 `dcaron_bridge`（串口桥接、轨迹客户端）。
这不是Nav2导航器，也不包含车端固件。车端须支持v6且已激活/校准；所有运动参数使用SI单位。

## 目录

- [1. 构建与启动](#section-1)
- [2. 运行一条或整组动作](#section-2)
- [3. ROS接口](#section-3)
- [4. 配置](#section-4)
- [5. 检查与验证边界](#section-5)

平台选择、函数速查与分层验收标准见[根 README](../README.md)。

<a id="section-1"></a>
## 1. 构建与启动

需要已激活的ROS 2环境、Python>=3.10、colcon、pyserial和PyYAML。
Windows完整环境步骤见 [Windows教程](docs/windows-tutorial.md)；Linux在工作区安装对应ROS依赖。
从仓库的 `ros2` 目录执行：

```sh
colcon build --merge-install --base-paths . --packages-up-to dcaron_bridge
# Linux / bash:
source install/setup.bash
# Windows CMD请改用: call install\local_setup.bat
ros2 launch dcaron_bridge bridge.launch.py port:=COM5
```

COM5换为实际串口；Linux例如 `/dev/ttyUSB0`。波特率默认460800，TX/RX交叉、GND共地、3.3V TTL。
默认只读遥测，**不接受运动输入**。另开同样加载环境的终端：

```sh
ros2 topic echo /odom --once
ros2 topic hz /odom
```

一个串口只由本节点持有，串口助手、直接探测脚本和另一控制板不能同时发送。

<a id="section-2"></a>
## 2. 运行一条或整组动作

在桥接终端开启Action，再从另一已加载环境的终端发目标：

```sh
ros2 launch dcaron_bridge bridge.launch.py port:=COM5 enable_motion_actions:=true
```

单条前进10cm、速度6cm/s、中档18：

```sh
ros2 action send_goal /motion dcaron_interfaces/action/Motion "{command: 100, x: 0.1, speed: 0.06, profile: 18, timeout: 10.0}" --feedback
```

**profile必须显式填16..20，包括原地旋转目标。** ROS字段的缺省0不是有效队列档位。
`goal accepted`只表示ROS接收目标，入队ACK只表示车端接受任务，最终 `success: true` 才表示车端报告成功。

从仓库 `ros2` 目录先校验轨迹，不连接串口、不发送运动：

```sh
ros2 run dcaron_bridge trajectory_sequence dcaron_bridge/examples/square.yaml
```

确认要执行后加 `--execute`：

```sh
ros2 run dcaron_bridge trajectory_sequence dcaron_bridge/examples/square.yaml --execute
ros2 run dcaron_bridge trajectory_sequence dcaron_bridge/examples/stadium.yaml --execute
```

- square：前进0.50m、左转90°，四次，共8段。
- stadium：倒行半圆、后退、倒行半圆、后退并转360°，共4段；要求全向底盘。
- 默认连续排队，最多8段在途、1..128段一组；加 `--sequential` 则逐段完成后再发。
- 示例YAML的60秒是**整个动作组**的上限，与STM32每次Wait的时限不同。
- 每次命令只运行一次；YAML数据结构是 `timeout` 加 `segments` 数组。
- 安装后的示例位于 `share/dcaron_bridge/examples`；可用 `ros2 pkg prefix --share dcaron_bridge` 定位。

目标字段、完成语义和十字标边界见 [动作说明](docs/motion.md)，报文见 [队列v6协议](../docs/DFLink-v6.md)。

<a id="section-3"></a>
## 3. ROS接口

| 接口（未设置namespace时） | 类型 | 说明 |
|---|---|---|
| `/odom` | nav_msgs/Odometry | 连续里程计位置，车体线速度；姿态四元数 |
| `/imu/data` | sensor_msgs/Imu | 仅 `telemetry:=odom` 且设备提供Pro全量回包时发布 |
| `/cmd_vel` | geometry_msgs/Twist | 默认关闭，独立启用 `enable_cmd_vel:=true` |
| `/motion` | dcaron_interfaces/action/Motion | 单动作；默认队列v6编码 |
| `/trajectory` | dcaron_interfaces/action/Trajectory | 整组v6队列轨迹 |
| `/tf` | TF | 可选odom→base_link，默认关闭 |

默认 `telemetry=velpos` 使用标准35字节VelPos；它不提供陀螺仪角速度，odom角速度零是缺失数据占位，不能据此判断车辆没有旋转。
Pro Odom v4为51字节，可提供IMU；`telemetry:=odom`还需要底盘能力/授权。
frame名改变不会自动旋转数据，当前数据按FLU解释。单一节点持有串口，两个Action互斥；有活动目标时不自动抢占，先取消或等待。

<a id="section-4"></a>
## 4. 配置

配置文件为 `dcaron_bridge/config/bridge.yaml`。launch直接接受 `port`、`telemetry`、`enable_cmd_vel`、`enable_motion_actions`、`namespace`、`params_file`。
前四项的launch值会覆盖配置文件同名值；其他参数从配置读取，例如 `motion_protocol`。

默认：`motion_protocol=queue`、任务30s、最大时限120s、位移2m、转角2π、线速度0.5m/s、角速度1rad/s、遥测失联2s、重连间隔3s。
Trajectories即使配置为legacy也使用v6；legacy只选择 `/motion` 的无ID编码，不是自动降级。
十字标仍是专用无ID接口，不能夹入队列轨迹。

开启cmd_vel后，0.5秒无新速度输入尝试发零；任何有效cmd_vel会打断当前Action，非法/超限速度也会终止并发零。
主机看门狗不等于断线硬件看门狗。取消、超时、失联会尝试取消队列和停车，但链路断开时不能确认车端收到。
重连不会重放旧目标；重新发目标前须确认现场状态和设备队列。

<a id="section-5"></a>
## 5. 检查与验证边界

在仓库根目录运行 `python3 tests/run_checks.py`，统一检查文档链接、示例、C客户端、Arduino和不依赖ROS的Python协议测试。
在真正的ROS环境另运行：

```sh
colcon test --merge-install --packages-select dcaron_bridge --event-handlers console_direct+
colcon test-result --verbose
```

CI包含ROS Jazzy构建与内存串口集成测试；一次离线检查不代表当前提交的ROS运行或实车已验证。
本版整理未进行真车轨迹测试，速度、到位与微调成功均以车端回包为依据，不代替外部测量。

与Livox搭配见 [接入说明](docs/livox.md)，雷达驱动独立运行，本包不依赖雷达SDK。
