# DcarON ROS 2 客户端

一个 Python 节点将 DFLink 串口变成 ROS 2 标准话题。与本仓库 STM32F1、Arduino
例程并列维护；ROS 2 运行在电脑上，电脑通过 USB-TTL 直接连接 **Dcar 底盘通信口**，
不需要串接 STM32F1 示例板，也不要连接示例板的 115200 日志口。

```text
你的 ROS 2 程序 / RViz / rosbag
          ↕ 标准话题
     dcaron_bridge（电脑）
          ↕ DFLink / COM 或 /dev/ttyUSB*
        Dcar 底盘
```

## 能力与范围

| ROS 接口（默认无 namespace） | 类型 | 行为 |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | 收到有效的新回包时发布；位置在 odom 系，速度在机体系 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 仅 `telemetry=odom` 时发布加速度和角速度 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 仅 `enable_cmd_vel=true` 时订阅，映射为 `A=0x02 B=0x62` |
| `/motion` | `dcaron_interfaces/action/Motion` | 旋转、位移、平移并转向、圆弧、十字标，带进度/结果/取消 |
| `/tf` | TF | 可选 `odom → base_link`，默认关闭 |

- 默认 `telemetry=velpos`：标准 VelPos v2，35 字节；不虚构 IMU 数据。
- `telemetry=odom`：Odom v4，51 字节，需要底盘当前档位/授权支持 Pro 全量回传。
- SI：m、m/s、rad、rad/s。+X 前、+Y 左、+Z 上，Yaw 逆时针为正。
- 原始串口订阅每次连接只发送一次；断线/数据超时会重连，丢弃旧命令。
- 0.2 版补齐有终点运动与十字标；详见 [动作接口与轨迹示例](docs/motion.md)。

## Windows 原生安装与运行

无需 WSL、Ubuntu 或 Docker。采用 **ROS 官方 Windows 预编译包**，使用发行版配套的
Pixi 环境；不要把系统 Python 的 DLL/依赖与 ROS 环境混用。

- Windows 11 可按 [ROS 2 Lyrical 原生安装指南](https://docs.ros.org/en/lyrical/Get-Started/Installation/Windows-Install-Binary.html) 安装。
- 官方文档的 [GitHub 源文](https://github.com/ros2/ros2_documentation/blob/lyrical/source/Get-Started/Installation/Windows-Install-Binary.rst) 可作为备用入口。
- 下载 [官方 ROS 2 release](https://github.com/ros2/ros2/releases) 的 Windows amd64 包；固定发行版和 release，不使用 Rolling 作为交付环境。
- 使用短路径，例如 `C:\dev\ros2_lyrical`；实际路径以解压结果为准。
- 在含 `pixi.toml` 的发行包目录运行 `pixi install`，按该发行包说明执行
  `pixi run python preinstall_setup_windows.py`，仅当该文件存在且该版指南要求时执行。
- 在 CMD 中运行 `pixi shell`，再 `call <ROS安装目录>\local_setup.bat`。
  下列所有命令均在这个已激活的终端中执行。

在 **ROS 配套 Pixi 环境**中添加串口和测试依赖，并保留生成的环境锁文件：

```bat
pixi add "pyserial==3.5" pytest
python -c "import rclpy, serial; from nav_msgs.msg import Odometry; print('OK')"
```

首次在两个激活后的终端分别运行官方 `demo_nodes_cpp talker` 和
`demo_nodes_py listener`，确认 ROS 基础安装正常。
0.2 版新增 `dcaron_interfaces` Action 定义，需要安装 **Visual Studio 2022 C++ Build Tools**
或已有 VS 2022 的 C++ 工作负载。使用 x64 Native Tools 终端并激活 Pixi/ROS 后编译，
`colcon` 会先生成接口，再构建 Python 驱动。运行时不要求启动 Visual Studio IDE。
然后进入本仓库 `ros2` 目录：

```bat
colcon build --merge-install --base-paths . --packages-up-to dcaron_bridge
call install\local_setup.bat
python -m serial.tools.list_ports
ros2 launch dcaron_bridge bridge.launch.py port:=COM5
```

将 COM5 换成实际串口号。默认只读取遥测，不接收运动话题。新开终端同样激活 ROS
和本工作区后验证：

```bat
ros2 topic list
ros2 topic echo /odom --once
ros2 topic hz /odom
```

可选 PowerShell 入口（从已激活 CMD 启动，继承环境）：

```bat
powershell -NoProfile -File tools\windows.ps1 -Action check
powershell -NoProfile -File tools\windows.ps1 -Action build
powershell -NoProfile -File tools\windows.ps1 -Action test
powershell -NoProfile -File tools\windows.ps1 -Action run -Port COM5
```

脚本不改系统执行策略；若电脑限制脚本，可直接使用上面的 CMD 命令。

## Linux / 已有 ROS 2 环境

源码相同。Jazzy 示例（云端 CI 的 Linux 容器不要求你的电脑安装 Linux）：

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-serial python3-colcon-common-extensions python3-pytest
cd DCar-BaseLine/ros2
colcon build --merge-install --base-paths . --packages-up-to dcaron_bridge
source install/setup.bash
ros2 launch dcaron_bridge bridge.launch.py port:=/dev/ttyUSB0
```

串口使用 460800、8N1，USB-TTL TX/RX 与底盘 RX/TX 交叉并共地，采用匹配的
3.3V TTL 电平。关闭占用同一串口的 DFhelper 等程序，一个串口只由一个节点打开。

## 速度输入与 Pro 数据

确认场地可运动后显式开启速度订阅：

```text
ros2 launch dcaron_bridge bridge.launch.py port:=COM5 enable_cmd_vel:=true
```

另一个激活后的终端发送零速度验证连接：

```text
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"
```

默认 XY 合速度上限 0.5m/s、角速度上限 1rad/s；超范围、NaN、Inf 或非平面轴
命令会被拒绝并发零。默认 0.5 秒无新指令主动发零。检查使用单调时钟定时器，
不受 ROS 仿真时间暂停影响。遥测失效时禁止接受新的运动输入。

**底盘持续速度命令没有自动停车终点。** 节点正常退出/输入超时会尝试发零；
拔掉串口、强杀进程或电脑断电时，主机无法保证零速度抵达底盘。这不是 MCU
通信看门狗，不能替代设备急停。本包没有修改底盘固件的该行为。

Pro 全量数据：

```text
ros2 launch dcaron_bridge bridge.launch.py port:=COM5 telemetry:=odom
ros2 topic echo /imu/data --once
```

## 配置与数据约定

配置见 `dcaron_bridge/config/bridge.yaml`。复制到自用配置文件后可传入
`params_file:=绝对路径`。**launch 的 port、telemetry、enable_cmd_vel、enable_motion_actions 始终覆盖 YAML
同名值**（包括默认值）；这四个值从命令行指定，其余值从 YAML 读取。
参数启动时读取且只读，修改后重启。

- 消息使用收到有效帧时的主机 ROS 时间；MCU `Time_ms/us` 保留在协议解码结果中。
  首版未建立设备与电脑的时间同步，不把设备开机计数直接当 ROS 时间戳。
- IMU 已按固件 FLU 机体系解释，默认 `imu_frame=base_link`。改名不会自动旋转数据，
  其他安装坐标系需要正确的外参转换。
- 不将 Odom yaw 和传感器 roll/pitch 组合成 IMU 姿态测量：IMU
  `orientation_covariance[0]=-1` 表示不可用，角速度/加速度协方差全零表示未知。
- VelPos 无角速度，`/odom` 角速度为占位零、协方差 1e6，不能当作静止测量；
  需要角速度时使用 Pro Odom/独立 IMU。
- Odom 协方差是保守初值，尚未实测标定；接入融合算法时按实际设备整定。
- TF 默认关闭，仅在本节点拥有 `odom → base_link` 发布权时通过配置开启。
  多车用 namespace 区分话题，并分别设置 frame 名。
- `/cmd_vel` 为 **Twist**，不是 TwistStamped；上层输出后者时需转换或配置上层。
- 数据发布为 reliable / volatile / depth 10；命令订阅为 reliable / volatile /
  depth 1。订阅者使用兼容 QoS。

## 验证与维护

已完成首版检查，具体环境和结果见 [2026-09-10 验证记录](docs/validation-2026-09-10.md)。

无需 ROS 的协议测试（Windows/Linux 均可）：

```text
cd ros2/dcaron_bridge
python -m unittest discover -s test -p test_protocol.py -v
python -m unittest discover -s test -p test_motion.py -v
```

`.github/workflows/ros2.yml` 配置了 Windows/Ubuntu 协议测试，以及 ROS 2 Jazzy
容器的 colcon 构建、安装入口、launch 参数和真实 ROS 话题＋模拟串口集成检查。
覆盖报文、单位、粘包分包、噪声、版本、超时、地址过滤和重连等行为。

**原生 Windows ROS 运行态和真车仍需单独验证。** Windows 协议检查通过不等于
硬件联调或所有 ROS 发行版验证通过。

协议修改先更新仓库主协议文档和报文测试，再更新 `protocol.py`；ROS 转换集中在
`node.py`。不复制 MCU 固件进 ROS 包，不提交安装环境或 colcon 生成目录。

与 Livox 搭配见 [Livox 接入说明](docs/livox.md)。两套驱动独立发布话题，本包不依赖雷达 SDK。
