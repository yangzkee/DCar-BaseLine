# DCar ROS 2 客户端

官网：[differ-tech.pages.dev](https://differ-tech.pages.dev)

淘宝搜索 **DCar**，认准 **“点浮科技”** 购买。

电脑串口直连车端DFLink口，默认队列v6。`dcaron_interfaces`定义动作/轨迹消息，`dcaron_bridge`提供串口桥、双流遥测与测试客户端；不包含车端固件，也不是Nav2导航器。

## 目录

- [构建](#构建)
- [一条命令持续操场测试](#一条命令持续操场测试)
- [双遥测与话题](#双遥测与话题)
- [自定义动作与只读应用](#自定义动作与只读应用)
- [配置与异常行为](#配置与异常行为)
- [验收与检查](#验收与检查)

## 构建

ROS 2、Python>=3.10、colcon、pyserial、PyYAML；Windows完整环境步骤见 [Windows教程](docs/windows-tutorial.md)。从仓库ros2目录执行：

```sh
colcon build --merge-install --base-paths . --packages-up-to dcaron_bridge
# Linux/bash:
source install/setup.bash
# Windows CMD改用: call install\local_setup.bat
```

车端须已激活/校准、支持队列v6和相应运动能力。USB-TTL 3.3V TTL，460800 / 8N1，TX/RX交叉、GND共地，不经过STM32示例板。关闭占用同一串口的助手和脚本。

## 一条命令持续操场测试

```sh
ros2 launch dcaron_bridge stadium_test.launch.py port:=COM5
```

替换实际串口，Linux例如 `/dev/ttyUSB0`。这一个入口自动启动桥接、Action和持续测试客户端，无需选择模式或第二条执行命令。
四段：半径0.30m倒行半圆→后退0.60m→同样倒行半圆→后退0.60m同时旋转2π，速度请求0.15m/s、档位18。需要全向/麦克纳姆底盘。
每圈四段以队列执行，YAML整圈时限60秒；整圈最终success=true后立即下一圈，无圈间延时。客户端打印 `[LAP] DONE n` 及每圈反馈/结果，圈数持续增加。
失败结束测试并关闭本launch；Ctrl+C退出时尝试取消和停车。断线不能确认停车。

## 双遥测与话题

每次串口连接独立发Odom订阅、VelPos订阅和状态查询，相邻请求间至少20ms；每种遥测请求10Hz，不根据是否收到某一种来轮换订阅。
两类数据分别解析和保存；状态与遥测缺失只报告，不拒绝运动、不单独触发重连。任务必须按自身ACK / DONE / 超时完成。

| 默认无namespace接口 | 类型 | 来源 / 用途 |
|---|---|---|
| `/telemetry/odom` | nav_msgs/Odometry | 仅真实Odom v4回包，不用VelPos补数 |
| `/telemetry/velpos` | nav_msgs/Odometry | 仅真实VelPos v2回包，不用Odom补数 |
| `/telemetry/status` | std_msgs/String，JSON | 每秒输出bytes、selected、state及两类wire/ok/reject/len/ver/fresh/age_s |
| `/odom` | nav_msgs/Odometry | 优先最近1秒内Odom，缺失则用新鲜VelPos；两者都过期不发布旧值 |
| `/imu/data` | sensor_msgs/Imu | 仅选中的真实新鲜Odom提供；VelPos无IMU，不发布伪造IMU |
| `/motion` | dcaron_interfaces/action/Motion | 单动作结果 / 反馈 |
| `/trajectory` | dcaron_interfaces/action/Trajectory | 1..128段队列轨迹 |
| `/cmd_vel` | geometry_msgs/Twist | 可选持续速度输入 |
| `/tf` | TF | 可选odom→base_link，默认关闭 |

VelPos35字节/version=2，Odom51字节/version=4，Odom需相应车端能力/授权。未收到Odom时VelPos仍工作，Odom恢复自动使用。诊断两类ok计数必须分别增长才能判双流通过。
VelPos没有角速度，Odometry角速度零仅为缺失数据占位且协方差设为未知大值，不能据此判断车辆不旋转。IMU orientation_covariance[0]=-1，不把混合姿态当测量姿态。
frame名称只标识坐标系，不自动旋转数据；当前按FLU（前/左/上）解释。

## 自定义动作与只读应用

只读应用可单独启动桥接（不会自动运动）：

```sh
ros2 launch dcaron_bridge bridge.launch.py port:=COM5
```

自定义动作可给此通用桥接入口加 `enable_motion_actions:=true`，从另一加载环境的终端发送：

```sh
ros2 action send_goal /motion dcaron_interfaces/action/Motion "{command: 100, x: 0.1, speed: 0.06, profile: 18, timeout: 10.0}" --feedback
```

profile必须显式16..20，包括原地旋转；目标缺省0无效。`goal accepted`不是车端完成，入队ACK也不是；最终success=true才表示协议成功。
通用YAML工具可离线校验或执行一次：

```sh
ros2 run dcaron_bridge trajectory_sequence dcaron_bridge/examples/stadium.yaml
ros2 run dcaron_bridge trajectory_sequence dcaron_bridge/examples/stadium.yaml --execute
```

不带execute只校验，加sequential逐段执行。持续测试直接使用上面的stadium_test.launch.py，不需要给工具额外选循环模式。
安装后的YAML位于 `share/dcaron_bridge/examples`；用 `ros2 pkg prefix --share dcaron_bridge` 定位。详细字段见[动作说明](docs/motion.md)，报文见[队列协议](../docs/DFLink-v6.md)。

## 配置与异常行为

`dcaron_bridge/config/bridge.yaml`配置频率、地址、运动范围、时限等；launch接受port、enable_cmd_vel、enable_motion_actions、namespace、params_file，前三项直接覆盖配置文件。双流默认始终订阅，不设置telemetry选择器。
默认：任务30s、最大120s、位移2m、转角2π、线速度0.5m/s、角速度1rad/s；telemetry_timeout=2s仅用于缺失告警，重连间隔3s仅用于实际串口故障。
一个节点独占串口，两个Action互斥，不自动抢占；先取消或等结果。`motion_protocol=queue`默认v6，legacy仅影响单动作编码，不是自动降级；Trajectory始终v6。

cmd_vel默认关闭，启用后0.5秒无新速度输入尝试零速；非法/超限速度同样终止当前动作并零速。cmd_vel有效输入会打断Action。
遥测/状态不作为运动门禁；参数、串口连接、队列能力、ACK / DONE、取消及任务时限仍有效。串口故障中止当前任务，重连不重放旧动作；缺失遥测只报告，不关闭仍可用的运动通道。

## 验收与检查

另一个加载环境的终端执行：

```sh
ros2 topic echo /telemetry/status
ros2 topic hz /telemetry/odom
ros2 topic hz /telemetry/velpos
```

两类wire和ok同时持续增加至少60秒，reject不增加；只有单流时另一流不得被补数。Odom断更1秒后 `/odom` 自动选择新鲜VelPos，Odom恢复后自动选择它；两种都断更时不发布旧值。
确认每圈结果success=true、completed=4、圈号持续增长，再按[物理验收表](../README.md#验收标准)测量四段动作和重复性。

根目录执行 `python3 tests/run_checks.py`；真正ROS环境另执行：

```sh
colcon test --merge-install --packages-select dcaron_bridge --event-handlers console_direct+
colcon test-result --verbose
```

CI运行ROS Jazzy构建与内存串口集成，不代替实车验证。Windows环境、车辆与其他发行版按对应环境验收。
Livox独立驱动接入见[说明](docs/livox.md)，本包不依赖雷达SDK。


## 2026-10-20 统一 Odometry

STM32 和 ROS 默认统一订阅 `A=0x04/B=0x80`，回传 `A=0x6C/B=0x80`（v4、51B）。已删除其 VelPos 0x81 订阅和解析入口；下文历史双流描述以本节为准。保留新鲜度、DMA接收、队列任务ID及连续操场功能。Pro返回完整数据，Std的roll/pitch、三轴加速度、三轴角速度填0；Mini/SE不开放里程计。ROS只有明确设置 `publish_imu: true` 才发布 IMU，默认false以免把Std填零当作测量。BDB的A02/B81定位命令不受影响。
