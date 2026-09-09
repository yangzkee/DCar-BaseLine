# ROS 2 首版验证记录 — 2026-09-10

实现提交：`efbde99900e57fc0be0ad08a698e73e77577b8e3`。

- [开发 PR #1](https://github.com/yangzkee/DCar-BaseLine/pull/1)
- [GitHub Actions 完整记录](https://github.com/yangzkee/DCar-BaseLine/actions/runs/34377073953)

| 环境 | 检查 | 结果 |
|---|---|---|
| 本机 Windows 11 / Python 3.12 | 11 项协议测试、语法编译、XML 解析 | 通过 |
| GitHub windows-latest / Python 3.12 | 11 项协议测试 | 通过 |
| GitHub ubuntu-latest / Python 3.12 | 11 项协议测试 | 通过 |
| ROS 2 Jazzy / Ubuntu 容器 / Python 3.12.3 | colcon build、安装后 executable/launch 查询 | 通过 |
| 同上 | 11 项协议 + 5 项 ROS 集成测试 | **16 passed，0 errors，0 failures，0 skipped** |

ROS 集成使用真正的 rclpy 节点、DDS 话题和标准消息，串口端点为内存模拟。
覆盖：遥测发布和单位、速度转帧、只读默认值、地址过滤、超时发零、非法指令拒绝、
断线旧命令清除、重连后需新遥测、部分写失败和正常退出清理。

未验证：本机原生 ROS 2 运行态、真实 COM 设备、真车运动、Livox 设备、跨电脑发现、
传感器时间同步及协方差标定。本次没有烧录固件或下发真实运动命令。

ROS 包的维护主源是本仓库 `ros2/dcaron_bridge`，与 STM32F1/Arduino 示例并列。
本记录之后若仅变更文档，以上结果仍对应这里列出的实现提交；代码变化应追加新的验证记录。
