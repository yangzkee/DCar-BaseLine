# 完整运动接口与真车分步验证 — 2026-09-10

代码基础提交：`9fe15c8953a38c303adcebb430c9330dafcb2022`。
本轮后续调整仅涉及 Windows UTF-8 环境、日志、诊断入口和文档。

## 已完成

- 与原 STM32F1、Arduino 示例并列补充 ROS 2 motion Action。
- 旋转、直线（三种 profile）、平移并转向、圆弧、十字标 Record/Localize/NEXT 组合。
- 进度、完成、拒绝、RC/新命令打断、取消、超时、断线和同命令迟到回包隔离。
- 离散轨迹序列（逐段等 MCU 成功）；旋转与方形 YAML 示例。

## 软件检查

| 环境 | 结果 |
|---|---|
| GitHub Windows / Ubuntu | 19 项纯协议与动作编码测试通过 |
| GitHub ROS 2 Jazzy 容器 | 28 项测试通过，0 skipped，包含真实 DDS Action / topic 与模拟串口 |
| 本机 Windows 11 / 原生 ROS 2 Lyrical | 两个包 colcon build 通过，28 项 pytest 全部通过 |

[GitHub 运行记录](https://github.com/yangzkee/DCar-BaseLine/actions/runs/34378964115)。

中文 Windows 在 ROS 模板展开时出现 GBK 解码错误，通过当前进程
`PYTHONUTF8=1`、`PYTHONIOENCODING=utf-8`、CMD `chcp 65001` 解决，未改系统语言。
本机使用已有 Visual Studio 2022 Community C++ 工具；未安装 Linux/WSL/Docker。

## 真板识别

ATK CMSIS-DAP 与 STM32F407 目标可见。Windows USB 串口为 **COM5**；底盘物理口是
**USART1，460800，8N1**，不能把它误解为 Windows COM1。
本轮未烧录、擦除、切换固件模式或改写授权/校准数据。

## 第一步：纯 Python → DFLink → 底盘

入口：`ros2/tools/probe_rotation.py`。
先读到 VelPos v2 再执行一次逆时针15°、最大角速度0.2rad/s、8秒超时。

- 原始运动帧：`DF 01 97 02 63 08 39 0A 00 00 D0 07 00 00 FD FB 03`。
- 收到连续 `A=0x6F, B=0x63` 进度，最后 payload `FF 00`。
- 起始 yaw −0.06875°，观察末端 yaw 14.82815°，增量 **14.89690°**。
- 底盘回传相对目标偏差约 −0.10310°。
- 结束发送 `0x62` 零速度和单次访问以关闭连续回传。

## 第二步：Windows 原生 ROS Action → DFLink → 底盘 → ROS 结果

入口：`ros2/tools/probe_ros_rotation.py`，使用已安装的两个 ROS 包；真实 ActionClient
和 ActionServer，经 ROS 接口发送同样15°目标，COM5是真串口。额外的 ROS 订阅者
读取 `/odom`，串口发送记录与第一步完全同帧。

- 收到38次 Action feedback，终止 process=255、percent=100、notice=0。
- ROS 结果 **status=4 (SUCCEEDED), success=true, notice=0**。
- `/odom` 共观察23条；航向增量 **14.76512°**，相对目标偏差约 −0.23488°。
- 结束发送零速度、关闭连续回传并释放COM5。

以上角度均是底盘自身遥测值，未用外部角度仪标定；两次是独立相对旋转测试。
目前只证明该配置下旋转及进度/完成/遥测链路可用，不证明所有动作的外部精度。

## 本机复现信息

- 原生包：`release-lyrical-20260807`，`ros2-lyrical-2026-08-07-windows-AMD64.zip`。
- SHA256：`ab3305fc9961848072d0665e7a7f2a772d4ae5a9499a3ac4c902e9d6f9f6bc5c`。
- Pixi 0.80.0 Windows x64 SHA256：`7700e558c4abef7d9b12f6caffabef39aec50b86fb76ac86cafa24f7c6c49bf5`。
- 环境和依赖锁文件：`C:\dev\dcar_ros2\runtime\ros2-windows`，添加 pyserial=3.5。
- 构建、安装、日志：`C:\dev\dcar_ros2\build`、`install`、`log`，避免Windows长路径。
- 原始硬件日志在 DcarON 的 `Delivery_Files/Debug_Logs/2026-09-10_ros2_direct_rotation_15deg.jsonl`
  和 `2026-09-10_ros2_native_rotation_15deg.jsonl`。

## 未完成的现场项目

直线位移/横移、平移并转向、圆弧、方形整段、十字标、Pro IMU，以及取消/急停在
真实运动中的行为，尚未逐项实测。本次未为这些动作发送非零目标。
这些项目应按现场空间和底盘能力分别测试，不能以“接口已编码、CI已通过”替代真车结果。
