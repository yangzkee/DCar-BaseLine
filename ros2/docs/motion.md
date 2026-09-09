# 运动 Action 与轨迹组合（0.2）

同一个 GitHub 仓库包含 STM32F1、Arduino 和 ROS 2 客户端。ROS 目录下分为两个
**包**：`dcaron_interfaces` 只定义一个 Action，`dcaron_bridge` 负责串口与执行；
它们不是两个 Git 仓库，也不包含新的底盘固件。

采用 ROS 原生 Action，区分“已接受、执行中、成功、失败、取消”。连续速度仍走
`cmd_vel`；有终点的动作走 `motion`，只有 MCU 正常终止回包才成功。

## 命令映射

| command | 名称 | Goal 使用字段 | 固件语义 |
|---|---|---|---|
| 99 / 0x63 | ROTATE | yaw、angular_speed | 相对旋转，正值逆时针 |
| 100 / 0x64 | LINEAR | x、y、speed、profile | 相对位移，profile 0匀速/1梯形/2距离闭环 |
| 101 / 0x65 | LINEAR_WITH_YAW | x、y、yaw、speed、profile | 平移并转向；固件当前强制匀速 |
| 102 / 0x66 | ARC | radius、yaw、speed、profile | 半径正值，yaw 正负决定转向；固件当前强制匀速 |
| 128 / 0x80 | CROSS_RECORD | mode | 0 FIRST / 1 NEXT，均立即 FULL，回捕获位姿 |
| 129 / 0x81 | CROSS_LOCALIZE | mode、可选 next_command 和位移字段 | FIRST 立即 FULL；NEXT 紧邻绑定一次直线 |

位置 m、速度 m/s、角度 rad、角速度 rad/s。x/y/yaw 相对本条命令起点，
不是全局绝对坐标。差速底盘不支持横移，具体能力由底盘决定。
Action 消息字段默认 0，所以 CLI 要显式写 profile（若需梯形则为 1）；
轨迹 YAML 示例客户端的缺省 profile 是 1，与 MCU 省略 profile 的默认一致。

## 启动与发送旋转

完成两包构建并激活 ROS 与工作区，在一个终端运行：

```text
ros2 launch dcaron_bridge bridge.launch.py port:=COM5 enable_motion_actions:=true
```

另一终端发送 15° 左转，最大角速度 0.2rad/s，等待进度和结果：

```text
ros2 action send_goal /motion dcaron_interfaces/action/Motion "{command: 99, yaw: 0.2617993878, angular_speed: 0.2, timeout: 8.0}" --feedback
```

参数 `enable_motion_actions` 与 `enable_cmd_vel` 独立，均默认关闭。
如同时开启，任何收到的有效 cmd_vel 会打断当前 Action；非有限/超限速度会打断
并发零。不要让后台速度发布器和轨迹客户端同时争用控制。

## 轨迹逐段执行

`examples/rotation_probe.yaml` 和 `examples/square.yaml` 是可读的步骤列表。
每段成功后才发下一段；拒绝、取消、失败或超时立即停止序列。
从 `ros2` 目录（已激活安装环境）运行：

```text
ros2 run dcaron_bridge motion_sequence dcaron_bridge/examples/square.yaml
```

默认只检查，不发送运动。确认运动空间后加 `--execute`；日志用 `--log 文件.jsonl`。
15° 测试例：

```text
ros2 run dcaron_bridge motion_sequence dcaron_bridge/examples/rotation_probe.yaml --execute --log rotation.jsonl
```

可将旋转、直线、平移转向和圆弧按顺序组合。此处是离散命令序列，不是 Nav2 路径跟踪器，
段间等待完成回包；不承诺跨段连续速度。原有轨迹规划仍在 MCU 内执行。

## 完成与异常

- Feedback 原始 process 1..254 的百分比是 `(process-1)/254*100`。
- `FF/00` 为普通运动成功；版本/参数拒绝、RC 接管和新命令打断返回 aborted 与原 notice。
- `notice=254` 保留为本地主机失败/取消/超时；message 说明原因，不伪装为 MCU 回包。
- `timeout=0` 使用配置的 motion_timeout（默认 30s）；可设置正值但不能超过
  max_motion_timeout（默认120s）。位移/转角/线速度/角速度均做上限检查，圆弧还检查
  弧长和由 speed/radius 推导出的角速度。
- Action 等待使用非阻塞 Future；遥测、取消和定时器继续工作。持续速度的 0.5s
  watchdog 不会误停正在执行的有终点动作，后者有自己的截止时间。
- 取消/超时/关闭尝试发零。新目标不会隐式抢占旧目标，需等待结果或先取消。
- DFLink 没有序列号。取消或断线后，相同命令号的迟到回包会隔离到旧终止帧到达；
  未收齐旧终止帧时拒绝新同类型目标。隔离状态跨串口重连保留；如果终止帧确实丢失，
  先确认车辆停止再重启节点恢复，不能靠超时猜测归属并错误报告成功。
- MCU 终止说明其完成判定成立，不等于外部测量已证明全部精度达标。

## 十字标 NEXT 组合

`CROSS_LOCALIZE, mode=1` 必须同时指定 `next_command=100`（LINEAR）或101
（LINEAR_WITH_YAW），并填写绑定直线的字段。桥接把“武装＋直线”两个帧放在同一次
串口写入，保证没有其他控制帧插入。整个 Action 等待 `B=0x81` 的最终回包，
不会把普通直线回包误认为组合任务完成。

只有 `LEN=19, process=255, notice=0` 且 mode 匹配，result.pose_valid 才为 true；
返回连续 odom 系的捕获位姿。失败和打断不提供有效 pose。
标记 ID、地图、相邻标记变换仍由上层维护。实际捕获需光电和现场标记，尚未在本次中测试。

## 不经过 ROS 的诊断

根目录运行（直接 COM，不能与桥接同时占用）：

```text
python ros2/tools/probe_rotation.py --port COM5 --degrees 15 --rate 0.2 --timeout 8 --log rotation-direct.jsonl
```

该脚本先确认 VelPos 遥测，再发送旋转，记录进度/完成/前后航向，退出时发零并关闭
连续订阅。省略 degrees 时只读遥测；诊断工具限制在 ±30°、0.3rad/s 内。
