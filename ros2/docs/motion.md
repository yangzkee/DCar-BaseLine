# 当前运动接口与轨迹格式

默认 `motion_protocol: queue`，运动能力为DFLink队列v6。先阅读 [ROS快速开始](../README.md)。
所有输入使用m、rad、m/s、rad/s；位移与转角相对每段起点，正X前、正Y左、正yaw逆时针。

## 持续操场测试

`ros2 launch dcaron_bridge stadium_test.launch.py port:=COM5` 同时启动桥接和持续操场客户端。
每圈四段入队、整圈成功后立即下一圈，任一圈失败结束测试；不需额外执行参数或切换模式。
Odom和状态仅独立报告，不作为运动启动条件；任务握手、ACK/DONE与自身超时必须通过。
下方单动作和YAML入口用于自定义应用。

## 单动作 /motion

| command | 含义 | 必填运动字段 |
|---|---|---|
| 99 / 0x63 | 原地旋转 | yaw、正angular_speed、profile |
| 100 / 0x64 | 直线或平移并转向 | x/y、正speed、profile；需要同时旋转时填yaw |
| 101 / 0x65 | 平移转向输入别名 | x/y、yaw、正speed、profile；线上编码仍是0x64 |
| 102 / 0x66 | 圆弧 | 正radius、yaw、非零speed、profile；负speed倒行 |

profile必须显式填16/17/18/19/20，示例用18。旋转帧不发送该字段，但ROS输入仍校验它。
timeout单位秒，0使用配置的motion_timeout，正值不能超过max_motion_timeout。

前进10cm：

```text
ros2 action send_goal /motion dcaron_interfaces/action/Motion "{command: 100, x: 0.1, speed: 0.06, profile: 18, timeout: 10.0}" --feedback
```

原地左转90°：

```text
ros2 action send_goal /motion dcaron_interfaces/action/Motion "{command: 99, yaw: 1.5707963268, angular_speed: 0.5, profile: 18, timeout: 10.0}" --feedback
```

## 整组 /trajectory

Goal包含 `segments`（1..128个MotionSegment）、`sequential`（默认false）、`timeout`（整组秒数，0用节点默认值）。
默认最多8条在途；每条发送等入队确认，不等完成即可继续填队列。sequential=true时每段成功才发下一段。
Feedback包含task_id、segment_index（从0计）、process、notice和percent；Result包含success、notice、message和completed（成功段数）。

`trajectory_sequence`使用这样的YAML：

```yaml
timeout: 60.0
segments:
  - {command: 100, x: 0.5, speed: 0.15, profile: 18}
  - {command: 99, yaw: 1.5707963267948966, angular_speed: 0.5, profile: 18}
```

缺省profile由此客户端填18；直接ROS Action不会替你填。
不加 `--execute` 只离线检查；加它执行一次，加 `--sequential`逐段执行。
完整路线在 [正方形](../dcaron_bridge/examples/square.yaml) 和 [操场](../dcaron_bridge/examples/stadium.yaml)。
这不是Nav2路径跟踪器，也不保证段间恒速；车端负责闭环、加减速、曲率限制和末端微调。

## 结果与异常

`process=0/notice=0x10`仅入队；`254/0x25`仍微调；`255/0`或`255/0x27`才成功。
`255/0x26`超差失败，其他错误notice保留原值；本地主机取消/超时/失败使用notice=254，并在message说明原因。
同一task ID重复完成不会重复计数。ACK丢失重发原ID原字节；完成丢失查原ID结果，不创建新动作。

队列开始时要求设备支持v6且队列为空。目标参数、串口连接或占用检查不通过会拒绝；两个Action不会并发执行。
取消、超时或故障尝试取消并发零速；断线不能保证停车，重连不自动重放。
关闭发送目标的终端不等于取消车端已接受目标。

## 专用接口和诊断工具

十字标通过 `/motion` 的128 CROSS_RECORD和129 CROSS_LOCALIZE调用，不支持Trajectory。
mode=0 FIRST、1 NEXT；CROSS_LOCALIZE/NEXT须绑定next_command=100或101，并填写位移字段。
该专用路径使用无ID编码和profile=0/1/2；需光电与底盘固件支持，不能夹入活动队列。
成功捕获回包才设置pose_valid，位姿是连续odom系，不是地图定位真值。

`motion_sequence`是逐个调用 `/motion` 的辅助客户端，输入为YAML列表而不是Trajectory的字典格式。
[小角度示例](../dcaron_bridge/examples/rotation_probe.yaml)适用它；默认按queue校验，只有显式 `--protocol legacy` 才用无ID校验，并须匹配节点配置。

`tools/probe_rotation.py`直接占用串口，`tools/probe_ros_rotation.py`自带桥接节点，两者当前都使用队列v6。
不填 `--degrees` 只读遥测；需要运动时限制在±30°内，均须 `--log 文件.jsonl`。
不能和另一个串口桥接/调试进程同时运行。它们是诊断入口，不构成实车精度认证。
