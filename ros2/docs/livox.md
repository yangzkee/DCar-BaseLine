# Livox 的 ROS 发布订阅方式与 Dcar 对接

研究来源（2026-09-10）：

- [Livox 官方驱动](https://github.com/Livox-SDK/livox_ros_driver2)
- [数据发布实现](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp)
- [MID360 启动示例](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/launch_ROS2/msg_MID360_launch.py)

Livox 驱动经 SDK 接收设备数据，再由 ROS 发布者输出消息。ROS 2 中间件负责节点
发现和消息传输，使用方不必再次连接雷达或解析设备报文。Dcar 也采用这种组织方式：
一个节点读串口，其他进程订阅 `odom`、`imu/data`。

```text
Livox 雷达 → Livox 驱动 → 点云话题 ─┐
                                 ├→ 你的程序 / RViz / rosbag
Dcar 底盘 → Dcar 桥接 → 里程计话题 ─┘
```

跨电脑并非无条件互通：需相同 `ROS_DOMAIN_ID`、可达的网络发现范围、防火墙放行，
以及匹配的话题、类型和 QoS。默认发现不会自动穿越公网/NAT；优先统一 ROS 发行版
和 RMW。点云和底盘组合使用还需时间同步、`base_link → livox_frame` 外参。

先用 `ros2 topic list`、`ros2 topic info <话题> --verbose`、
`ros2 topic echo <话题> --once` 分别定位发现、类型和数据问题。

Livox 常用输出包括 `sensor_msgs/msg/PointCloud2` 和自己的 `CustomMsg`。
MID360 示例 `xfer_format=1` 选自定义消息，0 选 PointCloud2。
消费者需支持实际消息类型，并检查点时间等字段的语义；CustomMsg 订阅端需要匹配
的消息定义。Dcar 首版使用标准消息，减少客户端额外安装。

当前官方 livox_ros_driver2 README 列出的构建环境为 Ubuntu（含 ROS 2 Jazzy）。
**ROS 2 支持 Windows 不等于 Livox 官方驱动已支持 Windows。** 本次未移植雷达
驱动或安装 SDK，具体型号尚待确认。

可让 Dcar 驱动在 Windows 运行、Livox 驱动在另一台受支持设备运行，再经 ROS 2
网络订阅。若雷达也必须在本机 Windows 运行，需要单独验证该型号的 SDK、驱动构建
和网卡收包，不能仅修改 launch 文件就保证可用。

本次仅借鉴发布订阅组织方式，未复制 Livox 代码或将雷达 SDK 设为底盘依赖。
