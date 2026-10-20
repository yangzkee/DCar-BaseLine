# STM32F103C8T6 完整工程

当前运动接口是 **DFLink队列v6**，固定使用米、弧度、m/s、rad/s。
打开 [USER/Template.uvprojx](USER/Template.uvprojx)，运动代码直接看 [USER/main.c](USER/main.c)。
这是外部控制板程序，不是小车主控固件。

## 编译与接线

使用 Keil MDK5 / ARM Compiler 5，Target中启用 **Use MicroLIB**。
板卡为STM32F103C8T6；工程的标准库、启动文件和系统时钟已包含。

| STM32引脚 | 连接 | 波特率 |
|---|---|---|
| PA9 / USART1 TX | 车端DFLink RX | 460800，8N1 |
| PA10 / USART1 RX | 车端DFLink TX | 460800，8N1 |
| GND | 车端GND | 必须共地 |
| PA2 / USART2 TX | USB-TTL RX | 115200，8N1，printf终端 |
| GND | USB-TTL GND | 必须共地 |

串口为3.3V TTL。车端运动前须激活/校准，并支持队列v6。

## 怎么启用示例

主函数顶部只有两个运行配置：

```c
#define DEMO_ENABLE_MOTION 0
#define DEMO_ROUTE DEMO_FORWARD
```

默认0：启动时尝试停车两次，检查状态和标准VelPos回传，打印 `DISABLED`，不会跑路线。
确认诊断通过后改为1，重新编译烧录。程序等3秒，确认v6能力和车端队列空闲，执行所选路线一次；结束后继续打印遥测，不循环跑。

| 路线 | 实际动作 | 修改位置 |
|---|---|---|
| `DEMO_FORWARD` | 前进0.10m，速度0.06m/s | main内的单条 `DFQueue_Linear` |
| `DEMO_SQUARE` | 前进0.50m、左转π/2，重复四次 | main内的for循环 |
| `DEMO_STADIUM` | 两段倒行半圆及两段后退，最后边后退边转2π | main内的四条调用，要求全向底盘 |

`DEMO_SPEED_MPS` 控制两条完整路线的请求速度；入门直线单独写为0.06m/s。
`DEMO_GRADE`默认 `DF_ACCEL_MEDIUM`（18）。只要修改主函数就能看清动作；没有隐藏在头文件中的路线实现。

## 如何改一个动作

```c
id = DFQueue_Linear(0.50f, 0, 0.15f, 0, DF_ACCEL_MEDIUM);  /* 前进50cm */
id = DFQueue_Rotate(1.5707963f, 0.50f);                   /* 左转90度 */
id = DFQueue_Arc(0.30f, 3.1415927f, -0.15f, DF_ACCEL_MEDIUM); /* 倒行半圆 */
```

这些是调用示意，实际使用须先成功 `DFQueue_Begin`、检查每个id非零，并按id等待结果。
`DFQueue_Linear(x,y,speed,yaw,grade)`：x/y为每段起点的相对位移，speed必须为正；负x表示后退。yaw=0只平移，非零表示边平移边转向。
`DFQueue_Rotate(yaw,omega)`：yaw正左转、负右转，omega为正。
`DFQueue_Arc(radius,yaw,signed_speed,grade)`：半径为正，负速度倒行。

先发多条再Wait就是排队；发一条立即Wait就是逐段执行。最多8个本地槽，成功Wait后释放。
`DFQueue_Wait(id, timeout_ms)`从本次调用开始计时，0无效；返回 `DF_QUEUE_WAIT_DONE` 才能认为该任务成功。
任务入队、到位微调、最终完成是不同状态，详见 [协议与状态说明](../docs/DFLink-v6.md)。

## 日志和排查

- `[INIT] state_reply=... imu_calibrated=... new_velpos_frames=...`：串口回包、校准、遥测是否正常。
- `[DEMO] DISABLED`：程序主动处于只诊断模式，按提示修改main。
- `[DEMO] FAILED stage=begin/submit/wait reason=... notice=...`：对应握手、提交、等待阶段，失败不自动重跑。
- `[DEMO] DONE`：车端报告整组任务成功，不等于外部测量已验证轨迹精度。
- `[VELPOS]`：位置为米、速度为m/s、打印Yaw为度；`g_velpos.yaw_rad`仍是弧度。

默认仅订阅标准VelPos，不依赖Pro全量Odom。需要完整IMU时，可订阅Odom并在 `DFCom_Print.c` 选择 `Odom_Print`，还须车端支持。
关闭打印可设置 `g_print_odom_enable=0`，不要关闭TIM2 tick；队列超时依赖tick和串口中断。
TIM2当前在中断中打印，打印耗时会影响本地毫秒计时精度，不用于高精度主机计时。

无任务ID的 `Cmd_Move_*` / `WaitMoveDone` 仍有明确声明，供独立调用场景使用，不能夹入活动队列；其单位开关不作用于本页示例。
完整仓库入口、平台边界及检查命令见 [根README](../README.md)。
