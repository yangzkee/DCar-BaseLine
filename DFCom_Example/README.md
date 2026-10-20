# STM32F103C8T6 完整工程

官网：[differ-tech.pages.dev](https://differ-tech.pages.dev)

淘宝搜索 **DCar**，认准 **“点浮科技”** 购买。

外部控制板程序，使用 DFLink 队列 v6。打开 [USER/Template.uvprojx](USER/Template.uvprojx)，实际运动直接看 [USER/main.c](USER/main.c)。

## 目录

- [编译、下载与接线](#编译下载与接线)
- [自动操场循环](#自动操场循环)
- [收帧、日志与独立功能](#收帧日志与独立功能)
- [诊断与验收](#诊断与验收)

## 编译、下载与接线

使用 Keil MDK5 / ARM Compiler 5，启用 Use MicroLIB。工程目标 STM32F103C8，Flash `0x08000000/0x10000`，RAM `0x20000000/0x5000`。
在 Debug / Utilities 选择自己的下载器和 SWD，使用与实物芯片相符的配置；BOOT0=0、建议 BOOT1=0。工程不绑定个人 ST-Link 序列号。

| 引脚 | 连接 | 参数 |
|---|---|---|
| PA9 / PA10 | USART1 TX→车 RX、RX←车 TX，GND 共地 | 460800，8N1，3.3V TTL |
| PA2 | USART2 TX→USB-TTL RX，GND 共地 | 115200，8N1，printf 日志 |

车端应先激活 / 校准，并支持队列 v6。本例程自动运动，需要全向 / 麦克纳姆底盘和可运行操场的场地。

## 自动操场循环

烧录后无须改模式或运动开关。上电两次零速之间等待1000ms，独立订阅Odom和VelPos（间隔20ms），固定观察500ms，等待50ms启用业务会话，再倒计时3000ms。
状态 / 遥测缺失只报告，不延长观察窗口、不阻止运动；队列能力、空闲状态、ACK、DONE和超时仍必须通过。

每圈连续入队四条：半径0.30m倒行半圆 → 后退0.60m → 同样倒行半圆 → 后退0.60m同时转2π。速度请求0.15m/s，加速度档位18。
四条均成功后立即开始下一圈，无圈间延时，无人工触发。每次Wait上限60000ms，从本次调用计时。
失败打印 `FAILED stage=begin/submit/wait`，尝试取消 / 零速后停止新任务，继续收帧。排除故障后复位。
具体动作和物理测量标准见[根README验收表](../README.md#验收标准)。

## 收帧、日志与独立功能

USART1用1024字节循环DMA持续接收；DMA TC中断记录环回，USART错误中断记录错误，主循环 `DFCom_Poll()` 调用 `USART1_ReceivePoll()` 分块解析。解析期间DMA保持接收。
TIM2只维护1ms tick，不打印、不解析。主循环和等待循环调用 Poll；默认延时每5ms服务，队列等待每1ms服务。
USART2使用2048字节非阻塞发送环和TXE中断；满时丢日志并累计 `g_usart2_log_dropped`，不会等待日志端。

main默认开启Odom订阅、VelPos订阅、状态查询和日志，四项独立。关闭任一订阅或日志不会停止运动；日志关闭也必须继续Poll收帧。
日志总开关为 `g_print_odom_enable`，`g_print_velpos_enable/g_print_full_odom_enable` 分别控制两种输出。这些是业务定制入口，运行现成测试无需修改。

`g_odom` 与 `g_velpos` 分别保存真实数据；不会交替订阅或复制另一种数据作为本流。Odom最近1秒有更新时与VelPos一起打印，Odom缺失只用VelPos；VelPos超时显示 `NO FRESH DATA`。
解析与打印均在主循环，串口IRQ只处理硬件状态。不要在ISR再次喂同一串字节。

## 诊断与验收

- `[INIT]`：状态、校准和两类帧数，仅报告。
- `[QUEUE] ... segment=n/4 ... DONE notice=...`：逐段结果；入队ACK不算完成。
- `[LAP] START/DONE n`：完成圈数持续增长，整圈四段都成功才计数。
- `[UART1] bytes/dropped/overflows/errors`：字节与接收异常，后三项应保持0。
- `[RX] ... wire/ok/reject/len/ver`：外层有效帧、成功解析、负载拒绝和最近格式。Odom要求len>=51、ver=4；VelPos要求len>=35、ver=2。
- 两类wire与ok应同时持续增长至少60秒，多次复位检查；只有一种更新不能当双流通过。

`wire`增长但`ok`不增长时查看负载长度 / 版本；`wire=0`表示未收到该类型有效帧。
接收服务超过缓冲可覆盖的时间未运行时，驱动记录溢出、丢弃残帧并从完整有效帧重新同步。1024字节约容纳22ms满速流量，业务必须按默认频率服务Poll。
任务DONE不代替实车轨迹精度验收；遥测新鲜度提示不是硬件停车保障，断线时不能确认取消 / 零速送达。


## 2026-10-20 统一 Odometry

STM32 和 ROS 默认统一订阅 `A=0x04/B=0x80`，回传 `A=0x6C/B=0x80`（v4、51B）。已删除其 VelPos 0x81 订阅和解析入口；下文历史双流描述以本节为准。保留新鲜度、DMA接收、队列任务ID及连续操场功能。Pro返回完整数据，Std的roll/pitch、三轴加速度、三轴角速度填0；Mini/SE不开放里程计。ROS只有明确设置 `publish_imu: true` 才发布 IMU，默认false以免把Std填零当作测量。BDB的A02/B81定位命令不受影响。
