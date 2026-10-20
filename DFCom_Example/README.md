# STM32F103C8T6 完整工程

当前运动接口是 **DFLink队列v6**，固定使用米、弧度、m/s、rad/s。
打开 [USER/Template.uvprojx](USER/Template.uvprojx)，运动代码直接看 [USER/main.c](USER/main.c)。
这是外部控制板程序，不是小车主控固件。

## 目录

- [编译与接线](#section-1)
- [自动循环测试](#section-2)
- [如何改一个动作](#section-3)
- [日志和排查](#section-4)

平台选择、函数速查与分层验收标准见[根 README](../README.md)。

<a id="section-1"></a>
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

<a id="section-2"></a>
## 自动循环测试

无需启用开关：编译烧录后，程序自动诊断、倒计时3秒，持续循环操场路线。
没有 `DEMO_ENABLE_MOTION` 或 `DEMO_ROUTE`，不用切换模式，也不用每圈复位。
正常完成一圈后立即下一圈，无额外圈间等待；故障会取消/零速并停止后续循环。
需要全向/麦克纳姆底盘，程序启动后即可能运动。

<a id="section-3"></a>
## 如何改一个动作

四条实际调用均在main内：倒行半圆R=0.30m、转角π、速度-0.15m/s；后退0.60m；同样倒行半圆；后退0.60m同时旋转2π。
参数固定米、弧度、m/s、rad/s，中档加速度18。修改main中的参数即可，不另设路线模式。
每圈连续提交4个ID，再按ID等待；全部成功后开始下一圈。
完整API、四段验收表和日志见[根README](../README.md#stm32测试)。

<a id="section-4"></a>
## 日志和排查

- `[INIT]`：state_reply=1、imu_calibrated=1、new_velpos_frames>0 才通过启动诊断。
- `[LAP] START n` / `[LAP] DONE n`：圈号持续递增，每圈4条任务全部成功。
- `[QUEUE] ... segment=n/4 ... DONE notice=...`：逐段完成结果；ACK不是完成。
- `[DEMO] FAILED stage=begin/submit/wait`：本次失败，尝试取消/零速后不再发新路线。
- `[VELPOS]`：基础位置与速度；fr应增长。`NO FRESH DATA`为缺失/超过本地1秒未更新，不能算通信通过。
- `[ODOM]`：启动自动尝试订阅；最近本地1秒内有更新时，与VelPos一起打印；不可用时自动保留VelPos，恢复后自动重新打印。

无需手动切换Odom/VelPos。Odom可用时两种帧计数均应增长，Odom额外提供IMU数据。
TIM2维护tick并打印，printf耗时会影响本地计时精度；遥测新鲜度不是硬件看门狗。串口断开时不能保证取消/停车请求送达。
软件DONE只证明车端报告成功；实车各段轨迹、旋转与多圈误差仍须外部测量，验收项目见根README。
