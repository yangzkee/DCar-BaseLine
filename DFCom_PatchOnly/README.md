# STM32 通信核心移植包

这里提供与 [完整工程](../DFCom_Example/README.md) 相同的通信核心，**不是独立可编译工程**。
本目录不提供含缺失依赖的Keil工程文件。`USER/main.c`是集成示例；需要由你的工程提供启动文件、STM32标准库、时钟和系统辅助代码。
当前运动示例使用队列v6，所有 `DFQueue_*` 参数固定为米、弧度、m/s、rad/s。

## 目录

- [必须接入的部分](#section-1)
- [初始化与业务入口](#section-2)
- [调度与独立诊断](#调度与独立诊断)

平台选择、函数速查与分层验收标准见[根 README](../README.md)。

<a id="section-1"></a>
## 必须接入的部分

1. 将 `HARDWARE/DFCom.h`、`DFCom_Tx.c`、`DFCom_Rx.c`、`DFCom_Print.c` 加入工程。
2. 接入 `SYSTEM/usart/usart.c/.h`，或提供等效的双向串口。`USART1_Send_By_DMA` 返回0成功、非0失败；成功时须在返回前复制发送字节，不能保留调用者栈指针。
3. USART1使用1024字节循环DMA；接入DMA1_Channel5完成中断与USART1错误中断。主循环至少每5ms调用 DFCom_Poll()，其中USART1_ReceivePoll消费DMA并调用解析器。不要在IDLE中断停DMA后解析，也不要主循环与ISR重复喂同一字节；若用自有串口驱动，提供等效USART1_ReceivePoll入口和接收计数。保留地址/帧尾/校验和验证。
4. 启用TIM2的1ms中断并维护 `g_local_tick_ms`，或用自己的单调毫秒tick替代。若替代，不要同时链接两个tick定义或两个TIM2中断。
5. 提供 `delay_ms` / `delay_us`。`Begin`、提交和 `Wait` 在主循环调用，等待期间必须保持串口接收和tick中断运行。
6. 若保留printf，配置USART2、MicroLIB及非阻塞重定向（含USART2 TXE中断），在主循环定期调用 DFCom_Poll()，队列等待内部已调用；若移除打印，仍须保留队列需要的毫秒tick。

USART1车端口460800，PA9 TX → 车RX、PA10 RX ← 车TX、GND共地。
USART2调试口115200，PA2 TX → USB-TTL RX。车端须支持队列v6、完成激活/校准。

<a id="section-2"></a>
## 初始化与业务入口

按 `USER/main.c` 的次序：初始化 → 隔离运动回包 → 发零速 → 等待1000ms → 再发零速 → 独立订阅并查询状态 → 固定观察500ms → 等50ms → 开始接收业务回包 → 等3000ms → 成功 `DFQueue_Begin` → 发动作 → 按ID等待。
集成后固定观察状态/遥测并持续循环操场四段，无运动开关或模式选择；每圈成功后立即开始下一圈。所有运动调用都在main内。自动同时订阅VelPos/Odom，Odom无回传时只使用VelPos；逐段与整圈验收见[根README](../README.md#stm32测试)。

```c
u16 id;
if (DFQueue_Begin()) {
    id = DFQueue_Linear(0.10f, 0, 0.06f, 0, DF_ACCEL_MEDIUM);
    if (!id || DFQueue_Wait(id, 10000) != DF_QUEUE_WAIT_DONE) {
        printf("%s notice=0x%02X\r\n", DFQueue_LastError(), DFQueue_LastNotice());
        DFQueue_Cancel();
    }
} else {
    printf("begin failed: %s\r\n", DFQueue_LastError());
}
```

先排队再Wait最多跟踪8条；成功Wait释放槽。`Wait`时限必须大于0，从调用时开始计时。
取消后下一批须重新Begin。串口断开时，取消/停车指令不能保证到达。
无任务ID运动接口与队列接口不能混发；`g_dfcom_unit_mode`不影响队列。

公共文件在两份STM32目录中保持一致，由 `python3 tests/run_checks.py` 检查。
API与状态字段见 [DFCom.h](HARDWARE/DFCom.h)，详细单位和报文见 [DFLink队列v6](../docs/DFLink-v6.md)。

## 调度与独立诊断

上电1000ms、订阅间隔20ms、观察500ms、会话前50ms和运动前3000ms均保留，等待期间也服务Poll。
观察窗口固定结束，Odom / VelPos / 状态缺失不阻止运动；队列握手、ACK、DONE及任务时限必须通过。
日志总开关不影响接收，USART2日志需非阻塞发送，移植时避免在接收/计时中断printf。
两类数据分别保存与计数，不交替订阅、不复制；Odom可用时两类同时处理，否则VelPos单独工作。
驱动调度、全部计数与日志说明见[完整工程](../DFCom_Example/README.md#收帧日志与独立功能)，逐段及双流验收见[根README](../README.md#验收标准)。
