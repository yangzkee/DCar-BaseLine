/******************************************************************************
 * usart.c - USART1 / USART2 配置
 * ---------------------------------------------------------------------------
 *   USART1 (PA9/PA10) @ 460800
 *     - 与 DcarON 小车通信（DMA RX + DMA TX）
 *     - 循环DMA持续接收，主循环USART1_ReceivePoll解析；不在解析期间停DMA
 *
 *   USART2 (PA2/PA3)  @ 115200
 *     - printf 重定向到这里
 *     - 接 USB-TTL 接电脑串口工具看 ODOM 数据
 *
 *   本客户端只配置USART1和USART2。
 ******************************************************************************/
#include "sys.h"
#include "usart.h"
#include "DFCom.h"

/* ===========================================================================
 * USART1 缓冲区
 * ===========================================================================*/
volatile u8 USART1_RX_BUFF[USART1_RX_BUFF_SIZE];   /* 循环DMA缓冲，主循环只消费，不清空/停用。 */
u8 USART1_TX_BUFF[100];
u8 USART1_cnt;

/* ===========================================================================
 * printf 重定向 → USART2（看终端数据）
 *   注意：标准库下需要 #pragma import(__use_no_semihosting)
 *   Keil 项目里也要勾选 "Use MicroLIB" 才能正常工作
 * ===========================================================================*/
#if defined(__CC_ARM)
#pragma import(__use_no_semihosting)
struct __FILE {
    int handle;
};
FILE __stdout;
void _sys_exit(int x) { x = x; }
#endif

/* 单生产者(主循环) / 单消费者(USART2 IRQ)环形缓冲。
 * 满时丢日志而不等待；未连接无线串口不会阻塞运动。 */
#define LOG_TX_SIZE 2048
static u8 log_tx[LOG_TX_SIZE];
static volatile u16 log_head, log_tail;
volatile u32 g_usart2_log_dropped;

int fputc(int ch, FILE *f)
{
    u16 next = (u16)((log_head + 1) % LOG_TX_SIZE);
    (void)f;
    if (!g_print_odom_enable) return ch;
    if (next == log_tail) {
        g_usart2_log_dropped++;
        return ch;
    }
    log_tx[log_head] = (u8)ch;
    log_head = next;
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    return ch;
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        if (log_tail != log_head) {
            USART_SendData(USART2, log_tx[log_tail]);
            log_tail = (u16)((log_tail + 1) % LOG_TX_SIZE);
        } else {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
    }
}

/* ===========================================================================
 * USART1备用接收状态变量；当前接收路径使用循环DMA
 * ===========================================================================*/
#if EN_USART1_RX
u8  USART_RX_BUF[USART_REC_LEN];
u16 USART_RX_STA = 0;
#endif

/* ===========================================================================
 * USART1 初始化 (与 DcarON 通信)
 *   bound: 推荐 460800（DcarON 默认）
 * ===========================================================================*/
void uart_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* PA9  - USART1_TX  - 复用推挽 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA10 - USART1_RX  - 浮空输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* NVIC */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART 参数 */
    USART_InitStructure.USART_BaudRate            = bound;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* 收帧不依赖IDLE，不读DR清IDLE，避免与DMA竞争接收字节。 */
    USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
    USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
    USART_Cmd(USART1, ENABLE);

    USART1_DMA_Rx_Configuration();
    USART_DMA_Tx_Configuration();
}

/* ===========================================================================
 * USART2 初始化 (printf 终端)
 *   bound: 推荐 115200
 * ===========================================================================*/
void usart2_init(u32 bound)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* USART2 挂在 APB1, GPIOA 在 APB2 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* PA2 - USART2_TX  - 复用推挽 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA3 - USART2_RX  - 浮空输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART 参数 */
    USART_InitStructure.USART_BaudRate            = bound;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    log_head = log_tail = 0;
    g_usart2_log_dropped = 0;
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART2, ENABLE);
}

/* ===========================================================================
 * USART1 DMA RX 配置（DMA1_Channel5）
 * ===========================================================================*/
/* TC仅记录DMA绕回次数；协议解析始终在主循环中运行。 */
static volatile u32 rx_dma_wraps;
static u32 rx_read_total;
static volatile u8 rx_discontinuity;
volatile u32 g_usart1_rx_bytes;
volatile u32 g_usart1_rx_dropped;
volatile u32 g_usart1_rx_overflows;
volatile u32 g_usart1_rx_errors;

void USART1_DMA_Rx_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)USART1_RX_BUFF;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize         = USART1_RX_BUFF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    rx_dma_wraps = 0;
    rx_read_total = 0;
    rx_discontinuity = 0;
    g_usart1_rx_bytes = g_usart1_rx_dropped = 0;
    g_usart1_rx_overflows = g_usart1_rx_errors = 0;
    DFCom_RxReset();
    DMA_ClearFlag(DMA1_FLAG_GL5);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Channel5, ENABLE);
}

/* ===========================================================================
 * USART1 DMA TX 配置（DMA1_Channel4）
 * ===========================================================================*/
void USART_DMA_Tx_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStruct;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (USART1_BASE + 0x04);
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)USART1_TX_BUFF;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStruct.DMA_BufferSize         = 100;
    DMA_Init(DMA1_Channel4, &DMA_InitStruct);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel4, DISABLE);
}

/* DMA持续接收；TC中断只记环回，不能在此处停DMA或解析协议。 */
void DMA1_Channel5_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC5) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_TC5);
        rx_dma_wraps++;
    }
}

/* 只处理硬件错误；正常接收不依赖IDLE中断。 */
void USART1_IRQHandler(void)
{
    u32 status = USART1->SR;
    if (status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE)) {
        (void)USART_ReceiveData(USART1); /* SR->DR清错误；计数不伪装成协议成功。 */
        g_usart1_rx_errors++;
        rx_discontinuity = 1;
    }
}

/* 同时处理“DMA已绕回而TC中断尚未执行”的瞬间，避免计数倒退。
 * 临界区只读寄存器，DMA不会被关闭；不能在这里格式化或解析。
 */
static u32 rx_produced(void)
{
    u32 wraps;
    u32 primask = __get_PRIMASK();
    u16 remaining;
    __disable_irq();
    wraps = rx_dma_wraps;
    remaining = DMA_GetCurrDataCounter(DMA1_Channel5);
    if (DMA_GetFlagStatus(DMA1_FLAG_TC5) != RESET) {
        wraps++;
        remaining = DMA_GetCurrDataCounter(DMA1_Channel5);
        if (remaining == 0) remaining = USART1_RX_BUFF_SIZE;
    }
    if (!primask) __enable_irq();
    return wraps * USART1_RX_BUFF_SIZE + USART1_RX_BUFF_SIZE - remaining;
}

/* 仅由主循环调用；每次最多消费进入时已收到的一个环大小。
 * 在小块副本上解析，解析期间DMA仍能接收下一帧，不受日志开关影响。
 */
void USART1_ReceivePoll(void)
{
    u8 chunk[64];
    u16 i, count;
    u32 target = rx_produced();
    u32 available = target - rx_read_total;
    u32 after_copy;
    g_usart1_rx_bytes = target;
    if (rx_discontinuity) {
        rx_discontinuity = 0;
        DFCom_RxReset();
    }
    if (available > USART1_RX_BUFF_SIZE) {
        g_usart1_rx_dropped += available - USART1_RX_BUFF_SIZE;
        g_usart1_rx_overflows++;
        rx_read_total = target - USART1_RX_BUFF_SIZE;
        available = USART1_RX_BUFF_SIZE;
        DFCom_RxReset();
    }
    while (available) {
        count = available > sizeof(chunk) ? sizeof(chunk) : (u16)available;
        for (i = 0; i < count; i++)
            chunk[i] = USART1_RX_BUFF[(rx_read_total + i) % USART1_RX_BUFF_SIZE];
        after_copy = rx_produced();
        if ((u32)(after_copy - rx_read_total) > USART1_RX_BUFF_SIZE) {
            /* 副本可能已被覆盖，丢弃并等待下次服务，不能混拼成有效帧。 */
            g_usart1_rx_dropped += after_copy - rx_read_total;
            g_usart1_rx_overflows++;
            rx_read_total = after_copy;
            DFCom_RxReset();
            return;
        }
        rx_read_total += count;
        available -= count;
        DFCom_RxParse(chunk, count);
    }
}

/* ===========================================================================
 * USART1 DMA 发送（DFCom_Tx 用）
 *   返回: 0=成功, 1=失败/超时
 * ===========================================================================*/
u8 USART1_Send_By_DMA(u8 *data, u8 size)
{
    u32 start_tick;
    u32 spin_guard;
    u8 i;

    if (data == 0 || size == 0 || size > 100) return 1;

    /* 仅在Channel4启用时等待上一帧TC；启动首帧直接复制并发送。
     * 等待上限10ms，tick未运行时由spin_guard限制等待。
     */
    if ((DMA1_Channel4->CCR & DMA_CCR4_EN) != 0) {
        start_tick = g_local_tick_ms;
        spin_guard = 720000;  /* tick 尚未启动/中断被关时的兜底，避免永久卡死 */

        while (DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET) {
            if ((g_local_tick_ms - start_tick) >= 10 || spin_guard-- == 0) {
                DMA_Cmd(DMA1_Channel4, DISABLE);
                DMA_ClearFlag(DMA1_FLAG_GL4);
                return 1;
            }
        }
    }

    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_GL4);

    for (i = 0; i < size; i++) USART1_TX_BUFF[i] = data[i];

    DMA1_Channel4->CMAR = (uint32_t)USART1_TX_BUFF;
    DMA_SetCurrDataCounter(DMA1_Channel4, size);
    DMA_Cmd(DMA1_Channel4, ENABLE);

    return 0;
}
