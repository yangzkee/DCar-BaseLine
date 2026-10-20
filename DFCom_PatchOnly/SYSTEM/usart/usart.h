#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "sys.h"

/* ===========================================================================
 * USART 配置说明
 *   USART1 (PA9/PA10) @ 460800 → 接 DcarON
 *   USART2 (PA2/PA3)  @ 115200 → 接 USB-TTL 看 printf 输出
 * ===========================================================================*/

#define USART_REC_LEN   200
#define EN_USART1_RX    1

extern u8  USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;

/* USART1（与 DcarON 通信） */
void uart_init(u32 bound);
void USART1_DMA_Rx_Configuration(void);
void USART_DMA_Tx_Configuration(void);
u8   USART1_Send_By_DMA(u8 *data, u8 size);
void USART1_IRQHandler(void);

/* 循环DMA持续接收，不在解析时停用；1024字节约容纳22ms满速460800流量。
 * 主循环至少每5ms调用USART1_ReceivePoll（DFCom_Poll已包含），禁止在ISR调用。
 * 溢出计数为字节，不代表帧；可从下一完整有效帧重新同步。
 */
#define USART1_RX_BUFF_SIZE 1024
void USART1_ReceivePoll(void);
void DMA1_Channel5_IRQHandler(void);
extern volatile u32 g_usart1_rx_bytes;
extern volatile u32 g_usart1_rx_dropped;
extern volatile u32 g_usart1_rx_overflows;
extern volatile u32 g_usart1_rx_errors;
extern volatile u8 USART1_RX_BUFF[USART1_RX_BUFF_SIZE];
extern u8 USART1_TX_BUFF[100];
extern u8 USART1_cnt;

/* USART2（printf 终端） */
void usart2_init(u32 bound);
void USART2_IRQHandler(void);
extern volatile u32 g_usart2_log_dropped; /* 日志缓冲满的丢字节数，不影响车端TX/RX。 */

#endif
