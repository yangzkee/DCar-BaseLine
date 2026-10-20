#ifndef TEST_USART_H
#define TEST_USART_H

#include <stdio.h>
#include "sys.h"

void uart_init(u32 baud);
void usart2_init(u32 baud);
void USART1_ReceivePoll(void);
extern volatile u32 g_usart1_rx_bytes, g_usart1_rx_dropped, g_usart1_rx_overflows, g_usart1_rx_errors;
u8 USART1_Send_By_DMA(u8 *data, u8 size);

#endif
