#ifndef TEST_USART_H
#define TEST_USART_H

#include <stdio.h>
#include "sys.h"

void uart_init(u32 baud);
void usart2_init(u32 baud);
u8 USART1_Send_By_DMA(u8 *data, u8 size);

#endif
