#ifndef DRIVER_TEST_SYS_H
#define DRIVER_TEST_SYS_H
#include "../stubs/sys.h"
#include <stdio.h>
#include <string.h>
#define ENABLE 1
#define DISABLE 0
#define RESET 0
typedef struct {
    u32 GPIO_Pin;
    u32 GPIO_Speed;
    u32 GPIO_Mode;
    u32 NVIC_IRQChannel;
    u32 NVIC_IRQChannelPreemptionPriority;
    u32 NVIC_IRQChannelSubPriority;
    u32 NVIC_IRQChannelCmd;
    u32 USART_BaudRate;
    u32 USART_WordLength;
    u32 USART_StopBits;
    u32 USART_Parity;
    u32 USART_HardwareFlowControl;
    u32 USART_Mode;
    u32 DMA_PeripheralBaseAddr;
    u32 DMA_MemoryBaseAddr;
    u32 DMA_DIR;
    u32 DMA_BufferSize;
    u32 DMA_PeripheralInc;
    u32 DMA_MemoryInc;
    u32 DMA_PeripheralDataSize;
    u32 DMA_MemoryDataSize;
    u32 DMA_Mode;
    u32 DMA_Priority;
    u32 DMA_M2M;
} MockInit;
typedef MockInit GPIO_InitTypeDef;
typedef MockInit USART_InitTypeDef;
typedef MockInit DMA_InitTypeDef;
typedef MockInit NVIC_InitTypeDef;
typedef struct { u32 SR, DR; } MockUSART;
typedef struct { u32 CCR, CMAR; u16 CNDTR; } MockDMA;
extern MockUSART mock_usart1,mock_usart2;
extern MockDMA mock_dma4,mock_dma5;
#define USART1 (&mock_usart1)
#define USART2 (&mock_usart2)
#define DMA1_Channel4 (&mock_dma4)
#define DMA1_Channel5 (&mock_dma5)
#define GPIOA ((void *)0)
#define USART_IT_TXE 100
#define RCC_APB2Periph_USART1 102
#define RCC_APB2Periph_GPIOA 103
#define RCC_APB2Periph_AFIO 104
#define GPIO_Pin_9 105
#define GPIO_Speed_50MHz 106
#define GPIO_Mode_AF_PP 107
#define GPIO_Pin_10 108
#define GPIO_Mode_IN_FLOATING 109
#define USART1_IRQn 110
#define USART_WordLength_8b 111
#define USART_StopBits_1 112
#define USART_Parity_No 113
#define USART_HardwareFlowControl_None 114
#define USART_Mode_Rx 115
#define USART_Mode_Tx 116
#define USART_IT_IDLE 117
#define USART_IT_ERR 118
#define RCC_APB1Periph_USART2 120
#define GPIO_Pin_2 121
#define GPIO_Pin_3 122
#define USART2_IRQn 123
#define RCC_AHBPeriph_DMA1 125
#define DMA_DIR_PeripheralSRC 126
#define DMA_PeripheralInc_Disable 127
#define DMA_MemoryInc_Enable 128
#define DMA_PeripheralDataSize_Byte 129
#define DMA_MemoryDataSize_Byte 130
#define DMA_Mode_Circular 1
#define DMA_Priority_VeryHigh 132
#define DMA_M2M_Disable 133
#define DMA1_FLAG_GL5 240
#define DMA_IT_TC 135
#define DMA1_Channel5_IRQn 136
#define USART_DMAReq_Rx 137
#define USART1_BASE 138
#define DMA_Priority_High 139
#define DMA_DIR_PeripheralDST 140
#define DMA_Mode_Normal 0
#define USART_DMAReq_Tx 142
#define DMA1_IT_TC5 32
#define USART_FLAG_ORE 8
#define USART_FLAG_NE 4
#define USART_FLAG_FE 2
#define DMA1_FLAG_TC5 32
#define DMA_CCR4_EN 1
#define DMA1_FLAG_TC4 2
#define DMA1_FLAG_GL4 15

static inline void RCC_APB2PeriphClockCmd(u32 x,u32 y) { (void)x; (void)y; }
static inline void RCC_APB1PeriphClockCmd(u32 x,u32 y) { (void)x; (void)y; }
static inline void RCC_AHBPeriphClockCmd(u32 x,u32 y) { (void)x; (void)y; }
static inline void GPIO_Init(void *x,MockInit *y) { (void)x; (void)y; }
static inline void NVIC_Init(MockInit *x) { (void)x; }
static inline void USART_Init(MockUSART *x,MockInit *y) { (void)x; (void)y; }
static inline void USART_ITConfig(MockUSART *x,u32 y,u32 z) { (void)x; (void)y; (void)z; }
static inline void USART_Cmd(MockUSART *x,u32 y) { (void)x; (void)y; }
static inline void USART_DMACmd(MockUSART *x,u32 y,u32 z) { (void)x; (void)y; (void)z; }
static inline void DMA_ITConfig(MockDMA *x,u32 y,u32 z) { (void)x; (void)y; (void)z; }
void DMA_DeInit(MockDMA *x);
void DMA_Init(MockDMA *x,MockInit *y);
void DMA_Cmd(MockDMA *x,u32 enable);
u16 DMA_GetCurrDataCounter(MockDMA *x);
void DMA_SetCurrDataCounter(MockDMA *x,u16 count);
u32 DMA_GetFlagStatus(u32 flag);
u32 DMA_GetITStatus(u32 flag);
void DMA_ClearFlag(u32 flag);
void DMA_ClearITPendingBit(u32 flag);
u32 USART_GetFlagStatus(MockUSART *x,u32 flag);
u32 USART_GetITStatus(MockUSART *x,u32 flag);
u16 USART_ReceiveData(MockUSART *x);
void USART_ClearFlag(MockUSART *x,u32 flag);
void USART_SendData(MockUSART *x,u16 data);
#define USART_FLAG_IDLE 16
#endif
