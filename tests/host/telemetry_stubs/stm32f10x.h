/* Host boundary for exercising the real timer/telemetry printing logic. */
#ifndef TEST_STM32_TIMER_H
#define TEST_STM32_TIMER_H
#include "sys.h"
typedef struct {
    u32 TIM_Period, TIM_Prescaler, TIM_ClockDivision, TIM_CounterMode;
} TIM_TimeBaseInitTypeDef;
typedef struct {
    u32 NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority;
    u32 NVIC_IRQChannelSubPriority, NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;
#define TIM2 2
#define RCC_APB1Periph_TIM2 2
#define TIM_CKD_DIV1 0
#define TIM_CounterMode_Up 0
#define TIM_IT_Update 1
#define TIM2_IRQn 2
#define ENABLE 1
#define RESET 0
#define RCC_APB1PeriphClockCmd(timer, enable) ((void)(timer), (void)(enable))
#define TIM_TimeBaseInit(timer, cfg) ((void)(timer), (void)(cfg))
#define TIM_ClearITPendingBit(timer, bit) ((void)(timer), (void)(bit))
#define TIM_ITConfig(timer, bit, enable) ((void)(timer), (void)(bit), (void)(enable))
#define NVIC_Init(cfg) ((void)(cfg))
#define TIM_Cmd(timer, enable) ((void)(timer), (void)(enable))
#define TIM_GetITStatus(timer, bit) ((void)(timer), (void)(bit), 1)
#endif
