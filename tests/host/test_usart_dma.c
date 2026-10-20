/* Link the real USART driver and parser. Inject new wire bytes during parsing. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sys.h"
#include "usart.h"
#include "DFCom.h"
MockUSART mock_usart1,mock_usart2;
MockDMA mock_dma4,mock_dma5;
volatile u32 g_local_tick_ms;
volatile u8 g_print_odom_enable = 1;
static u32 dma_flags,rx_mode;
static unsigned hardware_lost,rx_disabled,parse_calls;
static int auto_wrap_irq = 1;
static u8 inject[104];
static u16 inject_size;
void Actual_DFCom_RxParse(u8 *data,u16 size);
void DMA_DeInit(MockDMA *x) { memset(x,0,sizeof(*x)); }
void DMA_Init(MockDMA *x,MockInit *y) { x->CNDTR=(u16)y->DMA_BufferSize; if(x==DMA1_Channel5) rx_mode=y->DMA_Mode; }
void DMA_Cmd(MockDMA *x,u32 enabled) {
    if(enabled) x->CCR|=1; else { x->CCR&=~1u; if(x==DMA1_Channel5) rx_disabled++; }
}
u16 DMA_GetCurrDataCounter(MockDMA *x) { return x->CNDTR; }
void DMA_SetCurrDataCounter(MockDMA *x,u16 count) { x->CNDTR=count; }
u32 DMA_GetFlagStatus(u32 flag) { return dma_flags&flag; }
u32 DMA_GetITStatus(u32 flag) { return dma_flags&flag; }
void DMA_ClearFlag(u32 flag) { dma_flags&=~flag; }
void DMA_ClearITPendingBit(u32 flag) { DMA_ClearFlag(flag); }
u32 USART_GetFlagStatus(MockUSART *x,u32 flag) { return x->SR&flag; }
u32 USART_GetITStatus(MockUSART *x,u32 flag) { (void)x; (void)flag; return 0; }
u16 USART_ReceiveData(MockUSART *x) { x->SR=0; return (u16)x->DR; }
void USART_ClearFlag(MockUSART *x,u32 flag) { x->SR&=~flag; }
void USART_SendData(MockUSART *x,u16 data) { x->DR=data; }
void delay_ms(u32 ms) { g_local_tick_ms+=ms; }
void delay_us(u32 us) { (void)us; }
void DFCom_Poll(void) {
#ifndef DFCOM_EXPECT_LEGACY_GAP
    USART1_ReceivePoll();
#endif
}
static void wire(u8 *data,u16 size) {
    u16 i;
    for(i=0;i<size;i++) {
        if(!(DMA1_Channel5->CCR&1) || DMA1_Channel5->CNDTR==0) { hardware_lost++; continue; }
        USART1_RX_BUFF[USART1_RX_BUFF_SIZE-DMA1_Channel5->CNDTR]=data[i];
        DMA1_Channel5->CNDTR--;
        if(DMA1_Channel5->CNDTR==0) {
            dma_flags|=DMA1_FLAG_TC5;
            if(rx_mode==DMA_Mode_Circular) DMA1_Channel5->CNDTR=USART1_RX_BUFF_SIZE;
            else DMA1_Channel5->CCR&=~1u;
#ifndef DFCOM_EXPECT_LEGACY_GAP
            if(auto_wrap_irq) DMA1_Channel5_IRQHandler();
#endif
        }
    }
}
void DFCom_RxParse(u8 *data,u16 size) {
    u16 pending=inject_size;
    parse_calls++;
#ifndef DFCOM_EXPECT_LEGACY_GAP
    assert(DMA1_Channel5->CCR&1); /* Parser must never stop reception. */
#endif
    inject_size=0;
    if(pending) wire(inject,pending); /* Partner stream arrives while first frame is parsed. */
    Actual_DFCom_RxParse(data,size);
}
static u16 frame(u8 *out,u8 b) {
    u8 len=b==0x80?51:35;
    u16 sum=0;
    unsigned i;
    memset(out,0,len+9);
    out[0]=0xdf; out[1]=DFCOM_PC_ID; out[2]=DFCOM_ROBOT_ID;
    out[3]=0x6c; out[4]=b; out[5]=len; out[6]=b==0x80?4:2;
    out[len+6]=0xfd;
    for(i=0;i<(unsigned)len+7;i++) sum+=out[i];
    out[len+7]=(u8)sum; out[len+8]=(u8)(sum>>8);
    return (u16)(len+9);
}
static void reset(void) {
    DMA1_Channel5->CCR=0; dma_flags=0; auto_wrap_irq=1;
    hardware_lost=rx_disabled=parse_calls=0; inject_size=0;
    memset((void *)&g_odom,0,sizeof(g_odom));
    memset((void *)&g_dfcom_rx_diag,0,sizeof(g_dfcom_rx_diag));
    DFCom_RxReset();
    USART1_DMA_Rx_Configuration();
    rx_disabled=0;
}
int main(void) {
    u8 a[60],b[60],pair[104],noise[USART1_RX_BUFF_SIZE+100];
    u16 an,bn;
    unsigned order,i,prefix;
    (void)pair; (void)noise; (void)i;
    for(order=0;order<2;order++) {
        for(prefix=1;prefix<=3;prefix++) {
            reset(); an=frame(a,order?0x81:0x80); bn=frame(b,order?0x80:0x81);
            wire(a,an);
            memcpy(inject,b,prefix); inject_size=(u16)prefix;
            USART1->SR|=USART_FLAG_IDLE;
            USART1_IRQHandler();
#ifdef DFCOM_EXPECT_LEGACY_GAP
            assert(hardware_lost==prefix);
            wire(b+prefix,(u16)(bn-prefix));
            USART1->SR|=USART_FLAG_IDLE; USART1_IRQHandler();
            assert(g_odom.frame_count==(order?0:1));
#else
            assert(parse_calls==0);
            USART1_ReceivePoll();
            wire(b+prefix,(u16)(bn-prefix));
            USART1_ReceivePoll();
            assert(g_odom.frame_count==1);
            assert(hardware_lost==0 && rx_disabled==0);
#endif
        }
    }
#ifdef DFCOM_EXPECT_LEGACY_GAP
    puts("REPRODUCED: baseline stops DMA during parse and loses 1..3 partner header bytes and rejects that stream, either order");
#else
    reset(); an=frame(pair,0x80); bn=frame(pair+an,0x81);
    /* Continuous joined frames span many circular wraps without IDLE events. */
    for(i=0;i<100;i++) { wire(pair,an+bn); USART1_ReceivePoll(); }
    assert(g_odom.frame_count==100 && rx_disabled==0);
    for(i=0;i<(unsigned)(an+bn);i++) { wire(pair+i,1); USART1_ReceivePoll(); }
    assert(g_odom.frame_count==101);
    assert(g_usart1_rx_overflows==0 && g_usart1_rx_dropped==0);
    reset(); memset(noise,0,sizeof(noise)); auto_wrap_irq=0;
    wire(noise,USART1_RX_BUFF_SIZE); /* TC pending, IRQ not yet serviced, write index==0. */
    USART1_ReceivePoll();
    assert(g_usart1_rx_bytes==USART1_RX_BUFF_SIZE && g_usart1_rx_overflows==0);
    DMA1_Channel5_IRQHandler(); auto_wrap_irq=1;
    USART1_ReceivePoll(); /* Pending TC must not be counted twice. */
    wire(pair,an+bn); USART1_ReceivePoll();
    assert(g_usart1_rx_bytes==(u32)USART1_RX_BUFF_SIZE+an+bn);
    assert(g_odom.frame_count==1);
    reset();
    wire(noise,sizeof(noise)); wire(pair,an+bn); USART1_ReceivePoll();
    assert(g_usart1_rx_overflows==1 && g_usart1_rx_dropped==100u+an+bn);
    assert(g_odom.frame_count==1);
    reset();
    wire(pair,20); USART1_ReceivePoll(); /* Partial Odom frame before a hardware gap. */
    USART1->SR|=USART_FLAG_ORE; USART1_IRQHandler();
    wire(pair,an+bn); USART1_ReceivePoll();
    assert(g_usart1_rx_errors==1 && g_odom.frame_count==1);
    assert(rx_disabled==0 && hardware_lost==0);
    puts("PASS: live DMA during parse, both orders, joined/fragmented frames, wraps, pending TC, overflow and ORE recovery");
#endif
    return 0;
}
