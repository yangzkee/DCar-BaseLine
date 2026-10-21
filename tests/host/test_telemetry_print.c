#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "DFCom.h"
volatile OdomData_t g_odom;
volatile DFComRxDiag_t g_dfcom_rx_diag;
volatile u32 g_usart1_rx_bytes, g_usart1_rx_dropped, g_usart1_rx_overflows, g_usart1_rx_errors;
static unsigned odom_lines, stale_lines, rx_polls;
void USART1_ReceivePoll(void) { rx_polls++; }
void TIM2_IRQHandler(void);
int test_printf(const char *format, ...) {
    if (strstr(format, "[ODOM] NO FRESH DATA")) stale_lines++;
    else if (strstr(format, "[ODOM]")) odom_lines++;
    return 0;
}
static void tick_at(u32 before) {
    g_local_tick_ms = before;
    {
        unsigned total = odom_lines + stale_lines;
        TIM2_IRQHandler();
        assert(total == odom_lines + stale_lines); /* ISR never prints. */
    }
    DFCom_Poll();
}
int main(void) {
    Odom_PrintTimer_Init();
    tick_at(99); /* No first sample: report missing data. */
    assert(stale_lines==1 && odom_lines==0);
    g_odom.frame_count=1; g_odom.local_tick_ms=100;
    tick_at(199); assert(odom_lines==1);
    tick_at(1099); assert(stale_lines==2 && odom_lines==1);
    g_odom.local_tick_ms=1200;
    tick_at(1199); assert(odom_lines==2);
    g_odom.local_tick_ms=0xFFFFFFFFu-100;
    tick_at(99); assert(odom_lines==3); /* Tick wrap stays fresh. */
    g_print_odom_enable=0;
    { unsigned before=rx_polls; tick_at(299); assert(rx_polls==before+1); }
    assert(odom_lines==3);
    g_print_odom_enable=1; tick_at(399); assert(odom_lines==4);
    puts("Odom timer: freshness, missing data, recovery, tick wrap and logging independence passed");
    return 0;
}
