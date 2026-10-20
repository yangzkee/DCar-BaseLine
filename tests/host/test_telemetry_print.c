#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "DFCom.h"
volatile OdomData_t g_odom;
volatile VelPosData_t g_velpos;
volatile DFComRxDiag_t g_dfcom_rx_diag;
volatile u32 g_usart1_rx_bytes, g_usart1_rx_dropped, g_usart1_rx_overflows, g_usart1_rx_errors;
static unsigned velpos_lines, odom_lines, stale_lines, rx_polls;
void USART1_ReceivePoll(void) { rx_polls++; }
void TIM2_IRQHandler(void);
int test_printf(const char *format, ...) {
    if (strstr(format, "[VELPOS] NO FRESH DATA")) stale_lines++;
    else if (strstr(format, "[VELPOS]")) velpos_lines++;
    if (strstr(format, "[ODOM]")) odom_lines++;
    return 0;
}
static void tick_at(u32 before) {
    g_local_tick_ms = before;
    {
        unsigned total = velpos_lines + odom_lines + stale_lines;
        TIM2_IRQHandler();
        assert(total == velpos_lines + odom_lines + stale_lines); /* ISR never prints. */
    }
    DFCom_Poll();
}
int main(void) {
    Odom_PrintTimer_Init();
    g_velpos.frame_count = 1;
    g_velpos.local_tick_ms = 100;
    tick_at(99); /* No Odom support: only VelPos, never stale Odom zeros. */
    assert(velpos_lines == 1 && odom_lines == 0 && stale_lines == 0);
    g_odom.frame_count = 1;
    g_odom.local_tick_ms = 100;
    tick_at(199); /* Both streams available. */
    assert(velpos_lines == 2 && odom_lines == 1);
    g_velpos.local_tick_ms = 1100;
    tick_at(1099); /* Odom stops: fall back without a mode switch. */
    assert(velpos_lines == 3 && odom_lines == 1);
    g_odom.local_tick_ms = 1200;
    tick_at(1199); /* Odom resumes automatically. */
    assert(velpos_lines == 4 && odom_lines == 2);
    tick_at(2199); /* Both streams stale: don't print old telemetry as current. */
    assert(stale_lines == 1 && velpos_lines == 4 && odom_lines == 2);
    g_velpos.local_tick_ms = g_odom.local_tick_ms = 0xFFFFFFFFu - 100;
    tick_at(99); /* Unsigned freshness check across tick wrap. */
    assert(velpos_lines == 5 && odom_lines == 3);
    g_print_odom_enable = 0;
    { unsigned before=rx_polls; tick_at(299); assert(rx_polls==before+1); }
    assert(g_local_tick_ms == 300 && velpos_lines == 5 && odom_lines == 3);
    g_print_odom_enable = 1;
    g_print_velpos_enable = 0;
    tick_at(399);
    assert(velpos_lines == 5 && odom_lines == 4);
    g_print_velpos_enable = 1;
    g_print_full_odom_enable = 0;
    tick_at(499);
    assert(velpos_lines == 6 && odom_lines == 4);
    puts("real telemetry timer: dual stream, fallback, stale data, recovery and tick wrap passed");
    return 0;
}
