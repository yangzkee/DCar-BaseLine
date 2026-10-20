/* TIM2只维护1ms时基；DFCom_Poll在主循环调度日志，USART2非阻塞发送。 */
#include "DFCom.h"
#include "stm32f10x.h"
#include "usart.h"
#include "stdio.h"

/* ---- 本地 tick (ms) —— DFCom_Rx.c 用来给 ODOM 数据打时间戳 ---- */
volatile u32 g_local_tick_ms = 0;

/* ---- 控制是否自动打印（学生可以临时关掉） ---- */
volatile u8 g_print_odom_enable = 1;
volatile u8 g_print_full_odom_enable = 1;

/* ===========================================================================
 * TIM2 初始化：1ms 中断
 *   APB1 = 36MHz, TIM2 计数频率 = 72MHz / Prescaler
 *   实际Prescaler=72-1，Period=1000-1，1MHz/1000=1kHz
 *   不在TIM2中断内打印。
 * ===========================================================================*/
void Odom_PrintTimer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* 72MHz / 72 = 1MHz, 1MHz / 1000 = 1kHz = 1ms 中断 */
    TIM_TimeBaseStructure.TIM_Period        = 1000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* NVIC 配置 —— 优先级比 USART1（0,2）低一些 */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

/* 中断不调用printf，不访问USART2，不依赖任何遥测开关。 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        g_local_tick_ms++;
    }
}

/* 仅从主循环调用；错过的日志不补打，避免积压影响运动任务。 */
void DFCom_Poll(void)
{
    static u32 last_print, last_diag;
    u32 now;
    USART1_ReceivePoll(); /* 必须在日志开关检查之前，关闭日志仍持续收帧。 */
    now = g_local_tick_ms;
    if (!g_print_odom_enable) return;
    if ((u32)(now - last_print) >= 100) {
        last_print = now;
        if (g_print_full_odom_enable && g_odom.frame_count &&
            (u32)(now - g_odom.local_tick_ms) < 1000) Odom_Print();
    }
    if ((u32)(now - last_diag) >= 1000) {
        last_diag = now;
        printf("[UART1] bytes=%lu dropped=%lu overflows=%lu errors=%lu\r\n",
               (unsigned long)g_usart1_rx_bytes, (unsigned long)g_usart1_rx_dropped,
               (unsigned long)g_usart1_rx_overflows, (unsigned long)g_usart1_rx_errors);
        printf("[RX] odom wire=%lu ok=%lu reject=%lu len=%u ver=%u; "
               "\r\n",
               (unsigned long)g_dfcom_rx_diag.odom_wire, (unsigned long)g_odom.frame_count,
               (unsigned long)g_dfcom_rx_diag.odom_rejected,
               (unsigned)g_dfcom_rx_diag.odom_len, (unsigned)g_dfcom_rx_diag.odom_version);
    }
}





void Odom_Print(void)
{
    /* 取一份"瞬时快照" */
    OdomData_t snap;
    snap.roll_rad     = g_odom.roll_rad;
    snap.pitch_rad    = g_odom.pitch_rad;
    snap.yaw_rad      = g_odom.yaw_rad;
    snap.n_pos_x_m    = g_odom.n_pos_x_m;       /* N 系: 累积位置 */
    snap.n_pos_y_m    = g_odom.n_pos_y_m;
    snap.b_vel_x_mps  = g_odom.b_vel_x_mps;     /* B 系: 瞬时速度 */
    snap.b_vel_y_mps  = g_odom.b_vel_y_mps;
    snap.acc_x_mps2   = g_odom.acc_x_mps2;      /* B 系: IMU 加速度 */
    snap.acc_y_mps2   = g_odom.acc_y_mps2;
    snap.acc_z_mps2   = g_odom.acc_z_mps2;      /* 静止平放 ≈ +9.81 */
    snap.gyro_z_rps   = g_odom.gyro_z_rps;      /* B 系 Z 角速度 */
    snap.frame_count  = g_odom.frame_count;

    if (snap.frame_count == 0) {
        printf("[ODOM] (no data yet, send Cmd_Subscribe_Odom?)\r\n");
        return;
    }

    /* 打印格式说明 (字段名标了坐标系 + 单位, 避免混淆):
     *
     *   Roll/Pitch (deg)   = 姿态欧拉角 (FLU, 平地车 R/P ≈ 0)
     *   Yaw (deg, CCW+)    = 车头方向相对开机初始朝向偏转角度
     *   X(N,fwd) Y(N,left) = N 系 (开机时拍快照, 不跟车转) 累积位置 (m)
     *   Vx(B) Vy(B)        = B 系 (跟车转) 瞬时速度 (m/s)
     *   Ax/Ay/Az (B,m/s²)  = B 系 IMU 加速度 (Az 含重力反作用 ≈ +9.81)
     *   Gz (B,rad/s)       = B 系 Z 轴角速度 (CCW+) */
    printf("[ODOM] R=%6.2f P=%6.2f Yaw=%7.2f X(N,fwd)=%7.3f Y(N,left)=%7.3f "
           "Vx(B)=%5.2f Vy(B)=%5.2f Ax=%5.2f Ay=%5.2f Az=%5.2f Gz=%5.2f fr=%lu\r\n",
           (double)DFCOM_RAD2DEG(snap.roll_rad),
           (double)DFCOM_RAD2DEG(snap.pitch_rad),
           (double)DFCOM_RAD2DEG(snap.yaw_rad),
           (double)snap.n_pos_x_m,
           (double)snap.n_pos_y_m,
           (double)snap.b_vel_x_mps,
           (double)snap.b_vel_y_mps,
           (double)snap.acc_x_mps2,
           (double)snap.acc_y_mps2,
           (double)snap.acc_z_mps2,
           (double)snap.gyro_z_rps,
           (unsigned long)snap.frame_count);
}
