/******************************************************************************
 * DFCom_Print.c - 终端打印 + 本地 tick
 * ---------------------------------------------------------------------------
 * 提供两个东西：
 *   1. g_local_tick_ms：本地毫秒计数（TIM2 1ms 中断维护，给 WaitMoveDone 等用）
 *   2. Odom_Print()：把 g_odom 打到 USART2（printf 已重定向到 USART2）
 *      TIM2 计数到 100ms 时自动调用一次（10Hz 刷屏）
 *
 * TIM2 中断：72MHz / 7200 = 10kHz 计数器，1ms 一次中断
 *   - 每次中断 g_local_tick_ms++
 *   - 每 100 次中断（100ms） Odom_Print() 一次
 *
 * 学生在 USART2（USB-TTL 接 PC）终端看到的输出（坐标系 ROS REP-103）：
 *   X(fwd) = +X 前进方向 (m)        Y(left) = +Y 左方 (m)
 *   Yaw 协议层是弧度 (rad)，但终端打印转成度 (deg) 学生看着方便，CCW+ 逆时针为正
 *
 *   [ODOM] Yaw=  0.12 X(fwd)= 0.000 Y(left)= 0.000 Vx(fwd)= 0.00 Vy(left)= 0.00 fr=10
 *   [ODOM] Yaw=  0.15 X(fwd)= 0.024 Y(left)= 0.000 Vx(fwd)= 0.25 Vy(left)= 0.00 fr=11
 ******************************************************************************/
#include "DFCom.h"
#include "stm32f10x.h"
#include "usart.h"
#include "stdio.h"

/* ---- 本地 tick (ms) —— DFCom_Rx.c 用来给 ODOM 数据打时间戳 ---- */
volatile u32 g_local_tick_ms = 0;

/* ---- 控制是否自动打印（学生可以临时关掉） ---- */
volatile u8 g_print_odom_enable = 1;

/* ===========================================================================
 * TIM2 初始化：1ms 中断
 *   APB1 = 36MHz, TIM2 计数频率 = 72MHz / Prescaler
 *   设 Prescaler = 7200 - 1, Period = 10 - 1 → 10kHz/10 = 1kHz = 1ms
 *   (或者 Prescaler=72-1, Period=1000-1，也是 1ms，原理一样)
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


void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        g_local_tick_ms++;

        /* 10Hz 打印 */
        if (g_print_odom_enable && (g_local_tick_ms % 100) == 0) {
            Odom_Print();   /* 统一 Odom：Std 基础字段，Pro 完整字段 */
        }
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
