/******************************************************************************
 * DCar STM32F103C8T6 客户端：DFLink 队列 v6。
 *
 * 烧录后自动测试：诊断通过后等待3秒，持续循环操场路线，无模式开关。
 * 每圈：倒行半圆 -> 后退 -> 倒行半圆 -> 后退同时旋转360度。
 * 需要全向/麦克纳姆底盘。正常完成立即开始下一圈，失败停止，不把失败当通过。
 * 始终订阅VelPos，同时尝试Odom；Odom有新数据时一同打印，否则只用VelPos。
 *
 * 所有 DFQueue_* 入参固定为米、弧度、m/s、rad/s。
 * 0.10f 表示 10cm，PI/2 表示 90 度；不使用 g_dfcom_unit_mode。
 * +X 向前、+Y 向左、正 yaw 逆时针。位移/转角相对每段动作起点。
 *
 * 接线：PA9(TX) -> 车 RX，PA10(RX) <- 车 TX，GND 共地，460800/8N1。
 * 调试：PA2(TX) -> USB-TTL RX，GND 共地，115200/8N1。
 * Keil：完整工程见 DFCom_Example/USER/Template.uvprojx，启用 Use MicroLIB。
 * DFCom_PatchOnly中的本文件仅供移植参考，不包含独立Keil工程。
 * 本程序运行在外部 STM32 板上，不是小车主控固件。
 ******************************************************************************/
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "DFCom.h"

#define DEMO_SPEED_MPS       0.15f
#define DEMO_GRADE           DF_ACCEL_MEDIUM
#define DEMO_START_DELAY_MS  3000
#define DEMO_WAIT_MS         60000
#define DEMO_PI              3.14159265358979323846f

static void System_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    uart_init(460800);
    usart2_init(115200);
    Odom_PrintTimer_Init();  /* TIM2 维护毫秒 tick，并每100ms打印新鲜遥测。 */
}

/* 这里只检查串口、VelPos 和校准状态；队列 v6 能力由 DFQueue_Begin 单独确认。
 * 始终订阅标准VelPos，同时尝试全量Odom；Odom无回传不会阻止VelPos测试。
 */
static u8 Startup_Diagnose(void)
{
    u32 start;
    u32 frames_before;
    u32 frames_received;

    printf("\r\n[DCar] STM32 client 0.3.1 / queue v6\r\n");
    printf("[INIT] car=USART1 460800; terminal=USART2 115200\r\n");
    printf("[INIT] queue units=m, rad, m/s, rad/s\r\n");
    frames_before = g_velpos.frame_count;
    Cmd_Subscribe_Odom(ODOM_MODE_CONTINUOUS, ODOM_FREQ_10HZ);
    delay_ms(20);
    Cmd_Subscribe_VelPos(ODOM_MODE_CONTINUOUS, ODOM_FREQ_10HZ);
    g_dcar_state.received = 0;
    Cmd_Query_DcarState();
    start = g_local_tick_ms;
    while ((u32)(g_local_tick_ms - start) < 500) {
        if (g_dcar_state.received && g_velpos.frame_count != frames_before) break;
        delay_ms(5);
    }
    frames_received = g_velpos.frame_count - frames_before;
    printf("[INIT] state_reply=%u imu_calibrated=%u new_velpos_frames=%lu\r\n",
           (unsigned)g_dcar_state.received, (unsigned)g_dcar_state.imu_calibrated,
           (unsigned long)frames_received);
    if (!g_dcar_state.received || frames_received == 0) {
        printf("[INIT] FAIL: check car power, TX/RX/GND, 460800 and telemetry support\r\n");
        return 0;
    }
    if (!g_dcar_state.imu_calibrated) {
        printf("[INIT] FAIL: activate/calibrate the car before running motion\r\n");
        return 0;
    }
    printf("[INIT] link and calibration OK; queue capability not checked yet\r\n");
    return 1;
}

int main(void)
{
    u16 ids[4];
    u32 lap = 0;
    u8 i;
    const u8 count = 4;
    u8 result;
    u8 diagnostic_ok;
    const char *stage = "begin";

    System_Init();
    /* STM32 复位时车端可能仍在运行。先发零速，等车端供电启动后再发一次。
     * 隔离此阶段的运动回包，避免带入正式任务；停车能否到达仍取决于链路。
     */
    DFCom_MoveSessionQuarantine();
    Cmd_Move_Vel(0, 0, 0);
    delay_ms(1000);
    Cmd_Move_Vel(0, 0, 0);
    diagnostic_ok = Startup_Diagnose();
    delay_ms(50);
    DFCom_MoveSessionStart();

    if (!diagnostic_ok) {
        printf("[DEMO] Not started: startup diagnosis failed\r\n");
        goto idle;
    }
    printf("[DEMO] continuous stadium; start in %lu ms\r\n",
           (unsigned long)DEMO_START_DELAY_MS);
    delay_ms(DEMO_START_DELAY_MS);
    while (1) {
        stage = "begin";
        if (!DFQueue_Begin()) goto failed;
        lap++;
        printf("[LAP] START %lu: 1=reverse arc; 2=reverse line; 3=reverse arc; 4=reverse line+yaw\r\n",
               (unsigned long)lap);
        stage = "submit";
        /* 四条任务按顺序连续入队；运动调用直接放在main内，便于修改和验收。
         * 半径30cm、每段后退60cm；末段平移同时旋转，仅适用于全向底盘。
         */
        ids[0] = DFQueue_Arc(0.30f, DEMO_PI, -DEMO_SPEED_MPS, DEMO_GRADE);
        if (!ids[0]) goto failed;
        ids[1] = DFQueue_Linear(-0.60f, 0, DEMO_SPEED_MPS, 0, DEMO_GRADE);
        if (!ids[1]) goto failed;
        ids[2] = DFQueue_Arc(0.30f, DEMO_PI, -DEMO_SPEED_MPS, DEMO_GRADE);
        if (!ids[2]) goto failed;
        ids[3] = DFQueue_Linear(-0.60f, 0, DEMO_SPEED_MPS, 2*DEMO_PI, DEMO_GRADE);
        if (!ids[3]) goto failed;

        stage = "wait";
        for (i = 0; i < count; i++) {
            printf("[QUEUE] lap=%lu segment=%u/4 task=%u waiting\r\n",
                   (unsigned long)lap, (unsigned)(i+1), (unsigned)ids[i]);
            /* 每次Wait的上限，不是整圈的总截止时间；0不表示无限等。 */
            result = DFQueue_Wait(ids[i], DEMO_WAIT_MS);
            if (result != DF_QUEUE_WAIT_DONE) goto failed;
            printf("[QUEUE] lap=%lu segment=%u/4 task=%u DONE notice=0x%02X\r\n",
                   (unsigned long)lap, (unsigned)(i+1), (unsigned)ids[i],
                   (unsigned)DFQueue_LastNotice());
        }
        printf("[LAP] DONE %lu: 4 tasks completed; next lap immediately\r\n",
               (unsigned long)lap);
        /* 不延时、不等用户触发；完成确认后立即开始下一圈。
         * 车端仍可能在段末减速/微调；循环运行不等于全程速度不为零。
         */
    }

failed:
    printf("[DEMO] FAILED stage=%s reason=%s notice=0x%02X\r\n",
           stage, DFQueue_LastError(), (unsigned)DFQueue_LastNotice());
    DFQueue_Cancel();  /* 尝试取消队列并发零速；不会自动换接口或重跑路线。 */
idle:
    while (1) {
        delay_ms(1000);  /* 故障后保留遥测，不自动重试运动；排除故障后复位。 */
    }
}
