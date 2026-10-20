/******************************************************************************
 * DCar STM32F103C8T6 客户端：DFLink 队列 v6。
 *
 * 从这里开始：
 *   1. 保持 DEMO_ENABLE_MOTION=0，先编译、烧录并观察 USART2 诊断日志。
 *   2. 确认车端支持队列 v6，将 DEMO_ENABLE_MOTION 改为 1。
 *   3. 选择 DEMO_ROUTE；三个示例的实际运动调用都在本文件 main() 内。
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

#define DEMO_FORWARD 1             /* 入门：前进 0.10m */
#define DEMO_SQUARE  2             /* 前进 0.50m + 左转90度，重复四次 */
#define DEMO_STADIUM 3             /* 倒行操场路线，要求全向/麦克纳姆底盘 */

/* 这两个配置是示例的唯一运行入口；默认仅诊断，不自动行驶。 */
#ifndef DEMO_ENABLE_MOTION
#define DEMO_ENABLE_MOTION 0
#endif
#ifndef DEMO_ROUTE
#define DEMO_ROUTE DEMO_FORWARD
#endif
#define DEMO_SPEED_MPS       0.15f
#define DEMO_GRADE           DF_ACCEL_MEDIUM
#define DEMO_START_DELAY_MS  3000
#define DEMO_WAIT_MS         60000
#define DEMO_PI              3.14159265358979323846f

#if DEMO_ENABLE_MOTION != 0 && DEMO_ENABLE_MOTION != 1
#error DEMO_ENABLE_MOTION must be 0 or 1
#endif
#if DEMO_ROUTE < DEMO_FORWARD || DEMO_ROUTE > DEMO_STADIUM
#error DEMO_ROUTE must be DEMO_FORWARD, DEMO_SQUARE or DEMO_STADIUM
#endif

static void System_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    uart_init(460800);
    usart2_init(115200);
    Odom_PrintTimer_Init();  /* TIM2 维护毫秒 tick，并默认每100ms打印 VelPos。 */
}

/* 这里只检查串口、VelPos 和校准状态；队列 v6 能力由 DFQueue_Begin 单独确认。
 * 默认订阅标准 VelPos，不要求 Pro 全量 Odom，队列动作在车端闭环执行。
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
    u16 ids[8];
    u8 i;
    u8 count = 0;
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

    if (!DEMO_ENABLE_MOTION) {
        printf("[DEMO] DISABLED: set DEMO_ENABLE_MOTION=1 in USER/main.c\r\n");
        goto idle;
    }
    if (!diagnostic_ok) {
        printf("[DEMO] Not started: startup diagnosis failed\r\n");
        goto idle;
    }
    printf("[DEMO] route=%u; start in %lu ms; run once\r\n",
           (unsigned)DEMO_ROUTE, (unsigned long)DEMO_START_DELAY_MS);
    delay_ms(DEMO_START_DELAY_MS);
    if (!DFQueue_Begin()) goto failed;
    stage = "submit";

    /* 先发动作组，每条只等入队确认；再逐条 Wait 确认车端完成。
     * 若希望严格逐段执行，把每条 Wait 紧接在对应发送之后。
     * ID=0 表示发送/入队失败，不能将它当成有效任务。
     */
    if (DEMO_ROUTE == DEMO_FORWARD) {
        /* 最小运动示例：前进10cm，速度6cm/s，中档加速度。 */
        ids[0] = DFQueue_Linear(0.10f, 0, 0.06f, 0, DEMO_GRADE);
        if (!ids[0]) goto failed;
        count = 1;
    } else if (DEMO_ROUTE == DEMO_SQUARE) {
        /* 每次沿当前车头前进50cm，再原地左转90度；共8条任务。 */
        for (i = 0; i < 4; i++) {
            ids[2*i] = DFQueue_Linear(0.50f, 0, DEMO_SPEED_MPS, 0, DEMO_GRADE);
            if (!ids[2*i]) goto failed;
            ids[2*i+1] = DFQueue_Rotate(DEMO_PI / 2, 0.50f);
            if (!ids[2*i+1]) goto failed;
        }
        count = 8;
    } else {
        /* 全向底盘：倒行半圆R=30cm -> 后退60cm -> 倒行半圆 -> 后退并转360度。
         * 最后一段在固定直线路径上同时旋转车身，不适用于普通差速底盘。
         */
        ids[0] = DFQueue_Arc(0.30f, DEMO_PI, -DEMO_SPEED_MPS, DEMO_GRADE);
        if (!ids[0]) goto failed;
        ids[1] = DFQueue_Linear(-0.60f, 0, DEMO_SPEED_MPS, 0, DEMO_GRADE);
        if (!ids[1]) goto failed;
        ids[2] = DFQueue_Arc(0.30f, DEMO_PI, -DEMO_SPEED_MPS, DEMO_GRADE);
        if (!ids[2]) goto failed;
        ids[3] = DFQueue_Linear(-0.60f, 0, DEMO_SPEED_MPS, 2*DEMO_PI, DEMO_GRADE);
        if (!ids[3]) goto failed;
        count = 4;
    }

    stage = "wait";
    for (i = 0; i < count; i++) {
        printf("[QUEUE] waiting task=%u (%u/%u)\r\n",
               (unsigned)ids[i], (unsigned)(i+1), (unsigned)count);
        /* 此处是每次 Wait 的上限，不是整个动作组的总截止时间；0不表示无限等。 */
        result = DFQueue_Wait(ids[i], DEMO_WAIT_MS);
        if (result != DF_QUEUE_WAIT_DONE) goto failed;
    }
    printf("[DEMO] DONE: %u tasks completed; no automatic repeat\r\n", (unsigned)count);
    goto idle;

failed:
    printf("[DEMO] FAILED stage=%s reason=%s notice=0x%02X\r\n",
           stage, DFQueue_LastError(), (unsigned)DFQueue_LastNotice());
    DFQueue_Cancel();  /* 尝试取消队列并发零速；不会自动换接口或重跑路线。 */
idle:
    while (1) {
        delay_ms(1000);  /* 继续接收/打印遥测，不再发送非零运动。复位可重新启动。 */
    }
}
