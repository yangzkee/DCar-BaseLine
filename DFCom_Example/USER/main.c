/******************************************************************************
 * DCar STM32F103C8T6 客户端：DFLink 队列 v6。
 *
 * 烧录后自动测试：上电等待及遥测观察后等待3秒，持续循环操场路线，无模式开关。
 * 每圈：倒行半圆 -> 后退 -> 倒行半圆 -> 后退同时旋转360度。
 * 需要全向/麦克纳姆底盘。正常完成立即开始下一圈，失败停止，不把失败当通过。
 * 只订阅Odom，车端按型号提供普通/全量字段，应用无需切换Odom模式。
 * Pro含完整姿态/IMU；普通款仅yaw/速度/位置/时间有效，增强字段零占位。
 * 遥测/状态诊断仅报告，不作为运动启动条件。
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

#define DEMO_START_DELAY_MS  3000
#define DEMO_WAIT_MS         60000
/* 独立开关，关闭回传或日志不会关闭运动；不重复订阅，避免重置车端推流。 */
#define DEMO_QUERY_STATE      1
#define DEMO_LOG_ENABLE       1


static void System_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    uart_init(460800);
    usart2_init(115200);
    g_print_odom_enable = DEMO_LOG_ENABLE;
    Odom_PrintTimer_Init();  /* TIM2只计时，打印在主循环服务中执行。 */
}

/* 延时仍完整执行；每5ms服务一次日志，不在中断中格式化/等待串口。 */
static void Demo_Delay(u32 ms)
{
    while (ms) {
        u32 step = ms > 5 ? 5 : ms;
        delay_ms(step);
        DFCom_Poll();
        ms -= step;
    }
}

/* 诊断是观察窗口，不根据状态或任一遥测的有无决定是否发运动任务。 */
static void Startup_Diagnose(void)
{
    printf("\r\n[DCar] STM32 client 0.4.1 / circular RX / Odom telemetry / queue v6\r\n");
    printf("[INIT] car=USART1 460800; terminal=USART2 115200\r\n");
    printf("[INIT] queue units=m, rad, m/s, rad/s\r\n");
    Cmd_Subscribe_Odom(ODOM_MODE_CONTINUOUS, ODOM_FREQ_10HZ);
    Demo_Delay(20);  /* 保留Odom订阅与状态查询之间的间隔。 */
    g_dcar_state.received = 0;
    if (DEMO_QUERY_STATE) Cmd_Query_DcarState();
    Demo_Delay(500); /* 固定观察窗口，无遥测时也不会无限等待。 */
    printf("[INIT] state_reply=%u imu_calibrated=%u odom=%lu (report only)\r\n",
           (unsigned)g_dcar_state.received, (unsigned)g_dcar_state.imu_calibrated,
           (unsigned long)g_odom.frame_count);
    if (g_dcar_state.received && !g_dcar_state.imu_calibrated)
        printf("[INIT] WARN: car reports uncalibrated; check car calibration\r\n");
    printf("[INIT] telemetry/state do not gate motion; queue ACK/DONE still required\r\n");
}

int main(void)
{
    u16 ids[4];
    u32 lap = 0;
    u8 i;
    const u8 count = 4;
    u8 result;
    const char *stage = "begin";

    System_Init();
    /* STM32 复位时车端可能仍在运行。先发零速，等车端供电启动后再发一次。
     * 隔离此阶段的运动回包，避免带入正式任务；停车能否到达仍取决于链路。
     */
    DFCom_MoveSessionQuarantine();
    Cmd_Move_Vel(0, 0, 0); /* vx=0前后速度、vy=0左右速度、wz=0角速度：全部填0停车；本函数默认cm/s、度/s，正值前/左/逆时针，负值反向。 */
    Demo_Delay(1000);
    Cmd_Move_Vel(0, 0, 0); /* vx=0前后速度、vy=0左右速度、wz=0角速度：全部填0停车；本函数默认cm/s、度/s，正值前/左/逆时针，负值反向。 */
    Startup_Diagnose();
    Demo_Delay(50);
    DFCom_MoveSessionStart();

    printf("[DEMO] continuous stadium; start in %lu ms\r\n",
           (unsigned long)DEMO_START_DELAY_MS);
    Demo_Delay(DEMO_START_DELAY_MS);
    while (1) {
        DFCom_Poll();
        stage = "begin";
        if (!DFQueue_Begin()) goto failed;
        lap++;
        printf("[LAP] START %lu: 1=reverse arc; 2=reverse line; 3=reverse arc; 4=reverse line+yaw\r\n",
               (unsigned long)lap);
        stage = "submit";
        /* 四条任务按顺序连续入队；运动调用直接放在main内，便于修改和验收。
         * 各行直接填写数值，单独修改一行不影响其他段；数值须符合车端能力。
         * 角度用弧度：1.57079633约90度、3.14159265约180度、6.28318531约360度。
         * grade为16极低、17低、18中、19高、20极高；不是直接填写m/s²。
         * 半径30cm、每段后退60cm；末段平移同时旋转，仅适用于全向底盘。
         */
        ids[0] = DFQueue_Arc(0.30f, 3.14159265f, -0.15f, 18); /* radius=0.30米须>0，增大使弧更宽；yaw=3.14159265弧度约+180°，正逆时针/负顺时针，绝对值越大转得越多；speed=-0.15米/秒须非0，负倒行/正前行，绝对值越大越快；grade=18中档，可填16/17/18/19/20，档位越高加减速越强，实际限值由车端决定。 */
        if (!ids[0]) goto failed;
        ids[1] = DFQueue_Linear(-0.60f, 0.0f, 0.15f, 0.0f, 18); /* x=-0.60米后退，正值前进，绝对值越大走得越远；y=0米不侧移，正左/负右；speed=0.15米/秒须>0，越大越快，后退改x为负而非改speed；yaw=0弧度不附加转向，正逆时针/负顺时针；grade=18中档，可填16/17/18/19/20，越高加减速越强。 */
        if (!ids[1]) goto failed;
        ids[2] = DFQueue_Arc(0.30f, 3.14159265f, -0.15f, 18); /* radius=0.30米须>0，越大弧越宽；yaw=3.14159265弧度约+180°，正逆时针/负顺时针，绝对值控制转角；speed=-0.15米/秒须非0，负倒行/正前行，绝对值控制快慢；grade=18中档，可填16/17/18/19/20，越高加减速越强；本行参数可独立于第一段调整。 */
        if (!ids[2]) goto failed;
        ids[3] = DFQueue_Linear(-0.60f, 0.0f, 0.15f, 6.28318531f, 18); /* x=-0.60米后退，正前/负后；y=0米不侧移，正左/负右，位移绝对值越大走得越远；speed=0.15米/秒须>0，越大越快；yaw=6.28318531弧度约+360°，边平移边逆时针转一圈，负值顺时针、0不转；grade=18中档，可填16/17/18/19/20，越高加减速越强。 */
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
    while (1) {
        Demo_Delay(1000);  /* 故障后保留遥测，不自动重试运动；排除故障后复位。 */
    }
}
