/* DCar发送实现。当前队列v6 API位于本文件后半段，参数固定SI。
 * 无ID Cmd_Move_*读取单位开关，发送后不等运动完成；不能夹入活动队列。
 * 两类接口共用组帧和DMA发送。队列回包由Rx验证后交给DFQueue_Receive。
 * 运动S32定点编码为SI值乘10000；详见docs/DFLink-v6.md。
 */
#include "DFCom.h"
#include "usart.h"
#include "delay.h"

/* Rx 内部的任务槽交接点，不属于应用层 API。 */
void DFCom_MoveCommandSent(u8 cmd_code);
void DFCom_MoveCommandSendFailed(u8 cmd_code);

/* 字节拆分宏（标准库 BYTE0/1/2/3） */
#define BYTE0(v) ((u8)((v) & 0xFF))
#define BYTE1(v) ((u8)(((v) >> 8) & 0xFF))
#define BYTE2(v) ((u8)(((v) >> 16) & 0xFF))
#define BYTE3(v) ((u8)(((v) >> 24) & 0xFF))

/* 运动发送缩放因子：SI量 ×10000 编s32 LE（不用于遥测缩放）
 *   位置 / 速度 / 角度 / 角速度全用这一个缩放                      */
#define SCALE_SI     10000   /* SI 单位 → s32 (÷10000 还原) */

/* ===========================================================================
 *  单位模式全局变量（参见 DFCom.h 顶部 DFCom_UnitMode_e）
 *  ---------------------------------------------------------------------------
 *  仅影响无任务ID的 Cmd_Move_*；DFQueue_* 固定SI，不读取本变量。
 *  想用 SI 原生单位（米 + 弧度），在 main.c 把它设成 DFCOM_UNIT_M。
 *  本文件下面 5 个 Cmd_Move_* 函数顶部会按本变量做单位转换，
 *  协议层永远只发 SI（米 / rad / ×10000 s32）。
 * ===========================================================================*/
u8 g_dfcom_unit_mode = DFCOM_UNIT_CM;   /* 默认厘米模式 */

/*============================================================================
 *  通用打帧工具：build_header + finalize_frame + send_frame
 *  ──────────────────────────────────────────────────────────────────────────
 *  ★ DF_Link 发送帧总格式 ★
 *
 *   字节  内容                           说明
 *   ────  ────────────────────────────   ────────────────────────────────────
 *   [0]   0xDF                           固定帧头
 *   [1]   ROBOT_ID = 0x01                小车 ID（默认 1，可在 flash 改）
 *   [2]   PC_ID = 0x97                   上位机 ID（小车端不校验此字节）
 *   [3]   A                              指令类 (class)
 *   [4]   B                              子指令 (cmd)
 *   [5]   LEN                            payload 长度（不含头/尾/校验）
 *   [6..LEN+5]   payload                 数据域，字段顺序见每个指令注释
 *   [LEN+6]      0xFD                    固定帧尾
 *   [LEN+7/8]    checksum_lo / hi        u16 LE，从 [0] 累加到 [LEN+6]
 *============================================================================*/

static u8 build_header(u8 *buf, u8 A, u8 B)
{
    buf[0] = DFCOM_FRAME_HEADER;
    buf[1] = DFCOM_ROBOT_ID;
    buf[2] = DFCOM_PC_ID;
    buf[3] = A;
    buf[4] = B;
    /* buf[5] = LEN 由调用方填 */
    return 5;
}

static u8 finalize_frame(u8 *buf, u8 data_len)
{
    u8 cnt = 6 + data_len;
    u16 sc;
    u8 sum_len;
    u8 i;
    buf[cnt++] = DFCOM_FRAME_TAIL;
    /* 校验和：从帧头到帧尾全部累加 */
    sc = 0;
    sum_len = data_len + 7;   /* header 6 + tail 1 */
    for (i = 0; i < sum_len; i++) sc += buf[i];
    buf[cnt++] = BYTE0(sc);
    buf[cnt++] = BYTE1(sc);
    return cnt;
}

static u8 send_frame(u8 *buf, u8 len)
{
    u8 retry;
    /* DMA 发送，最多尝试 3 次；返回 1 表示本帧已经成功启动。 */
    for (retry = 0; retry < 3; retry++) {
        if (USART1_Send_By_DMA(buf, len) == 0) return 1;
        delay_us(200);
    }
    return 0;
}

/*
 * 运动槽只能在 DMA 成功启动之后切换：发送失败时，WaitMoveDone 会立即返回
 * TIMEOUT，而不会错误绑定到上一条同类型任务。
 */
static void send_motion_frame(u8 *buf, u8 len, u8 cmd_code)
{
    if (send_frame(buf, len)) {
        DFCom_MoveCommandSent(cmd_code);
    } else {
        DFCom_MoveCommandSendFailed(cmd_code);
        printf("[TX.FAIL] motion cmd=0x%02X DMA frame was not started\r\n",
               (unsigned)cmd_code);
    }
}

/* 把 4 个字节小端写入 buf[off..off+3]，给 s32 LE 用 (协议统一整数编码哲学) */
static void put_s32_le(u8 *buf, s32 v)
{
    buf[0] = BYTE0(v);
    buf[1] = BYTE1(v);
    buf[2] = BYTE2(v);
    buf[3] = BYTE3(v);
}


/* 无任务ID直线接口；单位由 g_dfcom_unit_mode 决定。payload: x,y,speed,profile。 */
void Cmd_Move_Linear(float px, float py, float speed_mps, u8 profile)
{
    /* ★ profile: 0=匀速, 1=梯形加减速, 2=高精度距离闭环 */
    u8 buf[32];
    u8 cnt;
    s32 ipx, ipy, ispd;

    if (g_dfcom_unit_mode == DFCOM_UNIT_CM) {
        px        *= 0.01f;
        py        *= 0.01f;
        speed_mps *= 0.01f;
    } else if (g_dfcom_unit_mode == DFCOM_UNIT_MM) {
        px        *= 0.001f;
        py        *= 0.001f;
        speed_mps *= 0.001f;
    }

    if (speed_mps <= 0.0f) {
        DFCom_MoveCommandSendFailed(CMD_LINEAR);
        return;
    }
    if (profile > 2) profile = 2;

    cnt = build_header(buf, 0x02, CMD_LINEAR);

    ipx  = (s32)(px        * SCALE_SI);
    ipy  = (s32)(py        * SCALE_SI);
    ispd = (s32)(speed_mps * SCALE_SI);

    buf[cnt++] = 13;                          /* LEN: 12B (Px Py Sp) + 1B profile */
    put_s32_le(&buf[cnt], ipx);  cnt += 4;
    put_s32_le(&buf[cnt], ipy);  cnt += 4;
    put_s32_le(&buf[cnt], ispd); cnt += 4;
    buf[cnt++] = profile;                     /* ★ byte[12] = Profile (0/1/2) */

    cnt = finalize_frame(buf, 13);
    send_motion_frame(buf, cnt, CMD_LINEAR);
}


/* 无任务ID平移转向；0x65 payload: x,y,yaw,speed,profile。当前队列使用扩展0x64。 */
void Cmd_Move_LinearWithYaw(float px, float py, float dyaw_rad, float speed_mps, u8 profile)
{
    /* ★ profile: 0=匀速, 1=梯形加减速, 2=高精度距离闭环 */
    u8 buf[32];
    u8 cnt;
    s32 ipx, ipy, idyaw, ispd;

    if (g_dfcom_unit_mode == DFCOM_UNIT_CM) {
        px        *= 0.01f;
        py        *= 0.01f;
        dyaw_rad  *= 0.01745329f;
        speed_mps *= 0.01f;
    } else if (g_dfcom_unit_mode == DFCOM_UNIT_MM) {
        px        *= 0.001f;
        py        *= 0.001f;
        dyaw_rad  *= 0.01745329f;
        speed_mps *= 0.001f;
    }

    if (speed_mps <= 0.0f) {
        DFCom_MoveCommandSendFailed(CMD_LINEAR_WITH_YAW);
        return;
    }
    if (profile > 2) profile = 2;

    cnt = build_header(buf, 0x02, CMD_LINEAR_WITH_YAW);

    ipx   = (s32)(px        * SCALE_SI);
    ipy   = (s32)(py        * SCALE_SI);
    idyaw = (s32)(dyaw_rad  * SCALE_SI);
    ispd  = (s32)(speed_mps * SCALE_SI);

    buf[cnt++] = 17;                          /* LEN: 16B + 1B profile */
    put_s32_le(&buf[cnt], ipx);   cnt += 4;
    put_s32_le(&buf[cnt], ipy);   cnt += 4;
    put_s32_le(&buf[cnt], idyaw); cnt += 4;
    put_s32_le(&buf[cnt], ispd);  cnt += 4;
    buf[cnt++] = profile;                     /* ★ byte[16] = Profile */

    cnt = finalize_frame(buf, 17);
    send_motion_frame(buf, cnt, CMD_LINEAR_WITH_YAW);
}


/* 无任务ID原地旋转；payload: yaw,omega。正yaw逆时针，omega为正数。 */
void Cmd_Move_Rot(float dyaw_rad, float omega_max_rad_s)
{
    u8 buf[32];
    u8 cnt;
    s32 idyaw, iomega;

    /* ★ 单位转换: 原地旋转只有角度参数, MM 跟 CM 一样用 deg
     *   CM / MM 模式: deg + deg/s → rad + rad/s
     *   M  模式:       rad + rad/s, 不转换 (协议原生) */
    if (g_dfcom_unit_mode == DFCOM_UNIT_CM ||
        g_dfcom_unit_mode == DFCOM_UNIT_MM) {
        dyaw_rad        *= 0.01745329f;
        omega_max_rad_s *= 0.01745329f;
    }

    if (omega_max_rad_s <= 0.0f) {
        DFCom_MoveCommandSendFailed(CMD_ROT);
        return;
    }

    cnt = build_header(buf, 0x02, CMD_ROT);

    idyaw  = (s32)(dyaw_rad        * SCALE_SI);
    iomega = (s32)(omega_max_rad_s * SCALE_SI);

    buf[cnt++] = 8;
    put_s32_le(&buf[cnt], idyaw);  cnt += 4;
    put_s32_le(&buf[cnt], iomega); cnt += 4;

    cnt = finalize_frame(buf, 8);
    send_motion_frame(buf, cnt, CMD_ROT);
}


/* 持续速度；无任务完成回包，不使用 WaitMoveDone。发(0,0,0)主动停车。 */
void Cmd_Move_Vel(float vx_mps, float vy_mps, float vz_rad_s)
{
    u8 buf[32];
    u8 cnt;
    s32 ivx, ivy, ivz;

    /* ★ 单位转换: 根据 g_dfcom_unit_mode 把入参转成 SI
     *   CM 模式: cm/s + deg/s → m/s + rad/s
     *   MM 模式: mm/s + deg/s → m/s + rad/s
     *   M  模式: m/s + rad/s, 不转换 */
    if (g_dfcom_unit_mode == DFCOM_UNIT_CM) {
        vx_mps   *= 0.01f;
        vy_mps   *= 0.01f;
        vz_rad_s *= 0.01745329f;
    } else if (g_dfcom_unit_mode == DFCOM_UNIT_MM) {
        vx_mps   *= 0.001f;
        vy_mps   *= 0.001f;
        vz_rad_s *= 0.01745329f;
    }

    cnt = build_header(buf, 0x02, CMD_VEL);

    ivx = (s32)(vx_mps   * SCALE_SI);
    ivy = (s32)(vy_mps   * SCALE_SI);
    ivz = (s32)(vz_rad_s * SCALE_SI);

    buf[cnt++] = 12;
    put_s32_le(&buf[cnt], ivx); cnt += 4;
    put_s32_le(&buf[cnt], ivy); cnt += 4;
    put_s32_le(&buf[cnt], ivz); cnt += 4;

    cnt = finalize_frame(buf, 12);
    send_motion_frame(buf, cnt, CMD_VEL);
}


/* 无任务ID圆弧；payload: radius,yaw,speed,profile；单位由 g_dfcom_unit_mode 决定。 */
void Cmd_Move_Arc(float radius_m, float dyaw_rad, float speed_mps, u8 profile)
{
    /* ★ profile: 0=匀速, 1=梯形加减速, 2=高精度距离闭环 */
    u8 buf[32];
    u8 cnt;
    s32 ir, idyaw, ispd;

    if (g_dfcom_unit_mode == DFCOM_UNIT_CM) {
        radius_m  *= 0.01f;
        dyaw_rad  *= 0.01745329f;
        speed_mps *= 0.01f;
    } else if (g_dfcom_unit_mode == DFCOM_UNIT_MM) {
        radius_m  *= 0.001f;
        dyaw_rad  *= 0.01745329f;
        speed_mps *= 0.001f;
    }

    if (speed_mps <= 0.0f) {
        DFCom_MoveCommandSendFailed(CMD_ARC);
        return;
    }
    if (profile > 2) profile = 2;

    cnt = build_header(buf, 0x02, CMD_ARC);

    ir    = (s32)(radius_m  * SCALE_SI);
    idyaw = (s32)(dyaw_rad  * SCALE_SI);
    ispd  = (s32)(speed_mps * SCALE_SI);

    buf[cnt++] = 13;                          /* LEN: 12B + 1B profile */
    put_s32_le(&buf[cnt], ir);    cnt += 4;
    put_s32_le(&buf[cnt], idyaw); cnt += 4;
    put_s32_le(&buf[cnt], ispd);  cnt += 4;
    buf[cnt++] = profile;                     /* ★ byte[12] = Profile */

    cnt = finalize_frame(buf, 13);
    send_motion_frame(buf, cnt, CMD_ARC);
}


/*============================================================================
 *  指令 6：ODOM 订阅 Cmd_Subscribe_Odom
 *  ──────────────────────────────────────────────────────────────────────────
 *  Class/Cmd: A = 0x04 (DataRecivViitClass), B = 0x80 (Time_V)
 *
 *  含义：统一订阅Odom，由车端按型号提供普通/全量数据。
 *  Pro提供姿态、IMU、两系速度、位置与时间；普通款提供yaw、两系速度、位置与时间。
 *
 *  ★ 这条指令的协议层格式未变（订阅机制本身不动），但回传里的 N_PosX/Y
 *    现在的含义是 ROS 坐标系：+X 前进、+Y 左方 ★
 *
 *  完整帧布局（共 11 字节）：
 *    [6]   type     u8    Vistt_type:
 *                         0x01 = 持续模式（小车自己 N Hz 推，最常用）
 *                         0x02 = 一次模式（小车发一帧后自动清零）
 *                         0x00 = 错误（小车会拒绝整个订阅！）
 *    [7]   freq     u8    频率枚举（不是 Hz 数字！）：
 *                         1=10Hz, 5=50Hz, 10=100Hz, 20=200Hz,
 *                         25=250Hz, 50=500Hz, 100=1000Hz
 *                         其他值小车也会拒绝。
 *    LEN = 2
 *
 *  ★ 重要：持续模式只要发一次！反复发会重置小车状态机。★
 *
 *  ★ 怎么"停止"持续推送？（官方文档原话）★
 *    "若已开启连续访问，只需发送单次访问指令即可自动关闭连续回传功能"
 *    所以本函数对 ODOM_MODE_STOP 内部转成 ONESHOT (0x02) 发出去——小车收到后
 *    把 Visit_Type 改成 0x02，发完下一帧自动清零，从此不再推。
 *============================================================================*/
void Cmd_Subscribe_Odom(u8 mode, u16 freq_hz)
{
    /* Hz → 小车端枚举值 */
    u8 fcode;
    u8 vtype;
    u8 buf[15];
    u8 cnt;

    if      (freq_hz == 10)  fcode = 1;
    else if (freq_hz == 50)  fcode = 5;
    else if (freq_hz == 100) fcode = 10;
    else if (freq_hz == 200) fcode = 20;
    else if (freq_hz == 250) fcode = 25;
    else if (freq_hz == 500) fcode = 50;
    else                     fcode = 1;   /* 默认 10Hz */

    /* mode 标准化：小车不支持 STOP，用 ONESHOT 实现"发完一次自停" */
    if      (mode == ODOM_MODE_CONTINUOUS) vtype = 0x01;
    else if (mode == ODOM_MODE_ONESHOT)    vtype = 0x02;
    else                                   vtype = 0x02;   /* STOP → ONESHOT */

    cnt = build_header(buf, 0x04, 0x80);
    buf[cnt++] = 2;
    buf[cnt++] = vtype;
    buf[cnt++] = fcode;
    cnt = finalize_frame(buf, 2);
    send_frame(buf, cnt);
}

/*============================================================================
 *  指令 7：查询小车激活/校准状态 Cmd_Query_DcarState
 *  ──────────────────────────────────────────────────────────────────────────
 *  Class/Cmd: A = 0x0B (RobAttribViitClass), B = 0x40 (ActivateState_V = 64)
 *
 *  含义：问小车 "你校准好了吗 / 参数 OK 吗 / IMU 健康吗"。
 *        小车回传 A=0x8C, B=0x40, LEN=12，含 IMU 校准标志等。
 *
 *  发送帧（共 9 字节，无 payload）：
 *    [0..5]  0xDF 0x01 0x97 0x0B 0x40 0x00       LEN = 0
 *    [6]     0xFD
 *    [7..8]  checksum LE
 *
 *  回传帧 payload (12 字节):
 *    [0]    ifIMUcali   u8    1 = IMU 校准好，0 = 未校准（不能用！）
 *    [1]    ifFinetune  u8    1 = 控制参数已微调
 *    [2..3] gyro_z_off  s16   gyro Z 偏置 × 100
 *    [4..5] imu_par2    s16
 *    [6..7] imu_temp    s16
 *    [8..9] ctrl_par1   s16   控制 Par1 × 100
 *    [10..11] ctrl_par2 s16   控制 Par2 × 100
 *============================================================================*/
void Cmd_Query_DcarState(void)
{
    u8 buf[12];
    u8 cnt = build_header(buf, 0x0B, 0x40);
    buf[cnt++] = 0;                  /* LEN = 0，无 payload */
    cnt = finalize_frame(buf, 0);
    send_frame(buf, cnt);
}

/* ===========================================================================
 * 队列 v6：应用调用 DFQueue_*，接收中断经 DFQueue_Receive 更新任务状态。
 * 共享本文件的组帧/DMA发送函数。使用固定8槽，无堆内存；仅接收函数在中断调用。
 * 与前面的无任务ID Cmd_Move_* 编码不同，单位固定为 SI。
 * ===========================================================================*/
#include <math.h>
#include <string.h>

typedef struct {
    volatile u16 id;
    volatile u8 process;
    volatile u8 notice;
    volatile u8 received;
} DFQueueSlot;

static DFQueueSlot q_slots[DF_QUEUE_MAX_TASKS];
static volatile u16 q_query_id;
static volatile u8 q_query_notice;
static volatile u8 q_query_process;
static volatile u8 q_query_ready;
static volatile u8 q_enabled;
static u16 q_next;
static const char *q_error = "none";
static u8 q_last_notice;

const char *DFQueue_LastError(void) { return q_error; }
u8 DFQueue_LastNotice(void) { return q_last_notice; }

/* 管理帧：0查能力、7查下一ID、5查指定结果、4取消。 */
static u8 q_manage(u8 operation, u16 id)
{
    u8 frame[12];
    u8 length = (operation == 5) ? 3 : 1;
    build_header(frame, 2, 0x85);
    frame[5] = length;
    frame[6] = operation;
    if (length == 3) {
        frame[7] = (u8)id;
        frame[8] = (u8)(id >> 8);
    }
    return send_frame(frame, finalize_frame(frame, length));
}

void DFQueue_Receive(const u8 *payload, u8 length)
{
    u8 i;
    u16 id;
    if (length != 4) return;
    id = (u16)payload[2] | ((u16)payload[3] << 8);
    if (payload[1] == 0x21 || payload[1] == 0x28) {
        q_query_id = id;
        q_query_notice = payload[1];
        q_query_process = payload[0];
        q_query_ready = 1;
        return;
    }
    if (!q_enabled || !id) return;  /* ID0不是某条任务的完成回包。 */
    for (i = 0; i < DF_QUEUE_MAX_TASKS; i++) {
        if (q_slots[i].id != id) continue;
        /* 终态保持：重复完成、迟到ACK或进度不能重新打开已经结束的任务。
         * 0x11是队列满，需要原ID重试，不是已完成。
         */
        if (q_slots[i].received && q_slots[i].process == 255 && q_slots[i].notice != 0x11) return;
        q_slots[i].process = payload[0];
        q_slots[i].notice = payload[1];
        q_slots[i].received = 1;
        return;
    }
}

static u8 q_query(u8 operation, u8 expected_notice)
{
    u8 attempt;
    u32 start;
    for (attempt = 0; attempt < 3; attempt++) {
        q_query_ready = 0;
        if (!q_manage(operation, 0)) {
            q_error = "DMA transmission failed";
            return 0;
        }
        start = g_local_tick_ms;
        while ((u32)(g_local_tick_ms - start) < 500) {
            if (q_query_ready && q_query_notice == expected_notice) return 1;
            DFCom_Poll();
            delay_ms(1);
        }
    }
    q_error = "queue query timed out; check link and v6 firmware";
    return 0;
}

u8 DFQueue_Begin(void)
{
    u8 i;
    q_error = "none";
    q_last_notice = 0;
    /* 尚有在途任务时不能重新握手丢失其身份。 */
    for (i = 0; i < DF_QUEUE_MAX_TASKS; i++) {
        if (q_slots[i].id && (!q_slots[i].received || q_slots[i].process != 255)) {
            q_error = "local tasks still active; wait or cancel first";
            return 0;
        }
    }
    q_enabled = 0;
    if (!q_query(0, 0x21)) return 0;
    if (q_query_id < 6) {
        q_error = "device does not support queue v6";
        return 0;
    }
    if (!q_query(7, 0x28)) return 0;
    if (!q_query_id || q_query_process != 16) {
        q_error = "device queue is not empty or next task ID is invalid";
        return 0;
    }
    memset(q_slots, 0, sizeof(q_slots));
    q_next = q_query_id;
    q_enabled = 1;
    return 1;
}

void DFQueue_Cancel(void)
{
    u8 frame[21];
    /* 两个发送都是尽力而为，断线时不能确认车端已停车。
     * 保留最近错误，供调用者打印；取消后下一批必须重新 Begin。
     */
    q_manage(4, 0);
    build_header(frame, 2, 0x62);
    frame[5] = 12;
    memset(frame + 6, 0, 12);
    send_frame(frame, finalize_frame(frame, 12));
    q_enabled = 0;
    memset(q_slots, 0, sizeof(q_slots));
}

u8 DFQueue_Status(u16 id, u8 *process, u8 *notice)
{
    u8 i;
    u32 irq;
    if (!process || !notice) return 0;
    irq = __get_PRIMASK();
    __disable_irq();
    for (i = 0; i < DF_QUEUE_MAX_TASKS; i++) {
        if (id && q_slots[i].id == id && q_slots[i].received) {
            *process = q_slots[i].process;
            *notice = q_slots[i].notice;
            if (!irq) __enable_irq();
            return 1;
        }
    }
    if (!irq) __enable_irq();
    return 0;
}

static u8 q_number(u8 *destination, float value)
{
    double scaled = (double)value * 10000.0;
    if (!isfinite(scaled) || scaled > 2147483647.0 || scaled < (-2147483647.0 - 1.0)) return 0;
    put_s32_le(destination, (s32)scaled);
    return 1;
}

static u16 q_submit(u8 command, const float *values, u8 count, u8 grade)
{
    u8 frame[28];
    u8 i;
    u8 slot = DF_QUEUE_MAX_TASKS;
    u8 length = (u8)(count * 4 + (command == CMD_ROT ? 0 : 1));
    u8 attempt;
    u8 process;
    u8 notice;
    u16 id;
    u32 start;
    q_error = "none";
    q_last_notice = 0;
    if (!q_enabled) {
        q_error = "call DFQueue_Begin successfully before submitting";
        return 0;
    }
    if (grade < DF_ACCEL_VERY_LOW || grade > DF_ACCEL_VERY_HIGH) {
        q_error = "acceleration grade must be 16..20";
        return 0;
    }
    for (i = 0; i < DF_QUEUE_MAX_TASKS; i++) {
        if (!q_slots[i].id) { slot = i; break; }
    }
    if (slot == DF_QUEUE_MAX_TASKS) {
        q_error = "8 local task slots used; consume completed results with Wait";
        return 0;
    }
    build_header(frame, 2, command);
    frame[5] = length + 2;
    for (i = 0; i < count; i++) {
        if (!q_number(frame + 6 + 4*i, values[i])) {
            q_error = "motion parameter is nonfinite or outside S32 fixed-point range";
            return 0;
        }
    }
    if (command != CMD_ROT) frame[6 + length - 1] = grade;
    id = q_next;
    frame[6 + length] = (u8)id;
    frame[7 + length] = (u8)(id >> 8);
    /* 先登记ID再启动DMA，保证极快的中断回包也能找到任务。 */
    q_slots[slot].received = 0;
    q_slots[slot].id = id;
    for (attempt = 0; attempt < 3; attempt++) {
        if (!send_frame(frame, finalize_frame(frame, length + 2))) {
            q_error = "DMA transmission failed";
            DFQueue_Cancel();
            return 0;
        }
        start = g_local_tick_ms;
        while ((u32)(g_local_tick_ms - start) < 500) {
            if (DFQueue_Status(id, &process, &notice)) {
                if (notice == 0 || notice == 0x10 || notice == 0x25 || notice == 0x27) {
                    q_next = id == 65535 ? 1 : (u16)(id + 1);
                    return id;  /* 入队确认或后续状态，不表示动作已结束。 */
                }
                if (notice != 0x11) {
                    q_last_notice = notice;
                    q_error = "device rejected submitted task";
                    DFQueue_Cancel();
                    return 0;
                }
            }
            DFCom_Poll();
            delay_ms(1);
        }
        /* ACK丢失或队列满：重发完全相同的ID和字节，不消耗新编号。 */
    }
    q_error = "task admission timed out or device queue stayed full";
    DFQueue_Cancel();
    return 0;
}

u16 DFQueue_Linear(float x, float y, float speed, float yaw, u8 grade)
{
    float values[4];
    if (!(speed > 0)) { q_error = "linear speed must be positive"; q_last_notice = 0; return 0; }
    values[0] = x; values[1] = y; values[2] = speed; values[3] = yaw;
    /* 普通直线和带旋转直线都用0x64，yaw非零时才扩展payload。 */
    return q_submit(CMD_LINEAR, values, yaw == 0 ? 3 : 4, grade);
}

u16 DFQueue_Rotate(float yaw, float omega)
{
    float values[2];
    if (!(omega > 0)) { q_error = "angular speed must be positive"; q_last_notice = 0; return 0; }
    values[0] = yaw; values[1] = omega;
    /* 原地旋转帧不含加速度档位。 */
    return q_submit(CMD_ROT, values, 2, DF_ACCEL_MEDIUM);
}

u16 DFQueue_Arc(float radius, float yaw, float speed, u8 grade)
{
    float values[3];
    if (!(radius > 0) || speed == 0) { q_error = "arc radius must be positive and speed nonzero"; q_last_notice = 0; return 0; }
    values[0] = radius; values[1] = yaw; values[2] = speed;
    return q_submit(CMD_ARC, values, 3, grade);
}

u8 DFQueue_Wait(u16 id, u32 timeout_ms)
{
    u32 start = g_local_tick_ms;
    u32 last = start;
    u8 process;
    u8 notice;
    u8 i;
    u8 found = 0;
    q_error = "none";
    q_last_notice = 0;
    for (i = 0; i < DF_QUEUE_MAX_TASKS; i++) if (id && q_slots[i].id == id) found = 1;
    if (!found || !timeout_ms) {
        q_error = "unknown task ID or zero wait timeout";
        return DF_QUEUE_WAIT_TIMEOUT;
    }
    while ((u32)(g_local_tick_ms - start) < timeout_ms) {
        if (DFQueue_Status(id, &process, &notice) && process == 255 && notice != 0x11) {
            q_last_notice = notice;
            if (notice != 0 && notice != 0x27) {
                q_error = "device reported task failure";
                DFQueue_Cancel();
                return DF_QUEUE_WAIT_FAULT;
            }
            for (i = 0; i < DF_QUEUE_MAX_TASKS; i++) if (q_slots[i].id == id) q_slots[i].id = 0;
            return DF_QUEUE_WAIT_DONE;
        }
        /* 已入队任务丢失终止帧时查结果，不重复创建运动。 */
        if ((u32)(g_local_tick_ms - last) >= 500) { q_manage(5, id); last = g_local_tick_ms; }
        DFCom_Poll();
        delay_ms(1);
    }
    q_error = "task completion timed out; cancellation attempted";
    DFQueue_Cancel();
    return DF_QUEUE_WAIT_TIMEOUT;
}
