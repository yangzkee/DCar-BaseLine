
#include "DFCom.h"
#include "usart.h"
#include "delay.h"
#include <string.h>

/* ===========================================================================
 * 全局数据存储 —— 应用层直接读
 * ===========================================================================*/
volatile OdomData_t   g_odom         = {0};   /* Odom v4 (CMD 0x6C 0x80, 51B) */
volatile MoveStatus_t g_move_status[2] = {{0}};
volatile DcarState_t  g_dcar_state   = {0};

#define MOVE_SLOT_NONE  0xFF

/* 无ID运动任务使用“当前槽 + 旧槽丢弃器”；队列v6状态在Tx中按ID维护。 */
static volatile u8 g_move_current_slot    = MOVE_SLOT_NONE;
static volatile u8 g_move_discard_slot    = MOVE_SLOT_NONE;
static volatile u8 g_move_discard_cmd     = 0;
static volatile u8 g_move_session_active  = 0;
static volatile u8 g_move_send_failed     = 0;
static volatile u8 g_move_failed_cmd      = 0;

/* 本地 ms tick（由 TIM2 中断 +1，在 DFCom_Print.c 维护） */
extern volatile u32 g_local_tick_ms;

/* ===========================================================================
 * 小端字节合成（DF_Link 协议所有多字节字段都是小端）
 * ===========================================================================*/
static s16 read_s16(const u8 *p) {
    return (s16)((u16)p[0] | ((u16)p[1] << 8));
}

static s32 read_s32(const u8 *p) {
    return (s32)((u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24));
}

static u32 read_u32(const u8 *p) {
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static u8 move_is_bounded_cmd(u8 cmd_code)
{
    return (cmd_code >= CMD_ROT && cmd_code <= CMD_ARC);
}

static void move_clear_slot(u8 slot)
{
    if (slot > 1) return;
    g_move_status[slot].cmd_code    = 0;
    g_move_status[slot].progress    = 0;
    g_move_status[slot].last_notice = 0;
    g_move_status[slot].fresh       = 0;
}

static u8 move_slot_is_terminal(u8 slot)
{
    return (slot <= 1 &&
            g_move_status[slot].fresh &&
            g_move_status[slot].progress == 0xFF);
}

static void move_release_discard(void)
{
    if (g_move_discard_slot <= 1) {
        move_clear_slot(g_move_discard_slot);
    }
    g_move_discard_slot = MOVE_SLOT_NONE;
    g_move_discard_cmd  = 0;
}

/*
 * 当前任务离开前台时，已终止的槽直接清空；未终止的槽变成丢弃器，
 * 专门吞掉该任务随后唯一的终止帧。
 */
static void move_retire_current(void)
{
    u8 slot = g_move_current_slot;
    u8 cmd_code;

    if (slot > 1) return;
    cmd_code = g_move_status[slot].cmd_code;

    if (move_slot_is_terminal(slot)) {
        move_clear_slot(slot);
    } else {
        if (g_move_discard_slot <= 1 &&
            g_move_discard_slot != slot) {
            move_release_discard();
        }
        move_clear_slot(slot);
        g_move_discard_slot = slot;
        g_move_discard_cmd  = cmd_code;
    }
    g_move_current_slot = MOVE_SLOT_NONE;
}

static u8 move_next_slot(void)
{
    if (g_move_current_slot <= 1) return g_move_current_slot ^ 1U;
    if (g_move_discard_slot <= 1) return g_move_discard_slot ^ 1U;
    return 0;
}

static u32 move_enter_critical(void)
{
    u32 primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void move_exit_critical(u32 primask)
{
    if ((primask & 1U) == 0) __enable_irq();
}

static void move_reset_session_state(u8 active)
{
    move_clear_slot(0);
    move_clear_slot(1);
    g_move_current_slot   = MOVE_SLOT_NONE;
    g_move_discard_slot   = MOVE_SLOT_NONE;
    g_move_discard_cmd    = 0;
    g_move_send_failed    = 0;
    g_move_failed_cmd     = 0;
    g_move_session_active = active;
}

void DFCom_MoveSessionQuarantine(void)
{
    u32 primask = move_enter_critical();
    move_reset_session_state(0);
    move_exit_critical(primask);
}

void DFCom_MoveSessionStart(void)
{
    u32 primask = move_enter_critical();
    move_reset_session_state(1);
    move_exit_critical(primask);
}

/*
 * 只能在运动帧成功启动 DMA 后调用。
 * 有界任务按 A/B/A/B 交替；若目标槽还是 N-2 的旧丢弃槽，直接强制清空复用，
 * 因此丢过一次终止帧也不会让槽永久占用。
 */
void DFCom_MoveCommandSent(u8 cmd_code)
{
    u32 primask;
    u8 next;

    if (cmd_code != CMD_VEL && !move_is_bounded_cmd(cmd_code)) return;

    primask = move_enter_critical();
    g_move_send_failed = 0;
    g_move_failed_cmd  = 0;

    /* 上电停车/诊断阶段仍发送帧，但所有运动回传都处于隔离态。 */
    if (!g_move_session_active) {
        move_exit_critical(primask);
        return;
    }

    /*
     * 持续速度没有可等待的自然终止帧，但它会打断当前有界任务。
     * 所以只把旧当前槽降级为丢弃器，不为 CMD_VEL 建立新当前槽。
     */
    if (cmd_code == CMD_VEL) {
        move_retire_current();
        move_exit_critical(primask);
        return;
    }

    next = move_next_slot();

    /*
     * A→B→A：轮到一个槽再次成为当前槽时，无条件放弃它保存的 N-2 旧任务。
     * 这是“终止帧在客户端丢失”时的硬兜底，不让旧槽永久等待。
     */
    if (g_move_discard_slot == next) {
        move_release_discard();
    }

    move_retire_current();
    move_clear_slot(next);
    g_move_status[next].cmd_code = cmd_code;
    g_move_current_slot = next;
    move_exit_critical(primask);
}

void DFCom_MoveCommandSendFailed(u8 cmd_code)
{
    u32 primask;
    if (!move_is_bounded_cmd(cmd_code)) return;

    primask = move_enter_critical();
    if (g_move_session_active) {
        g_move_send_failed = 1;
        g_move_failed_cmd  = cmd_code;
    }
    move_exit_critical(primask);
}


/*============================================================================
 *  共享 scale 表 (跟小车端 DF_link.h 完全一致)
 *  ──────────────────────────────────────────────────────────────────────────
 *  物理量          | scale  | 量程        | 精度
 *  --------------- |--------|-------------|----------
 *  角度 (rad)      | 10000  | ±3.276 rad  | 0.0057°
 *  加速度 (m/s²)   |   400  | ±8.36g      | 0.0025 m/s²
 *  角速度 (rad/s)  |  1800  | ±1043°/s    | 0.032°/s
 *  速度 (m/s)      |  5000  | ±6.55 m/s   | 0.0002 m/s
 *  位置 (m, s32)   |  1000  | ±2147 km    | 1 mm
 *============================================================================*/
#define DFCOM_SCALE_ANGLE_RAD   10000.0f
#define DFCOM_SCALE_ACC_MPS2      400.0f
#define DFCOM_SCALE_GYRO_RAD     1800.0f
#define DFCOM_SCALE_VEL_MPS      5000.0f
#define DFCOM_SCALE_POS_M        1000.0f

/*============================================================================
 *  解析 1：Odom v4 数据帧 (A=0x6C, B=0x80, 51B payload)
 *  ──────────────────────────────────────────────────────────────────────────
 *  全量状态: roll/pitch/yaw + acc + gyro + vel(body/world) + pos(world) + ts
 *  小车端发起: DF_CoreState.c::DF_IMU_TIME
 *
 *    偏移      字段           类型     scale    含义
 *    ────      ───────────    ──────  ───────  ─────────────────────────────
 *    [0]       version        u8       --       0x04 (协议版本)
 *    [1..2]    roll_rad       s16 LE   ×10000  ROS roll (FLU)
 *    [3..4]    pitch_rad      s16 LE   ×10000  ROS pitch (FLU)
 *    [5..6]    yaw_rad        s16 LE   ×10000  ROS yaw (wrap [-π,π])
 *    [7..8]    acc_x          s16 LE   ×400    m/s² (specific force)
 *    [9..10]   acc_y          s16 LE   ×400    m/s²
 *    [11..12]  acc_z          s16 LE   ×400    m/s² (静止 ≈ +9.81)
 *    [13..14]  gyro_x         s16 LE   ×1800   rad/s
 *    [15..16]  gyro_y         s16 LE   ×1800   rad/s
 *    [17..18]  gyro_z         s16 LE   ×1800   rad/s (CCW+)
 *    [19..20]  vel_body_x     s16 LE   ×5000   m/s
 *    [21..22]  vel_body_y     s16 LE   ×5000   m/s
 *    [23..24]  vel_body_z     s16 LE   ×5000   m/s
 *    [25..26]  vel_world_x    s16 LE   ×5000   m/s
 *    [27..28]  vel_world_y    s16 LE   ×5000   m/s
 *    [29..30]  vel_world_z    s16 LE   ×5000   m/s
 *    [31..34]  pos_world_x    s32 LE   ×1000   m
 *    [35..38]  pos_world_y    s32 LE   ×1000   m
 *    [39..42]  pos_world_z    s32 LE   ×1000   m
 *    [43..46]  timestamp_ms   u32 LE   --      启动后毫秒数
 *    [47..50]  timestamp_us   u32 LE   --      毫秒下的微秒部分
 *============================================================================*/
static void parse_odom_v4(const u8 *payload, u8 payload_len)
{
    if (payload_len < 51) return;
    if (payload[0] != 0x04) return;  /* version 字节检查 */

    g_odom.roll_rad    = read_s16(&payload[1])  / DFCOM_SCALE_ANGLE_RAD;
    g_odom.pitch_rad   = read_s16(&payload[3])  / DFCOM_SCALE_ANGLE_RAD;
    g_odom.yaw_rad     = read_s16(&payload[5])  / DFCOM_SCALE_ANGLE_RAD;

    g_odom.acc_x_mps2  = read_s16(&payload[7])  / DFCOM_SCALE_ACC_MPS2;
    g_odom.acc_y_mps2  = read_s16(&payload[9])  / DFCOM_SCALE_ACC_MPS2;
    g_odom.acc_z_mps2  = read_s16(&payload[11]) / DFCOM_SCALE_ACC_MPS2;

    g_odom.gyro_x_rps  = read_s16(&payload[13]) / DFCOM_SCALE_GYRO_RAD;
    g_odom.gyro_y_rps  = read_s16(&payload[15]) / DFCOM_SCALE_GYRO_RAD;
    g_odom.gyro_z_rps  = read_s16(&payload[17]) / DFCOM_SCALE_GYRO_RAD;

    g_odom.b_vel_x_mps = read_s16(&payload[19]) / DFCOM_SCALE_VEL_MPS;
    g_odom.b_vel_y_mps = read_s16(&payload[21]) / DFCOM_SCALE_VEL_MPS;
    g_odom.b_vel_z_mps = read_s16(&payload[23]) / DFCOM_SCALE_VEL_MPS;

    g_odom.n_vel_x_mps = read_s16(&payload[25]) / DFCOM_SCALE_VEL_MPS;
    g_odom.n_vel_y_mps = read_s16(&payload[27]) / DFCOM_SCALE_VEL_MPS;
    g_odom.n_vel_z_mps = read_s16(&payload[29]) / DFCOM_SCALE_VEL_MPS;

    g_odom.n_pos_x_m   = read_s32(&payload[31]) / DFCOM_SCALE_POS_M;
    g_odom.n_pos_y_m   = read_s32(&payload[35]) / DFCOM_SCALE_POS_M;
    g_odom.n_pos_z_m   = read_s32(&payload[39]) / DFCOM_SCALE_POS_M;

    g_odom.car_time_int_s  = read_u32(&payload[43]);
    g_odom.car_time_dec_us = read_u32(&payload[47]);

    g_odom.local_tick_ms = g_local_tick_ms;
    g_odom.frame_count++;
    g_odom.fresh = 1;
}





/*============================================================================
 *  解析 2：运动完成回传 ProgressBack (A=0x6F, B=cmd)
 *  ──────────────────────────────────────────────────────────────────────────
 *  小车在执行运动指令时定期回传进度，到位/被打断时也回传"完成"。
 *  payload 2 字节：
 *
 *    [0] Process  u8   0..0xFE = 进度比例（×255/100），0xFF = 完成
 *    [1] Notice   u8   0       = 正常完成
 *                      1       = 版本不支持或参数非法，任务未启动
 *                      0x0A    = 遥控器接管
 *                      0x62~66 = 被对应的新运动指令打断
 *
 *  B 字段 = 原指令的 cmd 码：
 *    0x62 = CMD_VEL          (持续速度无回传, 一般不会到这)
 *    0x63 = CMD_ROT          (原地旋转)
 *    0x64 = CMD_LINEAR       (直线位移)
 *    0x65 = CMD_LINEAR_WITH_YAW (无头位移)
 *    0x66 = CMD_ARC          (圆弧)
 *
 *  A/B 两槽绑定的是“本地发送次序”，不是命令类型。旧槽只吞帧，不保存状态；
 *  当前槽才会写入进度。这样连续两条相同 B 的指令也不会共用状态槽。
 *============================================================================*/
static void parse_move_progress(u8 cmd_code, const u8 *payload, u8 payload_len)
{
    if (payload_len < 2) return;
    if (!g_move_session_active) return;
    if (!move_is_bounded_cmd(cmd_code)) return;

    /*
     * 同类型连续指令时，DCar 回传里没有本地槽号。按协议顺序，上一任务会先
     * 回唯一的 0xFF 终止帧；旧槽在此之前吞掉同 B 的全部帧，见到 0xFF 后释放。
     * 若这唯一终止帧真的在客户端丢失，无法证明身份的同 B 帧仍按旧帧丢弃：
     * 最坏结果是当前 Wait 等满上限，而不是错误提前完成并跳过动作。
     */
    if (g_move_discard_slot <= 1 && cmd_code == g_move_discard_cmd) {
        if (payload[0] == 0xFF) {
            move_release_discard();
        }
        return;
    }

    if (g_move_current_slot <= 1 &&
        g_move_status[g_move_current_slot].cmd_code == cmd_code) {
        g_move_status[g_move_current_slot].progress    = payload[0];
        g_move_status[g_move_current_slot].last_notice = payload[1];
        g_move_status[g_move_current_slot].fresh       = 1;
    }
}


/*============================================================================
 *  解析 3：激活/校准状态 DcarParmState (A=0x8C, B=0x40)
 *  ──────────────────────────────────────────────────────────────────────────
 *  Cmd_Query_DcarState() 触发；小车端 DF_DcarParmState() 返回。
 *  payload 12 字节：
 *
 *    [0]      ifIMUcali   u8   ★ 1 = IMU 已校准（小车可正常工作）★
 *                              0 = 未校准（需要先用上位机校准/激活！）
 *                              判定标准：flash.acc_offset.z ∈ (1, 8000)
 *    [1]      ifFinetune  u8   1 = 控制参数已微调
 *    [2..3]   gyro_z_off  s16  Gyro Z 偏置 × 100
 *    [4..5]   imu_par2    s16  IMU 参数 2 × 10000
 *    [6..7]   imu_temp    s16  IMU 温度参数
 *    [8..9]   ctrl_par1   s16  控制 Par1 × 100
 *    [10..11] ctrl_par2   s16  控制 Par2 × 100
 *
 *  → 解析到 g_dcar_state；启动时 main 用它打印诊断
 *============================================================================*/
static void parse_dcar_state(const u8 *payload, u8 payload_len)
{
    if (payload_len < 12) return;
    g_dcar_state.imu_calibrated    = payload[0];
    g_dcar_state.is_finetune       = payload[1];
    g_dcar_state.imu_offset_z_x100 = read_s16(&payload[2]);
    g_dcar_state.imu_param2_x10000 = read_s16(&payload[4]);
    g_dcar_state.imu_temp_x10000   = read_s16(&payload[6]);
    g_dcar_state.ctrl_par1_x100    = read_s16(&payload[8]);
    g_dcar_state.ctrl_par2_x100    = read_s16(&payload[10]);
    g_dcar_state.received = 1;
}


/* 按完整且已验证的帧分发。队列回包与无ID回包使用不同B值。 */
volatile DFComRxDiag_t g_dfcom_rx_diag;

static void dispatch_frame(const u8 *frame)
{
    u8 a = frame[3];
    u8 b = frame[4];
    u8 length = frame[5];
    const u8 *payload = frame + 6;
    if (a == 0x6C) {
        if (b == 0x80) {
            g_dfcom_rx_diag.odom_wire++;
            g_dfcom_rx_diag.odom_len = length;
            g_dfcom_rx_diag.odom_version = length ? payload[0] : 0;
            if (length < 51 || payload[0] != 4) g_dfcom_rx_diag.odom_rejected++;
            parse_odom_v4(payload, length);
        }
    } else if (a == 0x6F) {
        if (b == 0x85) DFQueue_Receive(payload, length);
        else parse_move_progress(b, payload, length);
    } else if (a == 0x8C && b == 0x40) {
        parse_dcar_state(payload, length);
    }
}

/* 串口字节流入口，不把一次IDLE中断当作一帧。
 * 保存不完整帧；校验后可一次分发多帧。缓存固定128字节，接受payload<=60。
 * 扫描后续候选，坏LEN不会一直挡住它后面的有效帧。仅由接收上下文调用。
 */
static u8 stream[128];
static u16 used;

void DFCom_RxReset(void)
{
    used = 0; /* 丢弃断流前的残帧，不清除已收到的遥测与队列结果。 */
}

void DFCom_RxParse(u8 *data, u16 size)
{
    u16 input;
    u16 header;
    u16 incomplete;
    u16 total;
    u16 sum;
    u16 received_sum;
    u16 i;
    u8 length;
    u8 found;

    for (input = 0; input < size; input++) {
        if (used == sizeof(stream)) {
            memmove(stream, stream + 1, --used);
        }
        stream[used++] = data[input];
        do {
            found = 0;
            incomplete = used;
            for (header = 0; header < used; header++) {
                if (stream[header] != DFCOM_FRAME_HEADER) continue;
                if (used - header < 3) {
                    if (header < incomplete) incomplete = header;
                    continue;
                }
                if (stream[header+1] != DFCOM_PC_ID || stream[header+2] != DFCOM_ROBOT_ID) continue;
                if (used - header < 6) {
                    if (header < incomplete) incomplete = header;
                    continue;
                }
                length = stream[header+5];
                if (length > 60) continue;
                total = (u16)length + 9;
                if (used - header < total) {
                    if (header < incomplete) incomplete = header;
                    continue;
                }
                if (stream[header+total-3] != DFCOM_FRAME_TAIL) continue;
                sum = 0;
                for (i = 0; i < total - 2; i++) sum += stream[header+i];
                received_sum = (u16)stream[header+total-2] | ((u16)stream[header+total-1] << 8);
                if (sum != received_sum) continue;
                dispatch_frame(stream + header);
                used -= header + total;
                memmove(stream, stream + header + total, used);
                found = 1;
                break;
            }
            if (!found) {
                /* 丢弃噪声，只保留最早的可能残帧；没有候选则清空。 */
                used -= incomplete;
                memmove(stream, stream + incomplete, used);
            }
        } while (found);
    }
}


/* 无ID运动的本地阻塞等待，当前main的队列示例不调用它。
 * 绑定最近已发送的同cmd任务；0无限等，正时限到达只结束等待，不发送停车。
 * 返回DONE/TIMEOUT/INTERRUPTED；无ID协议仍有丢包后同类迟到回包的身份歧义。
 */
u8 WaitMoveDone(u8 cmd_code, u32 timeout_ms)
{
    u8 slot;
    u8 notice;
    u8 send_failed;
    u32 primask;
    u32 start;

    /*
     * 发送成功时已经完成切槽。这里只拍下当前槽，不清任何状态；
     * 因而零距离/参数拒绝等“比 Wait 入口更快”的终止帧也不会被抹掉。
     */
    primask = move_enter_critical();
    send_failed = (g_move_send_failed && g_move_failed_cmd == cmd_code);
    if (g_move_current_slot <= 1 &&
        g_move_status[g_move_current_slot].cmd_code == cmd_code) {
        slot = g_move_current_slot;
    } else {
        slot = MOVE_SLOT_NONE;
    }
    move_exit_critical(primask);

    start = g_local_tick_ms;
    printf("[WAIT.IN]  cmd=0x%02X slot=%u tmo=%lu start_tick=%lu\r\n",
           (unsigned)cmd_code, (unsigned)slot,
           (unsigned long)timeout_ms, (unsigned long)start);

    if (send_failed) {
        printf("[WAIT.TX_FAIL] cmd=0x%02X (DMA frame was not started)\r\n",
               (unsigned)cmd_code);
        return MOVE_WAIT_TIMEOUT;
    }

    if (slot == MOVE_SLOT_NONE) {
        printf("[WAIT.NO_TASK] cmd=0x%02X (not the latest bounded command)\r\n",
               (unsigned)cmd_code);
        return MOVE_WAIT_TIMEOUT;
    }

    while (1) {
        /* 终止帧统一由 progress=0xFF 标识，notice 决定自然完成或被打断。 */
        if (g_move_status[slot].fresh &&
            g_move_status[slot].progress == 0xFF) {
            notice = g_move_status[slot].last_notice;
            if (notice == MOVE_NOTICE_NONE) {
                printf("[WAIT.DONE] cmd=0x%02X waited=%lums\r\n",
                       (unsigned)cmd_code,
                       (unsigned long)(g_local_tick_ms - start));
                return MOVE_WAIT_DONE;
            }

            printf("[WAIT.INTERRUPTED] cmd=0x%02X notice=0x%02X waited=%lums\r\n",
                   (unsigned)cmd_code,
                   (unsigned)notice,
                   (unsigned long)(g_local_tick_ms - start));
            return MOVE_WAIT_INTERRUPTED;
        }
        if (timeout_ms != 0) {                /* timeout_ms == 0 ⇒ 永久等待 */
            if ((g_local_tick_ms - start) >= timeout_ms) {
                printf("[WAIT.TIMEOUT] cmd=0x%02X waited=%lums now_tick=%lu (no terminal progress=FF)\r\n",
                       (unsigned)cmd_code,
                       (unsigned long)(g_local_tick_ms - start),
                       (unsigned long)g_local_tick_ms);
                return MOVE_WAIT_TIMEOUT;
            }
        }
        /* 无ID等待每10ms检查一次；保持串口与tick中断运行。 */
        DFCom_Poll();
        delay_ms(10);
    }
}

u8 GetMoveProgress(u8 cmd_code)
{
    u8 slot = g_move_current_slot;
    if (slot <= 1 && g_move_status[slot].cmd_code == cmd_code) {
        return g_move_status[slot].progress;
    }
    return 0;
}

u8 GetMoveNotice(u8 cmd_code)
{
    u8 slot = g_move_current_slot;
    if (slot <= 1 && g_move_status[slot].cmd_code == cmd_code) {
        return g_move_status[slot].last_notice;
    }
    return 0;
}
