/******************************************************************************
 * DFCom.cpp  ——  Arduino 版 DcarON 通信库实现
 * 无任务ID的逐条DFLink接口；SI遥测和REP-103方向与其他平台一致，不支持队列v6。
 ******************************************************************************/
#include "DFCom.h"
#include <string.h>

/* ---- 协议常量 ---- */
static const uint8_t DF_HEAD = 0xDF;
static const uint8_t DF_TAIL = 0xFD;
static const uint8_t DF_ROBOT_ID = 0x01;   /* 小车 ID */
static const uint8_t DF_PC_ID    = 0x97;   /* 上位机 (本 Arduino) */
static const uint8_t MOVE_SLOT_NONE = 0xFF;

/* ---- 回传 scale 表 (与小车端一致) ---- */
#define SC_ANGLE 10000.0f
#define SC_ACC     400.0f
#define SC_GYRO   1800.0f
#define SC_VEL    5000.0f
#define SC_POS    1000.0f
#define SC_SI    10000.0f   /* 发送侧: 所有 SI 量 ×10000 编 s32 */

/* 全局实例 */
DFCom_t DFCom;

/* ---- 小端读写 ---- */
static int16_t  rd_s16(const uint8_t *p){ return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static int32_t  rd_s32(const uint8_t *p){ return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24)); }
static uint32_t rd_u32(const uint8_t *p){ return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void put_s32(uint8_t *b, int32_t v){ b[0]=(uint8_t)v; b[1]=(uint8_t)(v>>8); b[2]=(uint8_t)(v>>16); b[3]=(uint8_t)(v>>24); }

/* ===========================================================================
 * 初始化
 * ===========================================================================*/
void DFCom_t::begin(long baud, HardwareSerial &port)
{
    _ser = &port;
    _ser->begin(baud);
    _idx = 0;

    /*
     * Arduino 可能在 DCar 已经执行旧任务时复位。先隔离回传并连续停车两次：
     * 第一帧尽快止车，第二帧跨过 DCar 供电/串口稳定期后再次确认。
     */
    _resetMotionSession(0);
    moveVel(0, 0, 0);
    serviceDelay(1000);
    moveVel(0, 0, 0);
    serviceDelay(50);
    update();
    _idx = 0;
    _resetMotionSession(1);
}

/* ---- 单位转换 (按当前模式把入参折算成 SI) ---- */
float DFCom_t::_toM(float v)
{
    if (_unit == DFCOM_UNIT_CM) return v * 0.01f;
    if (_unit == DFCOM_UNIT_MM) return v * 0.001f;
    return v;   /* M 模式: 已是米 */
}
float DFCom_t::_toRad(float v)
{
    if (_unit == DFCOM_UNIT_M) return v;   /* M 模式: 已是弧度 */
    return v * 0.01745329f;                /* CM/MM 模式: 度→弧度 */
}

/* ===========================================================================
 * 建帧 + 校验 + 发送
 *   [0xDF][ROBOT_ID][PC_ID][A][B][LEN][payload..][0xFD][csum_lo][csum_hi]
 *   校验和 = [0] 累加到 [0xFD] (含), u16 LE
 * ===========================================================================*/
bool DFCom_t::_frame(uint8_t A, uint8_t B, const uint8_t *payload, uint8_t len)
{
    uint8_t f[72];
    uint8_t n = 0;
    f[n++] = DF_HEAD;
    f[n++] = DF_ROBOT_ID;
    f[n++] = DF_PC_ID;
    f[n++] = A;
    f[n++] = B;
    f[n++] = len;
    for (uint8_t i = 0; i < len; i++) f[n++] = payload[i];
    f[n++] = DF_TAIL;
    uint16_t sc = 0;
    uint8_t sum_len = (uint8_t)(len + 7);   /* header6 + tail1 */
    for (uint8_t i = 0; i < sum_len; i++) sc += f[i];
    f[n++] = (uint8_t)(sc & 0xFF);
    f[n++] = (uint8_t)((sc >> 8) & 0xFF);
    return _ser->write(f, n) == n;
}

bool DFCom_t::_isBoundedMotion(uint8_t cmd)
{
    return cmd >= DFCOM_CMD_ROT && cmd <= DFCOM_CMD_ARC;
}

void DFCom_t::_clearMoveSlot(uint8_t slot)
{
    if (slot > 1) return;
    _moveSlots[slot].cmd = 0;
    _moveSlots[slot].progress = 0;
    _moveSlots[slot].notice = 0;
    _moveSlots[slot].fresh = 0;
}

void DFCom_t::_releaseDiscard()
{
    if (_discardSlot <= 1) _clearMoveSlot(_discardSlot);
    _discardSlot = MOVE_SLOT_NONE;
    _discardCmd = 0;
}

void DFCom_t::_retireCurrent()
{
    uint8_t slot = _currentSlot;
    if (slot > 1) return;

    uint8_t cmd = _moveSlots[slot].cmd;
    bool terminal = _moveSlots[slot].fresh &&
                    _moveSlots[slot].progress == 0xFF;

    if (terminal) {
        _clearMoveSlot(slot);
    } else {
        if (_discardSlot <= 1 && _discardSlot != slot) _releaseDiscard();
        _clearMoveSlot(slot);
        _discardSlot = slot;
        _discardCmd = cmd;
    }
    _currentSlot = MOVE_SLOT_NONE;
}

uint8_t DFCom_t::_nextMoveSlot() const
{
    if (_currentSlot <= 1) return _currentSlot ^ 1U;
    if (_discardSlot <= 1) return _discardSlot ^ 1U;
    return 0;
}

void DFCom_t::_resetMotionSession(uint8_t active)
{
    _clearMoveSlot(0);
    _clearMoveSlot(1);
    _currentSlot = MOVE_SLOT_NONE;
    _discardSlot = MOVE_SLOT_NONE;
    _discardCmd = 0;
    _sendFailed = 0;
    _failedCmd = 0;
    _motionSessionActive = active;
}

void DFCom_t::_motionSent(uint8_t cmd)
{
    if (cmd != DFCOM_CMD_VEL && !_isBoundedMotion(cmd)) return;

    _sendFailed = 0;
    _failedCmd = 0;
    if (!_motionSessionActive) return;

    if (cmd == DFCOM_CMD_VEL) {
        _retireCurrent();
        return;
    }

    uint8_t next = _nextMoveSlot();
    if (_discardSlot == next) _releaseDiscard();
    _retireCurrent();
    _clearMoveSlot(next);
    _moveSlots[next].cmd = cmd;
    _currentSlot = next;
}

void DFCom_t::_motionSendFailed(uint8_t cmd)
{
    if (_motionSessionActive && _isBoundedMotion(cmd)) {
        _sendFailed = 1;
        _failedCmd = cmd;
    }
}

void DFCom_t::_routeMotionProgress(uint8_t cmd, uint8_t progressValue,
                                   uint8_t noticeValue)
{
    if (!_motionSessionActive || !_isBoundedMotion(cmd)) return;

    if (_discardSlot <= 1 && cmd == _discardCmd) {
        if (progressValue == 0xFF) _releaseDiscard();
        return;
    }

    if (_currentSlot <= 1 && _moveSlots[_currentSlot].cmd == cmd) {
        _moveSlots[_currentSlot].progress = progressValue;
        _moveSlots[_currentSlot].notice = noticeValue;
        _moveSlots[_currentSlot].fresh = 1;
    }
}

/* ===========================================================================
 * 运动指令 (class A=0x02 DataRecivControClass)
 * ===========================================================================*/
void DFCom_t::moveLinear(float px, float py, float speed, uint8_t profile)
{
    px = _toM(px); py = _toM(py); speed = _toM(speed);
    _lastCmd = DFCOM_CMD_LINEAR;
    if (speed <= 0.0f) {
        _motionSendFailed(DFCOM_CMD_LINEAR);
        return;
    }
    if (profile > 2) profile = 2;
    uint8_t p[13];
    put_s32(&p[0], (int32_t)(px * SC_SI));
    put_s32(&p[4], (int32_t)(py * SC_SI));
    put_s32(&p[8], (int32_t)(speed * SC_SI));
    p[12] = profile;
    if (_frame(0x02, DFCOM_CMD_LINEAR, p, 13)) {
        _motionSent(DFCOM_CMD_LINEAR);
    } else {
        _motionSendFailed(DFCOM_CMD_LINEAR);
    }
}

void DFCom_t::moveLinearWithYaw(float px, float py, float dyaw, float speed, uint8_t profile)
{
    px = _toM(px); py = _toM(py); dyaw = _toRad(dyaw); speed = _toM(speed);
    _lastCmd = DFCOM_CMD_LWY;
    if (speed <= 0.0f) {
        _motionSendFailed(DFCOM_CMD_LWY);
        return;
    }
    if (profile > 2) profile = 2;
    uint8_t p[17];
    put_s32(&p[0],  (int32_t)(px * SC_SI));
    put_s32(&p[4],  (int32_t)(py * SC_SI));
    put_s32(&p[8],  (int32_t)(dyaw * SC_SI));
    put_s32(&p[12], (int32_t)(speed * SC_SI));
    p[16] = profile;
    if (_frame(0x02, DFCOM_CMD_LWY, p, 17)) {
        _motionSent(DFCOM_CMD_LWY);
    } else {
        _motionSendFailed(DFCOM_CMD_LWY);
    }
}

void DFCom_t::moveRot(float dyaw, float omegaMax)
{
    dyaw = _toRad(dyaw); omegaMax = _toRad(omegaMax);
    _lastCmd = DFCOM_CMD_ROT;
    if (omegaMax <= 0.0f) {
        _motionSendFailed(DFCOM_CMD_ROT);
        return;
    }
    uint8_t p[8];
    put_s32(&p[0], (int32_t)(dyaw * SC_SI));
    put_s32(&p[4], (int32_t)(omegaMax * SC_SI));
    if (_frame(0x02, DFCOM_CMD_ROT, p, 8)) {
        _motionSent(DFCOM_CMD_ROT);
    } else {
        _motionSendFailed(DFCOM_CMD_ROT);
    }
}

void DFCom_t::moveVel(float vx, float vy, float vz)
{
    vx = _toM(vx); vy = _toM(vy); vz = _toRad(vz);
    uint8_t p[12];
    put_s32(&p[0], (int32_t)(vx * SC_SI));
    put_s32(&p[4], (int32_t)(vy * SC_SI));
    put_s32(&p[8], (int32_t)(vz * SC_SI));
    /* 持续速度无完成回传, 不动 _lastCmd / 不清槽 */
    if (_frame(0x02, DFCOM_CMD_VEL, p, 12)) {
        _motionSent(DFCOM_CMD_VEL);
    }
}

void DFCom_t::moveArc(float radius, float dyaw, float speed, uint8_t profile)
{
    radius = _toM(radius); dyaw = _toRad(dyaw); speed = _toM(speed);
    _lastCmd = DFCOM_CMD_ARC;
    if (radius <= 0.0f || speed == 0.0f) {
        _motionSendFailed(DFCOM_CMD_ARC);
        return;
    }
    if (profile > 2) profile = 2;
    uint8_t p[13];
    put_s32(&p[0], (int32_t)(radius * SC_SI));
    put_s32(&p[4], (int32_t)(dyaw * SC_SI));
    put_s32(&p[8], (int32_t)(speed * SC_SI));
    p[12] = profile;
    if (_frame(0x02, DFCOM_CMD_ARC, p, 13)) {
        _motionSent(DFCOM_CMD_ARC);
    } else {
        _motionSendFailed(DFCOM_CMD_ARC);
    }
}

/* ===========================================================================
 * 里程计订阅 (A=0x04 DataRecivViitClass, B=0x80) / 状态查询
 * ===========================================================================*/
static uint8_t visit_freq_code(uint16_t freqHz)
{
    if      (freqHz == 10)  return 1;
    else if (freqHz == 50)  return 5;
    else if (freqHz == 100) return 10;
    else if (freqHz == 200) return 20;
    else if (freqHz == 250) return 25;
    else if (freqHz == 500) return 50;
    return 1;     /* 其他 → 10Hz */
}

static uint8_t visit_type_code(uint8_t mode)
{
    return (mode == DFCOM_MODE_CONTINUOUS) ? 0x01 : 0x02;  /* STOP→ONESHOT */
}

void DFCom_t::subscribeOdom(uint16_t freqHz, uint8_t mode)
{
    uint8_t fcode = visit_freq_code(freqHz);
    uint8_t vtype = visit_type_code(mode);
    uint8_t p[2] = { vtype, fcode };
    _frame(0x04, 0x80, p, 2);
}

void DFCom_t::queryState()
{
    _frame(0x0B, 0x40, nullptr, 0);
}

/* ===========================================================================
 * 接收: 字节流解析状态机 (Arduino 无 IDLE 中断, 在 loop/waitDone 里逐字节喂)
 * ===========================================================================*/
void DFCom_t::update()
{
    while (_ser->available()) { rxBytes++; _feed((uint8_t)_ser->read()); }
}

void DFCom_t::serviceDelay(uint32_t ms)
{
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < ms) {
        update();
        delay(1);
    }
}

void DFCom_t::_feed(uint8_t b)
{
    if (_idx == sizeof(_buf)) {
        memmove(_buf, _buf + 1, --_idx);
    }
    _buf[_idx++] = b;
    bool found;
    do {
        found = false;
        uint8_t incomplete = _idx;
        for (uint8_t start = 0; start < _idx; start++) {
            if (_buf[start] != DF_HEAD) continue;
            if (_idx-start < 6) { if (start < incomplete) incomplete = start; continue; }
            if (_buf[start+1] != DF_PC_ID || _buf[start+2] != DF_ROBOT_ID || _buf[start+5] > 60) continue;
            uint8_t total = _buf[start+5] + 9;
            if (_idx-start < total) { if (start < incomplete) incomplete = start; continue; }
            if (_buf[start+total-3] != DF_TAIL) continue;
            uint16_t sum = 0;
            for (uint8_t i = 0; i < total-2; i++) sum += _buf[start+i];
            uint16_t received = (uint16_t)_buf[start+total-2] | ((uint16_t)_buf[start+total-1]<<8);
            if (sum != received) continue;
            _dispatch(_buf[start+3], _buf[start+4], _buf+start+6, _buf[start+5]);
            _idx -= start+total;
            memmove(_buf, _buf+start+total, _idx);
            found = true;
            break;
        }
        if (!found) {
            _idx -= incomplete;
            memmove(_buf, _buf+incomplete, _idx);
        }
    } while (found);
}

void DFCom_t::_dispatch(uint8_t A, uint8_t B, const uint8_t *p, uint8_t len)
{
    if (A == 0x6C && B == 0x80) {
        DFStreamDiag &diag = odomDiag;
        diag.wire++; diag.len = len; diag.version = len ? p[0] : 0;
        if (len < 51 || diag.version != 4) diag.rejected++;
    }
    if (A == 0x6C && B == 0x80 && len >= 51 && p[0] == 0x04) {
        /* DF_odom 全量 51B */
        odom.roll  = rd_s16(&p[1])  / SC_ANGLE;
        odom.pitch = rd_s16(&p[3])  / SC_ANGLE;
        odom.yaw   = rd_s16(&p[5])  / SC_ANGLE;
        odom.accX  = rd_s16(&p[7])  / SC_ACC;
        odom.accY  = rd_s16(&p[9])  / SC_ACC;
        odom.accZ  = rd_s16(&p[11]) / SC_ACC;
        odom.gyroX = rd_s16(&p[13]) / SC_GYRO;
        odom.gyroY = rd_s16(&p[15]) / SC_GYRO;
        odom.gyroZ = rd_s16(&p[17]) / SC_GYRO;
        odom.bVelX = rd_s16(&p[19]) / SC_VEL;
        odom.bVelY = rd_s16(&p[21]) / SC_VEL;
        odom.bVelZ = rd_s16(&p[23]) / SC_VEL;
        odom.nVelX = rd_s16(&p[25]) / SC_VEL;
        odom.nVelY = rd_s16(&p[27]) / SC_VEL;
        odom.nVelZ = rd_s16(&p[29]) / SC_VEL;
        odom.posX  = rd_s32(&p[31]) / SC_POS;
        odom.posY  = rd_s32(&p[35]) / SC_POS;
        odom.posZ  = rd_s32(&p[39]) / SC_POS;
        odom.carTimeS  = rd_u32(&p[43]);
        odom.carTimeUs = rd_u32(&p[47]);
        odom.localTickMs = millis();
        odom.frameCount++;
        odom.fresh = 1;
    }
    else if (A == 0x6F && len >= 2) {
        /* 运动完成/进度回传, B = 原 cmd 码 */
        _routeMotionProgress(B, p[0], p[1]);
    }
    else if (A == 0x8C && B == 0x40 && len >= 12) {
        /* 激活/校准状态 */
        state.imuCalibrated   = p[0];
        state.isFinetune      = p[1];
        state.gyroZoffX100    = rd_s16(&p[2]);
        state.imuParam2X10000 = rd_s16(&p[4]);
        state.imuTempX10000   = rd_s16(&p[6]);
        state.ctrlPar1X100    = rd_s16(&p[8]);
        state.ctrlPar2X100    = rd_s16(&p[10]);
        state.received = 1;
    }
}

/* ===========================================================================
 * 等运动完成 (阻塞, 内部持续 update 收解析; millis 计时)
 *   wait 只观察发送时已经绑定的当前槽，绝不在入口清状态。
 * ===========================================================================*/
bool DFCom_t::waitDone(uint8_t cmdCode, uint32_t timeoutMs)
{
    bool sendFailed = _sendFailed && _failedCmd == cmdCode;
    uint8_t slot = (_currentSlot <= 1 &&
                    _moveSlots[_currentSlot].cmd == cmdCode)
                       ? _currentSlot
                       : MOVE_SLOT_NONE;

    if (sendFailed || slot == MOVE_SLOT_NONE) return false;

    uint32_t start = millis();
    for (;;) {
        update();   /* 紧凑轮询, 及时清空硬件 RX 缓冲 (避免 64B 溢出) */
        if (_moveSlots[slot].fresh &&
            _moveSlots[slot].progress == 0xFF) {
            return _moveSlots[slot].notice == 0x00;
        }
        if (timeoutMs != 0 && (millis() - start) >= timeoutMs) return false;
    }
}

uint8_t DFCom_t::progress(uint8_t cmdCode)
{
    if (_currentSlot <= 1 && _moveSlots[_currentSlot].cmd == cmdCode) {
        return _moveSlots[_currentSlot].progress;
    }
    return 0;
}
