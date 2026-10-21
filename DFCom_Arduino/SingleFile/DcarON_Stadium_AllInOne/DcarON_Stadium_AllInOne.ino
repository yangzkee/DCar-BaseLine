/* 独立单文件操场例程；无需安装DFCom库。 */
/******************************************************************************
 * DFCom.h  ——  Arduino (ATmega328P) 版 DcarON 小车通信库
 * ----------------------------------------------------------------------------
 * Arduino Uno/Nano无任务ID逐条运动接口；未实现STM32/ROS的队列v6。
 * 官网: https://differ-tech.pages.dev/
 *
 *  ╔══════════════════════════════════════════════════════════════════════╗
 *  ║  接线 (Arduino Uno/Nano 5V  ↔  小车 USART3 3.3V):                      ║
 *  ║    Arduino D1(TX) → [电平转换 5V→3.3V] → 车 USART3_RX (PB11)           ║
 *  ║    Arduino D0(RX) ←────── 直连 ──────── 车 USART3_TX (PB10)            ║
 *  ║    Arduino GND   ←──────────────────── 车 GND                          ║
 *  ║  波特率固定 115200。调试想看打印 → 用 SoftwareSerial (见例程)。        ║
 *  ╚══════════════════════════════════════════════════════════════════════╝
 *
 *  坐标系 (ROS REP-103):  +X 前进 · +Y 左移 · +Yaw 逆时针(CCW) 为正
 *
 *  默认单位 = 厘米 + 度 (学生友好)。想用米+弧度: DFCom.useMeters();
 *
 *  最简用法:
 *    #include <DFCom.h>
 *    void setup(){ DFCom.begin(); DFCom.subscribeOdom(10); }
 *    void loop(){
 *      DFCom.moveLinear(50, 0, 30, 2); DFCom.waitDone(10000); // 前进 50cm
 *      DFCom.moveRot(90);             DFCom.waitDone(10000); // 左转 90°
 *    }
 ******************************************************************************/
#ifndef DFCOM_H
#define DFCOM_H

#include <Arduino.h>

/* ---- ODOM 订阅 ---- */
#define DFCOM_MODE_CONTINUOUS 0x01   /* 持续推送 (最常用) */
#define DFCOM_MODE_ONESHOT    0x02   /* 只发一帧 */
#define DFCOM_MODE_STOP       0x00   /* 停止推送 */

/* ---- 指令码 (waitDone 用) ---- */
#define DFCOM_CMD_VEL     0x62
#define DFCOM_CMD_ROT     0x63
#define DFCOM_CMD_LINEAR  0x64
#define DFCOM_CMD_LWY     0x65   /* LinearWithYaw */
#define DFCOM_CMD_ARC     0x66

/* ---- 单位模式 ---- */
#define DFCOM_UNIT_CM 0   /* 默认: 厘米 + 度 */
#define DFCOM_UNIT_M  1   /* 米 + 弧度 (SI 原生) */
#define DFCOM_UNIT_MM 2   /* 毫米 + 度 */

#define DFCOM_RAD2DEG(r) ((r) * 57.29578f)
#define DFCOM_DEG2RAD(d) ((d) * 0.01745329f)

/* 统一Odom，车端按DCar型号提供普通/全量字段，应用无需切换订阅。
 * Pro：rpy、acc、gyro、车体系/固定里程计系速度、位置与时间戳。
 * 普通款：yaw、两系速度、位置与时间；roll/pitch/acc/gyro为零占位。
 * SI原生单位：m、m/s、rad、rad/s；零占位不代表真实IMU测量。 */
struct DFOdom {
    float roll, pitch, yaw;          /* rad, yaw CCW+ */
    float accX, accY, accZ;          /* m/s² */
    float gyroX, gyroY, gyroZ;       /* rad/s */
    float bVelX, bVelY, bVelZ;       /* 车体系速度 m/s */
    float nVelX, nVelY, nVelZ;       /* 世界系速度 m/s */
    float posX, posY, posZ;          /* 世界系位置 m (开机累积) */
    uint32_t carTimeS, carTimeUs;
    uint32_t frameCount;
    uint32_t localTickMs;
    uint8_t  fresh;
};

/* 小车激活/校准状态 */
struct DFState {
    uint8_t received;        /* 1 = 收到过回传 (链路活) */
    uint8_t imuCalibrated;   /* 1 = IMU 已校准 (可用) */
    uint8_t isFinetune;
    int16_t gyroZoffX100;
    int16_t imuParam2X10000;
    int16_t imuTempX10000;
    int16_t ctrlPar1X100;
    int16_t ctrlPar2X100;
};

/* 独立统计真实有效外层帧与成功解码；遥测可用性不决定运动是否发送。 */
struct DFStreamDiag {
    uint32_t wire = 0, rejected = 0;
    uint8_t len = 0, version = 0;
};

class DFCom_t {
public:
    /* ---- 初始化（内部会先停车并隔离旧运动回传） ---- */
    void begin(long baud = 115200, HardwareSerial &port = Serial);

    /* ---- 单位 (默认厘米+度) ---- */
    void useCentimeters() { _unit = DFCOM_UNIT_CM; }
    void useMeters()      { _unit = DFCOM_UNIT_M;  }
    void useMillimeters() { _unit = DFCOM_UNIT_MM; }

    /* ---- 运动指令 (单位随当前模式; 默认 cm + deg) ----
     *   +X 前进(px/vx>0) · +Y 左移(py/vy>0) · +yaw 左转(dyaw>0, CCW)
     *   profile: 0=匀速  1=梯形加减速  2=高精度距离闭环 */
    void moveLinear(float px, float py, float speed, uint8_t profile = 2);
    void moveLinearWithYaw(float px, float py, float dyaw, float speed, uint8_t profile = 1);
    void moveRot(float dyaw, float omegaMax = 60);   /* 默认 60 deg/s */
    void moveVel(float vx, float vy, float vz);       /* 持续速度, 不自动停 */
    void moveArc(float radius, float dyaw, float speed, uint8_t profile = 1);
    void stop() { moveVel(0, 0, 0); }                 /* 主动停车 */

    /* ---- 里程计订阅 / 状态查询 ---- */
    void subscribeOdom(uint16_t freqHz = 10, uint8_t mode = DFCOM_MODE_CONTINUOUS);
    void queryState();

    /* ---- 必须在 loop() 里(或 waitDone 内部)调, 收+解析回传 ---- */
    void update();

    /* ---- 等运动完成 (阻塞, 内部自动 update) ----
     *   waitDone()            等最近一条运动指令
     *   waitDone(cmd, tmo_ms) 指定指令; tmo=0 表示永久等待
     *   tmo>0 是最长执行时间: 提前到位就提前返回, 到时可发下一条打断
     *   返回 true=自然完成, false=超时/中断/发送失败 */
    bool waitDone(uint32_t timeoutMs = 0) { return waitDone(_lastCmd, timeoutMs); }
    bool waitDone(uint8_t cmdCode, uint32_t timeoutMs);
    uint8_t progress(uint8_t cmdCode);

    /* ---- 数据 (SI 原生, 直接读) ---- */
    DFOdom   odom;
    DFState  state;
    DFStreamDiag odomDiag;
    uint32_t rxBytes = 0;
    bool odomAvailable() const { return odom.frameCount && (uint32_t)(millis()-odom.localTickMs)<1000; }
    void serviceDelay(uint32_t ms); /* 延时期间持续收帧；日志/状态不阻塞运动。 */

    /* ---- 友好取值 (换成 cm / 度) ---- */
    float xCm()    { return odom.posX * 100.0f; }
    float yCm()    { return odom.posY * 100.0f; }
    float yawDeg() { return DFCOM_RAD2DEG(odom.yaw); }
    bool  imuOK()  { return state.received && state.imuCalibrated; }

private:
    HardwareSerial *_ser = &Serial;
    uint8_t _unit = DFCOM_UNIT_CM;
    uint8_t _lastCmd = DFCOM_CMD_LINEAR;

    struct MoveSlot {
        uint8_t cmd;
        uint8_t progress;
        uint8_t notice;
        uint8_t fresh;
    };

    /* 两槽按本地发送顺序交替；上一任务的尾帧只进入丢弃槽。 */
    MoveSlot _moveSlots[2];
    uint8_t _currentSlot = 0xFF;
    uint8_t _discardSlot = 0xFF;
    uint8_t _discardCmd = 0;
    uint8_t _motionSessionActive = 0;
    uint8_t _sendFailed = 0;
    uint8_t _failedCmd = 0;

    /* 有界字节流缓存；保留残帧，扫描后续有效帧，支持遥测与任务回包粘连。 */
    uint8_t _buf[72];
    uint8_t _idx = 0;

    bool _frame(uint8_t A, uint8_t B, const uint8_t *payload, uint8_t len);  /* 建帧+校验+发送 */
    void _feed(uint8_t byteIn);                                              /* 字节流解析 */
    void _dispatch(uint8_t A, uint8_t B, const uint8_t *p, uint8_t len);
    static bool _isBoundedMotion(uint8_t cmd);
    void _clearMoveSlot(uint8_t slot);
    void _releaseDiscard();
    void _retireCurrent();
    uint8_t _nextMoveSlot() const;
    void _resetMotionSession(uint8_t active);
    void _motionSent(uint8_t cmd);
    void _motionSendFailed(uint8_t cmd);
    void _routeMotionProgress(uint8_t cmd, uint8_t progress, uint8_t notice);
    float _toM(float v);     /* 长度入参 → 米 (按单位模式) */
    float _toRad(float v);   /* 角度入参 → 弧度 (按单位模式) */
};

extern DFCom_t DFCom;

#endif /* DFCOM_H */

/******************************************************************************
 * DFCom.cpp  ——  Arduino 版 DcarON 通信库实现
 * 无任务ID的逐条DFLink接口；SI遥测和REP-103方向与其他平台一致，不支持队列v6。
 ******************************************************************************/

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
        /* 统一Odom 51B；Pro全量，普通款roll/pitch/acc/gyro为零占位。 */
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

/* Uno/Nano持续操场测试：Serial连接车端115200，TX经5V->3.3V转换。
 * 每圈四段；自然完成后立即下一段/下一圈，任一步失败停车并停止新任务。
 * 无ID接口按段等待，不模拟v6队列。需要全向底盘及相应车端运动能力。
 * 只订阅Odom，查询状态仅报告；不等待遥测才运动。
 * 数据与计数通过DFCom.odom/state/odomDiag读取。
 * Uno唯一硬件串口留给车端；不使用会屏蔽接收中断的SoftwareSerial日志。
 */


bool testStopped = false;
uint32_t completedLaps = 0;

void setup()
{
    DFCom.begin(115200);              // 双停车，上电1000ms与会话间隔50ms
    DFCom.subscribeOdom(10);
    DFCom.serviceDelay(20);
    DFCom.queryState();
    DFCom.serviceDelay(500);          // 固定观察窗口，不以状态/遥测作为门禁
    DFCom.serviceDelay(3000);         // 首圈前等待，后续各圈无延时
}

bool segmentDone()
{
    if (DFCom.waitDone(60000)) return true;
    DFCom.stop();                    // 无参数；发送全零速度停车，不提交后续段。
    testStopped = true;
    return false;
}

void loop()
{
    DFCom.update();                   // 即使运动失败也继续收真实遥测
    if (testStopped) return;
    // 以下每行独立调参；本例程保持默认厘米/度，实际速度与加速度由车端能力限制。
    DFCom.moveArc(30, 180, -15, 1);    // radius=30cm须>0，越大弧越宽；yaw=180度，正逆时针/负顺时针，绝对值越大转得越多；speed=-15cm/s须非0，负倒行/正前行，绝对值越大越快；profile=1梯形加减速，可填0匀速/1梯形/2距离闭环，不是16~20档位。
    if (!segmentDone()) return;
    DFCom.moveLinear(-60, 0, 15, 1);   // x=-60cm后退，正前/负后、绝对值越大越远；y=0cm不侧移，正左/负右；speed=15cm/s须>0，越大越快，后退用负x；profile=1梯形加减速，可填0匀速/1梯形/2距离闭环。
    if (!segmentDone()) return;
    DFCom.moveArc(30, 180, -15, 1);    // radius=30cm须>0，增大弧更宽；yaw=180度，正逆时针/负顺时针，绝对值控制转角；speed=-15cm/s须非0，负倒行/正前行，绝对值控制快慢；profile=1梯形，可填0匀速/1梯形/2距离闭环；本行可独立于第一段修改。
    if (!segmentDone()) return;
    DFCom.moveLinearWithYaw(-60, 0, 360, 15, 1); // x=-60cm后退，正前/负后；y=0cm不侧移，正左/负右、位移绝对值越大越远；yaw=360度，平移同时逆时针转一圈，负值顺时针、0不转；speed=15cm/s须>0，越大越快；profile=1梯形，可填0匀速/1梯形/2距离闭环。
    if (!segmentDone()) return;
    completedLaps++;                  // loop立即开始下一圈，不添加圈间延时
}
