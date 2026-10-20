/* Uno/Nano持续操场测试：Serial连接车端115200，TX经5V->3.3V转换。
 * 每圈四段；自然完成后立即下一段/下一圈，任一步失败停车并停止新任务。
 * 无ID接口按段等待，不模拟v6队列。需要全向底盘及相应车端运动能力。
 * 独立订阅Odom/VelPos，查询状态仅报告；不等待遥测才运动。
 * 数据与计数通过DFCom.odom/velpos/state/odomDiag/velposDiag读取。
 * Uno唯一硬件串口留给车端；不使用会屏蔽接收中断的SoftwareSerial日志。
 */
#include <DFCom.h>

bool testStopped = false;
uint32_t completedLaps = 0;

void setup()
{
    DFCom.begin(115200);              // 双停车，上电1000ms与会话间隔50ms
    DFCom.subscribeOdom(10);
    DFCom.serviceDelay(20);
    DFCom.subscribeVelPos(10);
    DFCom.queryState();
    DFCom.serviceDelay(500);          // 固定观察窗口，不以状态/遥测作为门禁
    DFCom.serviceDelay(3000);         // 首圈前等待，后续各圈无延时
}

bool segmentDone()
{
    if (DFCom.waitDone(60000)) return true;
    DFCom.stop();
    testStopped = true;
    return false;
}

void loop()
{
    DFCom.update();                   // 即使运动失败也继续收真实遥测
    if (testStopped) return;
    DFCom.moveArc(30, 180, -15, 1);    // 半径30cm，倒行半圆
    if (!segmentDone()) return;
    DFCom.moveLinear(-60, 0, 15, 1);   // 后退60cm
    if (!segmentDone()) return;
    DFCom.moveArc(30, 180, -15, 1);
    if (!segmentDone()) return;
    DFCom.moveLinearWithYaw(-60, 0, 360, 15, 1); // 后退同时旋转一圈
    if (!segmentDone()) return;
    completedLaps++;                  // loop立即开始下一圈，不添加圈间延时
}
