/* Uno/Nano持续操场测试：Serial连接车端115200，TX经5V->3.3V转换。
 * 每圈四段；自然完成后立即下一段/下一圈，任一步失败停车并停止新任务。
 * 无ID接口按段等待，不模拟v6队列。需要全向底盘及相应车端运动能力。
 * 只订阅Odom，查询状态仅报告；不等待遥测才运动。
 * 数据与计数通过DFCom.odom/state/odomDiag读取。
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
