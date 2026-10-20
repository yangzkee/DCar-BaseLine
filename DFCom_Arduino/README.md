# Arduino Uno / Nano 客户端

当前目录提供 **无任务ID的逐条运动接口**，库版本2.0.1。
它未实现STM32/ROS的队列v6；不能把 `DFQueue_*`、profile16..20或轨迹YAML直接套到这里。
它与其他平台使用相同的DFLink外层帧和ROS REP-103方向，但运动payload和完成回包不同。

## 入口与实际运行方式

- 库方式：把 `DFCom` 文件夹拷到Arduino的libraries目录，打开 `DFCom/examples/DcarON_Square/DcarON_Square.ino`。
- 单文件方式：打开 `SingleFile/DcarON_Square_AllInOne/DcarON_Square_AllInOne.ino`，不必安装库。
- 目标板为Uno/Nano（ATmega328P）。上传时断开D0/D1与车端连接，上传后再接。

**两个现有示例上电初始化后都会在loop中重复跑方形**，不像STM32示例默认只诊断、只运行一次。
实际动作是前进50cm、原地左转90°，重复四次后等1秒，再开始下一圈。
示例没有用wait返回值作为失败门禁；自定义可靠序列应检查返回值，失败就stop并退出本轮，不要照抄为实测成功保证。

## 接线和车端前提

| Arduino | 车端 / 调试 |
|---|---|
| D1 TX | 经5V→3.3V电平转换接车端USART3 RX/PB11 |
| D0 RX | 接车端USART3 TX/PB10，确认电平兼容 |
| GND | 与车端共地 |
| D3（可选调试TX） | USB-TTL RX，9600 |

车端USART3需支持 **115200双向DFLink**，包含里程计和完成回传。仅能接收指令、不能回传的固件不满足等待接口前提。
本仓库没有对应车端烧录包或完整兼容表，不能假设所有车已启用USART3回传。
运动前须激活/校准；Odom订阅还需相应能力，教学频率用10Hz，建议不超过50Hz。
SoftwareSerial调试可能屏蔽中断，避免大量打印造成硬件串口丢字节。

## 单位和API

库默认厘米、cm/s、度、度/s；`DFCom.useMeters()`切换米、m/s、弧度、rad/s。
单文件版函数参数固定厘米/度，不提供库对象的单位切换。
+X前、+Y左、正yaw逆时针；接收侧数据存储仍是SI。
profile=0匀速、1梯形加减速、2距离闭环；这些不是队列加速度档位。

```cpp
DFCom.begin(115200);       // 双停车和本地运动会话初始化，不是v6握手
DFCom.subscribeOdom(10);
DFCom.moveLinear(10, 0, 6, 2); // 前进10cm，6cm/s
if (!DFCom.waitDone(10000)) {
    DFCom.stop();          // 等待失败由应用决定停车/结束，库不自动推进序列
}
DFCom.update();            // loop中调用；waitDone内部也会解析回传
```

`waitDone(ms)`仅自然完成返回true，超时或中断返回false；0为永久等待，失联也不自行返回。
超时只结束本地等待，不自动取消车端任务。内部A/B本地槽不能创造车端任务ID，仍无法完全消除丢包后的同类回包歧义。
`moveVel`无自动终点，必须主动 `stop()`。

库API看 [DFCom.h](DFCom/src/DFCom.h)，修改动作看两个示例末尾的setup/loop。
离线回归统一从仓库根目录执行 `python3 tests/run_checks.py`；真实AVR编译、Flash/RAM和实车行为须在目标环境验证。
