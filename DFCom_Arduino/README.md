# Arduino Uno / Nano 客户端

官网：[differ-tech.pages.dev](https://differ-tech.pages.dev)

淘宝搜索 **DCar**，认准 **“点浮科技”** 购买。

目标ATmega328P。使用无任务ID的逐条运动接口，未实现队列v6；接收数据使用SI，与STM32 / ROS方向一致。

统一订阅 Odom，车端按 DCar 型号自动提供相应数据，用户无需选择 Odom 模式。专业版（Pro）提供全量 Odom；非专业版提供普通 Odom，包含航向、车体系/固定里程计系速度、位置和时间戳。普通 Odom 的 roll/pitch、三轴加速度、三轴角速度为零占位，不代表真实 IMU 测量。专业版还提供 roll/pitch、三轴加速度和三轴角速度；两类数据逐项对照见[Odom数据说明](../README.md#odom-data)。

## 目录

- [下载与运行](#下载与运行)
- [接线与车端能力](#接线与车端能力)
- [函数与数据](#函数与数据)
- [独立接收与诊断](#独立接收与诊断)
- [验收与检查](#验收与检查)

## 下载与运行

二选一，实际通信实现、API和操场动作相同：

- 库版：将 `DFCom` 放到Arduino libraries，打开 [DcarON_Stadium.ino](DFCom/examples/DcarON_Stadium/DcarON_Stadium.ino)。
- 单文件：打开 [DcarON_Stadium_AllInOne.ino](SingleFile/DcarON_Stadium_AllInOne/DcarON_Stadium_AllInOne.ino)，无需安装库。

选Uno / Nano对应板型，上传时断开D0/D1与车端连接，上传后接回。默认就运动，不用改模式。
上电两次停车之间等1000ms，会话间隔50ms；订阅Odom、间隔20ms后查询状态；固定观察500ms，等待3000ms后开始循环。等待期间持续收帧，查询结果只报告，不决定是否启动。

每圈逐条执行：倒行半圆（R=30cm，+180°，-15cm/s）→后退60cm→同样倒行半圆→后退60cm同时正向旋转360°，profile=1。
每段等待自然完成，上限60000ms；四段成功后 `completedLaps` 增长并立即开始下一圈，无1秒圈间等待。
任一步失败发送stop，`testStopped=true` 后不再发新动作，loop继续update。检查接线 / 车端状态后复位重启。

## 接线与车端能力

| Arduino | 连接 |
|---|---|
| D1 TX | 5V→3.3V电平转换后接车USART3 RX/PB11 |
| D0 RX | 接车USART3 TX/PB10，确认电平兼容 |
| GND | 共地 |

车端USART3需支持115200 / 8N1双向DFLink，无ID圆弧、平移、平移并转向和完成回包。路线需要全向底盘，运动前完成车辆激活 / 校准。
底盘须支持统一 Odom 回传；没有 Odom 时报告缺失，运动仍按完成回包判定结果。

Uno的唯一硬件串口留给车端，不向Serial打印文字。例程不使用SoftwareSerial日志，以免屏蔽硬件接收中断；遥测、计数和结果保存在公开变量里，业务可直接读取。
若使用额外硬件串口作日志，输出必须非阻塞、限频，接收服务独立于日志开关。车端串口不能同时连接上传程序 / 串口助手发送文本。

## 函数与数据

声明看 [DFCom.h](DFCom/src/DFCom.h)，实际动作看两个例程的setup/loop，单文件同样使用 `DFCom.` API。

| 函数 | 用法 |
|---|---|
| `begin(115200)` | 初始化硬件串口、双停车、本地运动会话 |
| `moveLinear(x,y,speed,profile)` | 相对平移，speed>0，负x后退 |
| `moveLinearWithYaw(x,y,yaw,speed,profile)` | 固定平移路径同时旋转车身 |
| `moveRot(yaw,omega)` | 原地相对旋转，omega>0 |
| `moveArc(radius,yaw,signed_speed,profile)` | radius>0、速度非零；负速度倒行 |
| `waitDone(ms)` | true自然完成；false超时 / 中断 / 发送失败，内部收帧；0无限等 |
| `stop()` / `moveVel(vx,vy,wz)` | 发零速 / 持续速度；持续速度无自动终点，不可等完成 |
| `subscribeOdom(10)` | 订阅 Odom 10Hz 连续回传，例程已调用 |
| `queryState()` | 请求状态，结果在 `.state` |
| `update()` | 消费串口字节，业务主循环持续调用 |
| `serviceDelay(ms)` | 延时期间持续收帧，不用长时间delay阻断接收 |
| `odomAvailable()` | Odom 确有有效回包且最近1秒更新过 |

发送默认厘米、cm/s、度、度/s；`useMeters()`切米、m/s、弧度、rad/s，`useMillimeters()`切毫米 / 度。+X前、+Y左、正yaw逆时针。
profile=0匀速、1梯形、2距离闭环，不能填队列档位16..20。`waitDone`超时本身只结束等待，停车由应用执行，现成例程已处理。
无ID回包不携带本地任务号，本地双槽不能完全排除丢包后的同类迟到帧歧义；不要与队列控制混发。

## 独立接收与诊断

只订阅 Odom；专业版读取全量数据，普通款读取航向、速度与位置：

- `DFCom.odom`：统一 Odom 数据结构，专业版包含完整姿态和 IMU，普通款增强字段为零占位；同时保存 `frameCount/localTickMs/fresh`。
- `DFCom.odomDiag`：`wire`外层校验通过帧、`rejected`长度/版本不符、`len/version`最近负载格式；对应数据的frameCount为成功解析数。
- `DFCom.rxBytes`：主循环已取出的串口字节数；它不包含AVR硬件/核心接收环可能丢失的字节，不可当作线端无丢包证明。
- `DFCom.state.received/imuCalibrated`：状态回包 / 校准位，只报告。

接收缓存有界，保留分片并扫描后续有效帧，支持多帧粘连及坏长度后的重同步。延时和waitDone持续update，日志不参与解析。
所有接收数据存SI；`xCm/yCm/yawDeg`把 Odom 位置/航向转换为厘米/度。
读取前检查 `odomAvailable()`；缺失或过期时不把旧值或零值当成新回传。

## 验收与检查

按[根README四段验收表](../README.md#验收标准)检查动作、完成结果及多圈漂移。Odom wire与frameCount持续增长，rejected保持0；断更后available为false，新帧到达后恢复。
两版业务同样检查逐段waitDone、失败停止后续任务和completedLaps增长。结果需结合外部测量，不承诺统一距离 / 角度精度。
仓库根目录 `python3 tests/run_checks.py` 包含主机回归；CI另编译两个Uno完整目标并报告Flash/RAM。真实AVR收帧稳定性与实车运动须在对应板卡验证，主机模拟不代替硬件验收。

<sub>老版本的 VelPos 已与 Odom 合并，会根据你手上的 DCar 版本自动调整到最合适的 Odom 版本。</sub>
