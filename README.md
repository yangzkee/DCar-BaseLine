# DCar 上位机通信例程

官网：[differ-tech.pages.dev](https://differ-tech.pages.dev)

淘宝搜索 **DCar**，认准 **“点浮科技”** 购买。

程序运行在外部 STM32、Arduino 或电脑上，通过串口控制 DCar / DcarON，接收任务结果、状态与里程计；本仓库不包含车端主控固件。

现成测试统一为**四段操场持续循环**，不需要选择路线或修改运动开关。遥测只订阅、解析和使用 Odom；遥测 / 状态只报告，不决定运动是否启动，运动必须获得相应任务完成回包。

## 目录

- [1. Example、PatchOnly、Arduino、ROS 怎么选](#选择)
- [2. 下载后有什么功能](#现成功能)
- [3. 下载、接线与车端要求](#下载接线)
- [4. STM32 怎么用](#stm32测试)
- [5. 发送运动的函数](#运动函数)
- [6. 接收回传的函数和数据](#回传函数)
- [7. 最小调用顺序](#最小调用)
- [8. Arduino / ROS 怎么用](#其他平台测试)
- [9. 四段动作与验收标准](#验收标准)
- [10. 日志与常见问题](#常见问题)
- [11. 源码目录](#源码目录)
- [12. 软件检查与验证范围](#验证范围)

<a id="选择"></a>
## 1. Example、PatchOnly、Arduino、ROS 怎么选

| 目录 | 适用情况 | 包含内容 | 使用入口 |
|---|---|---|---|
| [DFCom_Example](DFCom_Example/README.md) | STM32F103C8T6，直接编译运行 | Keil 工程、标准库、启动时钟、DMA 串口、通信核心、main 中的操场循环 | `USER/Template.uvprojx` |
| [DFCom_PatchOnly](DFCom_PatchOnly/README.md) | 已有自己的 STM32 工程 | 与 Example 相同的公共代码和 main；没有完整工程依赖 | 按移植说明接入自己的工程，不能独立编译 |
| [DFCom_Arduino](DFCom_Arduino/README.md) | Uno / Nano（ATmega328P） | 库版与免安装库的单文件版，API 和操场路线相同 | Arduino IDE 打开 `DcarON_Stadium.ino` 或 `DcarON_Stadium_AllInOne.ino` |
| [ros2](ros2/README.md) | 电脑或 ROS 2 应用 | 串口桥、Odom话题、IMU、单动作和队列轨迹 Action、持续操场测试 | `stadium_test.launch.py` |

Example 与 PatchOnly 的能力相同，差别在工程依赖是否齐全。STM32 和 ROS 使用队列 v6；Arduino 使用无任务 ID 的逐条接口，不能照抄 `DFQueue_*` 或加速度档位 16..20。四个目录任选适合的平台，不是依次安装的组件。

<a id="现成功能"></a>
## 2. 下载后有什么功能

| 平台 | 启动与运动 | 接收与日志 | 失败处理 |
|---|---|---|---|
| STM32 Example / PatchOnly | 双停车初始化；固定观察窗口；倒计时 3 秒；四条任务连续入队，整圈完成后立即下一圈 | USART1 循环 DMA 持续收帧，主循环解析；Odom 请求 10Hz；USART2 非阻塞日志；每秒字节数与Odom帧诊断 | v6 握手、ACK、DONE 和超时必须通过；任务失败取消 / 零速，停止发新圈，继续收帧 |
| Arduino 库版 / 单文件版 | 双停车初始化；固定观察窗口；等待 3 秒；四段逐条发、逐条等完成；完成后立即下一圈 | 硬件串口持续轮询；独立 Odom 数据、时间戳与计数；不使用 SoftwareSerial 日志 | 任一等待失败发零速，停止后续段；保留收帧服务 |
| ROS 持续测试入口 | 一条 launch 命令启动桥接及操场客户端；每圈成功后再提交下一圈 | 只请求 Odom，发布里程计和接收统计，IMU按配置发布；过期数据不继续发布 | 参数、串口、队列结果和任务时限约束；测试失败结束 launch；重连不重放旧目标 |

遥测或状态没有回传时，运动仍可按任务协议运行；**运动成功与遥测验收是两项独立结论**。先完成车辆激活 / 校准，程序的状态查询不会替你校准。
循环无额外圈间延时、无需按键或复位；车端仍可能逐段减速或微调，不能将“持续循环”理解为全程恒速、零停顿。

### Beta 队列警告兼容（2026-10-27）

STM32 Example/PatchOnly 与 ROS 握手确认队列版本 ≥7 后，将 `process=255` 且 `notice=0x24/0x26` 视为“本段警告退段”，保留原码并继续后续任务，不主动取消或发零速；这不表示本段达到精度。STM32 `DFQueue_Wait()` 新增 `DF_QUEUE_WAIT_WARN=4`，示例打印 WARNING；自有调用方也须显式处理此返回值。ROS 在 feedback 保留每段警告，最终结果的 notice/message 保留警告；success 表示队列流程结束，不代表所有段精确完成。版本6保持原有故障处理，通信超时及其他故障处理不变。Arduino 无任务ID的接口不作此放宽。

<a id="下载接线"></a>
## 3. 下载、接线与车端要求

下载 **main** 当前源码，或在 main 页面选 Code → Download ZIP：

```sh
git clone --branch main https://github.com/yangzkee/DCar-BaseLine.git
cd DCar-BaseLine
```

| 接口 | 接线 | 串口参数 |
|---|---|---|
| STM32 USART1，车端口 | PA9 TX → 车 RX；PA10 RX ← 车 TX；GND 共地 | 460800，8N1，3.3V TTL |
| STM32 USART2，日志口 | PA2 TX → USB-TTL RX；GND 共地 | 115200，8N1 |
| Arduino 硬件 Serial | D1 TX 经 5V→3.3V 转换 → 车 USART3 RX/PB11；D0 RX ← 车 TX/PB10；GND 共地，确认电平兼容 | 115200，8N1 |
| 电脑 ROS | USB-TTL TX/RX 与车 DFLink 口交叉，GND 共地；直接连车，不经过示例 STM32 | 460800，8N1，3.3V TTL |

操场末段需要边平移边旋转，要求全向 / 麦克纳姆底盘。STM32 / ROS 需要队列 v6 及带 ID 的完成回包；Arduino 需要无 ID 运动与完成回包。统一订阅 Odom，车端按 DCar 型号自动提供相应数据，用户无需选择 Odom 模式。专业版（Pro）提供全量 Odom；非专业版提供普通 Odom，包含航向、车体系/固定里程计系速度、位置和时间戳。普通 Odom 的 roll/pitch、三轴加速度、三轴角速度为零占位，不代表真实 IMU 测量。ROS默认不发布IMU，Pro需要时在配置中设置 `publish_imu: true`。
本仓库不提供可保证兼容的车端烧录包；队列能力由握手确认，遥测能力由实际回传格式确认。

<a id="stm32测试"></a>
## 4. STM32 怎么用

1. 使用 Keil MDK5 / ARM Compiler 5 打开 [Template.uvprojx](DFCom_Example/USER/Template.uvprojx)，启用 **Use MicroLIB**，编译。
2. Debug / Utilities 选择自己的 ST-Link，使用 SWD，确认目标为 STM32F103C8，Flash 起始 `0x08000000`、64KB，RAM 起始 `0x20000000`、20KB。BOOT0=0，建议 BOOT1=0。下载器需根据实物配置，不预选某个人的序列号。
3. 将程序烧录到外部板，按表接车端和 115200 日志终端。车辆放到适合操场运动的场地；复位后会自动运动。
4. 观察 `[LAP] START 1`、每圈四条 DONE、`[LAP] DONE 1`、`START 2` 持续出现，并按[验收表](#验收标准)检查。

每段运动的速度、加速度档位及其他参数直接写在调用行中，可逐行独立调整；行尾注释说明单位、正负方向、取值和修改效果。STM32改 `USER/main.c`，Arduino改草图的 `loop()`，ROS改 `ros2/dcaron_bridge/examples/stadium.yaml`。STM32/ROS队列档位填16～20；Arduino最后的0/1/2是运动模式，不是队列档位。

固定启动顺序：发零速 → 1000ms → 再发零速 → Odom 订阅 → 20ms → 状态查询 → 500ms 观察 → 50ms → 业务回包会话 → 3000ms → 队列握手与运动。
这些等待期间持续调用 `DFCom_Poll()`；没有遥测不会无限等待，也不阻止握手。每次 `DFQueue_Wait` 上限 60000ms，从该次调用开始计时。

主函数中四条真实运动调用见 [main.c](DFCom_Example/USER/main.c)，没有路线选择或运动模式开关。订阅、查询、日志各自独立，默认均开启；用户不需要修改它们才能运行。
PatchOnly 完成[移植步骤](DFCom_PatchOnly/README.md)后行为相同。

<a id="运动函数"></a>
## 5. 发送运动的函数

STM32 声明见 [DFCom.h](DFCom_Example/HARDWARE/DFCom.h)，组帧与任务管理见 [DFCom_Tx.c](DFCom_Example/HARDWARE/DFCom_Tx.c)。在主循环调用，不能从中断发起阻塞等待。

| 函数 | 参数、作用 | 返回值 / 判定 |
|---|---|---|
| `DFQueue_Begin()` | 查询队列 v6 能力、空闲状态与下一任务 ID | 1 成功、0 失败，不自动改用另一协议 |
| `DFQueue_Linear(x,y,speed,yaw,grade)` | 相对段起点平移 x/y 米；speed 为正 m/s；同时转 yaw 弧度；负 x 后退 | 非零 ID 表示确认入队，0 失败 |
| `DFQueue_Rotate(yaw,omega)` | 原地相对转角，正 yaw 左转，omega 为正 rad/s | 非零 ID 仅表示入队 |
| `DFQueue_Arc(radius,yaw,signed_speed,grade)` | 正半径米、相对转角弧度；有符号速度 m/s，负值倒行 | 非零 ID 仅表示入队 |
| `DFQueue_Wait(id,timeout_ms)` | 等任务终态，内部持续 Poll | 1 DONE；2 超时 / 未知 ID / 零时限；3 设备故障。timeout 必须 >0；成功释放槽 |
| `DFQueue_Status(id,&process,&notice)` | 非阻塞读取该 ID 最近回包，不释放槽 | 1 有回包，0 没有；返回 1 不表示成功完成 |
| `DFQueue_LastError()` / `DFQueue_LastNotice()` | 最近操作的原因 / 通知码 | 失败日志一起记录 |
| `DFQueue_Cancel()` | 尝试取消并发送零速，清理本地会话 | 无返回值；下一批重新 Begin，断线时不能确认停车 |

队列固定 SI：米、弧度、m/s、rad/s，`g_dfcom_unit_mode` 不影响队列。+X 前、+Y 左、正 yaw 逆时针。`0.50f`=50cm，π/2=90°。grade 16 极低、17 低、18 中、19 高、20 极高；仅直线 / 圆弧帧包含档位，原地旋转无此字段。
STM32 最多跟踪 8 条在途任务；当前操场每圈 4 条。连续提交再 Wait 是排队，发一条立即 Wait 是逐段执行。

Arduino 对应 `DFCom.moveLinear`、`moveLinearWithYaw`、`moveRot`、`moveArc`、`waitDone`、`stop`。默认厘米、cm/s、度、度/s，库 / 单文件均可 `useMeters()` 切 SI；profile 为 0 匀速、1 梯形、2 距离闭环，操场用 1。`waitDone(ms)` 为 true 才自然完成，false 为发送失败 / 超时 / 中断；0 无限等，超时本身不停车，例程负责 stop。`moveArc` 支持负速度倒行，半径必须正、速度非零。
STM32 的无 ID `Cmd_Move_*` 及 `WaitMoveDone` 用于独立场景，不能夹入活动队列。持续速度接口无自动终点、无完成回包，必须主动零速停止。

<a id="回传函数"></a>
## 6. 接收回传的函数和数据

**任务结果**回答“动作完成了吗”，**遥测**回答“当前测得的位置 / 速度 / 姿态是什么”，**状态**回答车端当前报告的校准等状态。三者不能相互替代。

| 平台 | 收帧服务 | 真实数据与诊断 |
|---|---|---|
| STM32 | `DFCom_Poll()` → `USART1_ReceivePoll()` → `DFCom_RxParse()`；有效任务帧分发 `DFQueue_Receive()` | `g_odom`、`g_dcar_state`、`g_dfcom_rx_diag`；USART 字节 / 溢出 / 错误计数 |
| Arduino 两版 | `DFCom.update()`；`serviceDelay()` / `waitDone()` 内部同样服务 | `DFCom.odom`、`.state`；`.odomDiag`、`.rxBytes` |
| ROS | 单线程执行器定期读串口和解析 | `/telemetry/odom` 发布真实 Odom；`/telemetry/status` JSON 统计；`/odom` 只发布新鲜 Odom |

应用不要手动调用 `DFQueue_Receive` 制造完成事件。移植时将接收字节接到解析器即可；完整 STM32 工程已接好。

### 数据与新鲜度

STM32 `n_pos_x_m/n_pos_y_m` 是开机累计世界系位置；`b_vel_x_mps/b_vel_y_mps` 是随车头旋转的车体系速度，`yaw_rad` 为弧度。日志位置米、速度 m/s、Yaw 转为度。Arduino 的 `posX/posY`、`bVelX/bVelY`、`yaw` 同样始终存 SI，不随发送单位切换。

<a id="odom-data"></a>
### 专业版全量 Odom 与普通 Odom

所有例程使用相同的 Odom 订阅和接收接口；数据能力由车端固件按 DCar 型号提供，不需选择另一种协议或切换模式。两类采用同一帧布局，区别在字段的有效内容，不能根据零值自动推断车辆型号。

| 数据 | 专业版：全量 Odom | 非专业版：普通 Odom | 单位 |
|---|---|---|---|
| 航向 yaw | 有效 | 有效 | rad，逆时针为正 |
| 横滚 roll、俯仰 pitch | 有效 | 零占位 | rad |
| 车体系速度 vx、vy、vz | 有效 | 有效 | m/s |
| 固定里程计系速度 vx、vy、vz | 有效 | 有效 | m/s |
| 固定里程计系位置 x、y、z | 有效 | 有效 | m |
| 三轴加速度 ax、ay、az | 有效 | 零占位 | m/s² |
| 三轴角速度 wx、wy、wz | 有效 | 零占位 | rad/s |
| 车端时间戳 timestamp_ms、timestamp_us | 提供 | 提供 | 两个原始 U32 字段 |

平面底盘的 z 位置/速度等字段可以为零；全量字段有效也不代表它们必须非零。普通 Odom 的增强字段占位不能用于判断车辆静止或计算 IMU 测量。ROS默认不发布 `/imu/data`；专业版需要独立IMU话题时配置 `publish_imu: true`，这不影响 Odom 的自动适配。ROS标准话题输出的位置和车体速度等字段见 [ROS说明](ros2/README.md#Odom遥测与话题)，完整线上字段见 [协议说明](docs/DFLink-v6.md#5-持续速度状态查询与遥测)。

### Odom 更新与断更

所有平台只保存 Odom 数据、计数和时间。最近 1 秒内更新视为新鲜；缺首帧或断更超过 1 秒则报告缺失，不把旧值或零值当成新回传。运动仍按自身协议处理。
STM32 定时器只计时，Poll 约每 100ms 在主循环打印；Odom 不新鲜时打印 `[ODOM] NO FRESH DATA`。日志开关不影响收帧 / 计时 / 运动。

### 任务回包判定

| process / notice | 含义 | 结果 |
|---|---|---|
| 0 / 0x10 | 入队 ACK | 未完成 |
| 1..254 / 0 | 进度缩放值 | 未完成，不能直接当作百分数 |
| 254 / 0x25 | 到位后微调中 | 未完成 |
| 255 / 0 | 自然完成 | 成功 |
| 255 / 0x27 | 微调时限到达、车端判在容差内 | 成功，保留 notice |
| 255 / 0x26 | 微调超差 | 失败 |

详细报文、队列满及其他通知见 [DFLink v6](docs/DFLink-v6.md)。Arduino 的无 ID 接口使用自己的两字节进度回包和 bool 完成语义，不能套用 v6 notice 判定。

<a id="最小调用"></a>
## 7. 最小调用顺序

STM32 初始化串口、毫秒 tick 和运动会话后，在主循环执行：

```c
u16 id;
if (DFQueue_Begin()) {
    id = DFQueue_Linear(0.10f, 0, 0.06f, 0, DF_ACCEL_MEDIUM);
    if (!id || DFQueue_Wait(id, 10000) != DF_QUEUE_WAIT_DONE) {
        printf("%s notice=0x%02X\r\n", DFQueue_LastError(), DFQueue_LastNotice());
        DFQueue_Cancel();
    }
} else {
    printf("begin failed: %s\r\n", DFQueue_LastError());
}
```

完整初始化按 main；业务空闲和自定义延时也必须调用 Poll，默认至少每 5ms 服务一次。队列等待内部已服务。不要从 ISR 解析同一串字节，也不要在 ISR 调用 printf。

<a id="其他平台测试"></a>
## 8. Arduino / ROS 怎么用

### Arduino

库版安装 `DFCom_Arduino/DFCom` 到 Arduino libraries，打开 [DcarON_Stadium.ino](DFCom_Arduino/DFCom/examples/DcarON_Stadium/DcarON_Stadium.ino)；单文件直接打开 [DcarON_Stadium_AllInOne.ino](DFCom_Arduino/SingleFile/DcarON_Stadium_AllInOne/DcarON_Stadium_AllInOne.ino)，不需要装库。二选一，API 与实际路线相同。
选择 Uno / Nano，上传时断开 D0/D1 与车端连接；上传后接回。完成固定启动等待即自动跑操场，每次循环四段，失败后 `testStopped=true`，成功圈数保存在 `completedLaps`。不需要打开运动模式。
Uno 的唯一硬件串口用于车端，不向它打印文字，不用会屏蔽接收中断的 SoftwareSerial。遥测与诊断保存在公开对象中，可在业务中读取；需要串口日志时使用有独立硬件串口的板卡并保持非阻塞输出。详见 [Arduino 说明](DFCom_Arduino/README.md)。

### ROS 2

按 [ROS 说明](ros2/README.md)构建并加载环境，执行：

```sh
ros2 launch dcaron_bridge stadium_test.launch.py port:=COM5
```

COM5 换实际口，Linux 例如 `/dev/ttyUSB0`。此入口自动开启 Action 和持续操场客户端，无需另外启动桥接或切换模式。正常每圈显示四段结果，整圈 `success: true` 后立即再提交；Ctrl+C 结束。
另一个已加载环境的终端可观察：

```sh
ros2 topic echo /telemetry/status
ros2 topic hz /telemetry/odom
```

`/odom` 只发布 1 秒内新鲜 Odom，过期后停止发布，收到新的有效 Odom 后恢复。`/imu/data` 仅在配置 `publish_imu: true` 时从 Odom 发布，Std不可据零占位开启。只读应用可单独启动 `bridge.launch.py`，不自动发路线；自定义动作、YAML 和单次校验工具见平台说明。
一个串口只由桥接节点占用，不能再开串口助手或直接串口脚本。

<a id="验收标准"></a>
## 9. 四段动作与验收标准

| 段 | 目标动作 | 要验收什么 |
|---|---|---|
| 1 | 半径 0.30m、相对 +π，速度 -0.15m/s 倒行半圆 | 倒行、半圆形状、半径、正向 180° 航向变化、任务完成 |
| 2 | 相对后退 0.60m，速度 0.15m/s，无附加转角 | 后退方向、距离、横向偏移、航向保持、任务完成 |
| 3 | 同第 1 段 | 第二半圆的形状、半径、转向、与第 1 段的一致性 |
| 4 | 固定直线路径后退 0.60m，同时车身正向转 2π | 后退与旋转同时进行、完整 360°、任务完成 |
| 整圈 | 四段按顺序自动重复 | 每圈四段成功、圈数持续增长、不需人工触发、多圈位置 / 姿态漂移 |

STM32 / ROS 用队列档位 18，Arduino 用无 ID profile 1。请求速度是上限，车端按曲率、加速度和旋转能力限速。
Yaw 在 ±π 回绕，不能只看第 4 段起终点 yaw 相同就判“转满一圈”；必须观察连续姿态和实际旋转。世界系累计位置不能直接当每段相对位移；外部板复位不会清零车端里程计。

| 验收层次 | 正常标准 |
|---|---|
| 构建与启动 | 编译无错误、上传成功；日志 / 运行变量对应当前程序 |
| 运动协议 | STM32 每个 ID 的 Wait=DONE；ROS 最终 success=true；Arduino 每次 waitDone=true；入队 ACK / goal accepted 不算完成 |
| Odom 遥测 | wire 和 ok 持续增长，格式正确，reject 不增长；航向、速度、位置来自有效 Odom；专业版另验收 roll/pitch、加速度和角速度 |
| 接收稳定性 | STM32 dropped / overflows / errors 保持 0；Odom连续观察至少 60 秒，并多次复位检查 |
| Odom 断更与恢复 | 断更 1 秒后不输出旧值；新有效帧到达后自动恢复；断更期间不能判遥测验收通过 |
| 独立性 | 无状态 / 无遥测时不因观察窗口卡住；运动仍由握手 / ACK / DONE 与超时判定；关闭日志仍收帧 |
| 物理运动 | 四段方向和顺序正确；外部测量距离、半径、转角与多圈重复误差满足事先约定的车辆 / 任务指标 |

仓库没有统一“±几毫米 / ±几度”的实车精度承诺。测试前约定距离 / 角度 / 停止速度容差及重复次数，再用尺量、角度测量或外部定位验收。记录代码提交、车端能力、底盘、地面、电量、每段 task / notice 和外部测量。DONE 或里程计显示目标值不能单独证明物理精度。

<a id="常见问题"></a>
## 10. 日志与常见问题

| 输出 / 现象 | 含义与检查 |
|---|---|
| `[INIT] ... (report only)` | 状态和遥测计数仅报告；校准位不是客户端运动开关，需事先完成车辆校准 |
| `[LAP] START/DONE n` | 当前圈号；只有四段全部成功才计入完成 |
| `[DEMO] FAILED stage=begin/submit/wait` | 握手 / 提交 / 等待失败，看 reason 和 notice；程序尝试取消 / 零速后不继续发新任务 |
| `[UART1] bytes=... dropped=... overflows=... errors=...` | 车端接收计数，后三项应为 0；检查线缆、供电、频率和 Poll 调度 |
| `[RX] odom wire/ok/reject/len/ver` | wire 为校验通过外层帧，ok 为负载解析成功；wire 增但 ok 不增时看长度 / 版本；wire=0 表示未收到该类型有效帧 |
| `[ODOM] NO FRESH DATA` | 缺首帧或本地超过 1 秒未更新；不能算遥测通过，不会因此自动切断运动 |
| Odom 长期缺失 | 检查 wire / reject、接线、波特率及车端 Odom 支持；缺失时不发布旧值 |
| Keil device mismatch | 核对 C8 目标、自己的 ST-Link 和实物芯片标识；SW-DP ID 不是完整芯片型号，不能仅凭它改选其他型号 |
| 没有 STM32 日志 | PA2→USB-TTL RX、GND、115200、MicroLIB；检查日志开关与发送丢字节计数 |
| Arduino 没有文本日志 | Uno 唯一硬件串口用于车端；查看公开对象 / 计数，不向车端串口打印文字 |
| 距离 / 角度不对 | 队列固定米 / 弧度，Arduino 默认厘米 / 度；不要混用单位或 profile |
| 更新 ROS 后行为没变 | 重新构建两个包，另开终端加载相同 install 环境 |

取消、停车均经过同一链路发送，失联时不能保证车端收到。遥测新鲜度提示不是硬件看门狗，车辆运动能力及停车保障由实际系统决定。

<a id="源码目录"></a>
## 11. 源码目录

```text
DFCom_Example/
  USER/main.c                   固定启动、独立订阅、四段操场循环
  USER/Template.uvprojx          Keil完整工程
  HARDWARE/DFCom.h               API、单位、数据结构与诊断字段
  HARDWARE/DFCom_Tx.c            组帧、队列提交/等待/取消
  HARDWARE/DFCom_Rx.c            有界流式解析、任务/状态/Odom分发
  HARDWARE/DFCom_Print.c         TIM2计时与主循环Poll服务
  SYSTEM/usart/                 1024字节循环DMA接收、非阻塞日志
DFCom_PatchOnly/                同一份公共代码的移植包
DFCom_Arduino/
  DFCom/src/                    Arduino通信库
  DFCom/examples/DcarON_Stadium/ 库版操场例程
  SingleFile/                   内嵌同一库实现的独立例程
ros2/
  dcaron_bridge/                 串口桥、Odom接收与诊断、动作/循环客户端
  dcaron_interfaces/             Motion、Trajectory与MotionSegment
  docs/                         Windows环境与动作说明
docs/DFLink-v6.md               报文和状态含义
tests/                         离线主机检查
```

<a id="验证范围"></a>
## 12. 软件检查与验证范围

Python >=3.10、PyYAML 及主机 C/C++ 编译器，在仓库根目录执行：

```sh
python3 tests/run_checks.py
```

统一覆盖文档与工程引用、两份 STM32 公共代码一致性、真实串口驱动模拟、遥测/任务粘连 / 分片 / DMA 环回 / 溢出重同步、队列与逐条运动、连续操场及失败处理。Windows C 主机检查可用：

```powershell
powershell -File tests/host/run_windows_tests.ps1 -Compiler C:\path\to\gcc.exe
```

ROS 环境另执行 `colcon test --merge-install --packages-select dcaron_bridge` 和 `colcon test-result --verbose`。CI 包含主机回归、ROS Jazzy 实际构建 / 内存串口集成以及 Uno 两种完整例程编译；具体结果看对应提交的 Actions。
主机模拟、目标编译和真实硬件测试分别验收。STM32 实测记录不能替代 ROS / Arduino 实车验证，也不能替代物理轨迹精度测量。

<sub>老版本的 VelPos 已与 Odom 合并，会根据你手上的 DCar 版本自动调整到最合适的 Odom 版本。</sub>
