# DCar 客户端例程

当前版本 **0.3.1**。本仓库运行在**外部 STM32 / Arduino 开发板或电脑**上，通过串口给 DCar / DcarON 发运动指令、收状态与里程计；**不是小车主控固件**。
STM32 和 ROS 2 路线使用 **DFLink 队列 v6**，Arduino 使用**无任务 ID 的逐条运动接口**。

## 目录

- [1. Example、PatchOnly、Arduino 怎么选](#选择)
- [2. 下载后已经具备哪些功能](#现成功能)
- [3. 下载与接线](#下载接线)
- [4. STM32 完整例程怎么用、怎么测](#stm32测试)
- [5. 发运动要看哪些函数](#运动函数)
- [6. 收回传要看哪些函数和数据](#回传函数)
- [7. 最小运动调用示例](#最小调用)
- [8. Arduino 和 ROS 2 怎么用、怎么测](#其他平台测试)
- [9. 测试通过的标准](#验收标准)
- [10. 常见问题](#常见问题)
- [11. 源码目录与进一步阅读](#源码目录)
- [12. 离线检查与验证范围](#验证范围)

<a id="选择"></a>
## 1. Example、PatchOnly、Arduino 怎么选

这里的 Arduino 就是 Uno / Nano 版本。三个目录不是安装顺序，也不用全部下载到板子里。

| 目录 | 给谁用 | 包含什么 | 你需要补什么 | 建议 |
|---|---|---|---|---|
| [DFCom_Example](DFCom_Example/README.md) | STM32F103C8T6，想直接编译例程 | Keil 工程、标准库、启动与时钟、串口 DMA、通信核心、main 中的循环操场测试 | Keil、外部开发板、车端和接线 | **STM32 新手从这里开始** |
| [DFCom_PatchOnly](DFCom_PatchOnly/README.md) | 已有自己的 STM32 工程 | 与 Example 相同的通信核心、串口参考实现和 main 集成示例 | 自己工程的启动、时钟、库、串口收发与毫秒计时集成 | **用于移植；不能独立编译**，不是功能增强包 |
| [DFCom_Arduino](DFCom_Arduino/README.md) | Arduino Uno / Nano（ATmega328P） | Arduino 库版和免安装库的单文件版，均带循环方形示例 | Arduino IDE、对应板卡、电平转换、支持该接口的车端 | **已有 Arduino 才选它**；不支持队列 v6 |
| [ros2](ros2/README.md) | 用电脑或 ROS 2 控车 | 串口桥、里程计、单动作与整组轨迹 Action、YAML 路线 | ROS 2 环境和 USB-TTL | 需要 ROS 话题 / Action 时选它 |

**Example 与 PatchOnly 的运动能力相同**，后者只省掉完整工程依赖。Arduino 的报文与等待方式不同，不能照抄 STM32 的 `DFQueue_*` 或档位 16..20。
这些程序均要求车端已有相应 DFLink 能力；更换外部控制板程序不会给车端增加协议支持。

<a id="现成功能"></a>
## 2. 下载后已经具备哪些功能

| 平台 | 上电 / 启动后默认做什么 | 现成运动示例 | 回传与异常处理 |
|---|---|---|---|
| STM32 Example | 两次尝试发零速；自动查询状态、订阅 10Hz VelPos 并尝试 Odom；诊断通过后自动运动 | 固定操场四段，完成一圈立即下一圈，持续循环；无需开关或模式选择 | 校准和通信门禁；v6 能力及空队列握手；按任务 ID 等完成、逐段打印结果；失败记录原因并尝试取消 / 零速，不继续下一圈 |
| STM32 PatchOnly | 集成后与 Example 相同 | 同一份 main，循环操场测试 | 同一份通信核心；正确运行依赖移植的中断、串口与 tick |
| Arduino 库版 | 双停车初始化、订阅 10Hz 全量 Odom、查询校准状态并打印 | **自动循环跑 50cm 方形**：前进 + 左转 90° 共四次，等 1 秒再跑 | 解析里程计与逐条完成回包；例程只提示校准异常，未把校准 / wait 失败作为停止门禁 |
| Arduino 单文件版 | 双停车初始化、订阅 10Hz 全量 Odom；**setup 没有查询校准状态** | 与库版相同，自动循环 | 自带解析和等待；同样未检查 wait 结果 |
| ROS 2 | 默认只订阅 / 发布遥测，不接受运动输入 | 开启 Action 后发送单动作；square / stadium YAML 默认仅校验，加 `--execute` 才运动 | 任务结果、取消、超时、遥测失联处理；重连不重放旧目标 |

STM32 操场每圈共 4 条任务（倒行半圆 → 后退 0.60m → 倒行半圆 → 后退 0.60m 同时转 2π），后者要求全向 / 麦克纳姆底盘。
请求速度是上限请求，车端会按加速度、曲率和旋转能力限速，不能把它当成全程恒速保证。

<a id="下载接线"></a>
## 3. 下载与接线

下载当前源码请选择 **main**：

```sh
git clone --branch main https://github.com/yangzkee/DCar-BaseLine.git
cd DCar-BaseLine
```

也可以在 main 页面选择 **Code → Download ZIP**，解压后使用。Release 附件不一定对应当前源码。

| 平台 / 串口 | 接线 | 参数 |
|---|---|---|
| STM32 USART1 车端口 | PA9 TX → 车 RX；PA10 RX ← 车 TX；GND 共地 | 460800，8N1，3.3V TTL |
| STM32 USART2 调试口 | PA2 TX → USB-TTL RX；GND 共地 | 115200，8N1，查看 printf |
| Arduino 硬件 Serial | D1 TX 经 5V→3.3V 转换 → 车 USART3 RX/PB11；D0 RX ← 车 TX/PB10；GND 共地 | 115200，8N1，确认 RX 电平兼容 |
| Arduino 可选调试口 | D3 TX → 另一 USB-TTL RX；GND 共地 | 9600，D0/D1 已用于车端，不能同时拿它看调试日志 |
| ROS 2 | USB-TTL TX/RX 与车端 DFLink 口交叉连接；GND 共地 | 460800，8N1，3.3V TTL；直接接车，不经过 STM32 示例板 |

先完成车辆激活 / 校准。STM32 / ROS 要求车端支持队列 v6 和带 ID 的完成回传；Arduino 要求 USART3 支持双向 DFLink 和无 ID 完成回传，默认 Odom 示例还需要全量 Odom 能力。
本仓库不提供可据此确认兼容的车端固件烧录包，也没有完整的“车端固件版本号 → 能力”对照表。STM32 自动同时请求 VelPos 与 Odom；Odom 无回传不阻止 VelPos 工作，不要求 Pro。Arduino 默认订阅全量 Odom，尚无自动兜底。

<a id="stm32测试"></a>
## 4. STM32 完整例程怎么用、怎么测

### 使用步骤：烧录后自动循环

1. 使用 Keil MDK5 / ARM Compiler 5 打开 [Template.uvprojx](DFCom_Example/USER/Template.uvprojx)，Target 启用 **Use MicroLIB**。
2. 编译并烧录到外部 STM32F103C8T6 板。运动直接写在 [USER/main.c](DFCom_Example/USER/main.c)，
3. 按接线表连接车辆和 115200 调试终端。车辆放在可运行操场路线的场地，完成激活 / 校准后，复位外部板。
4. 程序自动诊断；通过后倒计时 3 秒，开始操场循环。每圈成功后立即开始下一圈，不等待按键、不复位、不添加圈间延时。需要结束测试时主动停止设备。

本路线要求全向 / 麦克纳姆底盘。初次接通即可能运动；不要把“烧录完成后不动”作为预期现象。
循环不间断指程序持续运行、无人工触发或圈间等待；车端仍会在段末减速、到位微调，不能承诺全程零停顿或恒速。

### 自动启动与遥测现象

上电先尝试两次零速，约 1 秒后查询校准状态和新 VelPos；诊断窗口最多 500ms。

```text
[INIT] state_reply=1 imu_calibrated=1 new_velpos_frames=...（大于0）
[INIT] link and calibration OK; queue capability not checked yet
[DEMO] continuous stadium; start in 3000 ms
[LAP] START 1: ...
[QUEUE] lap=1 segment=1/4 task=... waiting
[QUEUE] lap=1 segment=1/4 task=... DONE notice=0x00
...（共四段）
[LAP] DONE 1: 4 tasks completed; next lap immediately
[LAP] START 2: ...
```

每圈开始重新握手，要求车端支持队列 v6 且队列空闲。所有任务均在 main 内连续入队，再按 ID 等待并记录结果；每次 Wait 上限 60000ms，不是要求每圈等待 60 秒。
诊断失败不运动；握手、提交或执行失败会打印 `FAILED stage=begin/submit/wait` 并尝试取消 / 零速，停止后续循环，不把失败伪装成完成。排除故障后复位重新开始。

### 每圈四段的验收项目

| 段 | 实际调用与参数 | 应观察 / 测量的现象 | 协议验收 |
|---|---|---|---|
| 1 | `DFQueue_Arc(0.30, π, -0.15, 18)` | 倒行半圆，半径目标 30cm，相对转角 +180°；检查倒行方向、圆弧形状与航向变化 | segment=1/4 DONE，记录 task 与 notice |
| 2 | `DFQueue_Linear(-0.60, 0, 0.15, 0, 18)` | 沿该段起点车头反方向后退 60cm，不附加旋转；检查距离与横向偏移 | segment=2/4 DONE |
| 3 | 与第 1 段相同 | 再倒行半圆；检查半径、转向与两段圆弧的一致性 | segment=3/4 DONE |
| 4 | `DFQueue_Linear(-0.60, 0, 0.15, 2π, 18)` | 沿该段固定直线路径后退 60cm，同时车身正向旋转 360°；检查两种运动确实同时发生 | segment=4/4 DONE |
| 整圈与循环 | 四段连续入队、完成后自动重复 | 顺序正确；LAP 编号递增；无需人工触发；比较多圈起终点与朝向漂移 | 每圈四条均 DONE 后才打印 LAP DONE，随后出现下一圈 START |

速度请求 0.15m/s，中档加速度 18。运动参数在 main 的四条调用中直接修改，不提供另一个模式入口。
Yaw 会在 ±180° 附近回绕：第 4 段不能只对比起终点 yaw 判定“转满一圈”，须观察连续姿态变化和真实车身旋转。
实际距离、角度、轨迹与重复性需要外部测量；具体容差见[验收标准](#验收标准)，不能凭 DONE 保证精度。

### VelPos / Odom 自动使用与验收

- 启动自动订阅两者，均请求 10Hz。VelPos 是必需的基础数据；Odom 属于可选增强，不需要用户切换。
- Odom 回传可用时，终端同时出现 `[VELPOS]` 和 `[ODOM]`，两种帧的 fr 均应持续增长；Odom 可额外验收姿态、加速度和陀螺仪。
- Odom 从未回传或本地超过 1 秒未更新时，不再打印其旧值，继续使用 / 打印 VelPos；恢复收到后自动一同打印，无需复位切换。
- VelPos 无首帧会导致启动失败；运行中超过本地 1 秒未更新打印 `NO FRESH DATA`，应判遥测验收失败，不能将旧值当成正常数据。该提示不是独立的运动失联看门狗。
- 停车静止时速度应接近零；全量 Odom 静止平放时 acc_z 应约 +9.81m/s²，gyro_z 接近零，旋转时正向角速度为正。噪声容差按车辆规格验收。

自动 Odom 增强 / VelPos 兜底是本次 STM32 Example 与 PatchOnly 的行为；Arduino 和 ROS 的订阅方式仍按各自平台说明。
PatchOnly 先完成[移植步骤](DFCom_PatchOnly/README.md)，之后使用相同循环和验收标准。复位外部板不清零车端累计位置。

<a id="运动函数"></a>
## 5. 发运动要看哪些函数

STM32 先看 [main.c 的实际调用](DFCom_Example/USER/main.c)，查参数看 [DFCom.h](DFCom_Example/HARDWARE/DFCom.h)，查组帧和等待实现看 [DFCom_Tx.c](DFCom_Example/HARDWARE/DFCom_Tx.c)。以下是当前例程使用的队列接口，均在主循环中调用。

| 函数 | 作用 / 参数 | 返回值与使用要点 |
|---|---|---|
| `DFQueue_Begin()` | 查询能力版本和下一任务 ID，开始一批任务 | 1 成功、0 失败；要求能力 >=6 且设备队列空闲；失败不自动换接口 |
| `DFQueue_Linear(x, y, speed, yaw, grade)` | 相对该段起点平移 x/y 米，同时转 yaw 弧度；speed 为正的路径速度 m/s | 返回已确认入队的非零 ID；0 失败。负 x 后退、正 y 左移；yaw=0 只平移，非零边走边转，需底盘支持 |
| `DFQueue_Rotate(yaw, omega)` | 原地转相对 yaw 弧度，omega 是正的最大角速度 rad/s | 正 yaw 左转、负 yaw 右转；不含加速度档；非零 ID 仅表示入队 |
| `DFQueue_Arc(radius, yaw, signed_speed, grade)` | 半径为正的圆弧，相对转角 yaw；速度可正可负 | 米 / 弧度 / m/s；负速度表示倒行；非零 ID 仅表示入队 |
| `DFQueue_Wait(id, timeout_ms)` | 阻塞等待该 ID 的车端结果；期间中断仍须运行 | 1=`DF_QUEUE_WAIT_DONE`；2=超时 / 未知 ID / 零时限；3=设备故障。必须传 >0 毫秒，从本次调用开始计时；成功后释放本地槽 |
| `DFQueue_Status(id, &process, &notice)` | 非阻塞查看该 ID 最近回包 | 1 表示有该 ID 回包、0 表示没有；输出进度与原因码，**返回 1 不代表任务完成**，不释放槽 |
| `DFQueue_LastError()` / `DFQueue_LastNotice()` | 读取最近一次操作的原因字符串 / 原因码 | 失败时一起打印，定位握手、提交或执行问题 |
| `DFQueue_Cancel()` | 尝试取消任务并发送零速，清理本地队列会话 | 无返回值，不能据此确认车端停车；下一批重新 Begin |

**单位固定 SI**：米、弧度、m/s、rad/s，`g_dfcom_unit_mode` 对这些函数无效。`0.50f` 是 50cm，`1.5707963f` 是 90°。
+X 向前、+Y 向左、正 yaw 逆时针；动作位移相对各段起点，遥测位置则是世界系累计值，二者不能直接当成同一组坐标。

| 队列加速度宏 | 值 | 含义 |
|---|---|---|
| `DF_ACCEL_VERY_LOW` | 16 | 极低 |
| `DF_ACCEL_LOW` | 17 | 低 |
| `DF_ACCEL_MEDIUM` | 18 | 中，示例默认 |
| `DF_ACCEL_HIGH` | 19 | 高 |
| `DF_ACCEL_VERY_HIGH` | 20 | 极高 |

这些档位用于队列直线和圆弧。先提交整组再按 ID Wait 是排队；发一条立即 Wait 是逐段执行。STM32 最多同时跟踪 8 条，成功 Wait 后释放槽；更多段需要分批提交。

STM32 另有无 ID 的 `Cmd_Move_Linear`、`Cmd_Move_LinearWithYaw`、`Cmd_Move_Rot`、`Cmd_Move_Arc`、`Cmd_Move_Vel`，当前操场循环不用它们。其单位由 `g_dfcom_unit_mode` 决定、默认厘米 / 度，profile 为 0/1/2；`WaitMoveDone` 等无 ID 完成，0 时限可无限等待。**不能混入活动队列**。持续速度 `Cmd_Move_Vel` 无自动终点、无完成回传，要主动发零速停止。

<a id="回传函数"></a>
## 6. 收回传要看哪些函数和数据

接收分两条线：**任务回包告诉你“这条动作完成了吗”**，**遥测告诉你“车当前在哪、速度多少”**。订阅遥测不会替代任务完成等待。

### 任务进度与完成

完整工程的串口接收会调用 [DFCom_RxParse(data, size)](DFCom_Example/HARDWARE/DFCom_Rx.c)，验证帧后分发给 `DFQueue_Receive` 更新任务槽。应用调用 `DFQueue_Status` / `DFQueue_Wait` 即可，**不要自己调用 Receive 伪造回包**；移植时才需要接通串口到解析器。

| process | notice | 意义 | 可判成功吗 |
|---|---|---|---|
| 0 | 0x10 | 入队确认 ACK | 否 |
| 1..254 | 0 | 执行进度，协议缩放值，不能直接打印成百分数 | 否 |
| 254 | 0x25 | 到位后仍在微调 | 否 |
| 255 | 0 | 正常完成 | 是 |
| 255 | 0x27 | 微调时限到达，但车端判定仍在容差内 | 是，保留该原因码用于记录 |
| 255 | 0x26 | 微调超差 | 否，失败 |

队列满 0x11 和其他通知详见[协议](docs/DFLink-v6.md)。用 `Wait` 的返回值作完成门禁，不能仅看最后一行位置或收到 ACK。

### 状态、位置、速度与姿态

| 要做什么 | STM32 入口 | 从哪里读结果 |
|---|---|---|
| 查询激活 / 校准状态 | `Cmd_Query_DcarState()` | `g_dcar_state.received` 和 `.imu_calibrated`；查询只发请求，要等回包，不会替你执行校准 |
| 连续接收简化位置 / 速度 | `Cmd_Subscribe_VelPos(ODOM_MODE_CONTINUOUS, ODOM_FREQ_10HZ)` | `g_velpos`；当前 main 已调用 |
| 接收全量 IMU / 里程计 | `Cmd_Subscribe_Odom(ODOM_MODE_CONTINUOUS, ODOM_FREQ_10HZ)` | `g_odom`；需车端支持全量 Odom，当前 main 已自动订阅；无回传不阻止 VelPos 运行 |
| 看调试日志 | `VelPos_Print()` / `Odom_Print()` | USART2 的 printf；当前 TIM2 约每 100ms 打印 VelPos，新鲜 Odom 存在时同时打印 |
| 判断是否有新遥测 | 比较 `.frame_count` 前后值，或消费 `.fresh`；检查 `.local_tick_ms` | 帧计数应增长；只看到重复打印的旧值不能说明链路仍正常 |

`g_velpos.n_pos_x_m / n_pos_y_m` 是世界系累计位置（米），`b_vel_x_mps / b_vel_y_mps` 是随车头旋转的车体系速度（m/s），`yaw_rad` 是航向角（弧度，跨 ±π 回绕）。日志 Yaw 为方便阅读转换成了度，数据字段仍是弧度。
全量 `g_odom` 另含 roll/pitch、加速度和陀螺仪；当前 STM32 例程已自动订阅并按新鲜度打印，无需改订阅或打印开关。停止订阅可用 `ODOM_MODE_STOP`，该实现会转为单次快照后停止；`ODOM_MODE_ONESHOT` 用于请求一帧。

接收数据由中断异步更新，直接读多个字段不是整帧原子快照；需要严格同步采样时应在自己的业务中实现一致性读取。默认 tick 与打印共用 TIM2，printf 耗时会影响本地毫秒计时精度，不用于精密性能测量。

<a id="最小调用"></a>
## 7. 最小运动调用示例

以下片段放在已完成初始化、回包会话启用、通信 / 校准诊断的主循环上下文中，不能脱离完整 main 直接运行：

```c
u16 id;
u8 result;
if (!DFQueue_Begin()) {
    printf("begin failed: %s\r\n", DFQueue_LastError());
} else {
    id = DFQueue_Linear(0.10f, 0, 0.06f, 0, DF_ACCEL_MEDIUM);
    if (!id) {
        printf("submit failed: %s notice=0x%02X\r\n",
               DFQueue_LastError(), (unsigned)DFQueue_LastNotice());
        DFQueue_Cancel();
    } else {
        result = DFQueue_Wait(id, 10000);
        if (result == DF_QUEUE_WAIT_DONE) {
            printf("task %u done\r\n", (unsigned)id);
        } else {
            printf("wait failed: %s notice=0x%02X\r\n",
                   DFQueue_LastError(), (unsigned)DFQueue_LastNotice());
            DFQueue_Cancel();
        }
    }
}
```

取消 / 零速是通过同一串口发出的请求，断线时无法保证送达；等待失败后不要自动提交下一条动作。

<a id="其他平台测试"></a>
## 8. Arduino 和 ROS 2 怎么用、怎么测

### Arduino

1. 库版：将 `DFCom_Arduino/DFCom` 放入 Arduino libraries，打开 [DcarON_Square.ino](DFCom_Arduino/DFCom/examples/DcarON_Square/DcarON_Square.ino)。单文件版：直接打开 [DcarON_Square_AllInOne.ino](DFCom_Arduino/SingleFile/DcarON_Square_AllInOne/DcarON_Square_AllInOne.ino)，无需安装库。二选一。
2. 选择 Uno / Nano 对应目标，上传时断开 D0/D1 车端连接，上传后再连接。**现成 loop 会自动运动**；首次只测通信时，先把 loop 改为仅调用 `DFCom.update()`（单文件为 `dcarUpdate()`），重新上传。
3. 库版查看 9600 调试口的 `car ready (IMU calibrated)`，并在 loop 打印 `DFCom.odom.frameCount`，确认持续增长。单文件版读取 `odomFresh` 后清零，确认会再次置 1；需要校准诊断则调用 `dcarQueryState()`，持续 `dcarUpdate()` 后检查 `carReceived/carImuOK`，现成 setup 没做这一步。
4. 全量 Odom 不可用时，选择支持的 `subscribeVelPos(10)` 并读 `DFCom.velpos`；`xCm/yCm/yawDeg` 仍读 Odom，不能用它们判断 VelPos。单文件的 Odom / VelPos 则更新同一组位置速度变量。
5. 通信通过后，建议先自定义一次 10cm 直线测试，再恢复方形。**每次检查 waitDone 的 bool 返回值，false 就 stop 并结束本轮**；原方形例程忽略返回值，仍可能超时后继续发下一条。

| 目的 | 库版 | 单文件版 |
|---|---|---|
| 初始化 / 解析回包 | `DFCom.begin(115200)` / `DFCom.update()` | `dcarBegin()` / `dcarUpdate()` |
| 平移 / 旋转 / 圆弧 | `moveLinear` / `moveRot` / `moveArc`（通过 `DFCom.` 调用） | `dcarMoveLinear` / `dcarMoveRot` / `dcarMoveArc` |
| 等上一条完成 | `DFCom.waitDone(ms)` | `dcarWaitDone(ms)` |
| 停车 | `DFCom.stop()` | `dcarStop()` |
| 订阅遥测 / 查状态 | `subscribeOdom` / `subscribeVelPos` / `queryState`（通过 `DFCom.` 调用） | `dcarSubscribeOdom` / `dcarSubscribeVelPos` / `dcarQueryState` |
| 看进度与结果 | `DFCom.progress(cmd)`，`waitDone` 返回值；`odom/velpos/state` | `dcarWaitDone` 返回值；`odomPosX/odomPosY/odomYaw/odomFresh`、`carReceived/carImuOK` |

库默认厘米、cm/s、度、度/s，可 `DFCom.useMeters()` 切为 SI；单文件固定厘米 / 度。profile=0 匀速、1 梯形加减速、2 距离闭环。
`waitDone` 仅自然完成返回 true，超时 / 中断 / 发送失败为 false；0 为无限等，超时本身不取消车端运动。`moveVel` / `dcarMoveVel` 无自动终点，必须主动停车。
正常方形是一圈四次 50cm 前进与 90° 左转、停约 1 秒再重复，库版打印 `pos x=`，单文件打印 `x=`。这些打印只代表走到了输出语句，**不代表八次等待都成功**。详细接线和限制见 [Arduino README](DFCom_Arduino/README.md)。

### ROS 2

按 [ROS README](ros2/README.md) 构建并加载环境，先以默认只读模式启动，执行 `ros2 topic echo /odom --once` 和 `ros2 topic hz /odom`：应收到新数据并持续更新（默认请求 10Hz，不要求墙钟严格等于 10Hz）。
开启 `enable_motion_actions:=true` 后测试 `/motion` 前进 0.1m；`goal accepted` 不是完成，最终 `success: true` 才是协议成功。再校验 square YAML，确认后加 `--execute` 跑一圈；不带该参数只校验、不动车。完整构建、启动和命令均在平台 README，反馈、目标与结果解释见[动作说明](ros2/docs/motion.md)。

<a id="验收标准"></a>
## 9. 测试通过的标准

按层记录，不能用上一层成功替代下一层。

| 层次 | 操作 | 通过标准 | 不足以判通过的现象 |
|---|---|---|---|
| 构建 / 上传 | 编译选定目标并上传外部板，或构建 ROS 包 | 构建无错误，上传成功，启动输出对应程序 | 只有编辑器能打开源码 |
| 双向通信 | 查询状态，连续订阅可用遥测 | 有状态回复；校准位为 1；接收帧 / 时间持续更新；没有诊断 FAIL | 只看到 TX 灯闪、固定旧值 |
| 任务协议 | 运行四段操场并观察多圈 | STM32 所有 ID 的 Wait 返回 DONE，并打印正确任务数；ROS 最终 success=true；Arduino 自定义门禁中每次 waitDone=true | 仅入队 ACK、ROS accepted、微调中、Arduino 循环走到打印位置 |
| 实际运动 | 地面标记起点、终点和朝向，尺量距离与角度，记录停止 | 方向和顺序符合所选路线；持续自动循环；结束静止；外部测量误差满足事先约定的车辆 / 任务指标 | 里程计恰好显示目标值或单独出现 DONE |
| 失败处理 | 在通信正常时制造可控的前置失败，例如 STM32 使用未校准车辆 | 不启动新路线，打印诊断失败；不得把未运行判成功。执行故障应看到失败结果，并在链路可用时观察停止 | 只看到程序调用 Cancel 就断定车已停 |

**本仓库没有实车标定数据，也没有可承诺的统一“±几毫米 / ±几度”标准。** 不把示例距离或车端内部到位容差当成实测精度保证。
实车测试至少记录：代码版本、平台、车端能力、底盘类型、地面 / 电量、路线参数、各任务结果 / notice、外部测量的起终点及朝向。各段从该段起点及起始朝向测量，不能把世界系累计位置直接当段位移；操场要检查各段与多圈重复误差，转满一圈后 yaw 可能回绕到起始值。
若需要数值验收，在测试前按车辆规格 / 应用需求约定距离误差、角度误差、停止速度阈值和重复次数，再用外部测量判定；当前例程不自动给出这些物理验收结论。

<a id="常见问题"></a>
## 10. 常见问题

| 现象 | 检查 |
|---|---|
| STM32 烧录后只有日志，没有动作 | 当前程序应自动循环；检查诊断、校准、v6 握手及 FAILED 原因，确认烧录的是当前版本 |
| 没有任何日志 | PA2 → USB-TTL RX、GND、115200、Use MicroLIB；Arduino 调试用 D3 / 9600 |
| 一直打印遥测，但车端断开也没变化 | 看 fr 是否增长；打印会重复最近值，不自带遥测失联报警 |
| `startup diagnosis failed` | 看 state_reply、imu_calibrated、new_velpos_frames；检查双向串口、校准和遥测能力 |
| `FAILED stage=begin` | 看 reason：v6 查询无回复、能力不足、设备队列非空等 |
| `FAILED stage=submit` | 看参数、档位、本地 8 槽、入队确认及 DMA 发送失败 |
| `FAILED stage=wait` | 看超时 / 设备故障和 notice，不能只看提交成功 |
| 距离差 100 倍、旋转角异常 | 队列固定米 / 弧度；Arduino 默认厘米 / 度；不要混用参数 |
| Arduino 上电就跑、失败后还发下一条 | 原例程循环运行且未检查 wait；先改为只收数据，再加失败门禁 |
| Odom 一直没数据 | 确认车端支持全量 Odom；标准 VelPos 和全量 Odom 是两种订阅与数据来源 |
| 更新 ROS 源码后行为没变 | 重新构建两个包，并在新终端加载安装环境 |

<a id="源码目录"></a>
## 11. 源码目录与进一步阅读

```text
DFCom_Example/                 STM32 完整工程
  USER/main.c                  自动诊断与连续操场四段的实际调用
  USER/Template.uvprojx         Keil 工程入口
  HARDWARE/DFCom.h              公共 API、参数、数据结构与返回值
  HARDWARE/DFCom_Tx.c           发送组帧、队列提交 / 等待 / 取消
  HARDWARE/DFCom_Rx.c           流式解析、回包分发、遥测更新
  HARDWARE/DFCom_Print.c        TIM2 毫秒 tick 和调试打印
  SYSTEM/usart/                串口 DMA 收发
DFCom_PatchOnly/               相同公共代码的移植包，非独立工程
DFCom_Arduino/                 库版与单文件版 Arduino 示例
ros2/                         ROS 接口、串口桥、轨迹与平台教程
docs/DFLink-v6.md              字节协议、单位和任务状态
tests/                        不连接车辆的离线回归与统一检查入口
```

- [STM32 完整工程说明](DFCom_Example/README.md)
- [STM32 移植步骤](DFCom_PatchOnly/README.md)
- [Arduino 说明](DFCom_Arduino/README.md)
- [ROS 2 说明](ros2/README.md)与 [Windows 环境教程](ros2/docs/windows-tutorial.md)
- [DFLink 队列 v6 协议](docs/DFLink-v6.md)

<a id="验证范围"></a>
## 12. 离线检查与验证范围

仓库根目录执行（Python >=3.10、PyYAML；C/C++ 检查需要主机编译器）：

```sh
python3 tests/run_checks.py
```

覆盖文档链接、工程引用、两份 STM32 公共代码一致性、报文编码、任务 ID、重复 / 迟到回包、查询恢复、超时和示例运行分支等。仅改文档时可定向执行 `python3 tests/run_checks.py --only static`。
CI 另有 ROS Jazzy 构建与内存串口集成检查，实际执行状态以对应提交的 GitHub Actions 为准。
离线测试不连接真车，不证明轨迹精度；Keil 目标编译、AVR 目标编译及 Flash/RAM、烧录和实车运动需在对应环境验证。
