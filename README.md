# DCar 客户端例程

当前版本 **0.3.1**。STM32 和 ROS 2 的路线控制使用 **DFLink 队列 v6**。
本仓库的程序运行在外部开发板或电脑上，通过串口控制 DCar / DcarON；不是小车主控固件。

## 从哪里开始

| 你使用的平台 | 打开的文件 | 当前能力 |
|---|---|---|
| STM32F103C8T6 + Keil | [完整工程](DFCom_Example/USER/Template.uvprojx)，[主函数](DFCom_Example/USER/main.c)，[使用说明](DFCom_Example/README.md) | v6任务队列，前进10cm、正方形、操场三个示例 |
| 已有 STM32 工程 | [移植说明](DFCom_PatchOnly/README.md) | 同一份通信核心，需接入自己的串口和毫秒tick |
| 电脑 + ROS 2 | [ROS使用说明](ros2/README.md) | `/motion` 单动作、`/trajectory` 多段轨迹、里程计及可选持续速度 |
| Arduino Uno / Nano | [Arduino使用说明](DFCom_Arduino/README.md) | 无任务ID的逐条运动接口；未实现v6队列，不能直接套用STM32的队列示例 |

下载当前源码请选择 **main**：

```sh
git clone --branch main https://github.com/yangzkee/DCar-BaseLine.git
cd DCar-BaseLine
```

也可以在 main 页面选择 **Code → Download ZIP**。Release附件与当前源码不一定一致。

## STM32：从主函数直接看懂运动

1. 用 Keil MDK5 / ARM Compiler 5 打开 `DFCom_Example/USER/Template.uvprojx`，启用 **Use MicroLIB**。
2. 接好车端串口和115200调试终端，编译、烧录外部STM32板。
3. 默认 `DEMO_ENABLE_MOTION=0`：发送启动停车指令，检查通信和校准状态，持续打印遥测，**不会自动跑路线**。终端会明确打印 `DISABLED`。
4. 在 `USER/main.c` 顶部将 `DEMO_ENABLE_MOTION` 改为 `1`，选择 `DEMO_ROUTE`，重新编译烧录。
5. 通信/校准检查通过后等待3秒，握手确认车端队列v6及空闲状态，运行一次。结束后不重复，复位才重新开始。

```c
#define DEMO_ENABLE_MOTION 1
#define DEMO_ROUTE DEMO_FORWARD  /* 也可选 DEMO_SQUARE / DEMO_STADIUM */
```

三个示例的实际 `DFQueue_Linear` / `DFQueue_Rotate` / `DFQueue_Arc` 调用都在 **main函数内**：

| 选择 | 动作 | 主要参数 |
|---|---|---|
| `DEMO_FORWARD`（默认选择） | 前进10cm | 位移0.10m，速度0.06m/s |
| `DEMO_SQUARE` | 前进50cm、原地左转90°，重复四次 | 速度0.15m/s，转速0.5rad/s |
| `DEMO_STADIUM` | 倒行半圆 → 后退60cm → 倒行半圆 → 后退60cm并旋转360° | 半径0.30m，速度0.15m/s；要求全向/麦克纳姆底盘 |

修改距离和角度只需修改main中的动作参数。请求速度是上限请求，不保证全程恒速；车端仍会按加速度、曲率和旋转能力限速。

## 单位、方向和加速度

**`DFQueue_*` 永远使用米、弧度、m/s、rad/s。** 不读取 `g_dfcom_unit_mode`。
`0.50f` 是50cm，`1.5707963f` 是90°；正X向前、正Y向左，正yaw为逆时针。
位移与转角相对各段动作起点；不是地图中的绝对目标。横移和边走边转的能力由底盘决定。

| 宏 | 数值 | 档位 |
|---|---|---|
| `DF_ACCEL_VERY_LOW` | 16 | 极低 |
| `DF_ACCEL_LOW` | 17 | 低 |
| `DF_ACCEL_MEDIUM` | 18 | 中，示例默认 |
| `DF_ACCEL_HIGH` | 19 | 高 |
| `DF_ACCEL_VERY_HIGH` | 20 | 极高 |

这些档位用于队列直线和圆弧；原地旋转帧不含档位。
Arduino的 `move*` 和STM32的无ID `Cmd_Move_*` 是不同接口，单位开关及profile取值见各自声明，不能与活动队列混发。

## 一个动作如何完成

```c
u16 id;
if (DFQueue_Begin()) {
    id = DFQueue_Linear(0.10f, 0, 0.06f, 0, DF_ACCEL_MEDIUM);
    if (!id || DFQueue_Wait(id, 10000) != DF_QUEUE_WAIT_DONE) {
        printf("reason=%s notice=0x%02X\r\n", DFQueue_LastError(), DFQueue_LastNotice());
        DFQueue_Cancel();
    }
} else {
    printf("begin failed: %s\r\n", DFQueue_LastError());
}
```

- `Begin`：查询车端能力和下一任务ID。车端必须支持v6、队列为空；失败不降级。
- 发送：返回非零任务ID只表示已确认入队，**不表示运动完成**。
- `Wait`：按ID等车端终止结果，成功后释放本地槽。返回1成功、2超时/未知ID/零时限、3设备故障。时限必须大于0，单位毫秒，从本次Wait调用开始计时。
- 连续路线：先提交整组，再按ID等待；STM32最多同时跟踪8条。严格逐段执行则每发一条立即Wait。
- 异常：示例记录失败阶段、原因与notice，尝试取消并发送零速，不自动重跑。断线时不能保证停车指令送达。

参数、编码和回包的完整说明见 [DFLink队列v6协议](docs/DFLink-v6.md)。

## 接线与固件要求

| 连接 | 参数 |
|---|---|
| STM32 PA9(TX) → 车RX，PA10(RX) ← 车TX，GND共地 | USART1，460800，8N1，3.3V TTL |
| STM32 PA2(TX) → USB-TTL RX，GND共地 | USART2，115200，8N1，用于printf日志 |
| 电脑ROS → 车 | USB-TTL直接接车端DFLink口，460800；不经过示例STM32板 |

车端必须提供队列v6及对应完成回传，运动前完成激活/校准。默认STM32示例使用标准VelPos，**不要求Pro全量Odom**。
本仓库不提供可据此确认兼容的v6车端固件烧录包；没有完整的“底盘固件版本号→能力”对照表，以实际握手结果为准。

## 代码分工

- `USER/main.c`：配置、初始化、诊断、可直接修改的动作示例。
- `HARDWARE/DFCom.h`：公共API、参数单位、返回值。
- `HARDWARE/DFCom_Tx.c`：发送组帧、任务ID、队列提交/等待/取消。
- `HARDWARE/DFCom_Rx.c`：串口流式解析，按B=0x85分发队列回包，更新遥测。
- `HARDWARE/DFCom_Print.c`：TIM2毫秒tick和遥测打印；打印Yaw为度，数据字段仍为弧度。
- `SYSTEM/usart`：DMA收发；`DFCom_PatchOnly`中的公共文件与完整工程保持一致。
- `ros2`：ROS接口定义、串口桥接、轨迹客户端与测试。
- `tests`：不连接车辆的主机回归；统一入口 `python3 tests/run_checks.py`（Python>=3.10，需PyYAML）。

## 遇到问题先看什么

| 现象 | 检查 |
|---|---|
| 烧录后只有日志，没有动作 | `DEMO_ENABLE_MOTION`默认0，改1后重新编译烧录 |
| 没有任何日志 | PA2接USB-TTL RX、115200、GND、Use MicroLIB |
| `startup diagnosis failed` | 日志中的state_reply、imu_calibrated、new_velpos_frames；检查通信或校准 |
| `FAILED stage=begin` | 查看reason；v6查询无回复、能力不足或车端队列非空 |
| `FAILED stage=submit` | 参数、档位、本地8槽、入队确认或DMA发送失败 |
| `FAILED stage=wait` | 任务超时或车端故障；同时查看notice，不把入队成功当完成 |
| 运行距离不对 | 队列入参为米，接收位置也是米；不要按厘米填数 |
| 更新ROS源码后行为没变 | 重新构建两包，并在新终端加载安装环境 |

## 验证边界

离线检查覆盖报文编码、任务ID、重复/迟到回包、查询恢复、超时和示例运行分支。
本次整理不连接真车、不烧录主控；离线测试不能证明实车轨迹精度。
Keil目标编译、ROS运行环境和底盘实测须在相应环境验证，不能把旧记录当成本版验证结果。
