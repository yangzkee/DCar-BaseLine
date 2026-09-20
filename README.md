# DCar 官方客户端例程与学习导航

通过串口控制 DCar / DcarON 小车，从第一个运动指令开始，学习接线、运动编排、里程计读取和通信调试。

本仓库提供运行在**外部开发板或电脑**上的客户端代码。本开发分支包含 STM32F103 Keil 工程、电赛固定路线、Arduino 库与示例、STM32 移植包，以及在电脑上运行的 ROS 2 桥接包。

**第一次来：先选平台，再按对应教程运行，最后修改动作。** 稳定版的各平台放在 `main` 的不同目录中，选平台不需要切换 Git 分支；开发中的新功能统一放在 `develop`。

[选择平台](#platforms) · [下载例程](#download) · [学习路线](#learning) · [STM32 快速开始](#stm32) · [Arduino 快速开始](#arduino) · [修改与调试](#debugging) · [协议与单位](#protocol) · [参与维护](#contributing)

[产品与完整教程](https://differ-tech.pages.dev/portal/view/dcar-fast-motion-control) · [DFLink V3 对外协议手册](DFLink_V3协议手册_对外版.md) · [版本下载](https://github.com/yangzkee/DCar-BaseLine/releases) · [问题反馈](https://github.com/yangzkee/DCar-BaseLine/issues)

<a id="platforms"></a>
> 当前是 `develop` 开发集成版。需要稳定入门例程，请前往 [main 首页](https://github.com/yangzkee/DCar-BaseLine/tree/main)。

## 1. 选择你的平台

| 你的目标 | 从这里开始 | 开发环境与使用前提 |
|---|---|---|
| 用 STM32 学习串口控制，修改 C 语言动作程序 | [STM32 完整工程教程](DFCom_Example/README.md) | STM32F103C8T6、Keil MDK5 / ARM Compiler 5；客户端通信口 460800 |
| 用 Arduino IDE 学习，调用库函数控制小车 | [Arduino 库与示例教程](DFCom_Arduino/README.md) | Uno / Nano（ATmega328P）；现有例程要求底盘 USART3 的 115200 双向通信支持，见下方前提 |
| 不安装库，直接查看一个完整 Arduino 程序 | [Arduino 单文件示例](DFCom_Arduino/SingleFile/DcarON_Square_AllInOne/DcarON_Square_AllInOne.ino) | 与 Arduino 库版相同的接线、开发板和底盘固件前提 |
| 把通信功能接入自己已有的 STM32 工程 | [STM32 移植包说明](DFCom_PatchOnly/README.md) | 需要目标工程原有的启动代码、标准库和系统辅助文件；不是独立完整工程 |
| 体验 ROS 2 与 Windows 教程 | [ROS 2 教程](ros2/README.md) | 开发版，尚未合入稳定分支；实际验证范围见该目录文档 |
| 体验 2026 电赛 H/D 固定路线 | [本页固定路线说明](#nuedc) | 开发版 STM32 路线，运行行为与稳定版不同，先阅读说明 |
| 使用 ROS 1 或独立 Python 入门例程 | 当前尚未提供对应独立教程 | 可以先阅读协议手册 |

> **底盘兼容性：** 选对客户端开发板之后，还要确认底盘固件支持对应通信口、波特率、运动指令和回传格式。当前仓库尚未列出完整的底盘固件版本兼容表，不能据此认为所有历史固件都能直接使用。

<a id="download"></a>
## 2. 下载例程

- **只想运行已有发布版：** 打开 [Releases](https://github.com/yangzkee/DCar-BaseLine/releases)，选择对应平台的附件。`v1.0.0` 提供 `DCar-STM32F103C8-Keil.zip` 和 `DCar-STM32F103C8-Arduino.zip`；附件具体支持的开发板以包内说明为准。
- **想学习或修改当前源码：** 点击仓库绿色 **Code → Download ZIP**，解压后进入所选平台目录。
- **使用 Git 持续维护：** 克隆仓库后进入所选目录，各平台独立使用，不必安装其他平台的开发环境。

```sh
git clone --branch develop https://github.com/yangzkee/DCar-BaseLine.git
cd DCar-BaseLine
```

`main` 是稳定发布分支，`develop` 是开发集成分支；Release / 标签用于保存正式发布时的版本。发布附件不一定包含分支后续的修复，反馈问题时请注明分支、标签或提交号。

<a id="learning"></a>
## 3. 从入门到修改的学习路线

| 步骤 | 学习内容 | 你应当确认的结果 |
|---|---|---|
| ① 接通通信 | 按平台教程接线，区分底盘通信口与电脑调试口 | 调试终端有启动输出，状态查询或里程计有回传 |
| ② 看懂原例程 | 找到 `main()` / `setup()` / `loop()`，区分初始化、订阅和运动指令 | 能指出程序在哪里发指令、在哪里等待、在哪里读数据 |
| ③ 修改一个动作 | 先改距离或角度中的一项，理解厘米/度与 SI 单位 | 能解释修改值、预期方向与实际动作之间的关系 |
| ④ 编排动作序列 | 组合直线、转向、圆弧，检查等待结果 | 能区分完成、超时和继续发送下一条指令 |
| ⑤ 读取反馈 | 查看位置、速度、航向与帧计数 | 能判断数据是否持续更新，并理解数据的单位和坐标系 |
| ⑥ 移植与深入调试 | 阅读发送、接收解析和协议手册，运行离线测试 | 能沿“发送 → 底盘处理 → 回传 → 客户端解析”定位问题 |

现有例程会发送运动指令。首次运行前预留运动空间；只想观察通信时，先注释业务运动段，保留初始化、订阅与接收处理。

<a id="stm32"></a>
## 4. STM32 快速开始

### 接线与环境

| STM32F103C8T6 引脚 | 连接目标 | 参数 |
|---|---|---|
| PA9 / USART1_TX | 底盘所选 DFLink 通信口 RX | 460800，8N1 |
| PA10 / USART1_RX | 底盘所选 DFLink 通信口 TX | 460800，8N1 |
| PA2 / USART2_TX | USB-TTL RX，用于电脑查看打印 | 115200，8N1 |
| PA3 / USART2_RX | USB-TTL TX，预留接收 | 115200，8N1 |
| GND | 底盘及 USB-TTL GND | 共地 |

TX/RX 交叉连接，使用匹配的 3.3V TTL 电平；底盘独立按产品要求供电，不用 USB-TTL 给电机供电。表中的 PA9/PA10 是**外部 STM32 开发板**引脚，底盘侧接口以对应产品接线说明为准。

1. 安装 Keil MDK5、ARM Compiler 5 和 STM32F1 Device Family Pack。
2. 打开 [Template.uvprojx](DFCom_Example/USER/Template.uvprojx)。
3. 检查目标器件 `STM32F103C8`、宏 `STM32F10X_MD, USE_STDPERIPH_DRIVER`、启动文件 `startup_stm32f10x_md.s`，并勾选 **Use MicroLIB**。
4. 编译并下载到外部 STM32F103 开发板，打开电脑调试串口（115200，8N1）。
5. 查看 `[INIT]` 诊断输出，确认状态回包及 `Telemetry flowing`；运行时查看里程计打印。

工具安装参考：[ARM Compiler / 芯片包获取说明](https://ucnerk2uhr85.feishu.cn/wiki/BaUMwzR4liGc6FkMES1c64c6nOb?renamingWikiNode=true#share-APaqdPwYPo1PIhxCeiac3fzJnih)。

### 从哪里开始改

本开发分支的 [USER/main.c](DFCom_Example/USER/main.c) 已接入电赛固定路线：启动握手完成后等待 3 秒，自动运行所选路线一次。参数入口是 [nuedc_2026_routes.h](DFCom_Example/USER/nuedc_2026_routes.h)，详见下方说明。稳定版 `main` 仍保留往返和转向示例。

开发版修改路线时保留初始化和启动运动会话处理，优先调整路线参数头文件；学习最基础的单条动作可以先使用稳定版 `main`。需要完整 API、订阅与反馈字段说明时，继续阅读 [STM32 详细教程](DFCom_Example/README.md)。

<a id="nuedc"></a>
## 2026 电赛 H/D 题固定赛道

STM32 主工程现提供两套独立的胶囊形固定路线，均从 A 点顺时针运行一圈：

- H 题：1.5 m 直线、0.50 m 半圆、1.5 m 直线、0.50 m 半圆；默认 0.34 m/s。
- D 题：1.5 m 直线、0.741 m 半圆、1.5 m 直线、0.741 m 半圆；默认 0.25 m/s。

路线使用连续速度指令，按局部里程投影结束直线、按累计航向结束半圆；四段之间
不停车，只在完成一圈或发生里程失联/超时时停车。实现不读取光电、灰度或其他
循线传感器。

唯一调参入口：

```text
DFCom_Example/USER/nuedc_2026_routes.h
```

修改下面一个宏即可选择烧录后执行 H 或 D，默认 H：

```c
#define NUEDC_2026_ACTIVE_ROUTE NUEDC_2026_ROUTE_H
```

主程序完成通信握手后等待 3 秒，自动执行所选路线一次并停车；需要重跑时按开发
板复位键。D 题半径按已确认的 741 mm 印刷地图设置；若使用官方 750 mm 场地，
将 `NUEDC_2026_D_RADIUS_M` 改为 `0.750f`。

路线主机仿真测试：

```sh
sh tests/host/run_nuedc_2026_routes_tests.sh
sh tests/host/run_nuedc_2026_integration_checks.sh
```

<a id="arduino"></a>
## 5. Arduino 快速开始

**先确认底盘前提：** 当前 Arduino 教程按 Uno / Nano（ATmega328P）和底盘 USART3（115200）编写，并依赖底盘在该接口回传里程计与运动完成信息。仅导入 Arduino 库不会给底盘增加这项能力；如果不确定底盘固件是否支持，先核对固件说明或向维护者反馈版本信息。

1. 按 [Arduino 教程](DFCom_Arduino/README.md) 确认接线及 5V → 3.3V 电平转换。
2. 将 [DFCom 库目录](DFCom_Arduino/DFCom) 放入 Arduino 库目录，或将该目录打包后用 IDE 的“添加 .ZIP 库”导入。
3. 打开 [DcarON_Square.ino](DFCom_Arduino/DFCom/examples/DcarON_Square/DcarON_Square.ino)，选择 Uno / Nano 并编译上传；上传时断开占用 D0/D1 的底盘连接，完成后再接回。
4. 可通过例程的 D3 调试输出连接 USB-TTL，以 9600 波特率查看状态。底盘通信仍为 115200。
5. 在 `loop()` 中修改方块边长、转角与等待时间。初学时保持订阅 10Hz，避免大量调试打印干扰接收。

不想安装库，可以使用 [单文件版](DFCom_Arduino/SingleFile/DcarON_Square_AllInOne/DcarON_Square_AllInOne.ino)。库版和单文件版是两种使用方式，选择一种即可。

<a id="debugging"></a>
## 6. 修改与调试导航

ROS 2 用户从 [Windows 逐步安装教程](ros2/docs/windows-tutorial.md) 开始，再读 [桥接包说明](ros2/README.md)、[运动动作与轨迹](ros2/docs/motion.md) 和 [已有实车验证记录](ros2/docs/motion-validation-2026-09-10.md)。该记录目前仅覆盖部分动作，不能当作所有轨迹已验证。

### 想改什么，就看哪里

| 目标 | STM32 | Arduino |
|---|---|---|
| 改距离、速度、转角、动作顺序 | [main.c](DFCom_Example/USER/main.c) | [DcarON_Square.ino](DFCom_Arduino/DFCom/examples/DcarON_Square/DcarON_Square.ino) |
| 查 API、指令码与数据结构 | [DFCom.h](DFCom_Example/HARDWARE/DFCom.h) | [DFCom.h](DFCom_Arduino/DFCom/src/DFCom.h) |
| 看指令打包与发送 | [DFCom_Tx.c](DFCom_Example/HARDWARE/DFCom_Tx.c) | [DFCom.cpp](DFCom_Arduino/DFCom/src/DFCom.cpp) |
| 看回传解析与运动等待 | [DFCom_Rx.c](DFCom_Example/HARDWARE/DFCom_Rx.c) | [DFCom.cpp](DFCom_Arduino/DFCom/src/DFCom.cpp) |
| 改串口配置 | [usart.c](DFCom_Example/SYSTEM/usart/usart.c) | 示例 `setup()` 中的 `DFCom.begin(115200)`，需与底盘匹配 |
| 改调试输出 | [DFCom_Print.c](DFCom_Example/HARDWARE/DFCom_Print.c) | 示例中的 `dbg` 打印 |

### 先看现象，再定位

| 现象 | 先检查 |
|---|---|
| 电脑没有任何输出 | 是否连接了调试口而非底盘通信口；波特率是否正确；STM32 的 MicroLIB 配置 |
| 有启动打印，没有状态或里程计 | 底盘供电、共地、TX/RX、底盘接口及波特率、固件回传支持；Arduino 尤其检查 USART3 前提 |
| 有回包但小车不动 | 底盘激活状态、校准状态和运行模式；收到某个状态包本身不等于具备全部运动条件 |
| 距离或转角差很多 | 默认是厘米/度，还是已经切换 SI；不要把 `0.5` 米误传为 `0.5` 厘米 |
| 方向不符合预期 | `+X` 前、`+Y` 左、`+Yaw` 逆时针；核对控制指令与反馈字段各自的坐标系 |
| 等待超时或动作被下一条打断 | 查看完成回包、等待返回值和时间上限；不要把超时当成成功到位 |
| Arduino 间歇丢数据 | 减少 SoftwareSerial 打印，降低订阅频率，检查接线与电平 |

### 运动等待的含义

- STM32：`WaitMoveDone(CMD_LINEAR, 10000)` 等待对应直线任务；Arduino：`DFCom.waitDone(10000)` 等待当前任务。两者都是“收到完成信息就返回，否则最多等待 10 秒”。
- **等待超时不会自动发停车指令，也不证明动作完成。** 应检查返回值，再决定重试、停车或发送下一条运动指令；下一条指令可以打断上一条。
- `timeout == 0` 表示永久等待，链路失联时可能一直等待。
- 持续速度指令没有同样的到位回传，不能依赖上述等待来停车；需要主动发送零速度（STM32：`Cmd_Move_Vel(0, 0, 0)`；Arduino：`DFCom.stop()`）。

客户端内部使用 A/B 运动槽隔离旧任务回包。由于协议回传不含 task id，某些旧终止帧丢失场景仍存在歧义，实现会丢弃模糊的完成帧，让当前等待走到超时，避免错误提前进入下一动作。细节见源码与离线测试。

<a id="protocol"></a>
## 7. 协议、坐标系与单位

**DFCom** 是客户端通信代码及 API 的名称，源码中保留 `DFCom v2` 的称呼；**DFLink V3** 是仓库所附对外协议手册的协议族名称。它们与底盘固件发布版本不是同一个版本号，也不表示客户端已实现手册中的所有功能。

| 量 | 默认 CM 模式的运动入参 | SI 模式的运动入参 |
|---|---|---|
| 距离 | 厘米（cm） | 米（m） |
| 线速度 | 厘米/秒（cm/s） | 米/秒（m/s） |
| 角度 | 度（deg） | 弧度（rad） |
| 角速度 | 度/秒（deg/s） | 弧度/秒（rad/s） |

STM32 用 `g_dfcom_unit_mode = DFCOM_UNIT_M` 切到 SI；Arduino 用 `DFCom.useMeters()`。修改单位模式时，要同步调整运动参数数值。

解码后的原始里程计字段使用 SI 单位，不随发送单位模式改变；Arduino 的 `xCm()`、`yCm()`、`yawDeg()` 等便捷接口会转换为厘米或度。在线协议的整数缩放规则以 [DFLink V3 协议手册](DFLink_V3协议手册_对外版.md) 为准。

坐标轴采用 `+X` 前、`+Y` 左、`+Yaw` 逆时针为正。反馈中包含车体系和世界系字段，编写闭环算法前应先确认字段所属坐标系。

<a id="contributing"></a>
## 8. 参与维护与反馈

仓库保持两个长期分支：

| 分支 | 用途 | 适合谁 |
|---|---|---|
| `main` | 稳定发布；开发内容验证后再合入 | 首次学习、按发布教程运行 |
| `develop` | 汇总新平台、新场景和待验证改动 | 参与开发、体验新功能 |

平台与场景用目录或配置区分，不各自长期占用一个分支。日常开发统一进入 `develop`，验证后通过 Pull Request 合入 `main`，正式发布再打标签。确有并行协作需要时可短期开任务分支，合入后及时清理。

维护例程时，请同步更新对应教程中的接线、开发环境、单位和底盘固件要求；新增平台应提供独立运行说明，并在首页标明实际支持状态。修改完整工程、移植包或 Arduino 库时，检查相关副本和单文件示例是否需要同步。

仓库已有离线回归测试。在安装 C/C++ 编译器及 `sh` 的主机上，从仓库根目录运行：

```sh
sh tests/host/run_move_slots_tests.sh
sh tests/arduino/run_arduino_tests.sh
sh tests/host/run_nuedc_2026_routes_tests.sh
sh tests/host/run_nuedc_2026_integration_checks.sh
```

ROS 2 的协议与运动逻辑可在未安装 ROS 的 Python 环境测试：

```sh
cd ros2/dcaron_bridge
python3 -m unittest discover -s test -p test_protocol.py -v
python3 -m unittest discover -s test -p test_motion.py -v
```

测试覆盖 STM32 完整工程与移植包、Arduino 库与单文件版的运动等待等逻辑；它们不能代替 Keil / AVR 目标编译和真实底盘通信验证。

反馈问题请使用 [GitHub Issues](https://github.com/yangzkee/DCar-BaseLine/issues)，附上：例程目录与提交号或发布标签、开发板型号、底盘固件版本、接线与波特率、复现步骤、预期行为和实际日志。请用 UTF-8 保存代码与文档。
