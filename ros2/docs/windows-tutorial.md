# Windows 原生 ROS 2：从下载到 DCar 真车运行

更新：2026-09-10。已验证环境：Windows 11 x64、ROS 2 Lyrical、VS 2022、ATK 串口。
本教程运行电脑端客户端，不涉及底盘固件烧录，不需要 WSL、Linux 虚拟机或 Docker。

## 1. GitHub 上为什么看不到 ros2 文件夹？

本次代码在 **`codex/windows-ros2-bridge`** 分支，尚未合并的记录为 [PR #1](https://github.com/yangzkee/DCar-BaseLine/pull/1)。

1. 打开 [开发分支首页](https://github.com/yangzkee/DCar-BaseLine/tree/codex/windows-ros2-bridge)。
2. 或点仓库左上角 `main` 下拉框，在搜索框输入 `codex/windows-ros2-bridge`，选择同名分支。
3. 进入 `ros2`。`Tags` 里的 `v1.0.0` 是旧版本标签，不是这次开发分支。
4. 不用再次点击 `Compare & pull request` 创建重复 PR；已有 PR #1。

`README.md` 是 GitHub 自动显示的说明文档，不是一条需要执行的命令。

```text
DCar-BaseLine/                 同一个 Git 仓库
├── DFCom_Example/             STM32F1 Keil 示例
├── DFCom_PatchOnly/           可移植通信核心
├── DFCom_Arduino/             Arduino 示例
└── ros2/
    ├── dcaron_interfaces/     ROS 包一：Motion Action 消息定义
    ├── dcaron_bridge/         ROS 包二：串口驱动、配置、launch、轨迹示例、测试
    ├── tools/                Python 诊断和 Windows 辅助入口
    ├── docs/                 本教程、动作说明、验证记录
    └── README.md             ROS 接口总说明
```

ROS 源码在电脑上执行。DcarON 固件仓库与这个客户端仓库分开维护；不必把整套 ROS 环境复制进固件目录。

## 2. 安装前准备

- Windows 11 x64。本文固定 Lyrical，不将 Foxy/Humble/Jazzy 的二进制混装进同一环境。
- [Git for Windows](https://git-scm.com/downloads/win)。安装后重新打开终端，检查 `git --version`。
- [Pixi 安装器或官方安装方式](https://pixi.prefix.dev/latest/installation/)。安装后重新打开终端，检查 `pixi --version`。
- [Visual Studio 2022 Build Tools](https://visualstudio.microsoft.com/vs/older-downloads/#visual-studio-2022-and-other-products)，选择“使用 C++ 的桌面开发”，包含 MSVC 和 Windows SDK。已有 VS 2022 C++ 工作负载可直接用。

我们的串口节点是 Python，但自定义 Action 需要生成接口，因此构建时仍需要 C++ 工具链。
不要求打开 Visual Studio IDE。以下代码块除特别注明外，均在 **CMD 命令提示符**运行，
不要直接粘进 PowerShell；`set`、`call`、`%USERPROFILE%` 是 CMD 语法。

## 3. 下载 ROS 并安装依赖（首次执行）

从 [已验证的官方 Release](https://github.com/ros2/ros2/releases/tag/release-lyrical-20260807)
下载 `ros2-lyrical-2026-08-07-windows-AMD64.zip`。
本次实测文件 SHA256：

```text
ab3305fc9961848072d0665e7a7f2a772d4ae5a9499a3ac4c902e9d6f9f6bc5c
```

下载后可用 CMD 的 `certutil -hashfile "下载文件的完整路径" SHA256` 核对。
解压到短路径，本文统一采用：

```text
C:\dev\dcar_ros2\runtime\ros2-windows
```

确认这个目录内直接包含 `pixi.toml`、`preinstall_setup_windows.py`、`local_setup.bat`。
如果解压后多嵌套了一层目录，要使用真正包含这些文件的目录。

在 CMD 中逐条执行，上一条失败先处理错误：

```bat
cd /d C:\dev\dcar_ros2\runtime\ros2-windows
pixi install
pixi run python preinstall_setup_windows.py
pixi add "pyserial==3.5"
```

该发行包已包含 pytest 依赖。保留生成的 `pixi.toml` 和 `pixi.lock`，用于复现。
不要在系统 Python 中随意安装同名 ROS 包来补环境错误。
ROS 升级时应重新验证，不直接覆盖正在使用的安装目录。

官方参考：[Lyrical Windows 安装指南](https://docs.ros.org/en/lyrical/Get-Started/Installation/Windows-Install-Binary.html)，
[文档源文件](https://github.com/ros2/ros2_documentation/blob/lyrical/source/Get-Started/Installation/Windows-Install-Binary.rst)。
官方 Windows 包包含 ROS base 和部分 desktop 包，不等于全部第三方 ROS 生态。

## 4. 获取正确分支的源码（首次执行）

在 CMD 中运行，默认放在桌面：

```bat
cd /d "%USERPROFILE%\Desktop"
git clone --branch codex/windows-ros2-bridge https://github.com/yangzkee/DCar-BaseLine.git
cd DCar-BaseLine
git branch --show-current
```

最后应显示 `codex/windows-ros2-bridge`。已有同名仓库时不要重复 clone，先查看状态：

```bat
cd /d "%USERPROFILE%\Desktop\DCar-BaseLine"
git status
git fetch origin
git switch codex/windows-ros2-bridge
git pull --ff-only
```

如有自己的未提交修改，先处理或保存它们；不要用强制重置覆盖文件。
也可在正确分支页面点 `Code → Download ZIP`，但日后更新建议使用 Git。

## 5. 激活环境并编译（首次或源码更新后）

从开始菜单打开 **x64 Native Tools Command Prompt for VS 2022**。
逐条执行；`pixi shell` 返回新 shell 提示符后继续后面的命令：

```bat
cd /d C:\dev\dcar_ros2\runtime\ros2-windows
pixi shell
set PYTHONNOUSERSITE=1
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
chcp 65001
call C:\dev\dcar_ros2\runtime\ros2-windows\local_setup.bat
python -c "import rclpy, serial, pytest; from nav_msgs.msg import Odometry; print('ROS and serial OK')"
cd /d "%USERPROFILE%\Desktop\DCar-BaseLine\ros2"
colcon --log-base C:/dev/dcar_ros2/log build --merge-install --base-paths . --packages-up-to dcaron_bridge --build-base C:/dev/dcar_ros2/build --install-base C:/dev/dcar_ros2/install
call C:\dev\dcar_ros2\install\local_setup.bat
```

期望看到 `2 packages finished`，包名分别是 `dcaron_interfaces` 和 `dcaron_bridge`。
这里将生成文件放到 C 盘短路径，源码仍在桌面；不使用教程之外的 `ros2/install` 路径。
UTF-8 设置只影响当前进程，解决中文 Windows 的 GBK 模板解码问题。

运行离线测试，不连接也不驱动车辆：

```bat
python -m pytest dcaron_bridge/test -q
ros2 interface show dcaron_interfaces/action/Motion
```

本次代码测试结果为 **28 passed**。如有 skipped，不能当作完整 ROS 集成测试通过。

## 6. 每次新开终端都要激活（无需重新安装或编译）

打开 CMD；以下整段在每个需要运行 ROS 的终端执行一次：

```bat
cd /d C:\dev\dcar_ros2\runtime\ros2-windows
pixi shell
set PYTHONNOUSERSITE=1
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
chcp 65001
call C:\dev\dcar_ros2\runtime\ros2-windows\local_setup.bat
call C:\dev\dcar_ros2\install\local_setup.bat
cd /d "%USERPROFILE%\Desktop\DCar-BaseLine\ros2"
```

两个终端需要相同的 ROS 环境和 `ROS_DOMAIN_ID`（未设置时用默认值）。
构建时使用第5节的 VS 工具终端；日常运行不需要启动 IDE。

## 7. 接线、识别串口、先读取回传

电脑 USB-TTL 直接接 **Dcar 底盘的 DFLink 口**，TX/RX 交叉、GND 共地，3.3V TTL。
本次使用 ATK 下载器自带串口接底盘 USART1，460800、8N1。
USART1 是 MCU 端口名，不代表 Windows 的 COM1；本机识别为 COM5，其他电脑可能不同。
这里不经过 STM32F1 示例板，也不使用它的 115200 printf 口。电机使用底盘规定电源。

在已激活的终端 A：

```bat
python -m serial.tools.list_ports -v
ros2 launch dcaron_bridge bridge.launch.py port:=COM5
```

将 COM5 换成 ATK 对应的实际端口，关闭占用它的 DFhelper 或串口助手。
默认不开启运动输入；节点读取 VelPos 遥测。保持终端 A 运行。

另开终端 B，先执行第6节，再运行：

```bat
ros2 topic list
ros2 topic echo /odom --once
ros2 topic hz /odom
```

`echo --once` 显示一条后退出；`hz` 连续显示接收频率，按 Ctrl+C 结束该观察命令。
位置单位米，速度米/秒，姿态使用四元数；+X 前、+Y 左、正 yaw 逆时针。
默认 VelPos 不提供角速度，`/odom` 中角速度零是占位值，不能据此判断车辆没在旋转。

## 8. 先验证小角度旋转，再扩展动作

以下命令会让真车运动。确认车轮周围无障碍、现场能随时停机后执行；只运行一个控制来源。

在终端 A 按 Ctrl+C 退出只读节点，然后开启 Action：

```bat
ros2 launch dcaron_bridge bridge.launch.py port:=COM5 enable_motion_actions:=true
```

终端 B（已激活）发送一次相对左转15°，最大角速度0.2rad/s，8秒超时：

```bat
ros2 action send_goal /motion dcaron_interfaces/action/Motion "{command: 99, yaw: 0.2617993878, angular_speed: 0.2, timeout: 8.0}" --feedback
```

观察 goal accepted、feedback 和最终结果；正常为 `SUCCEEDED`、`success: true`、`notice: 0`。
接收成功不等于执行完成，要等最终结果。关闭发送目标的终端不等于取消车上已接受的目标。
节点正常退出会尝试发零，但串口断开或主机失效时不能保证停止命令送达，现场仍要能停机。

本次两条实测路径：纯 Python 回传旋转14.90°；ROS Action + `/odom` 回传14.77°。
均为底盘自身测量值，详见 [原始验证说明](motion-validation-2026-09-10.md)。

也可用自带的完整 ROS 诊断脚本（它自己打开串口，必须先退出终端 A 的桥接节点）：

```bat
python tools/probe_ros_rotation.py --port COM5 --degrees 15 --rate 0.2 --log rotation-ros.jsonl
```

需要先绕过 ROS 排查串口时，同样保持串口无人占用，在当前环境运行：

```bat
python tools/probe_rotation.py --port COM5 --degrees 15 --rate 0.2 --timeout 8 --log rotation-direct.jsonl
```

诊断脚本省略 `--degrees` 时只读遥测，不发送非零旋转目标。

## 9. 轨迹、速度与更多数据

完整字段和动作语义见 [动作教程](motion.md)，包括直线、圆弧、平移并转向和十字标。
先验证 YAML，不运动：

```bat
ros2 run dcaron_bridge motion_sequence dcaron_bridge/examples/square.yaml
```

只有加 `--execute` 才会执行；需要已有开启 Action 的桥接节点。
轨迹逐段等待成功，失败即停止，不是 Nav2 连续路径跟踪。
目前仅旋转完成实车验证，直线/圆弧/整条方形等仍需现场分别验证。

持续速度需单独启用 `enable_cmd_vel:=true`，输入类型是 `geometry_msgs/msg/Twist`。
单位和零速度示例见 [ROS README](../README.md#速度输入与-pro-数据)。
持续速度无 MCU 自动终点，主机超时发零不等于断线硬件看门狗；不要让速度发布器和 Action 争用控制。

Pro 全量 IMU 数据用 `telemetry:=odom`，还要求底盘授权和固件支持。
默认模式没有 `/imu/data` 是预期行为。配置和协方差说明见 [接口文档](../README.md)。

## 10. 常见问题

| 现象 | 检查方式 |
|---|---|
| GitHub 没有 ros2 | 切到 `codex/windows-ros2-bridge`，不要停留在 main 或旧标签 |
| pixi / git 不是内部命令 | 安装后重新打开终端，确认 PATH；本机便携 Pixi 可用完整 exe 路径 |
| 找不到 rclpy / DLL 加载失败 | 执行第6节，`where python` 检查是否来自 ROS 的 Pixi 环境，不混用系统 Python |
| 找不到 dcaron_bridge / Motion | 确认两包构建成功，执行 C 盘短路径的 install/local_setup.bat |
| GBK / UnicodeDecodeError | 在启动编译进程前设置第5节三个环境变量并切 UTF-8 代码页 |
| 找不到 C++ 编译器 | 安装 VS 2022 C++ 工作负载，从 x64 Native Tools 终端编译 |
| RTI Connext 未找到 | 可选中间件提示；默认 Fast DDS 路径不要求安装 Connext |
| 串口拒绝访问 | 退出串口助手、DFhelper、其他桥接/诊断进程，一个串口只有一个拥有者 |
| 有串口但无 odom | 检查实际 COM、460800、底盘供电、TX/RX/GND 和所接固件是否支持 USART1 DFLink |
| Action 被拒绝 | 检查 enable_motion_actions、遥测新鲜度、已有任务、参数范围及节点日志 |
| 超时后相同动作一直被拒绝 | 可能在隔离旧任务迟到回包；先确认真车停止，再按动作文档恢复，不连续盲发 |
| 更新代码后行为没变 | 重新构建，新开终端重新加载安装环境，再启动节点 |

## 11. 当前开发机已经装好了，怎么对应？

当前开发机的源码是 `C:\Users\3164567\Desktop\DCar-BaseLine\ros2`，
ROS 环境和构建目录与本文 C 盘路径一致。
Pixi 采用便携版 `C:\dev\dcar_ros2\pixi\pixi.exe`；若 CMD 中 `pixi` 不在 PATH，
可在第6节第一条命令前执行 `set "PATH=C:\dev\dcar_ros2\pixi;%PATH%"`。
不需要重复下载或再次 clone。

DcarON 仓库的 `examples/ros/README.md` 是维护入口，真板日志存放在其
`Delivery_Files/Debug_Logs/`。不要将 `.pixi`、build、install、log 等环境产物提交到 GitHub。
