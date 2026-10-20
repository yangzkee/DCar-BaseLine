# Windows ROS 2：构建和运行当前客户端

本教程使用Windows 11 x64、ROS 2 Lyrical、VS 2022 C++工具链和CMD。
当前源码为0.3.1，队列运动要求车端v6。环境安装步骤不代表本版已完成Windows或真车验证。

## 1. 准备环境

安装Git、[Pixi](https://pixi.prefix.dev/latest/installation/)和Visual Studio 2022 Build Tools的“使用C++的桌面开发”。
按 [ROS官方Windows安装指南](https://docs.ros.org/en/lyrical/Get-Started/Installation/Windows-Install-Binary.html) 下载Lyrical二进制包。
自定义Action需要生成接口，因此即使串口节点是Python，也需要C++工具链。

将ROS解压到短路径，例如 `C:\dev\dcar_ros2\runtime\ros2-windows`，其中应直接有pixi.toml、preinstall_setup_windows.py、local_setup.bat。
按所下载发行包的官方步骤安装依赖；含Pixi的发行包可在CMD执行：

```bat
cd /d C:\dev\dcar_ros2\runtime\ros2-windows
pixi install
pixi run python preinstall_setup_windows.py
pixi add "pyserial==3.5" "pyyaml>=6,<7" pytest
```

不要混用系统Python或其他ROS发行版的二进制依赖；以下命令是CMD语法，不是PowerShell语法。

## 2. 下载并构建

```bat
cd /d "%USERPROFILE%\Desktop"
git clone --branch main https://github.com/yangzkee/DCar-BaseLine.git
```

打开 **x64 Native Tools Command Prompt for VS 2022**。执行pixi shell后，在它启动的shell继续：

```bat
cd /d C:\dev\dcar_ros2\runtime\ros2-windows
pixi shell
set PYTHONNOUSERSITE=1
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
chcp 65001
call C:\dev\dcar_ros2\runtime\ros2-windows\local_setup.bat
python -c "import rclpy, serial, yaml; from nav_msgs.msg import Odometry; print('dependencies OK')"
cd /d "%USERPROFILE%\Desktop\DCar-BaseLine\ros2"
colcon --log-base C:/dev/dcar_ros2/log build --merge-install --base-paths . --packages-up-to dcaron_bridge --build-base C:/dev/dcar_ros2/build --install-base C:/dev/dcar_ros2/install
call C:\dev\dcar_ros2\install\local_setup.bat
ros2 interface show dcaron_interfaces/action/Trajectory
```

两包均应成功：dcaron_interfaces、dcaron_bridge。更新源码后要重新构建，并重新加载同一个安装目录。
需要测试时，在此环境运行 `python -m pytest dcaron_bridge/test -q`；跳过项不等于ROS集成通过。

## 3. 每个新终端加载环境

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

两个终端使用相同ROS_DOMAIN_ID。不要把本教程的C盘安装路径与另一套ros2/install混用。

## 4. 接线并读取遥测

电脑USB-TTL直接接车端DFLink：TX→车RX、RX←车TX、GND共地，3.3V TTL，460800/8N1。
这里不经过STM32示例板，也不接它的115200 printf口。
在终端A：

```bat
python -m serial.tools.list_ports -v
ros2 launch dcaron_bridge bridge.launch.py port:=COM5
```

COM5换为实际端口，关闭串口助手及其他占用程序。在已加载环境的终端B：

```bat
ros2 topic echo /odom --once
ros2 topic hz /odom
```

默认只读遥测，不接受运动；VelPos不提供IMU角速度。

## 5. 运行当前队列示例

在终端A退出只读节点后开启Action：

```bat
ros2 launch dcaron_bridge bridge.launch.py port:=COM5 enable_motion_actions:=true
```

在终端B发送一次前进10cm（会让车运动）：

```bat
ros2 action send_goal /motion dcaron_interfaces/action/Motion "{command: 100, x: 0.1, speed: 0.06, profile: 18, timeout: 10.0}" --feedback
```

先离线检查正方形，再显式执行：

```bat
ros2 run dcaron_bridge trajectory_sequence dcaron_bridge/examples/square.yaml
ros2 run dcaron_bridge trajectory_sequence dcaron_bridge/examples/square.yaml --execute
```

操场用 `stadium.yaml`，要求全向底盘。每个命令运行一次；字段和结果见 [动作说明](motion.md)。
profile必须16..20，不能省略为0。车端队列v6能力不足、队列非空或遥测不新鲜时不会开始路线。

## 6. 排查

| 现象 | 检查 |
|---|---|
| 找不到rclpy / DLL | 先加载ROS和安装环境，检查where python，勿混用系统Python |
| 找不到Trajectory或新字段 | 重新构建两个包并加载正确install目录 |
| GBK解码错误 | 构建前设置UTF-8变量及65001代码页 |
| C++编译器缺失 | 安装VS C++工具链，从x64 Native Tools终端构建 |
| 串口拒绝访问 | 同一COM只能有一个持有进程 |
| 有串口但无odom | 检查COM、波特率、TX/RX/GND、供电和VelPos支持 |
| Goal被拒绝 | 查看桥接日志中的参数、profile、遥测或占用原因 |
| 队列查询超时 | 先确认车端v6支持，再排查链路；不盲目重发运动 |
| 更新后行为没变 | 重编译、新终端重新加载安装环境，重启桥接 |

取消或超时会尝试发停车；断线时不能保证到达。主机日志和车端成功回包不等于已外部测量轨迹精度。
