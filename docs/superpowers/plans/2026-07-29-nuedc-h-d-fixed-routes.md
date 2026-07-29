# 2026 电赛 H/D 题固定路线 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 STM32 DFCom v2 主线增加无光电、不中途停车的 H/D 胶囊赛道固定路线。

**Architecture:** 新路线模块以连续速度指令执行两条直线和两个顺时针半圆，使用相对里程投影与展开航向作为段结束条件。主程序只做初始化、模式选择和一次执行，所有调参项集中在头文件。

**Tech Stack:** C99、STM32F103C8T6、DFCom v2、Keil MDK、POSIX 主机测试。

## Global Constraints

- 分支必须基于 `origin/main` 的 `df07046454dd0efb97691c1c33117ac6aeb39885`。
- 不读取或依赖光电、灰度、循线传感器。
- H 直线 1.5 m、半径 0.50 m、默认速度 0.34 m/s。
- D 直线 1.5 m、半径 0.741 m、默认速度 0.25 m/s。
- 四个运动段之间不发送停车，只在完成或故障时停车。
- 默认模式为 H；修改一个宏即可切换 D。

---

### Task 1: 用仿真测试锁定路线合同

**Files:**
- Create: `tests/host/nuedc_stubs/DFCom.h`
- Create: `tests/host/nuedc_stubs/delay.h`
- Create: `tests/host/test_nuedc_2026_routes.c`
- Create: `tests/host/run_nuedc_2026_routes_tests.sh`
- Create: `DFCom_Example/USER/nuedc_2026_routes.h`
- Create: `DFCom_Example/USER/nuedc_2026_routes.c`

**Interfaces:**
- Consumes: `Cmd_Move_Vel`、`Cmd_Subscribe_Odom`、`g_odom`、`g_local_tick_ms`。
- Produces: `Nuedc2026_RunHRoute()`、`Nuedc2026_RunDRoute()`、`Nuedc2026_RunSelectedRoute()`。

- [ ] **Step 1: 编写失败测试**

测试仿真每 10 ms 积分一次速度与航向，断言 H/D 的五条速度命令、半径对应角速度、无中途停车、最终停车和总时间上限。

- [ ] **Step 2: 运行测试确认 RED**

Run: `sh tests/host/run_nuedc_2026_routes_tests.sh`

Expected: FAIL，因为路线头文件和源文件尚不存在。

- [ ] **Step 3: 实现最小路线模块**

实现稳定里程快照、直线前向投影、航向展开、ODOM 失联保护和统一错误停车；所有命令使用 SI 模式。

- [ ] **Step 4: 运行测试确认 GREEN**

Run: `sh tests/host/run_nuedc_2026_routes_tests.sh`

Expected: H/D、失联和非法模式测试全部 PASS。

### Task 2: 接入 STM32 主程序和 Keil 工程

**Files:**
- Modify: `DFCom_Example/USER/main.c`
- Modify: `DFCom_Example/USER/Template.uvprojx`
- Modify: `README.md`

**Interfaces:**
- Consumes: `Nuedc2026_RunSelectedRoute()` 和 `NUEDC_2026_START_DELAY_MS`。
- Produces: 上电握手后延时一次运行、结束后待机的固件入口。

- [ ] **Step 1: 增加静态失败检查**

检查主程序包含路线头文件、只调用一次所选路线、Keil 工程包含路线源文件，并且旧往返/原地旋转演示序列已移除。

- [ ] **Step 2: 运行静态检查确认 RED**

Expected: FAIL，因为主程序仍是 DFCom 回归演示。

- [ ] **Step 3: 替换主循环并更新工程与 README**

保留启动握手和诊断；启动运动会话后等待 3 秒，执行所选路线一次，打印状态并永久待机。

- [ ] **Step 4: 运行静态检查确认 GREEN**

Expected: 入口、工程文件和文档检查全部 PASS。

### Task 3: 完整回归和发布

**Files:**
- Verify: all modified files

- [ ] **Step 1: 运行新增路线测试**

Run: `sh tests/host/run_nuedc_2026_routes_tests.sh`

- [ ] **Step 2: 运行既有回归**

Run: `sh tests/host/run_move_slots_tests.sh && sh tests/arduino/run_arduino_tests.sh`

- [ ] **Step 3: 运行 C99 严格编译和 Keil XML 检查**

确认 `-Wall -Wextra -Werror` 通过，工程 XML 可解析并且源文件只登记一次。

- [ ] **Step 4: 检查差异并提交推送**

只提交 2026 路线、测试、入口、工程和文档文件，推送 `feature/2026-nuedc-h-d-fixed-routes`。
