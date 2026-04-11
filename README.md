# Pico2 抓取/投递控制系统

## 项目简介
本项目基于 Raspberry Pi Pico 2，使用三路红外传感器和两路舵机实现抓取与投递流程控制，并通过一个飞控触发引脚输出升空/起飞脉冲。

程序采用状态机驱动，目标是保证流程可控、动作有序、错误可恢复。

## 功能特性
- 三路红外输入检测（含防抖）
- 双舵机 50Hz PWM 控制（角度映射到脉宽）
- 舵机安全保护机制：
  - 重复角度命令去重
  - 最小命令间隔限制
  - 过频触发冷却保护
- 串口命令与状态强绑定，避免误触发
- 错误状态自动恢复并回到空闲

## 硬件引脚定义
默认引脚定义如下（可在代码宏中修改）：

- SERVO1_PIN: GPIO2（S1 舵机）
- SERVO2_PIN: GPIO3（S2 舵机）
- IR1_PIN: GPIO26（红外 1）
- IR2_PIN: GPIO27（红外 2）
- IR3_PIN: GPIO28（红外 3）
- FLY_TRIGGER_PIN: GPIO15（飞控触发）

约定：
- 红外被遮挡输出低电平
- 飞控触发采用高电平脉冲（约 1s）

## 项目逻辑讲解

### 1. 整体控制模式
主循环采用“收命令 + 跑状态机”的轮询结构：

1. 非阻塞读取串口命令
2. 执行一次状态机推进
3. 短暂延时后继续循环

这样可以避免流程被单一步骤长期阻塞，便于调试和扩展。

### 2. 状态机流程
状态顺序如下：

1. STATE_IDLE
2. STATE_GRAB_CHECK
3. STATE_GRAB_ACTUATE
4. STATE_WAIT_ASCEND
5. STATE_GRAB_VERIFY
6. STATE_HOLDING
7. STATE_RELEASE_ACTUATE
8. STATE_WAIT_TAKEOFF
9. STATE_RELEASE_VERIFY
10. STATE_ERROR

关键逻辑说明：

- IDLE：等待 `grab`
- GRAB_CHECK：读取 IR1/IR2/IR3 组合，判断是否允许抓取
- GRAB_ACTUATE：按红外组合决定 S1/S2 动作先后，完成后反馈 `grab_finished`
- WAIT_ASCEND：等待 `ascend`，收到后向飞控输出脉冲并进入抓取验证
- GRAB_VERIFY：以 IR3 作为抓取成功关键位，反馈 `grab_success` 或 `grab_failed`
- HOLDING：持货待命，等待 `release`
- RELEASE_ACTUATE：执行 S1->S2 复位动作，反馈 `release_finished`
- WAIT_TAKEOFF：等待 `takeoff`，收到后向飞控输出脉冲
- RELEASE_VERIFY：要求三路红外全部清空，反馈 `release_success` 或 `release_failed`
- ERROR：执行安全恢复（舵机回零、触发脚拉低），然后回到 IDLE

### 3. 红外判定与防抖
- 每路红外进行多次快速采样，按多数票决定是否有障碍
- 通过统一函数读取三路状态并打印调试信息，便于串口观察

### 4. 舵机控制与保护
舵机控制使用 50Hz PWM（20ms 周期），角度线性映射到约 0.5ms~2.5ms 脉宽。

为降低堵转和抖动风险，加入以下保护：
- 与当前目标角度相同的命令直接跳过
- 连续命令间隔过短时阻塞动作
- 在统计窗口内动作过频会进入冷却期

## 串口命令协议
命令以换行结束，支持：

- `grab`
- `ascend`
- `release`
- `takeoff`

命令只有在对应状态下才会被接受，不匹配时会输出 rejected 日志。

推荐时序：

1. `grab`
2. 等待 `grab_finished`
3. `ascend`
4. 等待 `grab_success`
5. `release`
6. 等待 `release_finished`
7. `takeoff`
8. 等待 `release_success`

## 环境搭建（Windows）

### 1. 软件与工具
建议通过 Raspberry Pi Pico VS Code 扩展安装与管理工具链。

当前工程配置对应：
- Pico SDK: 2.2.0
- Toolchain: 14_2_Rel1
- Ninja: 1.12.1
- picotool: 2.2.0-a4
- CMake: 3.13+

### 2. VS Code 准备
安装扩展：
- Raspberry Pi Pico
- C/C++

然后在 VS Code 打开本工程根目录。

### 3. SDK 环境
首次使用 Pico 扩展时完成 SDK 与工具链初始化。通常会在用户目录下生成 `.pico-sdk` 相关目录，项目 CMake 会自动引用。

### 4. 编译
本工程已配置 VS Code 任务，直接运行：
- Compile Project

或者命令行方式：

```powershell
cmake -B build -G Ninja
cmake --build build
```

### 5. 下载/烧录
可按硬件连接方式选择：
- Run Project（使用 picotool）
- Flash（使用 openocd）

如遇芯片状态异常，可尝试：
- Rescue Reset
- RISC-V Reset (RP2350)

## 目录说明
- `Pico2.c`：主程序（状态机、命令解析、红外与舵机控制）
- `CMakeLists.txt`：构建配置
- `pico_sdk_import.cmake`：Pico SDK 导入脚本
- `build/`：构建输出目录

## 调试建议
- 上电后先观察初始化日志
- 重点关注流程反馈：
  - `grab_finished` / `grab_success` / `grab_failed`
  - `release_finished` / `release_success` / `release_failed`
- 若频繁进入 ERROR：
  - 检查红外电平逻辑与接线是否一致
  - 检查舵机供电是否稳定且共地
  - 检查命令发送顺序是否符合状态机
