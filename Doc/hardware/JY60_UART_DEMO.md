# 原 ELRS 接口上的 JY60 独立串口 demo

## 接线与运行范围

ELRS 接收机已拔下。根据项目引脚表与用户提供的 PCB 插座图：

| 插座脚号 | MCU / PCB 功能 | JY60 |
| --- | --- | --- |
| 1 | GND | GND |
| 2 | VCC +5V | VCC（模块允许 3.3～5 V） |
| 3 | ELRS_RX / MCU PB7 / USART1_RX | TX |
| 4 | ELRS_TX / MCU PB6 / USART1_TX | RX（本 demo 不发配置，可以不接） |

配置为 9600 baud / 8N1，不启用 ELRS 解析。测试程序只初始化 GPIO、USART1、DMA 时钟和看门狗，使用前台轮询收字节及纯 C 帧解析器。DMA 接收和 RTOS 控制任务不启动；不初始化 TIM2 PWM、HX8 UART、HC13 或回零程序，也不向 JY60 发校准/休眠指令。

## 一键烧录及 RTT

在 MCU 项目根目录 PowerShell 执行：

```powershell
.\tools\flash_jy60_demo.bat
```

脚本使用单独目录 `cmake-build-jy60-demo`，设置 `JY60_UART_DEMO=ON`，编译、烧录、校验并启动 RTT。正常控制使用的 `cmake-build-debug` 不切换到此配置。两种固件交替烧录会替换板上程序，但源码和构建目录均保留。

VOFA：TCP 客户端，`127.0.0.1:9090`，RawData 文本，UTF-8。不需要发送 `h`；`d` 切换简洁/详细，`?` 显示帮助，换行选 `\n`。当前传感器 demo 不处理正常固件的 `g` / `p` / `s` 等控制台命令。

## 输出含义

默认每秒只输出两行英文，角度保留两位小数：

```text
[STATUS] ONLINE | Sensor=50ms Display=1000ms | ChecksumErrors=0 UartErrors=0
[ANGLE deg] Roll(X)=-0.85 | Pitch(Y)=+2.02 | Yaw(Z)=-5.21
```

Roll 是绕模块 X 轴转动，Pitch 绕 Y 轴，Yaw 绕 Z 轴；具体方向取决于模块安装方式，并非自动对应 HX8 关节角。Yaw 会漂移。断线/数据超过 200 ms 时不把缓存角度当成实时角度。

`Sensor=50ms` 是最近两条有效角度包的到达间隔（20 Hz），`Display=1000ms` 是日志打印间隔（1 Hz）；后者仅减少刷屏，接收解析仍持续运行。50 ms 不代表整体测量/传输/滤波延迟，也不代表内部 IMU 芯片的物理采样极限。该版模块资料规定固定 9600 baud、三包共 33 字节/组；8N1 下仅传输就需约 `33*10/9600=34.4ms`。不能只修改 MCU 波特率来获得更快输出。

发送 `d` 后追加下列技术信息，再次发送 `d` 返回简洁模式；重启默认简洁模式。

```text
[FW] JY60_UART_DEMO ... baud=9600 8N1; HX8/PWM/HC13 disabled
[JY60] ONLINE rx=660 good=60 acc=20 gyro=20 angle=20 checksum_err=0 uart_err=0 period=50 ms
[JY60] angle XYZ/RPY: ... deg tail_raw=... age=... ms
[JY60] acc XYZ: ... g tail_raw=... age=... ms
[JY60] gyro XYZ: ... deg/s tail_raw=... age=... ms
```

- `ONLINE`：收到过有效角度包且其年龄小于 200 ms；并非仅仅串口接收到字节。
- `rx`：收到的字节总数；`good`：校验通过的包总数。
- `acc` / `gyro` / `angle`：三类包各自的累计数量，正常应各约增加 20 包/秒。
- `checksum_err`：校验失败次数；`uart_err`：UART 硬件/接收错误次数。
- `period`：最近两条角度包的间隔，正常约 50 ms。
- `age`：该类型最近一次有效数据距今的时间；不应把缓存值当新数据。
- `NO_BYTES`：完全没有收到字节，检查供电、共地和 TX→PB7。
- `NO_VALID_RECENT_ANGLE`：有字节但无有效新角度包，检查波特率、帧内容、校验和时效；同时输出最初 16 字节供诊断。
- `tail_raw`：每包第 8～9 字节的原始有符号值。实测三类包尾字段不同，资料又存在温度公式差异，不将它统一宣称为已验证温度，也不用于保护。

`angle` 输出次序为 X/Roll、Y/Pitch、Z/Yaw。加速度包含重力。Yaw 积分漂移、欧拉角奇异性、标定和硬件修订限制见 [JY60_IMU.md](./JY60_IMU.md)。

## 已完成的验证（2026-10-19）

- 原 ELRS 接口读取到 `0x51/0x52/0x53` 三类有效包。
- 初始 1 秒采集为 660 字节、60 个有效包，三类各 20 个，角度周期 50 ms。
- 首次连续采样中有效包超过 1000，校验错误与 UART 错误均为 0。
- 示例姿态约 Roll −0.9°、Pitch +1.8°、Yaw −0.2°，加速度 Z 约 +1g。仅是当时放置状态，不是角度精度标定结果。
- `tools/test_jy60.ps1` 验证分段包、连续包、校验、噪声重同步、负数、角速度计算溢出和时间戳回绕。
- 普通固件与 demo 均可编译。demo 链接结果不包含 `BSP_PWM_Init`、`TaskMotion_Update`、`App_Control_Init` 等执行器启动符号。

## 恢复正常机械臂固件

```powershell
.\tools\flash_and_rtt.bat
```

该命令烧录正常构建目录。正常完整固件的 USART1 仍用于 ELRS，不能同时把这个接口上的 JY60 数据作为有效 ELRS 输入；恢复前应按所需控制方案重新安排硬件。
