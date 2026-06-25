# RTT Server And BOOT0 Troubleshooting

本文记录一次 DAPLink + OpenOCD + SEGGER RTT 调试链路无法输出日志的排查过程。

## 问题现象

固件烧录显示成功，甚至可以看到 `Verified OK`，但启动 OpenOCD RTT server 后始终出现：

```text
rtt: No control block found
```

VOFA+ 可以连接 `127.0.0.1:9090`，但没有任何日志输出。

## 根因 1: 烧录流程和 RTT 常驻服务混用

最初把烧录和 RTT server 放在同一条 OpenOCD 命令里，例如同时包含：

```text
-f daplink_rtt.cfg
-c "tcl_port disabled"
-c "program ...FOC_DEMO2.elf"
-c reset
-c shutdown
```

这会带来几个问题：

- `daplink_rtt.cfg` 中已经执行 `init`，命令行后续再执行 `tcl_port disabled` 会报错：

```text
Error: The 'tcl_port' command must be used before 'init'.
```

- `rtt setup/start` 在 `program` 之前执行时，固件还没有运行，RTT 控制块还没有被初始化。
- `shutdown` 会让 OpenOCD 刚启动 RTT TCP server 就退出，VOFA+ 没有机会持续读取数据。

正确做法是把烧录和 RTT server 分成两个独立流程：

```powershell
D:\STM32_Env\OpenOCD-20231002-0.12.0\bin\openocd.exe -s D:\STM32_Env\OpenOCD-20231002-0.12.0\share\openocd\scripts -c "tcl_port disabled" -c "gdb_port disabled" -c "telnet_port disabled" -f "C:\Users\Vincent Lu\Desktop\gitcode\AerialRoboArm\daplink.cfg" -c 'program "C:/Users/Vincent Lu/Desktop/gitcode/AerialRoboArm/cmake-build-debug/FOC_DEMO2.elf" verify reset exit'
```

然后单独启动常驻 RTT server：

```powershell
D:\STM32_Env\OpenOCD-20231002-0.12.0\bin\openocd.exe -s D:\STM32_Env\OpenOCD-20231002-0.12.0\share\openocd\scripts -f "C:\Users\Vincent Lu\Desktop\gitcode\AerialRoboArm\daplink_rtt_server.cfg"
```

VOFA+ 使用 TCP Client 连接：

```text
Host: 127.0.0.1
Port: 9090
Protocol: RawData / text
```

## 根因 2: 多个 OpenOCD 进程抢同一个 DAPLink

CLion 或手动终端中可能残留 OpenOCD 进程。如果两个 OpenOCD 同时访问同一个 CMSIS-DAP 探针，会出现 CMSIS-DAP command mismatch、连接异常或读写混乱。

操作前可以清理残留进程：

```powershell
Stop-Process -Name openocd -Force
```

同一时间只保留一个 OpenOCD 连接 DAPLink。

## 根因 3: CPU 没有从 Flash 启动

真正决定 RTT 能否出现的关键是 CPU 是否正在运行用户固件。排查时 halt 后看到：

```text
pc = 0x1ffff3b6
```

`0x1FFFFxxx` 位于 STM32F103 system memory，也就是芯片内置 ROM bootloader 区，不是用户 Flash。用户程序应运行在：

```text
0x0800xxxx
```

因此当 PC 落在 `0x1FFFFxxx` 时，固件没有运行，`SEGGER_RTT_Write()` 不会执行，RTT 控制块也不会创建。

典型原因是 BOOT0 在复位采样时为高电平：

| BOOT0 | BOOT1 | 启动区域 |
| --- | --- | --- |
| 0 | X | User Flash |
| 1 | 0 | System memory bootloader |
| 1 | 1 | SRAM |

本板正常运行时，BOOT0 应稳定为低电平。不按 BOOT/SW8 时应接近 `0 V`，按住 BOOT/SW8 时才应接近 `3.3 V`。

## 关键判断方法

只连接目标并读取 PC：

```powershell
D:\STM32_Env\OpenOCD-20231002-0.12.0\bin\openocd.exe -s D:\STM32_Env\OpenOCD-20231002-0.12.0\share\openocd\scripts -c "tcl_port disabled" -c "gdb_port disabled" -c "telnet_port disabled" -f "C:\Users\Vincent Lu\Desktop\gitcode\AerialRoboArm\daplink.cfg" -c "init" -c "reset halt" -c "reg pc" -c "shutdown"
```

判断标准：

```text
pc = 0x0800xxxx  -> 正常，从 Flash 启动
pc = 0x1FFFFxxx  -> 进入 system memory bootloader，检查 BOOT0
pc = 0xFFFFFFFE  -> 向量表/程序状态异常，重新 verify 烧录并复位
```

## 遗留提醒

如果通过 OpenOCD `reset run` 后日志正常，但断电冷启动后又没有日志，优先检查 BOOT0 硬件：

- BOOT0 到 GND 的下拉电阻是否焊接正常。
- BOOT/SW8 是否为常开按键，是否封装脚位接错。
- BOOT0 网络是否被其他电路拉高。
- 不按 BOOT/SW8 时 BOOT0 是否稳定接近 `0 V`。

RTT server 问题最终常常不是 RTT 本身，而是固件根本没有从 Flash 运行。
