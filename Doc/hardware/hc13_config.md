# HC13 Configuration And Project Parameters

本文档记录 HC13 模块的关键手册参数、当前项目固定配置，以及 PC 端 HC13 到 STM32 端 HC13 的测试方法。
HC13 在本项目中只作为透明无线串口使用，不参与 ELRS/CRSF 链路。

## Project Parameters

| Item | Project value | Note |
| --- | --- | --- |
| Link role | Transparent UART bridge | PC serial tool -> PC-side HC13 -> air link -> MCU-side HC13 -> STM32 `USART3`. |
| UART baud | `230400` | PC 串口助手、两个 HC13、STM32 `USART3` 必须一致。 |
| UART format | `8N1` | `8` data bits, no parity, `1` stop bit. |
| Wireless channel | `C043` / `433.4 MHz` | 两块 HC13 必须一致；这是手册/工具里常见默认信道。 |
| Wireless mode | `S7` | 普通模式最高速率；手册标称约 `19.0 KBytes/s`。 |
| RF power | `+20 dBm` | 当前模块查询值为 `OK+RP:+20dBm`。 |
| Max packet size | `< 1000 bytes` | HC13 单包不要超过 `1000 bytes`。当前控制帧远小于该限制。 |
| Duplex | Half-duplex | 不能同时空中收发；做遥控链路时应避免高频双向刷屏。 |

当前实测低延迟回显链路，发送 `V,35,300,90\n` 后 PC 端收到 `RX:V,35,300,90`，往返大约 `28..31 ms`。粗略估算单向延迟约 `14..16 ms`，实际控制延迟还会叠加串口助手、任务调度和解析周期。

## Electrical Parameters

| Item | Value | Note |
| --- | --- | --- |
| Supply voltage | `3.3 V ±0.3 V` | 不要接 `5 V` 到 HC13 `VCC`。 |
| Supply current | `>= 300 mA` recommended | 发射瞬间电流较高，电源不足会导致丢包、乱码或复位。 |
| Logic level | `3.3 V TTL UART` | 可直接接 STM32F103；USB-TTL 也应选择 `3.3 V` 电平。 |
| Antenna | Required for RF use | 两块模块都要接天线后再做远距离或高功率测试。 |

## KEY / AT Mode

| KEY state | Mode | UART baud used by AT command |
| --- | --- | --- |
| Floating or high | Transparent transmission | 使用已配置的串口波特率，例如 `230400`。 |
| Pulled low before power-on | AT configuration/query mode | 固定 `9600,N,1`，最稳妥的配置入口。 |
| Pulled low after power-on | AT configuration/query mode | 使用当前已配置的串口波特率。 |

退出 AT 模式时释放 `KEY`，等待至少 `30 ms` 后再发送透明传输数据。HC-T 串口助手里的“红键/AT 指令”就是把模块临时切到 AT 配置/查询模式；正常遥控数据测试时不要勾选 AT 指令。

## Factory Defaults

| Item | Default |
| --- | --- |
| UART baud | `9600` |
| UART format | `8N1` |
| Wireless channel | `C043` / `433.4 MHz` |
| Wireless mode | `S3` |

配置写入后会保存，断电重启仍然生效。

## Wiring For Configuration

单独配置每块 HC13 时，使用 USB-TTL 转接板：

| USB-TTL | HC13 |
| --- | --- |
| `3V3` | `VCC` |
| `GND` | `GND` |
| `TXD` | `RXD` |
| `RXD` | `TXD` |
| `GND` | `KEY` while configuring |

推荐先断电，`KEY` 接 `GND`，再上电进入固定 `9600,N,1` 的 AT 模式。

## MCU Wiring

配置完成后，MCU 端 HC13 按透明串口连接：

| HC13 | STM32F103 |
| --- | --- |
| `VCC` | `3V3` |
| `GND` | `GND` |
| `TXD` | `PB11 / USART3_RX` |
| `RXD` | `PB10 / USART3_TX` |
| `KEY` | Floating/high for transparent mode; `PB0` is reserved for future control. |

## AT Commands

常用配置/查询命令：

```text
AT          -> OK
AT+V        -> firmware version
AT+RX       -> query all parameters
AT+B230400  -> set UART baud to 230400
AT+C043     -> set wireless channel C043 / 433.4 MHz
AT+S7       -> set wireless mode S7
AT+RP       -> query RF power
AT+DEFAULT  -> restore factory defaults
```

两块 HC13 必须至少保持 `AT+C043` 和 `AT+S7` 一致；如果串口侧波特率不同，PC/MCU 侧也必须分别按各自模块的波特率打开，否则会出现乱码。

## Configure One Module

From the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File tools\hc13_config.ps1 -Port COM8
```

Repeat for the second module. The script sends:

```text
AT
AT+S7
AT+C043
AT+B230400
```

After it succeeds, release `KEY` high or disconnect it from `GND`, then wait at
least `30 ms` before using transparent mode.

## Query Only

```powershell
powershell -ExecutionPolicy Bypass -File tools\hc13_config.ps1 -Port COM8 -QueryOnly
```

The script probes `9600`, `115200`, and `230400` baud by default. This helps if
the module has already been configured before.

## PC To MCU Test

1. PC-side HC13: connect to USB-TTL, open `COM5` at `230400 8N1`.
2. MCU-side HC13: connect to STM32 `USART3` on `PB10/PB11`, keep `KEY` released.
3. Serial tool: disable AT mode, disable hex send, enable send-newline.
4. Send:

```text
V,35,300,90
```

The control firmware returns a rate-limited application acknowledgement after
the complete command has passed the MCU parser:

```text
ACK,1,35,90
```

The four fields are `ACK`, the cumulative parsed-frame count, accepted target
angle, and accepted confidence. Receiving this reply proves the wireless frame
reached the MCU parser. It does not prove that the servo has reached the target.
MCU control state does not need AUTO mode for the acknowledgement. AUTO/ELRS/
servo-online checks only affect whether parsed vision data may drive the arm.

From a CMD window in the repository root, the preferred test is:

```bat
tools\hc13_send.bat --angle 0
```

The tool sends through `COM5` at `230400 8N1` by default and prints `TX`, `RX`
and a final `PASS` or `FAIL`. Use `--port COMx` if the PC-side module has a
different port. Keep ELRS in MANUAL while testing communication only.

## Firmware Integration

| Firmware item | Current behavior |
| --- | --- |
| STM32 peripheral | `USART3` on `PB10/PB11`, `230400 8N1`. |
| RX path | `HAL_UART_RxCpltCallback()` forwards each byte to `DrvHC13_PushRxByte()`. |
| Application ACK | Parsed commands return `ACK,<count>,<angle>,<confidence>\r\n`, limited to 5 Hz. |
| Parser | `DrvHC13_PollVision()` accepts `V,<angle_deg>,<speed>,<confidence>` and `VISION,...` style frames. |
| Data TTL | `HC13_VISION_TTL_MS = 300 ms`; stale vision samples are ignored. |

## Vision AUTO control contract

The current PC vision runtime sends a calibrated absolute HX8 `g` command,
not the raw camera optical-axis offset:

```text
V,<integer_servo_g_deg>,0,<confidence_percent>\r\n
```

The speed field is in `deg/s`. The daily PC runtime deliberately sends `0`,
which selects the MCU's conservative `80 deg/s` AUTO default. If a future PC
runtime sends a non-zero speed, the MCU constrains it to `20..200 deg/s` before
passing it to the HX8 motion command.

`TaskArbiter` accepts the command only when all normal RC/servo/E-stop safety
conditions pass, RC requests AUTO, the sample is newer than the 200 ms arbiter
window, and confidence is at least 60%. Vision-stale AUTO keeps the previous
target. The former USART3 ISR line echo is disabled by default in control
builds so that it does not double radio traffic or busy-wait in an interrupt.

## Troubleshooting

- Garbled text usually means baud mismatch, wrong encoding display, wrong AT/transparent mode, or unstable power.
- `AT` commands returning no response usually means `KEY` is not low, the serial baud is wrong, or TX/RX are not crossed.
- Transparent data not arriving usually means the two modules use different channel/mode, one module is still in AT mode, or the MCU `USART3` wiring is reversed.
- High apparent delay in PC serial tools can come from timer send period, timestamp display, or the tool echoing TX/RX. Use one manual send and compare TX/RX timestamps for a rough RTT only.
