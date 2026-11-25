# HC13 无动作排障记录（2026-10-31）

本记录只使用 V2 只读查询，未发送运动、夹爪、模式或停止命令；未修改或烧录单片机代码。

## 本机实测

- Windows 枚举到 `COM4`，设备描述 `USB-SERIAL CH340`，USB VID:PID `1A86:7523`。这只能证明电脑看到一个 USB 串口，不能仅凭名称确认它接的是哪一块 HC13。
- 沙箱内首次尝试打开时收到 Windows `PermissionError(13)`；在获准的串口访问环境中，`COM4` 以 **230400、8N1、无流控**成功打开。
- 被动监听 2 秒没有收到字节。之后 `GET,<id>,INFO`、`GET,<id>,CAL`、`GET,<id>,STATE`、`PING,<id>` 四条只读命令分别完整写出 19、18、20、15 字节。每条之后都收到相同的 7 字节 `45 52 52 4F 52 0D 0A`，即 `ERROR\r\n`；没有有效 V2 帧。
- 带 UTC 时间、单调时间、完整 TX/RX 十六进制和请求号的原始记录：`../diagnostics/hc13_probe_20261031_164431.jsonl`。这次记录总接收 28 字节、有效 V2 帧 0 条。
- 用户再次运行只读探测，`../diagnostics/hc13_probe_20261031_170738.jsonl` 仍记录四次完整写出和四次相同的 `ERROR\r\n`，有效 V2 帧 0 条。
- 随后对 COM4 仅发送一次只读 `AT`（两个字节 `41 54`），收到 `OK\r\n`（`4F 4B 0D 0A`）。当时曾据此推测模块一直处于 AT 模式；后续对照测试推翻了这个持续状态判断。
- 用户提供的 XCOM 截图：同一个 `COM4`，230400、8N1、无流控，DTR/RTS 均未勾选，连续收到 `ACC,...`。pyserial 默认在打开串口时将 DTR/RTS 置高，即使流控关闭也如此。原上位机和探测脚本没有显式关闭这两条线，因此两套程序打开同一 COM 口时实际控制线状态不同。
- 修改上位机和探测脚本：在端口尚未打开时先设置 `DTR=False`、`RTS=False`，然后打开。新探测 `../diagnostics/hc13_probe_20261031_171444.jsonl` 收到 21,393 字节、759 个有效 V2 帧，其中 ACC 755、INFO/CAL/STATE/PONG 各 1。随后把读操作从固定等待 4096 字节改为优先读取当前可用字节，无数据时只等 1 字节；再次探测 `../diagnostics/hc13_probe_20261031_171832.jsonl` 用 3.5 秒收到 2,233 字节、73 个有效帧，四种只读查询均得到与请求号匹配的回复。
- 实际 GUI 在 COM4 上运行 2.5 秒：`link_online=True`，INFO、CAL、STATE 均已解析，图表收到 45 个 ACC 样本；仅自动发送只读握手/心跳查询，运动命令 0 条。关闭窗口后工作线程已退出。此前相同 GUI 在默认 DTR/RTS 下只收到裸 `ERROR`，这构成同一设备上的软件配置对照。

`ERROR\r\n` 不符合当前固件 `User/mod/Src/mod_hc13_host.c` 的返回格式。该模块查询会返回 `INFO/CAL/STATE/PONG`，协议拒绝是 `ERR,<id>,<code>`。**已确认的软件根因是上位机默认的 DTR/RTS 状态与能正常通信的 XCOM 配置不同**；关闭 DTR/RTS 后，同一 COM4 能收到 MCU V2 回包。DTR/RTS 具体通过哪根线影响模块工作模式尚未从实物接线测量，不能仅凭 `AT→OK` 判定 KEY 的实际电平。

## 静态链路核对

本机桌面快捷方式的目标是 Python 3.11 `pythonw.exe`，参数指向当前 `desktop_app/launch.pyw`；它导入 `main.py`，再创建 `arm_console.window.MainWindow`。因此本次检查针对实际启动入口，而非旧的 `103_a` 项目。

| 阶段 | 代码或配置证据 | 本轮结论 |
| --- | --- | --- |
| UI→本地 COM | `main.py`→`window.py`→`v1_state.py`→`serial_service.py`；单个 `SerialWorker` 同时读写一个 pyserial 实例，8N1、无流控、读超时 100 ms、写超时 300 ms | 只读探测的字节已实际写到 COM4；GUI 动作按钮只有通过握手、模式、标定和反馈门控才入队。新增日志区分入队与实际写出。 |
| 本地模块→空中链路 | 项目目标：两块 HC13 各 230400 8N1，C043、S7；上位机在端口打开前关闭 DTR/RTS | COM4 被动收到 MCU 风格 ACC，且只读请求得到匹配的 V2 回复，证明当前配置下端到端双向协议链路可用；信道、模式、供电值仍未仪表实测。 |
| 空中模块→MCU | `Core/Src/usart.c`：USART3 PB10 TX/PB11 RX，230400 8N1；`bsp_uart.c`：USART3 单字节接收中断、256 字节环形缓冲、错误后重启接收；主循环读取并送入 `ModHc13Host_Receive` | 该路径只在 `HC13_HOST_LINK=ON` 的专用构建有效；机载板当前镜像未知。不能从仓库现有 `.elf` 推断它已烧录。 |
| MCU 解析→ACK | `mod_hc13_host.c`：ASCII 逗号、LF/CRLF、最多 160 字节，无 CRC 字段，`req_id` 去重；GET/PING 可只读查询，动作回 `ACK`/`ERR`/`MOTION`/`DONE` | INFO/CAL/STATE/PONG 的请求号逐一匹配；只读查询已经到达 MCU V2 解析器并收到回复。动作 ACK 和执行器动作均未试验。 |
| MCU→执行器 | 固件配置将竖直位设为 0°、张开 135°、闭合 90°；后折位置及开闭爪等待时间对外为 `NA` | 即使后续握手成功，HOME/NORMAL/PREP、FOLDED、PICK 仍受标定门控，不能绕过。SERVO1 无位置/夹持反馈。 |

固件 `CMakeLists.txt` 的 `HC13_HOST_LINK` **默认 OFF**，专用构建脚本显式打开。源码中的传感器 USART1 PB6/PB7 为 115200 JY61P；USART2 PA2/PA3 为 115200 HX8；USART3 PB10/PB11 为 230400 HC13。HC13 专用构建内部 JY61P 约 200 Hz，非 VISION_PICK 模式通过无线约 20 Hz 回传 ACC。RTT 日志使用不阻塞模式；现有代码未见“ACC 必然阻塞控制接收”的确定证据，但实物中断/缓冲溢出仍需测量。

## 复测次序

1. 关闭 XCOM、VOFA 及其他占用 COM4 的程序。打开当前上位机，选择 COM4、230400、关闭模拟并连接；程序会在打开串口前关闭 DTR/RTS。
2. 确认“V2 在线”、ACC 实时值，以及日志中的实际 GET 写出和匹配 INFO/CAL/STATE 回复。此步骤不需要执行机构动作。
3. 如仍不在线，关闭上位机，运行 `python tools/probe_hc13_readonly.py --port COM4 --baud 230400`，把新生成的 JSONL 原始记录交给开发者。也可核对两块模块的 KEY、C043/S7、3.3 V 供电、共地、天线及机载 `TXD→PB11/USART3_RX`、`RXD←PB10/USART3_TX`。这些实物参数本轮未测量。
4. 当前 `CAL` 实测为 `CAL,...,0,0,-1,90000,NA,135000,90000,0,180000,NA,NA,4000`，后折位置和开闭爪等待时间仍是 `NA`。完成真实标定后才由操作者主动测试机构动作；保留标定门控。固件未修改或烧录。

## 软件验证范围

上位机单元测试和模拟串口覆盖编码、同实例写入、DTR/RTS 开口前状态、V2 回复解析与拒绝显示。将当前上位机编码器生成的 `GET,401,INFO`、`GET,402,CAL`、`GET,403,STATE`、`PING,404` 输入当前固件 C 模块的本机模拟端，四种回复均被当前上位机解析器识别；固件 C 测试也通过。固件模拟端的 35 行事件样本全部被上位机解析，0 个无效帧。COM4 上的只读 INFO/CAL/STATE/PONG 与 ACC 流进一步证实当前真实通信链路可用；**尚未验证动作命令被接受或执行器实物动作**。ARM 交叉编译器在本机触发 Windows DLL 重定位错误，且固件未更改，不能据此宣称新镜像已重新编译或实物动作通过。
