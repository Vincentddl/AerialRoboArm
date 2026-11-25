# HC13单向应用链路延迟实测

## 接线与测量边界

两个HC13分别通过各自USB转串口接入同一台电脑。模块保持既有透明传输配置：230400、8N1，信道与模式一致。USB串口TX接模块RXD，RX接TXD，共地。不要把两路推挽TX输出相连；本方法不要求连接STM32，也不需要刷固件。

软件时间边界：发送端Serial.write调用前，到接收端Serial.read返回包含完整匹配测试帧的时刻。两者使用同一主机perf_counter_ns。包括USB转串口、驱动/系统调度、UART和HC13无线链路；不是独立RF延迟，不含MCU任务解析、舵机或夹爪响应。不能把PC与STM32未同步的时钟直接相减。

测试只发送14字节L开头的探测帧，含本次会话号、序号、CRC16和换行。不发送AT配置、V/VISION控制命令，不打开执行器串口。包长与一个14字节视觉消息示例相同，内容不同，不能声称覆盖所有实际控制负载。

## 运行

先只列设备（不打开端口）：

```powershell
& 'D:\app\miniconda\envs\python312\python.exe' tools/measure_hc13_latency.py --list
```

确认两个串口分别连接HC13后指定端口，例如：

```powershell
& 'D:\app\miniconda\envs\python312\python.exe' tools/measure_hc13_latency.py --tx-port COM5 --rx-port COM8 --count 200 --interval 0.05
```

默认约20Hz，200包，发送结束后继续接收1秒。DTR/RTS在端口打开前保持不激活，避免某些HC-USB-T夹具进入AT模式。不自动修改模块参数。记录实际发送时间戳；Windows调度可能使发送间隔与请求值不同。

## 输出与解释

在outputs/hc13_latency/每轮独立目录保存summary.json和packets.csv。每个序号只采用第一次完整有效接收，拒绝CRC错误、外来会话、未发送序号及时间倒置。不将超时样本写成0ms，不把重复包重复统计。

统计中位数、P95、最小/最大、有效样本与未回收包数量；不足30个有效样本会提示。未回收包表示本轮采集窗口内未收到，不能直接当作纯RF丢包率。若未收到任何有效包，统计为null。Ctrl+C保留已有结果，并标记不完整。

测试应记录距离、遮挡、模块模式/信道、供电、天线、USB转接模块和背景流量。200包低负载测试不能代表所有运行条件。

## 当前接收端仍接STM32时

现有固件只对被解析的V/VISION帧回复ACK，且有200ms最小回复间隔。上述L探测帧不会得到该ACK。单个PC串口不能直接完成本工具的单向测量。现有ACK可以另做应用往返响应测量，但不能将往返时间除以二写成单向实测。请勿为获得ACK将探测帧改成可能驱动机械臂的控制消息。

## 独立纯通信往返测试固件

`tools/hc13_rtt_firmware/` 是独立构建的最小固件，不修改主工程入口或构建选项。它只初始化72MHz时钟、PB10/PB11 USART3和调试RTT，不启动USART1/2、PWM、RTOS、主臂或夹爪。收到合法14字节L探测帧后立即回复带相同会话/序号、独立CRC的14字节R帧，忽略所有运动消息。PC用 `tools/measure_hc13_rtt.py` 计时；不会把本地L回显误认成MCU的R回复。

需先备份板上完整Flash；刷写该独立固件后可按如下方式测量：

```powershell
python tools/measure_hc13_rtt.py --port COM6 --count 200 --warmup 5 --interval 0.05 --output-dir outputs/某次独立目录
```

记录同一主机发送前至收到完整匹配回复的RTT。此模式绕开正式控制固件20ms任务和200ms ACK限频，不能冒充正式控制应用响应，也不能据此直接测出单向延迟。测试结束须恢复原Flash并读回核验。2026-10-23实际测量与恢复证据保存在 `outputs/hc13_rtt_20261023_145603/`。
