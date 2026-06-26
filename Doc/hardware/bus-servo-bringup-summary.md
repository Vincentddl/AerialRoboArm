# 总线舵机与 RTT 联调问题总结

本文记录当前 PCB + STM32F103 + DAPLink + ELRS + HX8-U26H-M + PTK 7462W 联调过程中出现的问题、判断依据和最终处理方式。

## 1. RTT 服务器无数据

现象：

- 固件烧录显示 `Verified OK`。
- VOFA+ 连接 `127.0.0.1:9090` 没有日志。
- OpenOCD 报 `rtt: No control block found`。

原因分三层：

1. 烧录流程和 RTT 常驻服务器流程混用。
   - `program`、`rtt setup/start`、`shutdown` 放在同一条命令里，会导致 RTT 控制块还未建立就开始搜索，或者服务器刚启动就退出。
   - `tcl_port disabled` 必须在 OpenOCD `init` 之前执行，不能在 cfg 已经 `init` 后再追加。

2. 多个 OpenOCD 抢同一个 DAPLink。
   - CLion 集成 OpenOCD 和手动 OpenOCD 同时运行，会导致 CMSIS-DAP command mismatch、端口占用或探针状态异常。

3. BOOT0 不稳定导致 CPU 进入 STM32 ROM bootloader。
   - 读 PC 时如果是 `0x1FFFFxxx`，说明程序没有从 Flash 运行。
   - 正常运行用户固件时，PC 应在 `0x0800xxxx`。

解决：

- 烧录和 RTT 分离。
- 操作前清理旧 OpenOCD：

```powershell
Stop-Process -Name openocd -Force
```

- RTT 使用 attach-only 配置，不在启动 RTT 时 `reset run`，避免 BOOT0 浮空时复位重新进 bootloader。
- 新增 `daplink_rtt_attach.cfg` 和 `tools/rtt_attach.bat`。

推荐流程：

```powershell
.\tools\flash_and_rtt.bat
```

如果固件已经在跑，只重新打开 RTT：

```powershell
.\tools\rtt_attach.bat
```

遗留硬件提醒：

- BOOT0 默认必须稳定为低电平。
- 不焊 SW8 时，理论上 BOOT0 通过 R37 10k 下拉为 0。
- 如果 BOOT0 上电仍有约 1.8V，优先检查 R37、SW8 焊接、BOOT0 走线和是否有 3V3 异常漏电路径。

## 2. HX8 总线舵机最初无回包

现象：

```text
tx > 0
rx = 0
INIT BOOT
```

含义：

- MCU 已经通过 USART2 发 FSUS 命令。
- USART2 没有收到任何回包。
- 控制状态机停在启动/舵机在线探测阶段。

排查项：

- HX8 是否有独立 `9.0..12.6V` 供电。
- HX8 电源 GND、UC-01 GND、STM32 GND 是否共地。
- UC-01 电平开关是否为 `3.3V`。
- PA2/PA3 与 UC-01 的 T/R 是否接反。
- HX8 ID 是否为 `0`。
- HX8 波特率是否为 `115200`。

处理：

- 对比厂家 `stm32f103-sdk`，确认协议帧头、命令码、默认 ID、默认波特率与当前实现一致。
- 将启动在线探测从 `ServoMonitor` 改为厂家示例同款 `Ping`，Ping 成功后再进入运行阶段。

## 3. HX8 通信偶发超时

现象：

```text
MANUAL MANUAL_RC
wr=0 rd=0
pos 跟随 tgt
```

偶尔变成：

```text
rd=5
ERROR NO_SERVO
```

错误码说明：

```text
wr = last_write_result
rd = last_read_result

0 = OK
1 = BAD_HEADER
2 = BAD_ID
3 = BAD_SIZE
4 = BAD_CHECKSUM
5 = TIMEOUT
6 = BAD_FRAME
```

判断：

- `rd=5` 是回包超时，不是协议解析错误。
- 日志中 `rd=5` 后又恢复 `rd=0`，说明不是串口永久卡死，而是 HX8/UC-01 链路短时沉默。

软件处理：

- 增加 `wr/rd` 到 RTT 快照，便于区分超时、ID 错、校验错、帧头错。
- 将 `ServoMonitor` 回读从控制环频率降为 10Hz。
- 两次有效回读之间使用上一帧反馈缓存。
- 将在线宽限扩展为 3000ms，短暂 `rd=5` 不立刻踢出 `MANUAL`。
- timeout 后从实际结束时间重新计算下一次轮询，避免连续压总线。

当前关键配置：

```c
#define TASK_MOTION_SKIP_PING_FOR_BENCH (0)
#define TASK_MOTION_DEFAULT_VELOCITY    (360.0f)
#define TASK_MOTION_DEFAULT_T_ACC_MS    (80U)
#define TASK_MOTION_DEFAULT_T_DEC_MS    (80U)
#define TASK_MOTION_ONLINE_GRACE_MS     (3000U)
#define TASK_MOTION_FEEDBACK_PERIOD_MS  (100U)
```

## 4. ELRS 控制链路

验证结果：

- `rc=1`，`erx` 持续增长，说明 ELRS 接收机和 USART1/CRSF 解析正常。
- raw dump 可看到 CH1/CH4/SA/SB/SD 等通道变化。
- 手动模式下：

```text
MANUAL MANUAL_RC
wr=0 rd=0
pos 跟随 tgt
```

说明 ELRS -> 语义映射 -> HX8 主臂控制已打通。

当前 CH1 主臂控制参数：

```c
#define MOD_RC_CH1_STEP_INTERVAL_MS (50U)
#define MOD_RC_CH1_STEP_DEG         (10)
```

含义：

- CH1 推出死区后，目标角度约每 50ms 累加 10度。
- 当前比最初配置稳定，比保守测试配置更快。

## 5. PTK 7462W 小舵机 SERVO1 测试

当前软件链路：

```text
ELRS CH4
-> gripper_angle
-> TaskArbiter
-> TaskManipulator
-> ModActuator_SetGripperAngle
-> DrvServo_SetAngle
-> BSP_PWM_SetServoPulse(BSP_SERVO_1)
-> TIM2_CH1 / PA0
-> SERVO1
```

定时器配置：

```text
TIM2 prescaler = 72-1 => 1 tick = 1us
TIM2 period    = 3000-1 => 333Hz
SERVO1         = PA0 / TIM2_CH1
SERVO2         = PA1 / TIM2_CH2
脉宽范围       = 500..2500us
中位           = 1520us
```

这与 PTK 7462W 参数匹配：

```text
刷新率 333Hz
脉宽 500..2500us
中位 1520us
工作电压 4.8..7.4V
```

当前现象：

- RTT 中 `grip` 有变化，说明 ELRS 和软件上游已通。
- SERVO1 实物无响应，问题更可能在 PA0 PWM 输出之后。

检查顺序：

1. 使用 RTT 强制命令排除遥控器因素：

```text
p 0 90
p 0 30
p 0 150
p 0 90
P
```

2. 检查接线：

```text
SERVO1 红线       -> 7.4V+
SERVO1 黑/棕线    -> 7.4V GND
SERVO1 信号线     -> PA0 / SERVO1
7.4V GND          -> STM32 GND
```

3. 用示波器或逻辑分析仪测 PA0：

```text
p 0 90  -> 约 1520us
p 0 0   -> 约 500us
p 0 180 -> 约 2500us
频率     -> 约 333Hz
```

判断：

- 如果 PA0 有正确 PWM，问题在舵机供电、共地、线序或舵机本体。
- 如果 PA0 无 PWM，再回查 TIM2/PA0 硬件。

## 当前联调结论

已确认：

- 固件可从 Flash 启动。
- RTT attach-only 流程可避开 BOOT0 浮空复位问题。
- ELRS 输入链路正常。
- HX8 总线舵机控制链路已打通。
- HX8 的 `wr=0 rd=0`、`pos` 跟随 `tgt` 已验证。
- SERVO1 软件上游链路已打通，下一步重点在 PA0 PWM 实测与 7.4V 舵机供电/共地。
