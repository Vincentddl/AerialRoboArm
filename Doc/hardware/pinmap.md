# STM32F103C8T6 Pin Map

本文档记录当前 AerialRoboArm 控制板 PCB 与当前固件之间的 STM32F103C8T6 引脚占用关系。

状态说明：

- **当前启用**：当前 `Core/Src/main.c` 启动路径会初始化或运行时会使用。
- **PCB 已布线，固件未启用**：原理图/PCB 已连接到外部硬件，但当前固件还没有初始化该外设。
- **历史遗留，当前不占用**：旧方案或 CubeMX 残留配置中出现过，但当前运行路径不再使用。

当前固件启动路径：

```text
MX_GPIO_Init()
MX_DMA_Init()
MX_I2C1_Init()
MX_TIM2_Init()
MX_USART1_UART_Init()
MX_USART2_UART_Init()
MX_USART3_UART_Init()
MX_IWDG_Init()
BSP_UART_Init()
BSP_PWM_Init()
```

调试输出改为 DAPLink + OpenOCD RTT server，不再占用 USART3；USART3 当前用于 HC13 透明无线串口。

## 当前使用的硬件

| 硬件 | MCU 资源 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| STM32F103C8T6 | 主控 | 当前启用 | FreeRTOS + HAL 固件运行平台。 |
| DAPLink / CMSIS-DAP | PA13/PA14 SWD | 当前启用 | 下载、调试、OpenOCD RTT server。 |
| SEGGER RTT | SWD + SRAM RTT buffer | 当前启用 | 调试日志与控制台输入，经 OpenOCD 暴露为 `127.0.0.1:9090`。 |
| RadioMaster Pocket + ELRS-2.4G-NANO 接收机 | USART1 remap: PB6/PB7 | 当前启用 | 遥控输入，CRSF/ELRS 串口波特率 `420000`。 |
| UC-01 TTL/USB 转接调试板 | USART2: PA2/PA3 -> UC-01 -> HX8 | 当前启用 | 作为 MCU 与 HX8 总线舵机之间的半双工/全双工转换与供电转接板。 |
| HX8 / FSUS 串口执行器 | USART2: PA2/PA3，经 UC-01 | 当前启用 | 当前代码命名仍有 `FSUS/ST3215`，PCB 网络名为 `HX8_TX/HX8_RX`。 |
| PTK / PWM 舵机 1 | TIM2_CH1: PA0 | 当前启用 | `SERVO1`，PWM 输出。 |
| PTK / PWM 舵机 2 | TIM2_CH2: PA1 | 当前启用 | `SERVO2`，PWM 输出。 |
| 状态指示 IO | PC13 GPIO | 当前启用 | 状态 LED / 板载状态输出。 |
| HSE 外部晶振 | PD0/PD1 | 当前启用 | 系统时钟源。 |
| IWDG | 内部外设 | 当前启用 | 独立看门狗，不占用 GPIO。 |
| HC13 无线串口 | USART3: PB10/PB11 + PB0 KEY | 当前启用 | HC13 视觉/远程控制链路，`230400 8N1`，PB0 KEY 当前预留。 |
| VL53L1X 距离传感器 | I2C1: PB8/PB9 | 当前启用 | `PB8=SCL`、`PB9=SDA`，默认 7-bit 地址 `0x29`。 |

## 当前运行时引脚占用

| STM32 引脚 | PCB 网络名 | 当前固件功能 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| PC13 | `PC13` | GPIO_Output | Output | 状态 LED / 状态输出，由 `DevStatus_LedSet()` 间接控制。 |
| PD0 | `OSC_IN` | RCC_OSC_IN | Input | HSE 外部晶振输入。 |
| PD1 | `OSC_OUT` | RCC_OSC_OUT | Output | HSE 外部晶振输出。 |
| NRST | `NRST` | Reset | Input | 复位引脚。 |
| PA0 | `SERVO1` | TIM2_CH1 PWM | Output | 舵机/末端执行器 PWM 1。 |
| PA1 | `SERVO2` | TIM2_CH2 PWM | Output | 舵机/末端执行器 PWM 2。 |
| PA2 | `HX8_TX` | USART2_TX | Output | HX8/FSUS 串口发送。 |
| PA3 | `HX8_RX` | USART2_RX | Input | HX8/FSUS 串口接收。 |
| PB6 | `ELRS_TX` | USART1_TX remap | Output | USART1 remap TX，连接 ELRS 侧 RX。 |
| PB7 | `ELRS_RX` | USART1_RX remap | Input | USART1 remap RX，接收 ELRS 数据。 |
| PB10 | `HC13_TX` | USART3_TX | Output | 连接 MCU 端 HC13 `RXD`，当前配置 `230400 8N1`。 |
| PB11 | `HC13_RX` | USART3_RX | Input | 连接 MCU 端 HC13 `TXD`，当前配置 `230400 8N1`。 |
| PB8 | `I2C1_SCL` | I2C1_SCL | Output | VL53L1X I2C 时钟，复用开漏输出，PCB 4.7 kOhm 上拉保留。 |
| PB9 | `I2C1_SDA` | I2C1_SDA | Bidirectional | VL53L1X I2C 数据，复用开漏，PCB 4.7 kOhm 上拉保留。 |
| PA13 | `PA13/SWDIO` | SYS_JTMS-SWDIO | Bidirectional | SWD 数据线；DAPLink/OpenOCD/RTT 使用。 |
| PA14 | `PA14/SWCLK` | SYS_JTCK-SWCLK | Input | SWD 时钟线；DAPLink/OpenOCD/RTT 使用。 |

## PCB 已布线但当前固件未启用

| STM32 引脚 | PCB 网络名 | 预期用途 | 当前状态 | 说明 |
| --- | --- | --- | --- | --- |
| PB0 | `HC13_KEY` | HC13 KEY | 未初始化 | 可用于 HC13 配置/模式控制。 |
| PB1 | `BOOT1` | Boot 配置 | 未作为 GPIO 使用 | BOOT1 硬件配置脚。 |
| BOOT0 | `BOOT0` | Boot 配置 | 硬件配置 | 启动模式选择脚。 |

## 当前不应再视为占用的历史资源

| 资源/引脚 | 历史用途 | 当前处理 |
| --- | --- | --- |
| PA8/PA9/PA10 / TIM1_CH1~CH3 | 旧 BLDC 三相 PWM | 当前 `main.c` 不初始化 TIM1，PCB 图中 PA8/PA9/PA10 未接外部功能。 |
| PA11 / `MOTOR_EN` | 旧 SimpleFOC Mini / BLDC 功率级 EN | PCB 未连接该使能脚；当前应停止视为有效硬件资源。 |
| PB8/PB9 / AS5600 I2C | 旧 AS5600 位置反馈 | AS5600 路径未启用；PB8/PB9 当前作为 VL53L1X 的 I2C1 总线。 |
| PB10/PB11 / Debug UART | 旧 USART3 调试串口 | 调试已迁移到 RTT；PB10/PB11 当前用于 HC13。 |

## USART 与 DMA 资源

| 外设 | 引脚 | 当前用途 | DMA/中断 |
| --- | --- | --- | --- |
| USART1 | PB6 TX, PB7 RX | ELRS-2.4G-NANO 接收机，`420000` baud | RX: DMA1_Channel5 circular；TX: DMA1_Channel4 normal；USART1 IRQ enabled。 |
| USART2 | PA2 TX, PA3 RX | 通过 UC-01 连接 HX8/FSUS 串口执行器，`115200` baud | 当前 BSP 使用 blocking TX + interrupt RX；CubeMX 仍配置 DMA1_Channel7 TX / DMA1_Channel6 RX；USART2 IRQ enabled。 |
| USART3 | PB10 TX, PB11 RX | HC13 透明无线串口，`230400` baud | RX: single-byte interrupt into `DrvHC13_PushRxByte()`；USART3 IRQ enabled。 |

## RadioMaster Pocket + ELRS-2.4G-NANO 接收机参数

当前遥控链路由 `RadioMaster Pocket ELRS` 遥控器与 `ELRS-2.4G-NANO` 贝壳/Nano 接收机构成。接收机通过 CRSF/ELRS 串口连接到 MCU 的 `USART1 remap: PB6/PB7`。

```text
RadioMaster Pocket ELRS
  -> 2.4 GHz ExpressLRS link
  -> ELRS-2.4G-NANO receiver
  -> STM32 USART1 remap PB6/PB7
  -> drv_elrs / task_rc / mod_rc_semantic
```

| 参数 | 数值 | 固件/硬件相关性 |
| --- | --- | --- |
| 遥控器 | `RadioMaster Pocket`，ELRS 版 | 当前人工遥控输入源。 |
| 接收机型号 | `ELRS-2.4G-NANO` / ExpressLRS Receiver | 当前 PCB 侧 ELRS 接收机。 |
| 频段 | `2.4 GHz` | 需要与遥控器 ELRS 版本/频段匹配。 |
| 芯片 | `SX1281 + ESP8285` | 供应商图示参数。 |
| 射频前端 | `BPF + FEM`，PA: `SKYWORKS RFX24C01` | 供应商图示强调抗干扰与灵敏度。 |
| 回传功率 | `100 mW` | 遥测回传能力参考。 |
| 输入电压 | `3.6..5.5 V` | 不建议直接接 STM32 3.3 V；使用受控 `5 V`/BEC 供电，并与 MCU 共地。 |
| 串口协议 | CRSF/ELRS UART | 当前固件 `USART1` 配置为 `420000` baud。 |
| 通道数 | `16` | `drv_elrs` 解析 16 路 CRSF 通道。 |
| 天线 | `T` 型 2.4G 小天线，`IPEX1` 连接器 | 天线需接好后再长时间上电发射/回传。 |
| 固件版本 | 图示 `3.3.1` | 遥控器和接收机 ELRS major 版本应一致。 |
| 尺寸/重量 | 接收机 `11 x 18 mm` / `0.4 g`；含线约 `1.97 g` | 机械安装参考。 |
| BOOT 焊盘 | 短接进入升级/刷写模式 | 仅升级固件时使用，正常运行不要短接。 |

ELRS 接收机接口说明：

| 接收机焊盘 | 连接对象 | 说明 |
| --- | --- | --- |
| `GND` | STM32 GND / 电源 GND | 必须共地。 |
| `VIN 3.6..5.5V` | 受控 `5 V`/BEC 输出 | 不要接反；不要按 3.3 V 供电。 |
| `TX` | STM32 `PB7 / USART1_RX` | 接收机发出的 CRSF 数据进入 MCU。 |
| `RX` | STM32 `PB6 / USART1_TX` | MCU 到接收机的回传/配置串口方向。 |

## ELRS 通道映射与舵机测试说明

当前固件将 ELRS/CRSF 原始通道解析成系统语义，再分别驱动 HX8 总线舵机与两路 PTK PWM 小舵机。

| 遥控器物理量 | CRSF 通道 | 当前作用 | 实际舵机/系统行为 |
| --- | --- | --- | --- |
| `SA` | CH5 | 模式选择 | 高值 = `MANUAL`，低值 = `AUTO`。 |
| `SD` | CH10 | 急停 | 低值 = `ESTOP_ACTIVE`，高值 = `ESTOP_RELEASED`。 |
| `SB` | CH9 | 故障复位脉冲 | `SD` 释放后拨到高值位置，触发约 `150 ms` reset pulse。 |
| `CH1` 摇杆 | CH1 | 主臂角度增量 | HX8-U26H-M 总线大舵机。 |
| `CH4` 摇杆 | CH4 | 夹爪小舵机角度 | `SERVO1 / PA0`。 |
| `SF` 旋钮/拨轮 | CH8 | 姿态/滚转小舵机角度 | `SERVO2 / PA1`。 |
| `SC` | CH7 | 旧夹爪开/停/关语义 | 当前主要不用于角度测试；夹爪角度测试以 `CH4` 为准。 |

测试前状态：

1. 先确认 RTT 中 `[RC]` 原始通道值会跟随遥控器动作变化。
2. 将 `SD / CH10` 放到高值释放状态，使系统离开 `ESTOP_ACTIVE`。
3. 将 `SA / CH5` 放到高值手动状态，使系统进入 `MANUAL`。
4. 如果之前触发过急停、RC loss 或其他锁存故障，在 `SD` 释放后拨一下 `SB / CH9` 到高值位置，触发复位脉冲。
5. RTT 日志应看到 `MANUAL_RC`，不应停在 `ESTOP_SD`、`ESTOP_RCLOS` 或 `NO_SERVO`。

PTK 小舵机测试：

| 测试对象 | 遥控输入 | 输出引脚 | 预期现象 |
| --- | --- | --- | --- |
| 夹爪小舵机 | `CH4` 摇杆 | `PA0 / SERVO1` | 中位约 `90 deg`，两端分别接近 `0 deg` 与 `180 deg`；代码带中位死区，轻微抖动不会立即动作。 |
| 姿态/滚转小舵机 | `SF / CH8` 旋钮或拨轮 | `PA1 / SERVO2` | 从一端到另一端线性映射 `0..180 deg`。 |

RTT 里可重点观察：

```text
[RC] CH1=... CH4=... SA=... SC=... SF=... SB=... SD=...
[DBG] ... roll=xxx grip=xxx ...
```

HX8 总线大舵机测试：

| 条件/动作 | 说明 |
| --- | --- |
| `SA = MANUAL` | 只有手动模式下 `CH1` 才会生成主臂目标角度。 |
| `SD = RELEASED` | 急停释放后才允许主臂目标角度更新。 |
| RC link 正常 | 掉线会进入 `ESTOP_RCLOS`。 |
| `CH1` 偏正方向 | 目标角度每 `20 ms` 增加 `15 deg`，上限 `+180 deg`。 |
| `CH1` 偏负方向 | 目标角度每 `20 ms` 减少 `15 deg`，下限 `-180 deg`。 |
| `CH1` 回中 | 小于约 `15%` 死区时不再累加。 |

当前代码默认 `TASK_MOTION_SKIP_PING_FOR_BENCH = 1`，bench 模式会假装 HX8 在线，但不会真正向 `USART2 -> UC-01 -> HX8` 发送总线命令。因此当前优先用于测试两路 PTK 小舵机；若要实测 HX8，需要先将该宏改为 `0`，并确认 HX8 供电、UC-01 电平、共地、舵机 ID 与 USART2 波特率一致。

## UC-01 TTL/USB 转接调试板参数

UC-01 是当前 MCU 与 HX8-U26H-M 总线舵机之间的转接板：MCU 侧使用 `USART2: PA2/PA3`，经 UC-01 转换后连接 HX8 的 `PH2.0 3Pin` 总线接口。它也可通过 Type-C 连接 PC，用于总线舵机 ID 配置、参数设置与调试。

```text
STM32 USART2 PA2/PA3
  -> UC-01 TTL/USB 转接调试板
  -> HX8-U26H-M UART 总线舵机
```

| 参数 | 数值 | 固件/硬件相关性 |
| --- | --- | --- |
| 输入电压 | `6.0..15.0 V` | 可覆盖 HX8 的 `9.0..12.6 V` 工作电压范围，供电仍需按舵机峰值电流留余量。 |
| 转换信号 | 半双工转全双工 | 用于 MCU/PC 串口与总线舵机单线半双工通信之间的转换。 |
| TTL 电平 | `3.3 V` / `5.0 V` 可切换 | 接 STM32F103 时应使用 `3.3 V` TTL 电平。 |
| 最大承载电流 | `20 A` | 转接板承载能力；实际电源、线材和接口也必须满足 HX8 峰值电流。 |
| 串口输出 | `1` 组 | 当前用于 HX8 总线舵机链路。 |
| 数据接口 | Type-C | 连接 PC 时可做舵机 ID、参数配置和调试。 |
| 舵机接口 | `PH2.0 x 2` | 可接总线舵机，接线需确认电源、地、信号顺序。 |
| 保护功能 | 反接保护 | 不能代替接线检查，上电前仍需确认极性。 |
| 工作温度 | `-10..60 degC` | 与 HX8 工作温度范围一致。 |
| 固定孔位 | `M2.0 x 3` | 机械固定参考。 |
| 电源指示灯 | 红色 | 上电状态检查。 |
| 尺寸/重量 | `36 x 24 mm` / `5 g` | 机械安装参考。 |

UC-01 接口说明：

| 接口/标识 | 含义 | 当前接线建议 |
| --- | --- | --- |
| 顶部排针 `V` | TTL 侧电源/参考电压 | 接 MCU 时谨慎使用，优先确认是否需要由 UC-01 向外供电；通常控制信号只接 `T/R/G`。 |
| 顶部排针 `R` | UC-01 串口 RX | 接 STM32 `PA2 / USART2_TX`。 |
| 顶部排针 `T` | UC-01 串口 TX | 接 STM32 `PA3 / USART2_RX`。 |
| 顶部排针 `G` | GND | 必须与 STM32 GND、舵机电源 GND 共地。 |
| 左侧 `PH2.0` 舵机接口 | `- / + / S` | `-` 为电源负/GND，`+` 为舵机电源正，`S` 为总线信号；两组接口并联用于接总线舵机。 |
| 右侧蓝色端子 | `6.0..15.0 V` 电源输入 | 图示上端为 `-`，下端为 `+`；给 UC-01 与舵机侧供电。 |
| Type-C | USB 数据接口 | 接 PC 做舵机配置/调试时使用。 |
| `3.3V <-> 5V` 拨动开关 | TTL 电平选择 | 连接 STM32F103 时拨到 `3.3 V` 侧。 |

## HX8-U26H-M UART 总线舵机参数

HX8-U26H-M 是当前主臂的大舵机/总线执行器，PCB 网络名为 `HX8_TX/HX8_RX`，当前固件通过 `USART2: PA2/PA3` 经 UC-01 与其通信。

| 参数 | 数值 | 固件/硬件相关性 |
| --- | --- | --- |
| 控制与通信 | `UART / TTL` 半双工，双向通信 | 当前固件的 HX8/FSUS 路径使用 `USART2`。 |
| 支持波特率 | `9600 bps..1 Mbps` | 当前 `USART2` 配置为 `115200` baud；若舵机实际 ID/波特率不同，需要先用上位机或配置命令统一。 |
| ID 范围 | `0..254` | 当前代码默认舵机 ID 需要与实物配置一致。 |
| 工作电压 | `9.0..12.6 V` | 不能由 MCU 3.3 V 供电；舵机电源需与控制板共地。 |
| 马达/齿轮 | 无刷马达，全金属不锈钢齿轮组 | 主臂负载执行器。 |
| 位置传感器 | `12-bit` 非接触式绝对值磁编码器 | 支持位置与状态回读。 |
| 分辨率 | `4096` 阶 / `360 deg`，约 `0.088 deg` | 运动控制与回读精度参考。 |
| 有效角度 | 单圈 `+/-180 deg`；多圈 `+/-368640 deg`，即 `1024` 圈 | 支持断电角度记忆，可任意设定原点。 |
| 处理器 | `32-bit MCU` | 舵机内部闭环与保护由舵机侧 MCU 执行。 |
| 减速比 | `173:1` | 机械传动参数。 |
| 输出轴 | 不锈钢，`6 mm`，`25T` | 机械连接/舵盘匹配参考。 |
| 外壳/接口 | 全铝合金，`PH2.0 3Pin` | 接线时确认电源、地、信号定义。 |
| 尺寸/重量 | `40 x 20 x 40 mm` / `78 g` | 机械安装与重量估算参考。 |
| 工作温度 | `-10..60 degC` | 环境限制。 |
| 工作模式 | 单圈角度、多圈角度、阻尼模式 | 固件当前主路径按角度目标控制。 |
| 停止模式 | 锁力保持、失锁释放、阻尼控制 | 对应安全停机和急停策略。 |
| 保护功能 | 温度、电压、堵转、功率、电流保护，智能功率限制 | 急停/故障回读需要关注这些状态。 |

`@12 V` 特性参数：

| 参数 | 数值 |
| --- | --- |
| 静态堵转扭矩 | `2.55 N*m` / `26 kg-cm` |
| 最大动态扭矩 | `1.18 N*m` / `12 kg-cm` |
| 额定扭矩 | `0.44 N*m` / `4.5 kg-cm` |
| 空载转速 | `142 rpm`，约 `0.070 s/60 deg` |
| 额定转速 | `116 rpm`，约 `0.086 s/60 deg` |
| 峰值电流 | `5.5 A` |
| 空载电流 | `<300 mA` |
| 待机电流 | `<40 mA` |
| 轴向负载 | `20 N` |
| 径向负载 | `40 N` |

## PTK 7462W PWM 小舵机参数

供应商页面标称型号为 `PTK 7462W MG`，当前固件将其用于 `SERVO1/PA0` 与 `SERVO2/PA1` 两路 PWM 小舵机。

| 参数 | 数值 | 固件相关性 |
| --- | --- | --- |
| 控制类型 | 数字舵机，PWM 输入 | `drv_servo.c` 通过 PWM 脉宽控制角度。 |
| 角度范围 | 约 `180 deg`，供应商图示为 `180 deg +/- 10 deg` | 当前软件限位按 `0..180 deg` 使用。 |
| PWM 刷新率 | `333 Hz` | `BSP_PWM_SetServoPulse()` 注释按 333 Hz 舵机帧处理。 |
| PWM 脉宽范围 | `500..2500 us` | 固件在 `BSP_PWM_SetServoPulse()` 中钳位到该范围。 |
| 中位脉宽 | `1520 us` | `BSP_PWM_Init()` 上电预装载两路舵机到中位。 |
| 死区 | 约 `3 us` | 供应商图示参数，角度微调时需注意小变化可能无响应。 |
| 工作电压 | `4.8..7.4 V` | 舵机电源应独立确认电流余量，并与 MCU 共地。 |
| 运行电流 | `300 mA @ 7.4 V` | 这是运行电流参考值，不等同于堵转峰值电流。 |
| 堵转扭矩 | `3.8 kg-cm @ 5.0 V`; `4.0 kg-cm @ 6.0 V`; `4.5 kg-cm @ 7.4 V`; `4.9 kg-cm @ 8.4 V` | 标称工作电压到 7.4 V；8.4 V 数据来自供应商性能表，实际使用前需确认舵机/供电可承受。 |
| 运行速度 | `0.112 s/60 deg @ 5.0 V`; `0.096 s/60 deg @ 6.0 V`; `0.083 s/60 deg @ 7.4 V`; `0.078 s/60 deg @ 8.4 V` | 运动规划估算可优先按 7.4 V 下约 `0.083 s/60 deg` 计算。 |
| 净重 | `13 g` | 供应商标题写 9g，但参数图中净重为 13g，以实物称重为准。 |
| 尺寸 | `23.7 x 12 x 26.6 mm` | 机械安装避让参考。 |
| 齿轮/轴承 | 金属齿轮，滚珠轴承 | 用于末端执行器小舵机负载评估。 |
| 外壳/电机 | 塑料外壳，铁芯电机 | 供应商图示参数。 |
| 线长/插头 | `JR 250 mm` | 线序接入 PCB 时需确认信号、电源、地。 |

当前固件角度到脉宽映射为分段线性：

```text
0 deg   -> 500 us
90 deg  -> 1520 us
180 deg -> 2500 us
```

## 调试链路

当前调试链路不再占用硬件 UART：

```text
STM32 SEGGER_RTT buffer
  -> PA13/PA14 SWD
  -> DAPLink / CMSIS-DAP
  -> OpenOCD RTT server
  -> TCP 127.0.0.1:9090
  -> VOFA+ / PuTTY / MobaXterm
```

OpenOCD RTT server 配置文件为仓库根目录的 `daplink_rtt.cfg`。

## 注意事项

- PCB 没有为 PA11 预留电机使能连接，后续固件应移除或停止初始化 PA11 的 `MOTOR_EN` 历史配置。
- HC13 当前使用 PB10/PB11 的 USART3；调试日志继续走 RTT，避免重新占用 USART3。
- PB8/PB9 当前固定初始化为 I2C1，供 VL53L1X 距离传感器使用。
- `HAL_MspInit()` 当前关闭 JTAG、保留 SWD，因此 PA13/PA14 必须保留给调试；PA15/PB3/PB4 理论上释放，但当前 PCB 未使用。
