# TOFSense-F2 P I2C + STM32F103 独立工程

本工程面向 STM32F103C8T6（常见 Blue Pill，8 MHz HSE）：STM32 通过 I2C1
直接读取 TOFSense-F2 P 寄存器，并通过 USART2 把解析结果发送给 USB-TTL 模块。

工程包含 CubeMX `.ioc`、CubeF1 HAL 1.8.6、CLion/CMake Presets、GNU Arm GCC
工具链配置、ST-Link/OpenOCD 配置和独立的 TOFSense I2C 驱动。

## 1. 默认接线

### TOFSense-F2 P -> STM32F103

| TOFSense-F2 P | STM32F103 | 说明 |
| --- | --- | --- |
| VCC | 5 V | 传感器供电范围为 4.3-5.2 V |
| GND | GND | 必须共地 |
| SCL | PB6 / I2C1_SCL | I2C 时钟 |
| SDA | PB7 / I2C1_SDA | I2C 数据 |

GH1.25 4P 母座卡扣凹槽朝上时，I2C 模式从右向左为 `VCC、GND、SDA、SCL`。

TOFSense 供电是 5 V，但信号电平是 3.3 V。若总线上没有上拉电阻，应在 SCL 和 SDA
各增加一个约 2.2-4.7 kΩ 的电阻上拉到 **3.3 V**，不可上拉到 5 V。

### USB-TTL -> STM32F103

| USB-TTL | STM32F103 | 说明 |
| --- | --- | --- |
| RXD | PA2 / USART2_TX | 接收测距文本，必须连接 |
| TXD | PA3 / USART2_RX | 当前程序未使用，可选 |
| GND | GND | 必须共地 |
| 3.3V/5V | 不连接 | STM32 已供电时避免重复供电 |

USB-TTL 选择 3.3 V 逻辑电平，串口终端设置为 `115200, 8-N-1`。

### ST-Link -> STM32F103

| ST-Link | STM32F103 |
| --- | --- |
| SWDIO | PA13 |
| SWCLK | PA14 |
| GND | GND |
| 3.3V | 3.3 V 电压参考 |
| NRST | NRST（可选） |

## 2. I2C 参数与寄存器

- I2C1：PB6/PB7，400 kHz，7 位地址
- TOFSense-F I2C 地址：`0x08 + 模块 ID`
- 默认 `ID=0`，所以默认地址是 `0x08`
- 程序每 10 ms 从 `0x20` 开始连续读取 16 字节
- 所有多字节值采用小端序

| 寄存器 | 数据 | 程序解释 |
| --- | --- | --- |
| `0x20` | 系统时间，4 字节 | ms |
| `0x24` | 距离，4 字节 | mm |
| `0x28` | 状态 + 信号强度，4 字节 | 低 16 位状态，高 16 位信号强度 |
| `0x2C` | 精度 + 刷新率 + 滤波因子，4 字节 | 精度 1 字节、刷新率 2 字节、滤波 1 字节 |

F2 系列距离状态为 `0=无效，1=有效`。`range_precision=0` 表示小于 1 cm，
`0xFF` 表示大于 255 cm。

应用参数位于 `Core/Inc/app_config.h`：

```c
#define TOFSENSE_I2C_ID           0U
#define TOFSENSE_I2C_CLOCK_HZ     400000U
#define TOFSENSE_POLL_PERIOD_MS   10U
#define TOFSENSE_I2C_TIMEOUT_MS   20U
#define TOFSENSE_DEBUG_DECIMATION 10U
```

模块必须先通过 NAssistant 设置成 IIC 模式，并确认其 ID。若 ID 改为 3，程序地址会自动变为
`0x0B`。

## 3. CLion 和 CMake

在 CLion 中直接打开本目录，选择 `debug` CMake Preset。

命令行构建：

```powershell
cmake --preset debug
cmake --build --preset debug
```

输出位于 `cmake-build-debug`：

- `TOFSense_F2P_F103.elf`
- `TOFSense_F2P_F103.hex`
- `TOFSense_F2P_F103.bin`
- `TOFSense_F2P_F103.map`

工具链默认路径是 `D:/DevEnv/GNU-tools-for-STM32`。若安装位置不同，可配置：

```powershell
cmake --preset debug -DARM_GCC_ROOT=D:/your/arm-gnu-toolchain
```

## 4. OpenOCD 下载和调试

在 CLion 右上角选择 `TOFSense_F2P_F103 OpenOCD` 后再点击运行或调试。不要选择自动生成的
`TOFSense_F2P_F103` CMake 应用程序配置；后者会把 ARM `.elf` 当作 Windows 程序执行，
从而出现 `CreateProcess error=193`。

工程已提供共享运行配置 `.run/TOFSense_F2P_F103_OpenOCD.run.xml`。如果选项没有立即显示，
请重新加载 CMake 工程，或关闭后重新打开本项目目录。

使用 ST-Link V2 双击烧录 Debug ELF：

```text
flash_stlink_v2.bat
```

脚本默认烧录 `cmake-build-debug/TOFSense_F2P_F103.elf`，执行烧录、校验并复位 MCU。
也可以把任意兼容的 `.elf` 文件直接拖到脚本上。命令行无暂停运行方式：

```powershell
.\flash_stlink_v2.bat --no-pause
```

连接 ST-Link 后烧录：

```powershell
cmake --build --preset debug --target flash
```

OpenOCD 配置是 `openocd/stlink.cfg`，默认 SWD 速度 1 MHz、Flash 64 KB。

在 CLion 的 `OpenOCD Download & Run` 配置中：

1. Executable 选择 `openocd.exe`。
2. Board config 选择 `openocd/stlink.cfg`。
3. Target 选择 `TOFSense_F2P_F103`。
4. GDB 选择 `arm-none-eabi-gdb.exe`。

## 5. CubeMX 配置

使用 STM32CubeMX 打开 `TOFSense_F2P_F103.ioc`：

- MCU：STM32F103C8T6，LQFP48
- HSE 8 MHz、PLL x9、系统时钟 72 MHz
- I2C1：PB6/PB7，400 kHz
- USART2：PA2/PA3，115200 bps
- Serial Wire：PA13/PA14
- PC13：状态 LED

业务逻辑位于 `App` 和 `TOFSense` 目录；`main.c` 只在 CubeMX `USER CODE` 区域调用
`App_Init()` 和 `App_Loop()`。生成代码后，应确认 CMake 仍包含：

- `App/Src/app.c`
- `TOFSense/Src/tofsense_f2p_i2c.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c`

当前电脑未检测到 STM32CubeMX，因此没有实际调用 CubeMX GUI/CLI 重新生成；`.ioc` 按仓库
现有 CubeMX 6.16.1 / CubeF1 1.8.6 格式创建，源码由 ARM GCC 实际编译验证。

## 6. USB-TTL 输出

启动示例：

```text
TOFSense-F2 P STM32F103 I2C monitor
sensor: I2C1 PB6/PB7 @ 400000 Hz, id=0, address=0x08
debug : USART2 PA2/PA3 @ 115200
probe : device ready
```

测距示例：

```text
F2P time=36766 ms distance=2221 mm status=1(VALID) signal=100 precision=1 cm rate=100 Hz filter=5
STAT reads=100 i2c_err=0 hal=0 last_age=3 ms
```

若显示 `probe : no ACK` 或 `i2c_err` 持续增长，重点检查：

- 模块是否已切换到 IIC 模式
- `TOFSENSE_I2C_ID` 是否与 NAssistant 中的 ID 一致
- SCL/SDA 是否接反并正确上拉到 3.3 V
- MCU、传感器、USB-TTL 是否共地
- 传感器 VCC 是否为 4.3-5.2 V

## 7. 官方资料

- 协议和 IIC 寄存器表：https://support.nooploop.com/cn/tofsense/protocol/
- 示例代码页面：https://support.nooploop.com/cn/tofsense/example-code/
- ArduPilot IIC 地址说明：https://support.nooploop.com/cn/tofsense/arduPilot-for-tOfsense/
- 硬件接线：https://support.nooploop.com/cn/tofsense/hardware/
