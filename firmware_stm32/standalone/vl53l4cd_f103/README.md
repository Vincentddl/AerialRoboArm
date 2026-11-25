# VL53L4CD + STM32F103C8T6

这是一个可独立打开、编译和烧录的 STM32F103C8T6 工程。它通过 I2C1 连续读取 VL53L4CD 单区域近距离测距结果，并通过 USART2 输出到 USB 转 TTL 模块。

## 已配置功能

- MCU：STM32F103C8T6，外部 8 MHz 晶振，系统时钟 72 MHz
- 传感器：VL53L4CD，默认 20 ms timing budget，连续测量
- I2C1：PB6/SCL、PB7/SDA，400 kHz
- XSHUT：PB0
- GPIO1：PB1（已保留；当前程序采用轮询方式）
- 串口：USART2，PA2/TX、PA3/RX，115200、8N1
- 调试/烧录：ST-Link V2，SWD
- 状态灯：PC13；得到有效距离时翻转
- 默认 I2C 地址：`0x29`（7 位）；ST 驱动中的 8 位写法为 `0x52`

## 接线

| VL53L4CD 模块 | STM32F103 | 说明 |
|---|---|---|
| VCC/VDD | 3.3V | 本工程按 3.3V 逻辑使用 |
| GND | GND | 必须共地 |
| SCL | PB6 | I2C1 时钟 |
| SDA | PB7 | I2C1 数据 |
| XSHUT | PB0 | 低电平关闭传感器 |
| GPIO1/INT | PB1 | 可选，当前程序不依赖中断 |

| USB 转 TTL | STM32F103 |
|---|---|
| RXD | PA2 / USART2_TX |
| TXD | PA3 / USART2_RX |
| GND | GND |

| ST-Link V2 | STM32F103 |
|---|---|
| SWDIO | PA13 |
| SWCLK | PA14 |
| GND | GND |
| 3.3V | 3.3V（只保留一个可靠供电源） |

SCL、SDA 需要上拉到 3.3V。很多成品模块已带上拉电阻；若没有，可各加约 4.7 kΩ。不要把 5V TTL 电平直接接到传感器或 STM32 引脚。

## CLion 编译

1. 用 CLion 打开本目录。
2. 在 CMake 配置中选择 `debug` 或 `release` preset。
3. 构建目标 `VL53L4CD_F103`。
4. `.run/VL53L4CD_F103_OpenOCD.run.xml` 是已准备好的 OpenOCD 面板配置。

命令行也可执行：

```powershell
cmake --preset debug
cmake --build --preset debug
```

生成文件：`cmake-build-debug/VL53L4CD_F103.elf`、`.hex` 和 `.bin`。

## 烧录

双击 `flash_stlink_v2.bat`，会烧录 Debug ELF。也可以把其他 `.elf` 文件拖到这个批处理文件上。脚本默认使用：

`D:\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe`

若 OpenOCD 安装位置不同，可设置环境变量 `VL53_OPENOCD`。

## 串口输出

串口工具选择正确的 USB 转 TTL COM 口，设置为 115200、8N1。程序输出距离、Range Status、信号强度、环境光、Sigma 和有效 SPAD 数，并每秒输出统计信息。

启动成功时应显示芯片 ID `0xEBAA`。测量质量分为：

- `VALID`：状态 0，距离有效。
- `WARNING`：状态 1、2、6，距离有结果但需要结合信号、Sigma 和现场条件判断。
- `ERROR`：其余状态，当前距离不应直接用于控制。

若看到 `init failed`，优先检查模块是否为 3.3V 供电、是否共地、SCL/SDA 是否接反，以及两根 I2C 线是否已上拉到 3.3V。程序会自动复位传感器、恢复 I2C 并重试。

参数集中在 `App/Inc/app_config.h`。可调整 timing budget、测量间隔和 `VL53L4CD_PRINT_DECIMATION`。

## 资料命名说明

原压缩包里的 `vl53l4cx用户手册.pdf`，PDF 内部标题实际是 **VL53L4CD ULD 用户指南**，配套 ZIP 代码也确实是 VL53L4CD，所以本工程按 VL53L4CD 创建。压缩包内另一个 `vl53l4cx.pdf` 则是 VL53L4CX 数据手册，不适用于本项目，未混入驱动。

## CubeMX

`VL53L4CD_F103.ioc` 保存了引脚与外设配置。当前环境未安装 STM32CubeMX，因此该文件按现有 STM32F103 HAL 工程整理，但没有在 CubeMX 图形界面中重新生成验证。若以后用 CubeMX 重新生成代码，请先备份 `App`、`Compat`、`Vendor` 和 `CMakeLists.txt`，避免覆盖手工移植部分。

资料副本见 `docs/VL53L4CD_ULD_User_Manual.pdf`。
