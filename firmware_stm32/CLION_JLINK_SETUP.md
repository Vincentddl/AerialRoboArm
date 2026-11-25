# CLion J-Link 配置（2026-11-13）

工程目录是本文件所在目录。固件源码和 CMake 构建预设未修改。本次未烧录。

## 当前 CLion 配置

- 选择右上角 `FOC_DEMO2 J-Link SEGGER (烧录并运行)`，类型为“嵌入式 GDB 服务器”。配置存于 `.idea/runConfigurations/FOC_DEMO2_J_Link_SEGGER___.xml`。
- 构建配置为 `hc13-debug - hc13-debug`，目标及下载文件均为 `FOC_DEMO2.elf`；运行前先构建，启动调试时总是下载 ELF。
- 使用 `E:\software\jlink\JLink_V898\JLinkGDBServerCL.exe`，设备 `STM32F103C8`，SWD 100 kHz，J-Link 序列号 `20090928`，GDB 端口 `2331`，并启用下载后校验（`-vd`）。CLion 使用内置 GDB。
- 原 `FOC_DEMO2 DAPLink` 配置保留。OpenOCD 的 `jlink.cfg` 也保留作参考，但当前 Windows J-Link 驱动下，该 OpenOCD 路径报 `LIBUSB_ERROR_NOT_SUPPORTED`，请使用上面的 SEGGER 配置。

## 操作与确认

1. 将 J-Link USB 接入电脑，连接目标板的 SWDIO、SWCLK、GND、VTref，并按目标板原供电方式上电。关闭其他占用 J-Link 的程序。
2. 在 CLion 选择上述 SEGGER 配置，确认 CMake 为 `hc13-debug - hc13-debug`。点击右上角绿色调试虫子按钮，CLion 将先构建，再启动 GDB Server 并尝试下载固件；只有日志显示下载成功，才能认定烧录完成，单独的“构建完成”不算。当前配置使用 GDB 调试入口，绿色三角运行按钮可能不可用。
3. 若要先只读确认 USB 探针是否被 SEGGER 软件发现，可在工程目录执行：

   ```powershell
   & 'E:\software\jlink\JLink_V898\JLink.exe' -CommandFile .\jlink_probe_readonly.jlink
   ```

   应出现 `J-Link[0]`、序列号 `20090928`。该命令不会连接目标板或写入固件。

## 本次验证情况

- 重新插接后，`JLink.exe` 已列出 `SAM-ICE`、序列号 `20090928`；SEGGER 原生 USB 驱动和探针枚举正常。此前的 `Could not select J-Link` 是当时探针未枚举时的结果。
- J-Link 读取到目标参考电压 `VTref=3.300V`，SWD 显示 `DAP initialized successfully`。在 100 kHz 和 50 kHz 下均出现 `Timeout while waiting for CPU to halt`；自动的 connect-under-reset 重试也失败。因此目前尚不能读取 STM32 芯片 ID，CLion 烧录功能也不能认定可用。
- 下一步先检查 J-Link 到目标板的 SWDIO、SWCLK、GND、VTref、NRST 接线及 NRST 实测电平，并确认 MCU 的 3.3 V 供电与 BOOT0 电平。若这些均正常，再排查 MCU 时钟/复位电路和正在运行的固件是否影响调试连接。不要在 CLion 中尝试下载，直到 J-Link Commander 能成功连接目标。
- 未改动固件源码，也未向目标板写入数据；连接测试可能触发目标复位。
