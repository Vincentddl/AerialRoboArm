# STM32F103 HSE 最小测试

这个固件只测试外部高速晶振 HSE 是否起振，不使用 HAL、FreeRTOS、PLL 或看门狗。
CPU 始终由复位默认的内部 HSI 驱动，因此外部晶振失效时程序仍然可以报告结果。

## 结果指示

- `PC13` 慢闪：HSE 已起振。
- `PC13` 快闪：HSE 未在超时前起振。
- HSE 成功时，`PA8/MCO` 输出未经分频的 HSE，可用示波器测量，正常应约为 `8 MHz`。

PC13 外接 LED 的亮灭电平可能因电路而相反，但闪烁速度不受影响。

## 构建

在仓库根目录执行：

```powershell
cmake -S tools/hse_test -B cmake-build-hse-test -G Ninja
cmake --build cmake-build-hse-test
```

输出文件：

```text
cmake-build-hse-test/hse_test.elf
cmake-build-hse-test/hse_test.hex
cmake-build-hse-test/hse_test.bin
```

## 烧录

```powershell
D:\STM32_Env\OpenOCD-20231002-0.12.0\bin\openocd.exe `
  -s D:\STM32_Env\OpenOCD-20231002-0.12.0\share\openocd\scripts `
  -f daplink.cfg `
  -c "tcl_port disabled" `
  -c "gdb_port disabled" `
  -c "telnet_port disabled" `
  -c "program cmake-build-hse-test/hse_test.elf verify reset exit"
```

## 用 OpenOCD 读取精确结果

程序将结果结构固定放在 SRAM 地址 `0x20000000`。运行程序后执行：

```text
mdw 0x20000000 5
```

五个 32 位字依次是：

| 偏移 | 含义 | 正常值 |
| --- | --- | --- |
| `+0x00` | 魔数 | `0x48534554` |
| `+0x04` | 状态 | `1` = 起振，`2` = 失败 |
| `+0x08` | `RCC_CR` 快照 | 成功时 bit17 `HSERDY` 为 1 |
| `+0x0C` | `RCC_CFGR` 快照 | 成功时 MCO 选择 HSE |
| `+0x10` | 等待循环次数 | 起振所用或超时达到的近似等待次数 |

这个测试不会切换到 HSE 或 PLL，所以它只回答一个问题：晶振电路是否让 MCU 检测到了 `HSERDY`。
