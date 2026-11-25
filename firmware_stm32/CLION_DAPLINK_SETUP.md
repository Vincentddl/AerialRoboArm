# CLion 与 DAPLink 本机配置记录（2026-11-07）

本目录是 2026-11-06 复制的 `codex/settled-position-trim` 固件快照。以下“已完成并实测”记录的是首次配置与烧录结果；后续联动修改及其烧录状态见文末。

## 已完成并实测

- Arm GCC/GDB：`E:\software\xpack-arm-none-eabi-gcc-15.2.1-1.1\bin`，Windows x64 版，GCC 15.2.1。原有两套 32 位 GCC 均无法在本机启动，提示“无效的系统 DLL 重定位”。
- OpenOCD：`E:\software\openocd-0.12.0-7\xpack-openocd-0.12.0-7\bin\openocd.exe`。
- `CMakePresets.json` 的 `hc13-debug` 预设将上述编译器加入 PATH，启用 `HX8_JY60_CAPTURE`、`JY61P_CAPTURE`、`HC13_HOST_LINK`，使用 CLion 2025.3 附带的 Ninja。
- `.run/FOC_DEMO2_DAPLink.run.xml`：OpenOCD 运行配置，使用项目现有 `daplink.cfg` 和 `FOC_DEMO2.elf`。CLion 导入后生成的活动配置名为 `hc13-debug - hc13-debug`，运行配置已对应修正。**尚未在 CLion 界面中点运行/调试验证该配置。**
- 命令行 CMake 配置和构建成功：`FOC_DEMO2.bin` 35,432 B；Flash 54.06%，RAM 67.27%。链接器报告 RWX 段警告，未阻止构建。
- Windows 检测到 DAPLink 的 CMSIS-DAP、虚拟磁盘和 COM7。OpenOCD 通过 SWD 识别 Cortex-M3、器件 ID `0x20036410`，Flash 64 KiB。
- 烧录前读取的 64 KiB Flash 已备份至 `diagnostics/preflash_20261107.bin`。
- 用户连接目标 MCU 并授权进一步烧录后，使用 OpenOCD 对本次构建的 `FOC_DEMO2.bin` 编程、校验并复位。OpenOCD 输出 `Programming Finished`、`Verified OK`、`Resetting Target`，退出码 0。未验证执行器实物动作或 HC13 上位机通信。

## 在 CLion 中完成一次性设置

1. 在 CLion 使用“文件 → 打开”，选择**本文件所在的工程目录**，不是旧 `103_a`。若弹出项目信任或 Windows 确认窗口，请自行检查并操作。
2. “设置 → 构建、执行、部署 → 工具链”：检查 CMake、Ninja 可用；C/C++ 编译器和调试器指向上述 xPack `bin` 目录中的 `arm-none-eabi-gcc.exe`、`arm-none-eabi-g++.exe`、`arm-none-eabi-gdb.exe`。CMake 预设也会把 xPack 的 `bin` 放在 PATH 最前面。
3. “设置 → 构建、执行、部署 → CMake”：使用已启用的 `hc13-debug - hc13-debug`，不要启用默认 MinGW `Debug`。重新加载 CMake，构建目标是 `FOC_DEMO2.elf`。
4. “设置 → 构建、执行、部署 → 嵌入式开发 → OpenOCD”：将可执行文件设为上述 `openocd.exe`。
5. 右上角选择 `FOC_DEMO2 DAPLink（连接单片机后使用）`，检查 `daplink.cfg`、ELF 和构建配置，再点运行/调试。该 IDE 按钮仍需在 CLion 里实际试一次；若未识别预置配置，可手动新建 OpenOCD Download & Run 配置，使用相同文件。

## 命令行复现

在本工程目录执行：

```powershell
& 'E:\software\CLion 2025.3\bin\cmake\win\x64\bin\cmake.exe' --preset hc13-debug
& 'E:\software\CLion 2025.3\bin\cmake\win\x64\bin\cmake.exe' --build --preset hc13-debug --parallel 4
```

确认目标板接线、供电与镜像无误后，烧录命令为：

```powershell
& 'E:\software\openocd-0.12.0-7\xpack-openocd-0.12.0-7\bin\openocd.exe' -f daplink.cfg -c 'program cmake-build-hc13-debug/FOC_DEMO2.bin 0x08000000 verify reset exit'
```

`daplink.cfg` 使用 CMSIS-DAP/SWD、STM32F1、1 MHz；DAPLink 的 COM7 仅是探针虚拟串口，不代表 HC13 链路已通。下一步仍需在 CLion 验证一键下载配置，并通过 INFO/CAL/STATE 核对新固件和上位机协议，再进行机构动作测试。

## 2026-11-07 联动固件更新状态

本次已修改 HC13 实验固件：PICK 可从 PC 的 +50° 正常位或暂定 +100° 折叠位执行，在 HX8 新鲜实际位置跨过目标前 4° 时由 MCU 应用一次闭爪 PWM；实验构建角度上限为 +100°，正式 RC 构建仍为 +85°；运动中的短时反馈缺帧宽限为 400 ms，新动作准入仍要求小于 200 ms 的反馈。上位机已改为发送 PICK，待 MCU 回报完成与新鲜空闲状态后回到抓取起点。详见 `Doc/hardware/HC13_HOST_PROTOCOL_V2.md`。

构建与 HC13 C 测试通过；新 `FOC_DEMO2.bin` 为 35,512 B，SHA256 `9D87B72D1C3359B9C4AD42418085FA18862B69747BAEF34F50B8206CE3CA74F9`。常规隔离环境最初报告 `unable to find a matching CMSIS-DAP device`；取得本机设备访问权限后，OpenOCD 识别 Cortex-M3 和 `0x20036410`，烧录日志显示 `Programming Finished`、`Verified OK`、`Resetting Target`，退出码 0。新固件已写入并经 Flash 校验。无线 INFO/CAL/STATE 查询及机械运动、高速反馈和 +100° 折叠仍需实机试验。
