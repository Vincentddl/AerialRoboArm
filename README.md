# AerialRoboArm: single-axis aerial manipulator firmware

STM32F103C8T6 firmware for an HX8-U26H-M main-arm UART servo and a SERVO1 PWM gripper. The MCU generates targets, arbitrates control and supervises feedback. Image processing and angle mapping run on the PC; the HX8 closes its own position loop.

[中文完整说明](README-zh.md) · [Documentation](Doc/README.md) · [Current runtime](Doc/design/CURRENT_SYSTEM.md) · [Wiring](Doc/hardware/pinmap.md)

## Formal control

```text
ELRS receiver -- UART/CRSF -- manual input ----+
                                            +-- safety/mode arbitration -- HX8 UART / gripper PWM
Camera -- video receiver -- PC -- HC13 vision+
HX8 encoder and protection feedback --------> supervision
```

MANUAL integrates CH1 stick input into the arm target and holds it when centered; CH4 selects gripper angles. AUTO continuously follows valid vision targets and holds when invalid/stale. On AUTO entry the gripper receives 130 degrees, then CH4 takes control. Optional three-frame closing is off by default. Safety arbitration takes priority, including RC loss, emergency stop, stall, offline feedback and latched faults. Main-arm target limits are −90 to +85 degrees.

## Select the right firmware

| | Formal build (default) | HC13 V2 host experiment |
| --- | --- | --- |
| USART1 PB6/PB7 | ELRS, 420000 | JY61P, 115200 |
| USART2 PA2/PA3 | HX8 through UC-01, 115200 | Same |
| USART3 PB10/PB11 | HC13 vision, 230400 | HC13 V2 commands/telemetry, 230400 |
| Runtime | FreeRTOS, 20 ms control task | Dedicated capture loop |
| Example | `V,-30,100,90` (degrees, continuous target) | `MOVE,101,-30000,100` (millidegrees, manual move) |

The vision protocols are incompatible. A V2 `V` frame stores a candidate for explicit triggering; it does not continuously drive the arm. JY61P and ELRS share a connector and must not be connected together.

## Build

Use CMake 4+, Ninja and an Arm GNU toolchain on PATH. Prefer a fresh build directory to avoid cached experimental options.

```powershell
cmake -S . -B cmake-build-formal -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-formal --parallel
.\tools\build_hc13_host.ps1
.\tools\test_hc13_host.ps1
```

Outputs retain the CubeMX-compatible name `FOC_DEMO2.elf/.hex/.bin`; there is no active MCU BLDC/FOC controller. The V2 helper builds only. `tools/flash_and_rtt.bat` targets the formal debug build, not the V2 image.

DAPLink/CMSIS-DAP uses `daplink.cfg`; SEGGER RTT remains required for OpenOCD logging and capture. V2 commands go through HC13, not legacy RTT `g` commands. IMU sampling targets 200 Hz, while wireless ACC is approximately 20 Hz. Full-rate capture files are local in `outputs/` and must be checked independently.

V2 gripper wait times and folded position remain uncalibrated (NA), blocking dependent automatic operations. PWM gripper commands have no physical position/grasp feedback. Software tests or successful queries do not prove complete real-world grasping. The PC apps and vision models are outside this MCU repository.

This project evolved from [LaneTang/AerialRoboArm](https://github.com/LaneTang/AerialRoboArm). Commit history, source authorship and third-party license notices are retained. Retired plans are recoverable from Git/history backups rather than presented as the current implementation.
