# VL53L1X Gripper ToF Configuration

## Grasp Window

The end-effector ToF confirmation gate uses these fixed limits:

| Setting | Value |
| --- | ---: |
| Minimum accepted distance | 50 mm |
| Maximum accepted distance | 120 mm |
| Requested measurement frequency | 50 Hz |
| Measurement period | 20 ms |
| Consecutive valid samples | 2 |
| Confirmation latency | about 20--40 ms plus I2C/polling overhead |
| Stale timeout | 60 ms |

Both endpoints are inclusive. Any invalid sample or measurement outside 50--120 mm immediately clears the confirmation. A confirmed result also expires when no new sample arrives within 60 ms.

The gate implementation is sensor-independent and lives in `task_tof_gate.c`. It does not close the gripper by itself; the final control interlock must also require a valid vision command and an acceptable arm-angle error.

## Wiring

| VL53L1X module | STM32F103 | Notes |
| --- | --- | --- |
| `VIN` | `3.3V` | Keeps the module-side pull-ups in the MCU voltage domain. |
| `GND` | `GND` | Common ground is mandatory. |
| `SCL` | `PB8 / I2C1_SCL` | Remapped I2C1, 400 kHz. |
| `SDA` | `PB9 / I2C1_SDA` | Remapped I2C1, 400 kHz. |
| `XSHUT` | optional | The module pull-up keeps the sensor enabled. |
| `GPIO1` | optional | Firmware currently polls data-ready every 2 ms. |

The module pulls the sensor-side I2C and GPIO signals up to 3.3 V, so the imported ULD configuration enables the VL53L1X 2V8 I/O mode. Do not add external pull-ups to 5 V.

The module uses a different header order from the old GY-VL53L0X board. Verify the silkscreen instead of reusing the old connector order.

## Active Driver

The active build uses ST's BSD-licensed VL53L1X Ultra Lite Driver from the supplied `X-CUBE-53L1A1` package. Project integration consists of:

- `VL53L1X_api.c`: ST sensor setup and configuration routines.
- `vl53l1_platform.c`: 16-bit-register I2C adaptation to `bsp_i2c`.
- `drv_vl53l1x.c`: short mode, 20 ms timing budget, 20 ms inter-measurement period, and centered 8x8 ROI.
- `app_tof.c`: background acquisition, recovery, measurement-period statistics, and the gripper gate.

The legacy VL53L0X files remain in the repository for rollback but are excluded from the active build. VL53L0X and VL53L1X share address `0x29`; their register maps are not interchangeable.

## Runtime Check

Flash the normal firmware and press `t` in the RTT console. The command prints the latest non-blocking snapshot. A healthy sensor reports ID `0xEACC`, `online=1`, increasing `samples`, and a measured period close to 20 ms.

The firmware records the real interval between samples. Treat 50 Hz as verified only after the reported period remains near 20 ms on hardware; scheduling, wiring quality, and invalid measurements can reduce the effective rate.

## Bench Acceptance Test

1. Verify the sensor model and continuous-ranging frequency with timestamps.
2. Place the black foam target at 40, 50, 85, 120, and 130 mm.
3. Confirm that only 50, 85, and 120 mm enter the window.
4. Move the target rapidly through the window and check that two consecutive samples are required.
5. Remove the target and verify that confirmation clears within 60 ms.
