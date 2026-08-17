# GY-VL53L0X ToF Bring-up

## Wiring

| GY-VL53L0X | STM32F103 | Notes |
| --- | --- | --- |
| `VIN` | `3.3V` | The module contains an LDO and level shifting; 3.3 V keeps the bus in the MCU domain. |
| `GND` | `GND` | A common ground is mandatory. |
| `SCL` | `PB8 / I2C1_SCL` | I2C1 remap, 100 kHz. |
| `SDA` | `PB9 / I2C1_SDA` | I2C1 remap, 100 kHz. |
| `XSHUT` | optional | The four-pin board can leave it unconnected. |
| `GPIO1` | optional | Polling mode does not use the interrupt output. |

The supplied module schematic uses 10 kOhm pull-ups and bidirectional level shifting, so the firmware starts at a
conservative 100 kHz. Do not add another strong pull-up pair unless a scope shows that the bus rise time is too slow.

## Address Convention

The datasheet prints `0x52` for write and `0x53` for read. Those are 8-bit bus bytes. The firmware API uses the
7-bit address `0x29`; `bsp_i2c` performs the left shift required by STM32 HAL.

## Basic Usage

For the first hardware check, flash the normal `demo_v7` image, attach the RTT console, and press `t`. The debug task
probes address `0x29` and prints one bounded measurement without enabling a permanent sensor loop. A healthy module
should first report `model=0xEE`, followed by distance, validity, range status, signal count, and ambient count.

```c
#include "drv_vl53l0x.h"

static DrvVL53L0X_Context_t tof;

void Tof_Init(void)
{
    AraStatus_t status = DrvVL53L0X_Init(&tof,
                                         DRV_VL53L0X_DEFAULT_ADDRESS_7BIT,
                                         100U);
    if (status != ARA_OK) {
        /* Report status over RTT; do not block the control task forever. */
    }
}

bool Tof_ReadMillimeters(uint16_t *distance_mm)
{
    DrvVL53L0X_Result_t sample;
    if ((distance_mm == NULL) ||
        (DrvVL53L0X_ReadSingle(&tof, &sample) != ARA_OK) ||
        !DrvVL53L0X_IsRangeValid(&sample)) {
        return false;
    }
    *distance_mm = sample.distance_mm;
    return true;
}
```

`DrvVL53L0X_ReadSingle()` is blocking but bounded by the context timeout. Call it from a low-rate sensor task, not from
the 1 kHz actuator/control path or an interrupt. The result also exposes ambient count, signal count, device range
status, and the local timestamp.

## Scope

This is a compact power-on-default single-ranging driver derived from the supplied module example and the VL53L0X
control-interface specification. It validates model register `0xC0 == 0xEE`, triggers `SYSRANGE_START`, waits for a
new sample, reads the 12-byte result block, and clears the sensor interrupt.

The driver does not include ST's complete `STSW-IMG005` calibration/profile API. Add that package only if the project
needs custom timing budgets, long-range mode, multi-sensor address assignment, offset calibration, or cross-talk
calibration, and re-check the STM32F103C8 flash budget after integration.
