/**
 * @file drv_vl53l0x.h
 * @brief L2 basic single-shot driver for the GY-VL53L0X ToF module.
 *
 * This compact driver uses the sensor power-on ranging profile. It provides
 * bounded blocking reads and raw range status, but does not implement the
 * large ST profile/calibration API.
 */

#ifndef DRV_VL53L0X_H
#define DRV_VL53L0X_H

#include "ara_def.h"

#define DRV_VL53L0X_DEFAULT_ADDRESS_7BIT  (0x29U)
#define DRV_VL53L0X_EXPECTED_MODEL_ID     (0xEEU)
#define DRV_VL53L0X_DEFAULT_TIMEOUT_MS    (100U)

#define DRV_VL53L0X_RANGE_STATUS_VALID              (0U)
#define DRV_VL53L0X_RANGE_STATUS_MIN_RANGE_CLIPPED  (11U)

typedef struct {
    uint8_t address_7bit;
    uint8_t model_id;
    uint8_t revision_id;
    uint32_t timeout_ms;
    bool initialized;
} DrvVL53L0X_Context_t;

typedef struct {
    uint16_t distance_mm;
    uint16_t ambient_count;
    uint16_t signal_count;
    uint8_t range_status;
    uint32_t timestamp_ms;
} DrvVL53L0X_Result_t;

/** Probe the sensor and validate the reset model ID (0xEE at register 0xC0). */
AraStatus_t DrvVL53L0X_Init(DrvVL53L0X_Context_t *ctx,
                            uint8_t address_7bit,
                            uint32_t timeout_ms);

/** Start one measurement without waiting for completion. */
AraStatus_t DrvVL53L0X_StartSingle(DrvVL53L0X_Context_t *ctx);

/** Wait for a started measurement and decode the 12-byte result block. */
AraStatus_t DrvVL53L0X_ReadResult(DrvVL53L0X_Context_t *ctx,
                                  DrvVL53L0X_Result_t *result);

/** Start, wait, decode and clear one measurement. */
AraStatus_t DrvVL53L0X_ReadSingle(DrvVL53L0X_Context_t *ctx,
                                  DrvVL53L0X_Result_t *result);

/** Return true for the two device statuses that contain a usable distance. */
bool DrvVL53L0X_IsRangeValid(const DrvVL53L0X_Result_t *result);

#endif /* DRV_VL53L0X_H */
