/**
 * @file drv_vl53l1x.h
 * @brief Project wrapper for continuous VL53L1X short-range measurements.
 */

#ifndef DRV_VL53L1X_H
#define DRV_VL53L1X_H

#include "ara_def.h"
#include "vl53l1_platform.h"

#define DRV_VL53L1X_DEFAULT_ADDRESS_7BIT  (0x29U)
#define DRV_VL53L1X_EXPECTED_SENSOR_ID    (0xEACCU)
#define DRV_VL53L1X_DEFAULT_TIMEOUT_MS    (100U)
#define DRV_VL53L1X_TIMING_BUDGET_MS      (20U)
#define DRV_VL53L1X_INTERMEASUREMENT_MS   (20U)
#define DRV_VL53L1X_ROI_WIDTH             (8U)
#define DRV_VL53L1X_ROI_HEIGHT            (8U)
#define DRV_VL53L1X_RANGE_STATUS_VALID     (0U)

typedef struct {
    VL53L1_Dev_t dev;
    uint16_t sensor_id;
    uint8_t interrupt_polarity;
    bool initialized;
    bool ranging;
} DrvVL53L1X_Context_t;

typedef struct {
    uint16_t distance_mm;
    uint16_t signal_rate_kcps;
    uint16_t ambient_rate_kcps;
    uint8_t range_status;
    uint32_t timestamp_ms;
} DrvVL53L1X_Result_t;

AraStatus_t DrvVL53L1X_Init(DrvVL53L1X_Context_t *ctx,
                            uint8_t address_7bit,
                            uint32_t timeout_ms);
AraStatus_t DrvVL53L1X_StartContinuous(DrvVL53L1X_Context_t *ctx);
AraStatus_t DrvVL53L1X_Stop(DrvVL53L1X_Context_t *ctx);
AraStatus_t DrvVL53L1X_ReadIfReady(DrvVL53L1X_Context_t *ctx,
                                   DrvVL53L1X_Result_t *result);
bool DrvVL53L1X_IsRangeValid(const DrvVL53L1X_Result_t *result);

#endif /* DRV_VL53L1X_H */
