#ifndef TOFSENSE_F2P_I2C_H
#define TOFSENSE_F2P_I2C_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOFSENSE_F2P_I2C_BASE_ADDRESS 0x08U
#define TOFSENSE_F2P_I2C_DATA_REGISTER 0x20U
#define TOFSENSE_F2P_I2C_DATA_LENGTH 16U

typedef struct {
    uint32_t system_time_ms;
    int32_t distance_mm;
    uint16_t distance_status;
    uint16_t signal_strength;
    uint8_t range_precision_cm;
    uint16_t refresh_rate_hz;
    uint8_t filter_factor;
} TOFSense_F2P_I2C_Result;

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t module_id;
    uint16_t hal_address;
} TOFSense_F2P_I2C;

bool TOFSense_F2P_I2C_Init(TOFSense_F2P_I2C *device,
                           I2C_HandleTypeDef *i2c,
                           uint8_t module_id);

HAL_StatusTypeDef TOFSense_F2P_I2C_IsReady(TOFSense_F2P_I2C *device,
                                           uint32_t trials,
                                           uint32_t timeout_ms);

HAL_StatusTypeDef TOFSense_F2P_I2C_Read(TOFSense_F2P_I2C *device,
                                       TOFSense_F2P_I2C_Result *result,
                                       uint32_t timeout_ms);

static inline bool TOFSense_F2P_I2C_IsDistanceValid(
    const TOFSense_F2P_I2C_Result *result)
{
    return (result != NULL) && (result->distance_status == 1U);
}

#ifdef __cplusplus
}
#endif

#endif /* TOFSENSE_F2P_I2C_H */

