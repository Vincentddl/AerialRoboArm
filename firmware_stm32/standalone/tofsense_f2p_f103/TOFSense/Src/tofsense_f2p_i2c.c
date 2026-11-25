#include "tofsense_f2p_i2c.h"

#include <stddef.h>

static uint16_t ReadLittleEndian16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t ReadLittleEndian32(const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8U)
         | ((uint32_t)data[2] << 16U)
         | ((uint32_t)data[3] << 24U);
}

bool TOFSense_F2P_I2C_Init(TOFSense_F2P_I2C *device,
                           I2C_HandleTypeDef *i2c,
                           uint8_t module_id)
{
    /* 0x08 + ID must remain a valid 7-bit I2C address. */
    if ((device == NULL) || (i2c == NULL) || (module_id > 0x6FU)) {
        return false;
    }

    device->i2c = i2c;
    device->module_id = module_id;
    device->hal_address = (uint16_t)(TOFSENSE_F2P_I2C_BASE_ADDRESS + module_id) << 1U;
    return true;
}

HAL_StatusTypeDef TOFSense_F2P_I2C_IsReady(TOFSense_F2P_I2C *device,
                                           uint32_t trials,
                                           uint32_t timeout_ms)
{
    if ((device == NULL) || (device->i2c == NULL)) {
        return HAL_ERROR;
    }
    return HAL_I2C_IsDeviceReady(device->i2c,
                                 device->hal_address,
                                 trials,
                                 timeout_ms);
}

HAL_StatusTypeDef TOFSense_F2P_I2C_Read(TOFSense_F2P_I2C *device,
                                       TOFSense_F2P_I2C_Result *result,
                                       uint32_t timeout_ms)
{
    uint8_t data[TOFSENSE_F2P_I2C_DATA_LENGTH];

    if ((device == NULL) || (device->i2c == NULL) || (result == NULL)) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        device->i2c,
        device->hal_address,
        TOFSENSE_F2P_I2C_DATA_REGISTER,
        I2C_MEMADD_SIZE_8BIT,
        data,
        sizeof(data),
        timeout_ms);
    if (status != HAL_OK) {
        return status;
    }

    /* Registers 0x20..0x2F, all values little-endian. */
    result->system_time_ms = ReadLittleEndian32(&data[0]);
    result->distance_mm = (int32_t)ReadLittleEndian32(&data[4]);
    result->distance_status = ReadLittleEndian16(&data[8]);
    result->signal_strength = ReadLittleEndian16(&data[10]);
    result->range_precision_cm = data[12];
    result->refresh_rate_hz = ReadLittleEndian16(&data[13]);
    result->filter_factor = data[15];
    return HAL_OK;
}
