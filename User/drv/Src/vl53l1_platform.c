/**
 * @file vl53l1_platform.c
 * @brief VL53L1X ULD transport mapped onto the project I2C BSP.
 */

#include "vl53l1_platform.h"

#include "bsp_i2c.h"

#include <limits.h>

static VL53L1_Error map_status(AraStatus_t status)
{
    if (status == ARA_OK) {
        return VL53L1_ERROR_NONE;
    }
    if (status == ARA_ERR_PARAM) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    return VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_Dev_t *dev,
                               uint16_t index,
                               uint8_t *data,
                               uint32_t count)
{
    if ((dev == NULL) || (data == NULL) || (count == 0U) ||
        (count > UINT16_MAX)) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    return map_status(BSP_I2C_WriteReg16(dev->address_7bit,
                                         index,
                                         data,
                                         (uint16_t)count,
                                         dev->timeout_ms));
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_Dev_t *dev,
                              uint16_t index,
                              uint8_t *data,
                              uint32_t count)
{
    if ((dev == NULL) || (data == NULL) || (count == 0U) ||
        (count > UINT16_MAX)) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    return map_status(BSP_I2C_ReadReg16(dev->address_7bit,
                                        index,
                                        data,
                                        (uint16_t)count,
                                        dev->timeout_ms));
}

VL53L1_Error VL53L1_WrByte(VL53L1_Dev_t *dev, uint16_t index, uint8_t data)
{
    return VL53L1_WriteMulti(dev, index, &data, 1U);
}

VL53L1_Error VL53L1_WrWord(VL53L1_Dev_t *dev, uint16_t index, uint16_t data)
{
    uint8_t bytes[2] = {(uint8_t)(data >> 8), (uint8_t)data};
    return VL53L1_WriteMulti(dev, index, bytes, sizeof(bytes));
}

VL53L1_Error VL53L1_WrDWord(VL53L1_Dev_t *dev, uint16_t index, uint32_t data)
{
    uint8_t bytes[4] = {
        (uint8_t)(data >> 24),
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)data,
    };
    return VL53L1_WriteMulti(dev, index, bytes, sizeof(bytes));
}

VL53L1_Error VL53L1_RdByte(VL53L1_Dev_t *dev, uint16_t index, uint8_t *data)
{
    return VL53L1_ReadMulti(dev, index, data, 1U);
}

VL53L1_Error VL53L1_RdWord(VL53L1_Dev_t *dev, uint16_t index, uint16_t *data)
{
    uint8_t bytes[2];
    VL53L1_Error status;
    if (data == NULL) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    status = VL53L1_ReadMulti(dev, index, bytes, sizeof(bytes));
    if (status == VL53L1_ERROR_NONE) {
        *data = ((uint16_t)bytes[0] << 8) | bytes[1];
    }
    return status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_Dev_t *dev, uint16_t index, uint32_t *data)
{
    uint8_t bytes[4];
    VL53L1_Error status;
    if (data == NULL) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    status = VL53L1_ReadMulti(dev, index, bytes, sizeof(bytes));
    if (status == VL53L1_ERROR_NONE) {
        *data = ((uint32_t)bytes[0] << 24) |
                ((uint32_t)bytes[1] << 16) |
                ((uint32_t)bytes[2] << 8) |
                bytes[3];
    }
    return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *dev, int32_t wait_ms)
{
    (void)dev;
    if (wait_ms < 0) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    BSP_I2C_DelayMs((uint32_t)wait_ms);
    return VL53L1_ERROR_NONE;
}
