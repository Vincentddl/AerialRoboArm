/**
 * @file bsp_i2c.c
 * @brief STM32 HAL implementation of the sensor I2C bus.
 */

#include "bsp_i2c.h"

#include "i2c.h"

static AraStatus_t map_hal_status(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return ARA_OK;
    }
    if (status == HAL_BUSY) {
        return ARA_BUSY;
    }
    if (status == HAL_TIMEOUT) {
        return ARA_TIMEOUT;
    }
    if ((HAL_I2C_GetError(&hi2c1) & HAL_I2C_ERROR_AF) != 0U) {
        return ARA_ERR_NACK;
    }
    return ARA_ERR_IO;
}

AraStatus_t BSP_I2C_IsReady(uint8_t address_7bit, uint32_t timeout_ms)
{
    if (address_7bit > 0x7FU) {
        return ARA_ERR_PARAM;
    }
    return map_hal_status(HAL_I2C_IsDeviceReady(&hi2c1,
                                                (uint16_t)address_7bit << 1,
                                                2U,
                                                timeout_ms));
}

AraStatus_t BSP_I2C_WriteReg(uint8_t address_7bit,
                             uint8_t reg,
                             const uint8_t *data,
                             uint16_t length,
                             uint32_t timeout_ms)
{
    if ((address_7bit > 0x7FU) || (data == NULL) || (length == 0U)) {
        return ARA_ERR_PARAM;
    }
    return map_hal_status(HAL_I2C_Mem_Write(&hi2c1,
                                            (uint16_t)address_7bit << 1,
                                            reg,
                                            I2C_MEMADD_SIZE_8BIT,
                                            (uint8_t *)data,
                                            length,
                                            timeout_ms));
}

AraStatus_t BSP_I2C_ReadReg(uint8_t address_7bit,
                            uint8_t reg,
                            uint8_t *data,
                            uint16_t length,
                            uint32_t timeout_ms)
{
    if ((address_7bit > 0x7FU) || (data == NULL) || (length == 0U)) {
        return ARA_ERR_PARAM;
    }
    return map_hal_status(HAL_I2C_Mem_Read(&hi2c1,
                                           (uint16_t)address_7bit << 1,
                                           reg,
                                           I2C_MEMADD_SIZE_8BIT,
                                           data,
                                           length,
                                             timeout_ms));
}

AraStatus_t BSP_I2C_WriteReg16(uint8_t address_7bit,
                               uint16_t reg,
                               const uint8_t *data,
                               uint16_t length,
                               uint32_t timeout_ms)
{
    if ((address_7bit > 0x7FU) || (data == NULL) || (length == 0U)) {
        return ARA_ERR_PARAM;
    }
    return map_hal_status(HAL_I2C_Mem_Write(&hi2c1,
                                            (uint16_t)address_7bit << 1,
                                            reg,
                                            I2C_MEMADD_SIZE_16BIT,
                                            (uint8_t *)data,
                                            length,
                                            timeout_ms));
}

AraStatus_t BSP_I2C_ReadReg16(uint8_t address_7bit,
                              uint16_t reg,
                              uint8_t *data,
                              uint16_t length,
                              uint32_t timeout_ms)
{
    if ((address_7bit > 0x7FU) || (data == NULL) || (length == 0U)) {
        return ARA_ERR_PARAM;
    }
    return map_hal_status(HAL_I2C_Mem_Read(&hi2c1,
                                           (uint16_t)address_7bit << 1,
                                           reg,
                                           I2C_MEMADD_SIZE_16BIT,
                                           data,
                                           length,
                                           timeout_ms));
}

uint32_t BSP_I2C_GetTickMs(void)
{
    return HAL_GetTick();
}

void BSP_I2C_DelayMs(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}
