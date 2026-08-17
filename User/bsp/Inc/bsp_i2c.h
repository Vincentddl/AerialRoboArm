/**
 * @file bsp_i2c.h
 * @brief L1 blocking I2C access for low-rate sensors.
 */

#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "ara_def.h"

AraStatus_t BSP_I2C_IsReady(uint8_t address_7bit, uint32_t timeout_ms);
AraStatus_t BSP_I2C_WriteReg(uint8_t address_7bit,
                             uint8_t reg,
                             const uint8_t *data,
                             uint16_t length,
                             uint32_t timeout_ms);
AraStatus_t BSP_I2C_ReadReg(uint8_t address_7bit,
                            uint8_t reg,
                            uint8_t *data,
                            uint16_t length,
                            uint32_t timeout_ms);
AraStatus_t BSP_I2C_WriteReg16(uint8_t address_7bit,
                               uint16_t reg,
                               const uint8_t *data,
                               uint16_t length,
                               uint32_t timeout_ms);
AraStatus_t BSP_I2C_ReadReg16(uint8_t address_7bit,
                              uint16_t reg,
                              uint8_t *data,
                              uint16_t length,
                              uint32_t timeout_ms);
uint32_t BSP_I2C_GetTickMs(void);
void BSP_I2C_DelayMs(uint32_t delay_ms);

#endif /* BSP_I2C_H */
