/**
 * @file vl53l1_platform.h
 * @brief Minimal ST VL53L1X ULD platform contract for the ARA I2C BSP.
 */

#ifndef VL53L1_PLATFORM_H
#define VL53L1_PLATFORM_H

#include <stdint.h>

typedef int8_t VL53L1_Error;

#define VL53L1_ERROR_NONE               ((VL53L1_Error)0)
#define VL53L1_ERROR_INVALID_PARAMS     ((VL53L1_Error)-4)
#define VL53L1_ERROR_CONTROL_INTERFACE  ((VL53L1_Error)-13)

typedef struct {
    uint8_t address_7bit;
    uint32_t timeout_ms;
} VL53L1_Dev_t;

VL53L1_Error VL53L1_WriteMulti(VL53L1_Dev_t *dev,
                               uint16_t index,
                               uint8_t *data,
                               uint32_t count);
VL53L1_Error VL53L1_ReadMulti(VL53L1_Dev_t *dev,
                              uint16_t index,
                              uint8_t *data,
                              uint32_t count);
VL53L1_Error VL53L1_WrByte(VL53L1_Dev_t *dev, uint16_t index, uint8_t data);
VL53L1_Error VL53L1_WrWord(VL53L1_Dev_t *dev, uint16_t index, uint16_t data);
VL53L1_Error VL53L1_WrDWord(VL53L1_Dev_t *dev, uint16_t index, uint32_t data);
VL53L1_Error VL53L1_RdByte(VL53L1_Dev_t *dev, uint16_t index, uint8_t *data);
VL53L1_Error VL53L1_RdWord(VL53L1_Dev_t *dev, uint16_t index, uint16_t *data);
VL53L1_Error VL53L1_RdDWord(VL53L1_Dev_t *dev, uint16_t index, uint32_t *data);
VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *dev, int32_t wait_ms);

#endif /* VL53L1_PLATFORM_H */
