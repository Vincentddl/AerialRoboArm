/**
 * @file bsp_gpio.h
 * @brief GPIO Hardware Abstraction
 * @note  Status LED only. MOTOR_EN (PA11) removed — PCB not connected.
 */

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "ara_def.h"

/* --- Device Definition --- */
typedef enum {
    BSP_GPIO_LED_STATUS = 0,  // Onboard Debug LED (PC13)
    BSP_GPIO_QTY
} BspGpio_Pin_t;

/* --- API --- */

/**
 * @brief Write digital value to GPIO
 * @param pin Target Pin Enum
 * @param active_level true = High/Active, false = Low/Inactive
 */
void BSP_GPIO_Write(BspGpio_Pin_t pin, bool active_level);

/**
 * @brief Read digital value from GPIO
 * @return true = High, false = Low
 */
bool BSP_GPIO_Read(BspGpio_Pin_t pin);

#endif // BSP_GPIO_H