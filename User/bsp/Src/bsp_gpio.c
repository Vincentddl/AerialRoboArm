/**
 * @file bsp_gpio.c
 * @brief GPIO Hardware Abstraction Implementation
 * @note  Maps Logical Enum to Physical STM32 Port/Pin
 *        PA11 MOTOR_EN removed — PCB not connected (demo_v7).
 * @author ARA Project Coder
 */

#include "bsp_gpio.h"
#include "stm32f1xx_hal.h" // L1 Layer accesses HAL directly

/* --- Hardware Configuration --- */

// Status LED (BluePill Onboard LED) — PC13
#ifndef LED_STATUS_GPIO_PORT
#define LED_STATUS_GPIO_PORT GPIOC
#define LED_STATUS_PIN       GPIO_PIN_13
#endif

/* --- Internal Mapping Struct --- */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
} GpioHardware_t;

/* --- Look-up Table --- */
static const GpioHardware_t gpio_map[BSP_GPIO_QTY] = {
        [BSP_GPIO_LED_STATUS] = { LED_STATUS_GPIO_PORT, LED_STATUS_PIN }
};

/* --- Implementation --- */

void BSP_GPIO_Write(BspGpio_Pin_t pin, bool active_level)
{
    // 1. Safety Check: Index Bounds
    if (pin < 0 || pin >= BSP_GPIO_QTY) {
        return;
    }

    // 2. Convert Logical Bool to HAL State
    // true  => High Voltage (Enable/On)
    // false => Low Voltage (Disable/Off)
    GPIO_PinState state = active_level ? GPIO_PIN_SET : GPIO_PIN_RESET;

    // 3. Hardware Access
    HAL_GPIO_WritePin(gpio_map[pin].port, gpio_map[pin].pin, state);
}

bool BSP_GPIO_Read(BspGpio_Pin_t pin)
{
    // 1. Safety Check
    if (pin < 0 || pin >= BSP_GPIO_QTY) {
        return false; // Default safe value
    }

    // 2. Hardware Access
    GPIO_PinState state = HAL_GPIO_ReadPin(gpio_map[pin].port, gpio_map[pin].pin);

    // 3. Return Logic
    return (state == GPIO_PIN_SET);
}