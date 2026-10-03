#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void Error_Handler(void);

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define VL53_XSHUT_Pin GPIO_PIN_0
#define VL53_XSHUT_GPIO_Port GPIOB
#define VL53_GPIO1_Pin GPIO_PIN_1
#define VL53_GPIO1_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
