/**
 * @file bsp_pwm.c
 * @brief PWM Driver (TIM2 servo path only after demo_v7 BLDC retirement).
 *
 * The legacy 3-phase TIM1 path was removed when the BLDC stack was
 * retired in demo_v7 phase 1. Only the TIM2-driven hobby-servo PWM
 * channels remain (used for gripper / roll on the manipulator).
 */

#include "bsp_pwm.h"
#include "stm32f1xx_hal.h"

/* --- Hardware Resources --- */
extern TIM_HandleTypeDef htim2; // Servos (Auxiliary)

/* --- Configuration Maps --- */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
} ServoConfig_t;

static const ServoConfig_t servo_map[BSP_SERVO_NUM] = {
        [BSP_SERVO_1] = { &htim2, TIM_CHANNEL_1 },
        [BSP_SERVO_2] = { &htim2, TIM_CHANNEL_2 }
};

/* --- API Implementation --- */

void BSP_PWM_Init(void)
{
    /* TIM2 is a general-purpose timer, MOE not required. Channels are
     * started lazily on the first BSP_PWM_SetServoPulse() call so a
     * miswired channel does not generate spurious pulses at boot. */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void BSP_PWM_SetServoPulse(BspServo_Dev_t servo, uint16_t us)
{
    if (servo >= BSP_SERVO_NUM) return;

    /* Standard hobby-servo pulse range is 500-2500 us. */
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;

    /* Assumes TIM2 prescaler maps 1 tick = 1 us. */
    __HAL_TIM_SET_COMPARE(servo_map[servo].htim, servo_map[servo].channel, us);
}

void BSP_PWM_StopAll(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}
