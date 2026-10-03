/**
 * @file bsp_pwm.h
 * @brief PWM Driver for hobby servos (TIM2). 3-phase TIM1 path retired
 *        in demo_v7 phase 1.
 */

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "ara_def.h"

/* SERVO2/PA1 is not connected during the current bench bring-up. Set to 1
 * after the roll servo is installed to restore its PWM output. */
#ifndef BSP_PWM_ENABLE_SERVO_2
#define BSP_PWM_ENABLE_SERVO_2 (0U)
#endif

/* --- Device Definition --- */
typedef enum {
    BSP_SERVO_1 = 0,    // Gripper / Aux Axis
    BSP_SERVO_2,
    BSP_SERVO_NUM
} BspServo_Dev_t;

/* --- API --- */

void BSP_PWM_Init(void);

/**
 * @brief  Set Servo Pulse Width
 * @param  servo  Target Servo
 * @param  us     Pulse width in microseconds (500-2500)
 */
void BSP_PWM_SetServoPulse(BspServo_Dev_t servo, uint16_t us);

/**
 * @brief  Emergency Stop - Forces all PWM outputs to safe state.
 */
void BSP_PWM_StopAll(void);

#endif // BSP_PWM_H
