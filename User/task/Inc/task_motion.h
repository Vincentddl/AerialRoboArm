/**
 * @file task_motion.h
 * @brief ST3215 servo proxy (L4).
 *
 * Motion is now a thin agent over the ST3215 bus:
 *   - Encode target / read feedback via drv_st3215.
 *   - Issue half-duplex transactions via bsp_uart HD API.
 *   - De-duplicate writes by deadband + periodic keepalive.
 *
 * demo_v7 ships with TASK_MOTION_USE_MOCK=1 by default so the whole control
 * stack runs without the electric servo or USART2. The mock simulates a
 * first-order tracker (servo "follows" target at a finite slew rate),
 * producing plausible feedback.
 *
 * When the ST3215 is physically connected and USART2 is wired, flip the
 * macro to 0 and replace the bsp_uart HD stub with the real DMA flow.
 */

#ifndef TASK_MOTION_H
#define TASK_MOTION_H

#include "ara_def.h"
#include "drv_st3215.h"

/* ============================================================================
 * Compile-time configuration
 * ============================================================================= */

#ifndef TASK_MOTION_USE_MOCK
#define TASK_MOTION_USE_MOCK            (1)
#endif

#define TASK_MOTION_SERVO_ID            (1U)   /**< Default ST3215 ID. */

#define TASK_MOTION_DEADBAND_STEPS      (5)    /**< ~0.44 degree. */
#define TASK_MOTION_KEEPALIVE_MS        (200U) /**< Force periodic WritePos. */
#define TASK_MOTION_ANGLE_MIN_DEG       (0)
#define TASK_MOTION_ANGLE_MAX_DEG       (359)
#define TASK_MOTION_DEFAULT_ACC         (50U)
#define TASK_MOTION_DEFAULT_SPEED       (1000U)

/* ============================================================================
 * Command / state types
 * ============================================================================= */

typedef struct {
    bool     torque_on;              /**< If false, motion releases torque. */
    int16_t  target_angle_deg;       /**< Commanded joint angle. */
    uint16_t target_speed;           /**< step/s. 0 means servo default. */
    uint8_t  target_acc;             /**< 0..254. */
    bool     force_keepalive;        /**< Force WritePos regardless of deadband. */
} MotionCmd_t;

typedef struct {
    /* --- This-tick IO outcome --- */
    St3215_IoResult_t   last_write_result;
    St3215_IoResult_t   last_read_result;

    /* --- Latest feedback (valid iff last_read_result == OK) --- */
    St3215_Feedback_t   feedback;
    bool                feedback_valid;

    /* --- High-level motion classification --- */
    St3215_MotionStatus_t motion_status;

    /* --- Link health projected from driver context --- */
    bool                servo_online;
} MotionState_t;

/* ============================================================================
 * API
 * ============================================================================= */

/**
 * @brief Initialise driver context, clear counters. No IO. Safe pre-scheduler.
 */
void TaskMotion_Init(void);

/**
 * @brief Execute one 50 Hz motion step.
 * @param cmd     Commanded motion from the arbiter / manipulator FSM.
 * @param tick_ms Current system tick in milliseconds.
 * @param state   Output state structure.
 *
 * @note Must run in task context (HD WaitRx uses vTaskDelay). Time budget:
 *       - Mock mode: microseconds.
 *       - Real mode: <= 2 ms WritePos + 2 ms ReadFeedback = 4 ms worst-case.
 */
void TaskMotion_Update(const MotionCmd_t *cmd,
                       uint32_t           tick_ms,
                       MotionState_t     *state);

#endif /* TASK_MOTION_H */
