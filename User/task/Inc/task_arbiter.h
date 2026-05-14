/**
 * @file task_arbiter.h
 * @brief Pure-function dual-link arbitration (L4).
 *
 * The arbiter takes RC intent, Vision intent, and current servo state, and
 * decides on the single unified motion intent the system should pursue
 * this tick. It is intentionally a pure function:
 *   - no global state read/write
 *   - no DataHub touch
 *   - no RTOS calls
 *
 * This makes it trivially unit-testable on the PC and keeps arbitration
 * rules visible in one place.
 *
 * Priority ladder (high to low):
 *   1. E-Stop (operator or RC-loss-in-AUTO)
 *   2. MANUAL mode (RC intent)
 *   3. AUTO mode with fresh vision (vision intent)
 *   4. AUTO mode with stale vision (park hold)
 *   5. IDLE default
 */

#ifndef TASK_ARBITER_H
#define TASK_ARBITER_H

#include "ara_def.h"
#include "datahub.h"

/* ============================================================================
 * Output codes for observability (packed into ArbiterOutput.reason_code)
 * ========================================================================== */

typedef enum {
    ARB_REASON_BOOT              = 0,
    ARB_REASON_ESTOP_OPERATOR    = 1,
    ARB_REASON_ESTOP_RC_LOSS     = 2,
    ARB_REASON_MANUAL_RC         = 3,
    ARB_REASON_AUTO_VISION_FRESH = 4,
    ARB_REASON_AUTO_VISION_STALE = 5,
    ARB_REASON_IDLE_DEFAULT      = 6,
    ARB_REASON_SERVO_OFFLINE     = 7
} ArbiterReason_t;

/* ============================================================================
 * Arbitration input / output
 * ========================================================================== */

typedef struct {
    const RcControlData_t  *rc;
    const VisionIntent_t   *vision;
    /** Opaque handle-style, read: is the servo currently reachable. */
    bool                    servo_online;
    uint32_t                tick_ms;
    AraSysMode_t            prev_mode;
    /** Vision freshness window in ms. Past this, AUTO falls back to park. */
    uint32_t                vision_stale_ms;
} ArbiterInput_t;

typedef struct {
    AraSysMode_t      mode;
    bool              estop;
    /** True when the arbiter wants torque applied to the servo. */
    bool              torque_request;
    /** Target joint angle in degrees. */
    int16_t           target_angle_deg;
    /** Target traversal speed, step/s. 0 means servo default. */
    uint16_t          target_speed;
    /** Acceleration register value (0..254). */
    uint8_t           target_acc;
    AraGripperCmd_t   gripper_cmd;
    uint8_t           roll_degree;
    AraLedPattern_t   led_pattern;
    ArbiterReason_t   reason_code;
} ArbiterOutput_t;

/* ============================================================================
 * API
 * ========================================================================== */

/**
 * @brief Default vision staleness window: a vision sample older than this
 *        is ignored in AUTO mode and the system parks.
 */
#define ARBITER_DEFAULT_VISION_STALE_MS   (200U)

/**
 * @brief Pure function. Consumes ArbiterInput, populates ArbiterOutput.
 *        Does not touch any globals.
 */
void TaskArbiter_Decide(const ArbiterInput_t *in, ArbiterOutput_t *out);

#endif /* TASK_ARBITER_H */
