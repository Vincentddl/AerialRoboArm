/**
 * @file task_manipulator.h
 * @brief Manipulator FSM runnable (L4).
 *
 * The arbiter has already collapsed RC + Vision + servo state into one
 * unified intent. The manipulator FSM is now a thin behaviour layer that:
 *   - Translates ArbiterOutput into MotionCmd for task_motion.
 *   - Issues end-effector (gripper / roll) commands via mod_actuator.
 *   - Tracks demo state transitions (idle / manual / auto-pick / auto-done).
 *
 * No DataHub access. All inputs explicit. Pure-C state machine.
 */

#ifndef TASK_MANIPULATOR_H
#define TASK_MANIPULATOR_H

#include "ara_def.h"
#include "datahub.h"
#include "mod_actuator.h"
#include "task_arbiter.h"
#include "task_motion.h"

#ifndef TASK_MANIP_AUTO_GRIP_THREE_FRAME
#define TASK_MANIP_AUTO_GRIP_THREE_FRAME 0
#endif

#define TASK_MANIP_AUTO_OPEN_DEG          (ARBITER_AUTO_GRIPPER_ANGLE_DEG)
#define TASK_MANIP_AUTO_CLOSE_DEG         (90U)
#define TASK_MANIP_AUTO_FEEDBACK_MAX_MS   (140U)

/* ============================================================================
 * State enum
 * ============================================================================= */

typedef enum {
    MANIP_STATE_BOOT = 0,        /**< Waiting for first servo-online tick. */
    MANIP_STATE_IDLE,            /**< Safe idle, torque off. */
    MANIP_STATE_MANUAL,          /**< RC direct control. */
    MANIP_STATE_AUTO_GO,         /**< AUTO: traversing to vision target. */
    MANIP_STATE_AUTO_HOLD,       /**< AUTO: near target or no fresh target; grip open. */
    MANIP_STATE_ERROR_SAFE       /**< Latched fault / E-Stop. */
} ManipulatorState_t;

typedef struct {
    ManipulatorState_t current_state;
    uint32_t           state_enter_ms;
    float              last_target_angle_deg;
    ModActuator_Context_t actuator;
    bool               auto_active;
    bool               default_target_on_auto_entry;
    bool               auto_grip_closed;
    bool               auto_manual_grip_engaged;
    uint8_t            grip_stable_frames;
    uint32_t           last_vision_frame_id;
    uint8_t            gripper_command_deg;
} TaskManipulator_Context_t;

/* ============================================================================
 * API
 * ============================================================================= */

void TaskManipulator_Init(void);

/**
 * @brief Seed the manipulator's hold target from an absolute encoder reading.
 * @note  Call on startup and after an HX8 link recovery so AUTO_STALE / IDLE
 *        cannot fall back to the historical 0 degree initializer.
 */
void TaskManipulator_SeedCurrentAngle(float current_angle_deg);
/** Explicit default command, distinct from a measured encoder hold. */
void TaskManipulator_SeedDefaultPosition(void);

/**
 * @brief Run one 50 Hz manipulator step.
 * @param arb       Arbiter decision for this tick.
 * @param mot_state Latest motion feedback (read-only).
 * @param tick_ms   Current tick.
 * @param out_cmd   Output motion command for task_motion.
 */
void TaskManipulator_Update(const ArbiterOutput_t *arb,
                            const MotionState_t   *mot_state,
                            uint32_t               tick_ms,
                            MotionCmd_t           *out_cmd);

/**
 * @brief Read the current FSM state (for telemetry / DataHub publishing).
 */
ManipulatorState_t TaskManipulator_GetState(void);
uint8_t TaskManipulator_GetGripperAngle(void);
/** 0=inactive, 2=tracking/open, 3=closed and tracking, 4=holding/open. */
uint8_t TaskManipulator_GetAutoStage(void);
uint8_t TaskManipulator_GetGripStableFrames(void);
/** Isolated PWM path; never touches roll or HX8. */
bool TaskManipulator_DriveServo1Only(uint8_t degree);
/** Cancel a pick when RTT force control interrupts normal arbitration. */
void TaskManipulator_CancelAutoCycle(void);

#endif /* TASK_MANIPULATOR_H */
