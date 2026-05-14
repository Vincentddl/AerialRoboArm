/**
 * @file task_manipulator.c
 * @brief Manipulator FSM implementation for demo_v7.
 *
 * Stripped-down state machine. Heavy auto-flow logic from demo_v6 is gone;
 * AUTO is now driven directly by Arbiter (which itself is driven by Vision
 * Mock). This file just maps Arbiter intent to MotionCmd and tracks the
 * high-level demo state for observability.
 */

#include "task_manipulator.h"

#include <string.h>

static TaskManipulator_Context_t s_ctx;

static void enter_state(ManipulatorState_t s, uint32_t tick_ms)
{
    s_ctx.current_state  = s;
    s_ctx.state_enter_ms = tick_ms;
}

void TaskManipulator_Init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.current_state         = MANIP_STATE_BOOT;
    s_ctx.last_target_angle_deg = 0;
}

ManipulatorState_t TaskManipulator_GetState(void)
{
    return s_ctx.current_state;
}

void TaskManipulator_Update(const ArbiterOutput_t *arb,
                            const MotionState_t   *mot_state,
                            uint32_t               tick_ms,
                            MotionCmd_t           *out_cmd)
{
    if ((arb == NULL) || (mot_state == NULL) || (out_cmd == NULL)) {
        return;
    }

    /* Default safe command */
    out_cmd->torque_on        = false;
    out_cmd->target_angle_deg = s_ctx.last_target_angle_deg;
    out_cmd->target_speed     = 0U;
    out_cmd->target_acc       = 50U;
    out_cmd->force_keepalive  = false;

    /* Error / E-Stop short-circuits everything. */
    if (arb->estop || (arb->mode == ARA_MODE_ERROR)) {
        if (s_ctx.current_state != MANIP_STATE_ERROR_SAFE) {
            enter_state(MANIP_STATE_ERROR_SAFE, tick_ms);
        }
        out_cmd->torque_on = false;
        return;
    }

    /* Recovery from ERROR_SAFE: arbiter went non-error, transition to IDLE. */
    if (s_ctx.current_state == MANIP_STATE_ERROR_SAFE) {
        enter_state(MANIP_STATE_IDLE, tick_ms);
    }

    /* Boot waits for servo online. Arbiter rejects offline -> ERROR_SAFE. So
     * when we land here with non-error mode, servo is online. */
    if (s_ctx.current_state == MANIP_STATE_BOOT) {
        enter_state(MANIP_STATE_IDLE, tick_ms);
    }

    switch (arb->mode) {
    case ARA_MODE_MANUAL:
        if (s_ctx.current_state != MANIP_STATE_MANUAL) {
            enter_state(MANIP_STATE_MANUAL, tick_ms);
        }
        out_cmd->torque_on        = arb->torque_request;
        if (arb->target_angle_deg != ARBITER_TARGET_ANGLE_HOLD) {
            out_cmd->target_angle_deg   = arb->target_angle_deg;
            s_ctx.last_target_angle_deg = arb->target_angle_deg;
        } else {
            out_cmd->target_angle_deg   = s_ctx.last_target_angle_deg;
        }
        out_cmd->target_speed     = arb->target_speed;
        out_cmd->target_acc       = arb->target_acc;
        break;

    case ARA_MODE_AUTO:
        out_cmd->torque_on    = arb->torque_request;
        out_cmd->target_speed = arb->target_speed;
        out_cmd->target_acc   = arb->target_acc;

        if ((arb->reason_code == ARB_REASON_AUTO_VISION_FRESH) &&
            (arb->target_angle_deg != ARBITER_TARGET_ANGLE_HOLD)) {
            out_cmd->target_angle_deg   = arb->target_angle_deg;
            s_ctx.last_target_angle_deg = arb->target_angle_deg;
            if (mot_state->motion_status == ST3215_MOTION_ARRIVED) {
                if (s_ctx.current_state != MANIP_STATE_AUTO_HOLD) {
                    enter_state(MANIP_STATE_AUTO_HOLD, tick_ms);
                }
            } else {
                if (s_ctx.current_state != MANIP_STATE_AUTO_GO) {
                    enter_state(MANIP_STATE_AUTO_GO, tick_ms);
                }
            }
        } else {
            /* Stale vision OR explicit hold sentinel: hold last target. */
            out_cmd->target_angle_deg = s_ctx.last_target_angle_deg;
            if (s_ctx.current_state != MANIP_STATE_AUTO_HOLD) {
                enter_state(MANIP_STATE_AUTO_HOLD, tick_ms);
            }
        }
        break;

    case ARA_MODE_INIT:
    case ARA_MODE_IDLE:
    default:
        if (s_ctx.current_state != MANIP_STATE_IDLE) {
            enter_state(MANIP_STATE_IDLE, tick_ms);
        }
        out_cmd->torque_on = false;
        break;
    }
}
