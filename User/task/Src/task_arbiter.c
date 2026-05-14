/**
 * @file task_arbiter.c
 * @brief Pure-function dual-link arbitration implementation.
 */

#include "task_arbiter.h"

static int16_t rc_analog_to_angle_deg(int16_t ch1_percent)
{
    /* Map -100..+100 to 0..360 degrees linearly. Demo mapping only;
     * the real mechanism range is clamped by the ST3215 MIN/MAX limits. */
    int32_t deg = ((int32_t)ch1_percent + 100) * 360 / 200;
    if (deg < 0)   deg = 0;
    if (deg > 359) deg = 359;
    return (int16_t)deg;
}

void TaskArbiter_Decide(const ArbiterInput_t *in, ArbiterOutput_t *out)
{
    if ((in == NULL) || (out == NULL)) {
        return;
    }
    if ((in->rc == NULL) || (in->vision == NULL)) {
        return;
    }

    /* Safe defaults */
    out->mode             = ARA_MODE_IDLE;
    out->estop            = false;
    out->torque_request   = false;
    out->target_angle_deg = 0;
    out->target_speed     = 0U;
    out->target_acc       = 50U;
    out->gripper_cmd      = GRIPPER_CMD_STOP;
    out->roll_degree      = 90U;
    out->led_pattern      = LED_PATTERN_IDLE_SLOW_BLINK;
    out->reason_code      = ARB_REASON_IDLE_DEFAULT;

    const RcControlData_t *rc = in->rc;
    const VisionIntent_t  *vs = in->vision;

    const uint32_t vision_stale_ms = (in->vision_stale_ms != 0U)
                                         ? in->vision_stale_ms
                                         : ARBITER_DEFAULT_VISION_STALE_MS;

    /* ----- Rule 1: operator E-Stop absolute ----- */
    if (rc->is_link_up && (rc->estop_state == ESTOP_ACTIVE)) {
        out->mode           = ARA_MODE_ERROR;
        out->estop          = true;
        out->torque_request = false;
        out->led_pattern    = LED_PATTERN_ERROR_SOS;
        out->reason_code    = ARB_REASON_ESTOP_OPERATOR;
        return;
    }

    /* ----- Rule 2: servo offline - inhibit all motion ----- */
    if (!in->servo_online) {
        out->mode           = ARA_MODE_ERROR;
        out->estop          = true;
        out->torque_request = false;
        out->led_pattern    = LED_PATTERN_ERROR_SOS;
        out->reason_code    = ARB_REASON_SERVO_OFFLINE;
        return;
    }

    /* ----- Rule 3: RC link loss ----- */
    if (!rc->is_link_up) {
        /* In MANUAL (default/ambiguous), RC loss must also E-Stop because
         * operator cannot command. In AUTO (if prev_mode was AUTO), demo_v7
         * policy is also E-Stop (no headless vision run). */
        out->mode           = ARA_MODE_ERROR;
        out->estop          = true;
        out->torque_request = false;
        out->led_pattern    = LED_PATTERN_ERROR_SOS;
        out->reason_code    = ARB_REASON_ESTOP_RC_LOSS;
        return;
    }

    /* ----- Rule 4: MANUAL mode (RC up, operator present) ----- */
    if (rc->req_mode == ARA_MODE_MANUAL) {
        out->mode             = ARA_MODE_MANUAL;
        out->torque_request   = true;
        out->target_angle_deg = rc_analog_to_angle_deg(rc->ch1_percent);
        out->target_speed     = 1500U;
        out->target_acc       = 50U;
        out->gripper_cmd      = rc->gripper_cmd;
        out->roll_degree      = rc->roll_degree;
        out->led_pattern      = LED_PATTERN_MANUAL_HEARTBEAT;
        out->reason_code      = ARB_REASON_MANUAL_RC;
        return;
    }

    /* ----- Rule 5: AUTO mode ----- */
    if (rc->req_mode == ARA_MODE_AUTO) {
        const bool vision_fresh = vs->target_present &&
            ((uint32_t)(in->tick_ms - vs->last_update_tick_ms) < vision_stale_ms);

        if (vision_fresh) {
            out->mode             = ARA_MODE_AUTO;
            out->torque_request   = true;
            out->target_angle_deg = vs->target_angle_deg;
            out->target_speed     = (vs->target_speed != 0U) ? vs->target_speed : 1000U;
            out->target_acc       = 50U;
            out->gripper_cmd      = GRIPPER_CMD_STOP;
            out->roll_degree      = 90U;
            out->led_pattern      = LED_PATTERN_AUTO_SOLID;
            out->reason_code      = ARB_REASON_AUTO_VISION_FRESH;
            return;
        }

        /* Vision stale: park-hold at current position. */
        out->mode             = ARA_MODE_AUTO;
        out->torque_request   = true;
        out->target_angle_deg = 0; /* caller keeps last target via servo FSM */
        out->target_speed     = 0U;
        out->target_acc       = 50U;
        out->gripper_cmd      = GRIPPER_CMD_STOP;
        out->led_pattern      = LED_PATTERN_AUTO_SOLID;
        out->reason_code      = ARB_REASON_AUTO_VISION_STALE;
        return;
    }

    /* ----- Rule 6: default IDLE ----- */
    out->mode           = ARA_MODE_IDLE;
    out->torque_request = false;
    out->led_pattern    = LED_PATTERN_IDLE_SLOW_BLINK;
    out->reason_code    = ARB_REASON_IDLE_DEFAULT;
}
