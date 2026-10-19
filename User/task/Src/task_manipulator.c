/**
 * @file task_manipulator.c
 * @brief Manipulator FSM implementation for demo_v7.
 *
 * AUTO continuously follows fresh vision targets with the gripper open,
 * matching the supervised tracking behaviour introduced in August 2026.
 */

#include "task_manipulator.h"
#include "mod_rc_semantic.h"

#include <string.h>
#include <math.h>

static TaskManipulator_Context_t s_ctx;

static float resolve_auto_velocity(uint16_t requested_deg_per_s)
{
    if (requested_deg_per_s == 0U) {
        return TASK_MOTION_AUTO_VELOCITY;
    }

    float velocity = (float)requested_deg_per_s;
    if (velocity < TASK_MOTION_AUTO_VELOCITY_MIN) {
        velocity = TASK_MOTION_AUTO_VELOCITY_MIN;
    }
    if (velocity > TASK_MOTION_AUTO_VELOCITY_MAX) {
        velocity = TASK_MOTION_AUTO_VELOCITY_MAX;
    }
    return velocity;
}

static void enter_state(ManipulatorState_t s, uint32_t tick_ms)
{
    s_ctx.current_state  = s;
    s_ctx.state_enter_ms = tick_ms;
}

void TaskManipulator_Init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    (void)ModActuator_Init(&s_ctx.actuator);
    s_ctx.current_state         = MANIP_STATE_BOOT;
    s_ctx.last_target_angle_deg = 0.0f;
    s_ctx.gripper_command_deg   = TASK_MANIP_AUTO_CLOSE_DEG;
}

void TaskManipulator_SeedCurrentAngle(float current_angle_deg)
{
    if (current_angle_deg > (float)TASK_MOTION_ANGLE_MAX_DEG) {
        current_angle_deg = (float)TASK_MOTION_ANGLE_MAX_DEG;
    }
    if (current_angle_deg < (float)TASK_MOTION_ANGLE_MIN_DEG) {
        current_angle_deg = (float)TASK_MOTION_ANGLE_MIN_DEG;
    }
    s_ctx.last_target_angle_deg = current_angle_deg;
}

ManipulatorState_t TaskManipulator_GetState(void)
{
    return s_ctx.current_state;
}

void TaskManipulator_SeedDefaultPosition(void)
{
    s_ctx.last_target_angle_deg = (float)ARA_MAIN_ARM_DEFAULT_ANGLE_DEG;
    s_ctx.default_target_on_auto_entry = true;
}

uint8_t TaskManipulator_GetGripperAngle(void)
{
    return s_ctx.gripper_command_deg;
}

uint8_t TaskManipulator_GetAutoStage(void)
{
    if (!s_ctx.auto_active) return 0U;
    if (s_ctx.auto_grip_closed) return 3U; /* Main-arm tracking remains active. */
    switch (s_ctx.current_state) {
    case MANIP_STATE_AUTO_GO:   return 2U;
    case MANIP_STATE_AUTO_HOLD: return 4U;
    default: return 0U;
    }
}

void TaskManipulator_CancelAutoCycle(void)
{
    s_ctx.auto_active = false;
    s_ctx.auto_manual_grip_engaged = false;
    s_ctx.auto_grip_closed = false;
    s_ctx.grip_stable_frames = 0U;
}

uint8_t TaskManipulator_GetGripStableFrames(void)
{
    return s_ctx.grip_stable_frames;
}

static bool command_gripper(uint8_t angle)
{
    if (ModActuator_SetGripperAngle(&s_ctx.actuator, angle) != MOD_ACTUATOR_OK) {
        return false;
    }
    s_ctx.gripper_command_deg = angle;
    return true;
}

bool TaskManipulator_DriveServo1Only(uint8_t degree)
{
    return command_gripper(degree);
}

static void update_auto(const ArbiterOutput_t *arb, const MotionState_t *mot,
                        uint32_t now_ms, MotionCmd_t *cmd)
{
    const bool entering_auto = !s_ctx.auto_active;
    if (entering_auto) {
        s_ctx.auto_active = true;
        s_ctx.auto_manual_grip_engaged = false;
        s_ctx.auto_grip_closed = false;
        s_ctx.grip_stable_frames = 0U;
        s_ctx.last_vision_frame_id = arb->vision_frame_id;
        if (mot->feedback_valid && !s_ctx.default_target_on_auto_entry) {
            TaskManipulator_SeedCurrentAngle(mot->feedback.angle_deg);
        }
        s_ctx.default_target_on_auto_entry = false;
    }
    cmd->torque_on = arb->torque_request;
    cmd->target_angle_deg = s_ctx.last_target_angle_deg;
    cmd->velocity_deg_per_s = resolve_auto_velocity(arb->target_speed);
    cmd->t_acc_ms = TASK_MOTION_AUTO_T_ACC_MS;
    cmd->t_dec_ms = TASK_MOTION_AUTO_T_DEC_MS;
    cmd->power_mw = TASK_MOTION_DEFAULT_POWER_MW;
#if TASK_MANIP_AUTO_GRIP_THREE_FRAME
    bool grip_ok = command_gripper(entering_auto ? TASK_MANIP_AUTO_OPEN_DEG
                                    : arb->gripper_override_enabled ? arb->gripper_angle
                                    : (s_ctx.auto_grip_closed ? TASK_MANIP_AUTO_CLOSE_DEG
                                                             : TASK_MANIP_AUTO_OPEN_DEG));
#else
    /* Hold the entry opening until CH4 is operated; then follow CH4 normally. */
    if (entering_auto) {
        (void)command_gripper(TASK_MANIP_AUTO_OPEN_DEG);
    } else if (arb->gripper_override_enabled) {
        (void)command_gripper(arb->gripper_angle);
    } else {
        if (arb->gripper_angle != MOD_RC_CH4_MOMENTARY_DEFAULT_DEG)
            s_ctx.auto_manual_grip_engaged = true;
        (void)command_gripper(s_ctx.auto_manual_grip_engaged
                             ? arb->gripper_angle : TASK_MANIP_AUTO_OPEN_DEG);
    }
#endif

    const bool fresh_target = arb->reason_code == ARB_REASON_AUTO_VISION_FRESH &&
                              arb->target_angle_deg != ARBITER_TARGET_ANGLE_HOLD &&
                              isfinite(arb->target_angle_deg);
    const bool healthy = arb->torque_request && mot->servo_online && mot->feedback_valid &&
                         mot->feedback_age_ms < TASK_MANIP_AUTO_FEEDBACK_MAX_MS &&
                         mot->feedback.voltage_mv >= 9000 && mot->feedback.voltage_mv <= 12600 &&
                         !(mot->feedback.status & 0xFEU) && !mot->is_stalled &&
                         !mot->is_overload && !mot->encoder_jump_suspected;
#if TASK_MANIP_AUTO_GRIP_THREE_FRAME
    if (!s_ctx.auto_grip_closed) {
        if (!fresh_target || !healthy || !grip_ok || arb->gripper_override_enabled) {
            s_ctx.grip_stable_frames = 0U;
            s_ctx.last_vision_frame_id = arb->vision_frame_id;
        } else if (arb->vision_frame_id != 0U &&
                   arb->vision_frame_id != s_ctx.last_vision_frame_id) {
            s_ctx.last_vision_frame_id = arb->vision_frame_id;
            if (arb->vision_stable_frames == 0U) s_ctx.grip_stable_frames = 0U;
            else if (arb->vision_stable_frames == 1U) s_ctx.grip_stable_frames = 1U;
            else if (s_ctx.grip_stable_frames < 3U) s_ctx.grip_stable_frames++;
            if (s_ctx.grip_stable_frames >= 3U && arb->vision_stable_frames >= 3U &&
                command_gripper(TASK_MANIP_AUTO_CLOSE_DEG)) {
                s_ctx.auto_grip_closed = true;
            }
        }
    }
#endif
    if (fresh_target && healthy) {
        float target = arb->target_angle_deg;
        if (target < TASK_MOTION_ANGLE_MIN_DEG) target = TASK_MOTION_ANGLE_MIN_DEG;
        if (target > TASK_MOTION_ANGLE_MAX_DEG) target = TASK_MOTION_ANGLE_MAX_DEG;
        s_ctx.last_target_angle_deg = target;
        cmd->target_angle_deg = target;
        ManipulatorState_t state = fabsf(mot->feedback.angle_deg - target) <= 3.0f
                                      ? MANIP_STATE_AUTO_HOLD : MANIP_STATE_AUTO_GO;
        if (s_ctx.current_state != state) enter_state(state, now_ms);
    } else if (s_ctx.current_state != MANIP_STATE_AUTO_HOLD) {
        enter_state(MANIP_STATE_AUTO_HOLD, now_ms);
    }
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
    out_cmd->torque_on          = false;
    out_cmd->target_angle_deg   = s_ctx.last_target_angle_deg;
    out_cmd->velocity_deg_per_s = 0.0f;
    out_cmd->t_acc_ms           = 0U;
    out_cmd->t_dec_ms           = 0U;
    out_cmd->power_mw           = 0U;
    out_cmd->force_update       = false;

    /* Error / E-Stop short-circuits everything. */
    if (arb->estop || (arb->mode == ARA_MODE_ERROR)) {
        TaskManipulator_CancelAutoCycle();
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
        if (mot_state->feedback_valid && !s_ctx.default_target_on_auto_entry) {
            TaskManipulator_SeedCurrentAngle(mot_state->feedback.angle_deg);
        }
        enter_state(MANIP_STATE_IDLE, tick_ms);
    }

    (void)ModActuator_SetRoll(&s_ctx.actuator, arb->roll_degree);
    if (arb->mode != ARA_MODE_AUTO) {
        TaskManipulator_CancelAutoCycle();
        (void)command_gripper(arb->gripper_angle);
    }

    switch (arb->mode) {
    case ARA_MODE_MANUAL:
        s_ctx.default_target_on_auto_entry = false;
        if (s_ctx.current_state != MANIP_STATE_MANUAL) {
            enter_state(MANIP_STATE_MANUAL, tick_ms);
        }
        out_cmd->torque_on = arb->torque_request;
        if (arb->target_angle_deg != ARBITER_TARGET_ANGLE_HOLD) {
            out_cmd->target_angle_deg   = (float)arb->target_angle_deg;
            s_ctx.last_target_angle_deg = (float)arb->target_angle_deg;
        } else {
            out_cmd->target_angle_deg   = s_ctx.last_target_angle_deg;
        }
        out_cmd->velocity_deg_per_s = TASK_MOTION_DEFAULT_VELOCITY;
        out_cmd->t_acc_ms           = TASK_MOTION_DEFAULT_T_ACC_MS;
        out_cmd->t_dec_ms           = TASK_MOTION_DEFAULT_T_DEC_MS;
        out_cmd->power_mw           = TASK_MOTION_DEFAULT_POWER_MW;
        break;

    case ARA_MODE_AUTO:
        update_auto(arb, mot_state, tick_ms, out_cmd);
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
