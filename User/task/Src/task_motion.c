/**
 * @file task_motion.c
 * @brief FSUS servo proxy with Mock fallback.
 */

#include "task_motion.h"
#include "bsp_uart.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"

#include <string.h>

/* =============================================================================
 * Internal state
 * ============================================================================= */

/* Last-sent target tracking for change detection. */
static float    s_last_target_deg     = -999.0f;
static bool     s_torque_active       = false;
static uint32_t s_last_command_tx_ms  = 0U;
static uint32_t s_last_stop_tx_ms     = 0U;
static uint32_t s_last_feedback_ok_ms = 0U;
static uint32_t s_last_feedback_poll_ms = 0U;
static bool     s_last_feedback_valid = false;
static FsusFeedback_t s_last_feedback;

/* Encoder continuity diagnostics use only newly received monitor frames.
 * Cached feedback must never be counted as a new physical observation. */
static bool     s_encoder_prev_valid = false;
static FsusFeedback_t s_encoder_prev_feedback;
static uint32_t s_encoder_jump_count = 0U;

#define ENCODER_DIAG_MAX_GAP_MS          (500U)
#define ENCODER_DIAG_STATIC_BAND_DEG     (2.0f)
#define ENCODER_DIAG_STATIC_JUMP_DEG     (5.0f)
#define ENCODER_DIAG_DYNAMIC_MARGIN_DEG  (5.0f)

#if TASK_MOTION_USE_MOCK
/* Mock-mode servo simulator */
static float    s_mock_pos_deg        = 0.0f;
static float    s_mock_target_deg     = 0.0f;
static bool     s_mock_torque_on      = false;
static uint32_t s_mock_last_tick_ms   = 0U;

#define MOCK_SLEW_DEG_PER_SEC   (130.0f)
#endif

/* =============================================================================
 * Helpers
 * ============================================================================= */

static float map_angle_to_fsus(float deg)
{
    /* Final safety boundary shared by every command source (RC, RTT and
     * HC-13 vision). The protocol supports +/-180 deg, but the mechanism is
     * deliberately restricted to -100 deg forward .. +100 deg backward. */
    if (deg > (float)TASK_MOTION_ANGLE_MAX_DEG) return (float)TASK_MOTION_ANGLE_MAX_DEG;
    if (deg < (float)TASK_MOTION_ANGLE_MIN_DEG) return (float)TASK_MOTION_ANGLE_MIN_DEG;
    return deg;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void update_encoder_diagnostic(const MotionCmd_t *cmd,
                                      bool command_changed,
                                      MotionState_t *state)
{
    state->encoder_jump_suspected = false;
    state->encoder_delta_deg = 0.0f;
    state->encoder_jump_count = s_encoder_jump_count;

    if (!state->feedback_fresh) {
        return;
    }

    if (s_encoder_prev_valid) {
        const uint32_t dt_ms =
            state->feedback.timestamp_ms - s_encoder_prev_feedback.timestamp_ms;
        const float delta = abs_float(state->feedback.angle_deg -
                                      s_encoder_prev_feedback.angle_deg);
        state->encoder_delta_deg = delta;

        if (cmd->torque_on && (dt_ms > 0U) &&
            (dt_ms <= ENCODER_DIAG_MAX_GAP_MS)) {
            float allowed_delta =
                abs_float(cmd->velocity_deg_per_s) * (float)dt_ms / 1000.0f +
                ENCODER_DIAG_DYNAMIC_MARGIN_DEG;

            const bool target_was_settled =
                !command_changed &&
                (abs_float(s_encoder_prev_feedback.angle_deg -
                           map_angle_to_fsus(cmd->target_angle_deg)) <=
                 ENCODER_DIAG_STATIC_BAND_DEG);
            if (target_was_settled) {
                allowed_delta = ENCODER_DIAG_STATIC_JUMP_DEG;
            }

            if (delta > allowed_delta) {
                state->encoder_jump_suspected = true;
                s_encoder_jump_count++;
                state->encoder_jump_count = s_encoder_jump_count;
            }
        }
    }

    s_encoder_prev_feedback = state->feedback;
    s_encoder_prev_valid = true;
}

/**
 * @brief FSUS transaction: send, then sync on response header 0x05 0x1C.
 * @param tx_buf    Encoded request frame.
 * @param tx_len    Request length.
 * @param rx_buf    Response buffer (>= FSUS_RX_BUF_SIZE).
 * @param timeout_ms Max wait.
 * @return Total response bytes received, 0 on timeout, or -1 on TX failure.
 */
static int16_t fsus_transact(const uint8_t *tx_buf, uint16_t tx_len,
                             uint8_t *rx_buf, uint32_t timeout_ms)
{
    /* Drop stale bytes before every request. A half-old response in the ring
     * buffer would otherwise look like a valid but unrelated servo reply. */
    if (BSP_UART_Fsus_EnsureRxArmed() != ARA_OK) {
        return -1;
    }
    BSP_UART_Fsus_Flush();
    if (BSP_UART_Fsus_Send(tx_buf, tx_len) != ARA_OK) {
        return -1;
    }

    uint32_t deadline = HAL_GetTick() + timeout_ms;
    uint16_t total = 0U;

    /* 1. Collect enough bytes to identify the response and read SIZE. */
    while (total < 5U) {
        total += BSP_UART_Fsus_Recv(&rx_buf[total], (uint16_t)(5U - total));
        if (total >= 5U) break;
        if (HAL_GetTick() > deadline) return 0U;
        osDelay(1U);   /* yield so lower-prio tasks (housekeeping/IWDG) run */
    }

    /* 2. Sync to response header 0x05 0x1C. If noise or stale bytes are
     * present, slide one byte at a time until the frame boundary is aligned. */
    while (rx_buf[0] != 0x05U || rx_buf[1] != 0x1CU) {
        /* Shift buffer left by 1, read one more byte. */
        for (uint16_t i = 0U; i < total - 1U; i++) {
            rx_buf[i] = rx_buf[i + 1U];
        }
        total--;
        while (BSP_UART_Fsus_Recv(&rx_buf[total], 1U) == 0U) {
            if (HAL_GetTick() > deadline) return 0U;
            osDelay(1U);
        }
        total++;
    }

    /* 3. Read remaining bytes. SIZE is the content length.
     *     Frame length = header(2) + cmd(1) + size(1) + content(size) + ck(1)
     *                  = 5 + content_size. */
    uint8_t content_size = rx_buf[3];
    uint16_t frame_len = (uint16_t)(5U + content_size);
    if (frame_len > FSUS_RX_BUF_SIZE) return 0U;

    while (total < frame_len) {
        total += BSP_UART_Fsus_Recv(&rx_buf[total], (uint16_t)(frame_len - total));
        if (total >= frame_len) break;
        if (HAL_GetTick() > deadline) return 0U;
        osDelay(1U);
    }
    return total;
}

/* =============================================================================
 * Real-servo helpers
 * ============================================================================= */

#if !TASK_MOTION_USE_MOCK

static FsusParseResult_t do_stop(uint8_t mode, uint16_t power_mw)
{
    uint8_t tx[FSUS_TX_BUF_SIZE];

    uint16_t tx_len = DrvFsus_EncodeStop(tx, TASK_MOTION_SERVO_ID, mode, power_mw);
    if (tx_len == 0U) {
        return FSUS_PARSE_BAD_FRAME;
    }

    /* Stop has no defined response; this is a fire-and-forget control frame.
     * In unlock mode it releases the servo's holding torque. */
    BSP_UART_Fsus_Flush();
    return (BSP_UART_Fsus_Send(tx, tx_len) == ARA_OK)
               ? FSUS_PARSE_OK
               : FSUS_PARSE_TX_FAILED;
}

static FsusParseResult_t do_set_angle(float    angle_deg,
                                       float    velocity_deg_per_s,
                                       uint16_t t_acc_ms,
                                       uint16_t t_dec_ms,
                                       uint16_t power_mw)
{
    uint8_t tx[FSUS_TX_BUF_SIZE];

    /* FSUS expects no response for SetAngleByVelocity by default.
     * Send and return OK — feedback is obtained via ServoMonitor. */
    /* Encode a motion target. This updates the servo's internal target
     * trajectory; the servo then closes the loop with its own encoder. */
    uint16_t tx_len = DrvFsus_EncodeSetAngleByVelocity(tx, TASK_MOTION_SERVO_ID,
                                                       angle_deg, velocity_deg_per_s,
                                                       t_acc_ms, t_dec_ms, power_mw);
    if (tx_len == 0U) {
        return FSUS_PARSE_BAD_FRAME;
    }
    BSP_UART_Fsus_Flush();
    return (BSP_UART_Fsus_Send(tx, tx_len) == ARA_OK)
               ? FSUS_PARSE_OK
               : FSUS_PARSE_TX_FAILED;
}

static FsusParseResult_t do_ping(void)
{
    uint8_t tx[FSUS_TX_BUF_SIZE];
    uint8_t rx[FSUS_RX_BUF_SIZE];

    uint16_t tx_len = DrvFsus_EncodePing(tx, TASK_MOTION_SERVO_ID);
    if (tx_len == 0U) {
        return FSUS_PARSE_BAD_FRAME;
    }

    int16_t rx_len = fsus_transact(tx, tx_len, rx, FSUS_IO_TIMEOUT_MS);
    if (rx_len < 0) {
        return FSUS_PARSE_TX_FAILED;
    }
    if (rx_len < 5) {
        return FSUS_PARSE_TIMEOUT;
    }
    return DrvFsus_ParsePing(rx, (uint16_t)rx_len, TASK_MOTION_SERVO_ID);
}

static FsusParseResult_t do_read_feedback(uint32_t        tick_ms,
                                          FsusFeedback_t *out)
{
    uint8_t tx[FSUS_TX_BUF_SIZE];
    uint8_t rx[FSUS_RX_BUF_SIZE];

    /* Ask the servo to report the state measured by its internal controller:
     * angle, voltage, current, power, temperature and status bits. */
    uint16_t tx_len = DrvFsus_EncodeServoMonitor(tx, TASK_MOTION_SERVO_ID);
    if (tx_len == 0U) {
        return FSUS_PARSE_BAD_FRAME;
    }

    int16_t rx_len = fsus_transact(tx, tx_len, rx, FSUS_IO_TIMEOUT_MS);
    if (rx_len < 0) {
        return FSUS_PARSE_TX_FAILED;
    }
    if (rx_len < 5) {
        return FSUS_PARSE_TIMEOUT;
    }
    /* Convert raw UART bytes into a typed feedback struct for the FSM. */
    return DrvFsus_ParseServoMonitor(rx,
                                     (uint16_t)rx_len,
                                     TASK_MOTION_SERVO_ID,
                                     tick_ms,
                                     out);
}

#endif /* !TASK_MOTION_USE_MOCK */

/* =============================================================================
 * Public API
 * ============================================================================= */

void TaskMotion_Init(void)
{
    s_last_target_deg = -999.0f;
    s_torque_active   = false;
    s_last_command_tx_ms = 0U;
    s_last_stop_tx_ms = 0U;
    s_last_feedback_ok_ms = 0U;
    s_last_feedback_poll_ms = 0U;
    s_last_feedback_valid = false;
    memset(&s_last_feedback, 0, sizeof(s_last_feedback));
    s_encoder_prev_valid = false;
    memset(&s_encoder_prev_feedback, 0, sizeof(s_encoder_prev_feedback));
    s_encoder_jump_count = 0U;

#if TASK_MOTION_USE_MOCK
    s_mock_pos_deg       = 0.0f;
    s_mock_target_deg    = 0.0f;
    s_mock_torque_on     = false;
    s_mock_last_tick_ms  = 0U;
#endif
}

#if TASK_MOTION_USE_MOCK

void TaskMotion_Update(const MotionCmd_t *cmd,
                       uint32_t           tick_ms,
                       MotionState_t     *state)
{
    if ((cmd == NULL) || (state == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));

    if (s_mock_last_tick_ms == 0U) {
        s_mock_last_tick_ms = tick_ms;
    }
    uint32_t dt_ms = tick_ms - s_mock_last_tick_ms;
    if (dt_ms > 500U) dt_ms = 500U;
    s_mock_last_tick_ms = tick_ms;

    s_mock_torque_on = cmd->torque_on;
    if (cmd->torque_on) {
        s_mock_target_deg = map_angle_to_fsus(cmd->target_angle_deg);
    }

    /* First-order tracker */
    float err       = s_mock_target_deg - s_mock_pos_deg;
    float max_delta = MOCK_SLEW_DEG_PER_SEC * (float)(int32_t)dt_ms / 1000.0f;
    if (max_delta < 0.1f) max_delta = 0.1f;

    float delta;
    if (err > max_delta)        delta =  max_delta;
    else if (err < -max_delta)  delta = -max_delta;
    else                        delta =  err;

    s_mock_pos_deg += delta;

    /* Simulated feedback */
    FsusFeedback_t fb;
    memset(&fb, 0, sizeof(fb));
    fb.servo_id      = TASK_MOTION_SERVO_ID;
    fb.angle_deg     = s_mock_pos_deg;
    fb.voltage_mv    = 7400;
    fb.current_ma    = 0;
    fb.power_mw      = 0;
    fb.temp_raw      = 3000;
    fb.status        = 0;
    fb.circle_count  = 0;
    fb.timestamp_ms  = tick_ms;

    state->last_write_result = FSUS_PARSE_OK;
    state->last_read_result  = FSUS_PARSE_OK;
    state->feedback          = fb;
    state->feedback_valid    = true;
    state->feedback_fresh    = true;
    state->feedback_age_ms   = 0U;
    state->servo_online      = true;
    state->is_moving         = (delta > 0.5f || delta < -0.5f);
    state->is_stalled        = false;
    state->is_overload       = false;
    state->encoder_jump_suspected = false;
    state->encoder_jump_count = 0U;
    state->encoder_delta_deg = 0.0f;
}

#else /* ============ Real-servo path (FSUS) ============ */

void TaskMotion_Update(const MotionCmd_t *cmd,
                       uint32_t           tick_ms,
                       MotionState_t     *state)
{
    if ((cmd == NULL) || (state == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->feedback_age_ms = UINT32_MAX;
    state->encoder_jump_count = s_encoder_jump_count;

#if TASK_MOTION_SKIP_PING_FOR_BENCH
    /* Bench mode short-circuit: HX8 / UC01 not powered. Pretend the main
     * arm is online and silently swallow any motion command so the rest of
     * the FSM (arbiter, manipulator) is allowed to drive PA0/PA1 PTK
     * end-effectors. We do NOT touch the FSUS bus here. */
    (void)cmd;
    state->last_write_result = FSUS_PARSE_OK;
    state->last_read_result  = FSUS_PARSE_OK;
    state->feedback.servo_id     = TASK_MOTION_SERVO_ID;
    state->feedback.angle_deg    = 0.0f;
    state->feedback.timestamp_ms = tick_ms;
    state->feedback_valid    = true;
    state->feedback_fresh    = true;
    state->feedback_age_ms   = 0U;
    state->servo_online      = true;
    state->is_moving         = false;
    state->is_stalled        = false;
    state->is_overload       = false;
    update_encoder_diagnostic(cmd, false, state);
    return;
#endif

    float target_deg = map_angle_to_fsus(cmd->target_angle_deg);

    /* Bring-up probe: first prove ID/baud/electrical reachability, then read
     * the absolute encoder before torque is enabled. The control FSM uses
     * this feedback to hold the current physical angle instead of moving to
     * an assumed zero position during startup. */
    if ((!cmd->torque_on) && cmd->force_update) {
        state->last_write_result = FSUS_PARSE_OK;
        state->last_read_result  = do_ping();
        if (state->last_read_result == FSUS_PARSE_OK) {
            state->last_read_result = do_read_feedback(tick_ms, &state->feedback);
        }
        if (state->last_read_result == FSUS_PARSE_OK) {
            state->feedback_valid = true;
            state->feedback_fresh = true;
            state->feedback_age_ms = 0U;
            state->servo_online   = true;
            s_last_feedback = state->feedback;
            s_last_feedback_valid = true;
            s_last_feedback_ok_ms = tick_ms;
            s_last_feedback_poll_ms = tick_ms;
            update_encoder_diagnostic(cmd, false, state);
        }
        return;
    }

    bool command_changed = false;
    bool control_frame_sent = false;

    /* --- Torque transition ---
     * Protocol-level meaning:
     *   false -> Stop(unlock), release holding torque
     *   true  -> send position command, servo holds/follows target internally */
    if (cmd->torque_on != s_torque_active) {
        if (cmd->torque_on) {
            /* Enable: send motion command to lock at target. */
            state->last_write_result = do_set_angle(target_deg,
                                                    cmd->velocity_deg_per_s,
                                                    cmd->t_acc_ms,
                                                    cmd->t_dec_ms,
                                                    cmd->power_mw);
            if (state->last_write_result == FSUS_PARSE_OK) {
                s_torque_active   = true;
                s_last_target_deg = target_deg;
                s_last_command_tx_ms = tick_ms;
                s_last_stop_tx_ms = 0U;
                command_changed   = true;
                control_frame_sent = true;
            }
        } else {
            /* Disable: stop and unlock. */
            state->last_write_result = do_stop(FSUS_STOP_MODE_UNLOCK, 0U);
            if (state->last_write_result == FSUS_PARSE_OK) {
                s_torque_active   = false;
                s_last_target_deg = -999.0f;
                s_last_command_tx_ms = 0U;
                s_last_stop_tx_ms = tick_ms;
                s_encoder_prev_valid = false;
                control_frame_sent = true;
            }
        }
    } else if (cmd->torque_on) {
        /* --- Target change detection ---
         * Avoid spamming the bus with the same target every control tick.
         * The servo keeps running its internal closed loop after one command. */
        float diff = target_deg - s_last_target_deg;
        if (diff < 0.0f) diff = -diff;
        bool changed = (diff > 0.2f) || cmd->force_update;
        bool target_interval_due = (s_last_command_tx_ms == 0U) ||
            ((uint32_t)(tick_ms - s_last_command_tx_ms) >=
             TASK_MOTION_TARGET_INTERVAL_MS);
        bool refresh_due = (s_last_command_tx_ms == 0U) ||
            ((uint32_t)(tick_ms - s_last_command_tx_ms) >=
             TASK_MOTION_COMMAND_REFRESH_MS);

        if (target_interval_due && (changed || refresh_due)) {
            state->last_write_result = do_set_angle(target_deg,
                                                    cmd->velocity_deg_per_s,
                                                    cmd->t_acc_ms,
                                                    cmd->t_dec_ms,
                                                    cmd->power_mw);
            if (state->last_write_result == FSUS_PARSE_OK) {
                s_last_target_deg = target_deg;
                s_last_command_tx_ms = tick_ms;
                command_changed   = changed;
                control_frame_sent = true;
            }
        } else {
            state->last_write_result = FSUS_PARSE_OK;
        }
    } else {
        const bool stop_refresh_due = (s_last_stop_tx_ms == 0U) ||
            ((uint32_t)(tick_ms - s_last_stop_tx_ms) >=
             TASK_MOTION_STOP_REFRESH_MS);
        if (stop_refresh_due) {
            state->last_write_result = do_stop(FSUS_STOP_MODE_UNLOCK, 0U);
            if (state->last_write_result == FSUS_PARSE_OK) {
                s_last_stop_tx_ms = tick_ms;
                control_frame_sent = true;
            }
        } else {
            state->last_write_result = FSUS_PARSE_OK;
        }
    }

    /* --- Periodically read feedback ---
     * Command frames tell the servo what to do; monitor frames tell us what
     * actually happened inside the servo controller. Polling at the full
     * control-loop rate can overload a marginal bring-up bus, so telemetry is
     * intentionally decimated and cached between successful monitor frames. */
    bool should_poll = (!s_last_feedback_valid) ||
                       ((uint32_t)(tick_ms - s_last_feedback_poll_ms) >= TASK_MOTION_FEEDBACK_PERIOD_MS);

    /* Never place a monitor query immediately behind a motion/Stop frame.
     * Defer it to the next 20 ms control tick, comfortably beyond the
     * protocol's 5-10 ms minimum command interval. */
    if (control_frame_sent && s_last_feedback_valid) {
        should_poll = false;
    }

    if (should_poll) {
        state->last_read_result = do_read_feedback(tick_ms, &state->feedback);
        s_last_feedback_poll_ms = HAL_GetTick();
    } else {
        state->last_read_result = FSUS_PARSE_OK;
        state->feedback = s_last_feedback;
        state->feedback.timestamp_ms = tick_ms;
    }

    if (should_poll && (state->last_read_result == FSUS_PARSE_OK)) {
        state->feedback_valid = true;
        state->feedback_fresh = true;
        state->feedback_age_ms = 0U;
        state->servo_online   = true;
        s_last_feedback = state->feedback;
        s_last_feedback_valid = true;
        s_last_feedback_ok_ms = tick_ms;

        /* Motion classification from servo status byte. */
        uint8_t st = state->feedback.status;
        state->is_stalled  = ((st >> 2) & 0x01U) != 0U;  /* BIT2 = stall */
        state->is_overload = (st & 0xFEU) != 0U;           /* BIT1..7 faults */
        state->is_moving   = (st & 0x01U) != 0U;           /* BIT0 executing */
        update_encoder_diagnostic(cmd, command_changed, state);
    } else if (s_last_feedback_valid &&
               ((uint32_t)(tick_ms - s_last_feedback_ok_ms) <= TASK_MOTION_ONLINE_GRACE_MS)) {
        state->feedback = s_last_feedback;
        state->feedback.timestamp_ms = tick_ms;
        state->feedback_valid = true;
        state->feedback_fresh = false;
        state->feedback_age_ms = (uint32_t)(tick_ms - s_last_feedback_ok_ms);
        state->servo_online = true;

        uint8_t st = state->feedback.status;
        state->is_stalled  = ((st >> 2) & 0x01U) != 0U;
        state->is_overload = (st & 0xFEU) != 0U;
        state->is_moving   = (st & 0x01U) != 0U;
    }

    state->encoder_jump_count = s_encoder_jump_count;
}

#endif /* TASK_MOTION_USE_MOCK */
