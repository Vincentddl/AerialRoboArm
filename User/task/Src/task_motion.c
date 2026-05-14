/**
 * @file task_motion.c
 * @brief ST3215 servo proxy implementation with Mock fallback for demo_v7.
 *
 * The old BLDC/FOC/PID implementation has been retired. This file now
 * targets the ST3215-HS smart joint where closed-loop is inside the servo.
 */

#include "task_motion.h"
#include "bsp_uart.h"

#include <string.h>

/* =============================================================================
 * Internal state
 * ============================================================================= */

static DrvSt3215_Context_t s_st_ctx;

/* Deadband + keepalive bookkeeping */
static int16_t  s_last_written_steps   = INT16_MIN;
static uint32_t s_last_written_ms      = 0U;
static bool     s_torque_enabled       = false;

#if TASK_MOTION_USE_MOCK
/* Mock-mode servo simulator */
static int32_t  s_mock_pos_steps       = 0;
static int32_t  s_mock_target_steps    = 0;
static int32_t  s_mock_velocity        = 0;
static bool     s_mock_torque_on       = false;
static uint32_t s_mock_last_tick_ms    = 0U;

#define MOCK_SLEW_STEPS_PER_SEC   (2000)
#endif

/* =============================================================================
 * Helpers
 * ============================================================================= */

static int16_t clamp_angle_deg(int16_t deg)
{
    if (deg < TASK_MOTION_ANGLE_MIN_DEG) {
        return (int16_t)TASK_MOTION_ANGLE_MIN_DEG;
    }
    if (deg > TASK_MOTION_ANGLE_MAX_DEG) {
        return (int16_t)TASK_MOTION_ANGLE_MAX_DEG;
    }
    return deg;
}

/* =============================================================================
 * Public API
 * ============================================================================= */

void TaskMotion_Init(void)
{
    (void)DrvSt3215_Init(&s_st_ctx, TASK_MOTION_SERVO_ID);
    s_last_written_steps = INT16_MIN;
    s_last_written_ms    = 0U;
    s_torque_enabled     = false;

#if TASK_MOTION_USE_MOCK
    s_mock_pos_steps    = 0;
    s_mock_target_steps = 0;
    s_mock_velocity     = 0;
    s_mock_torque_on    = false;
    s_mock_last_tick_ms = 0U;
    /* Seed driver context as online so arbiter does not latch SERVO_OFFLINE. */
    DrvSt3215_NoteIoOk(&s_st_ctx, 1U);
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
        s_mock_target_steps = ST3215_DEG_TO_STEPS(
            clamp_angle_deg(cmd->target_angle_deg));
    }

    int32_t err       = s_mock_target_steps - s_mock_pos_steps;
    int32_t max_delta = (int32_t)MOCK_SLEW_STEPS_PER_SEC * (int32_t)dt_ms / 1000;
    if (max_delta < 1) max_delta = 1;

    int32_t delta;
    if (err > max_delta)       delta =  max_delta;
    else if (err < -max_delta) delta = -max_delta;
    else                       delta =  err;

    s_mock_pos_steps += delta;
    s_mock_velocity   = (dt_ms > 0U) ? (delta * 1000 / (int32_t)dt_ms) : 0;

    /* Simulated feedback */
    St3215_Feedback_t fb;
    memset(&fb, 0, sizeof(fb));
    fb.position     = (int16_t)s_mock_pos_steps;
    fb.speed        = (int16_t)s_mock_velocity;
    fb.load         = (int16_t)((delta != 0) ? 100 : 0);
    fb.voltage_dv   = 74U;
    fb.temp_c       = 35U;
    fb.moving       = (delta != 0);
    fb.current      = 0;
    fb.timestamp_ms = tick_ms;

    s_st_ctx.last_fb       = fb;
    s_st_ctx.last_fb_valid = true;
    DrvSt3215_NoteIoOk(&s_st_ctx, tick_ms);

    state->last_write_result = ST3215_IO_OK;
    state->last_read_result  = ST3215_IO_OK;
    state->feedback          = fb;
    state->feedback_valid    = true;
    state->servo_online      = true;
    state->motion_status     = DrvSt3215_ClassifyMotion(&fb,
                                                        (int16_t)s_mock_target_steps,
                                                        NULL);
}

#else /* ============ Real-servo path (HD stub until motor arrives) ============ */

static St3215_IoResult_t do_write_byte(uint8_t  reg_addr,
                                       uint8_t  value,
                                       uint32_t tick_ms)
{
    uint8_t tx[ST3215_TX_BUF_SIZE];
    uint8_t rx[ST3215_RX_BUF_SIZE];

    uint8_t tx_len = DrvSt3215_EncodeWriteByte(tx, s_st_ctx.servo_id,
                                               reg_addr, value);
    if (tx_len == 0U) {
        return ST3215_IO_BAD_FRAME;
    }

    AraStatus_t kr = BSP_UART_HalfDuplex_Transact(BSP_UART_ST3215, tx, tx_len,
                                                  rx, ST3215_ACK_FRAME_LEN);
    if (kr != ARA_OK) {
        DrvSt3215_NoteIoFail(&s_st_ctx, 0U);
        return DrvSt3215_ClassifyIoResult(&s_st_ctx, ST3215_PARSE_BAD_HEADER,
                                          false, tick_ms);
    }

    uint16_t rx_len = 0U;
    AraStatus_t rr  = BSP_UART_HD_WaitRx(BSP_UART_ST3215,
                                         ST3215_IO_TIMEOUT_MS_1M, &rx_len);
    if (rr == ARA_TIMEOUT) {
        BSP_UART_HD_AbortRx(BSP_UART_ST3215);
        DrvSt3215_NoteIoFail(&s_st_ctx, 0U);
        return DrvSt3215_ClassifyIoResult(&s_st_ctx, ST3215_PARSE_OK,
                                          true, tick_ms);
    }
    if (rr != ARA_OK) {
        DrvSt3215_NoteIoFail(&s_st_ctx, 0U);
        return DrvSt3215_ClassifyIoResult(&s_st_ctx, ST3215_PARSE_BAD_HEADER,
                                          false, tick_ms);
    }

    uint8_t err_bits = 0U;
    St3215_ParseResult_t pr = DrvSt3215_ParseAck(rx, (uint8_t)rx_len,
                                                 s_st_ctx.servo_id, &err_bits);
    if (pr == ST3215_PARSE_OK) {
        DrvSt3215_NoteIoOk(&s_st_ctx, tick_ms);
    } else {
        DrvSt3215_NoteIoFail(&s_st_ctx, err_bits);
    }
    return DrvSt3215_ClassifyIoResult(&s_st_ctx, pr, false, tick_ms);
}

static St3215_IoResult_t do_set_torque(bool enable, uint32_t tick_ms)
{
    return do_write_byte(ST3215_REG_TORQUE_ENABLE,
                         enable ? ST3215_TORQUE_ENABLE : ST3215_TORQUE_DISABLE,
                         tick_ms);
}

static St3215_IoResult_t do_write_pos(int16_t  target_steps,
                                      uint16_t speed,
                                      uint8_t  acc,
                                      uint32_t tick_ms)
{
    uint8_t tx[ST3215_TX_BUF_SIZE];
    uint8_t rx[ST3215_RX_BUF_SIZE];

    uint8_t tx_len = DrvSt3215_EncodeWritePos(tx, s_st_ctx.servo_id,
                                              target_steps, speed, acc);
    if (tx_len == 0U) {
        return ST3215_IO_BAD_FRAME;
    }

    AraStatus_t kr = BSP_UART_HalfDuplex_Transact(BSP_UART_ST3215, tx, tx_len,
                                                  rx, ST3215_ACK_FRAME_LEN);
    if (kr != ARA_OK) {
        DrvSt3215_NoteIoFail(&s_st_ctx, 0U);
        return DrvSt3215_ClassifyIoResult(&s_st_ctx, ST3215_PARSE_BAD_HEADER,
                                          false, tick_ms);
    }

    uint16_t rx_len = 0U;
    AraStatus_t rr  = BSP_UART_HD_WaitRx(BSP_UART_ST3215,
                                         ST3215_IO_TIMEOUT_MS_1M, &rx_len);
    if (rr == ARA_TIMEOUT) {
        BSP_UART_HD_AbortRx(BSP_UART_ST3215);
        DrvSt3215_NoteIoFail(&s_st_ctx, 0U);
        return DrvSt3215_ClassifyIoResult(&s_st_ctx, ST3215_PARSE_OK,
                                          true, tick_ms);
    }
    if (rr != ARA_OK) {
        DrvSt3215_NoteIoFail(&s_st_ctx, 0U);
        return DrvSt3215_ClassifyIoResult(&s_st_ctx, ST3215_PARSE_BAD_HEADER,
                                          false, tick_ms);
    }

    uint8_t err_bits = 0U;
    St3215_ParseResult_t pr = DrvSt3215_ParseAck(rx, (uint8_t)rx_len,
                                                 s_st_ctx.servo_id, &err_bits);
    if (pr == ST3215_PARSE_OK) {
        DrvSt3215_NoteIoOk(&s_st_ctx, tick_ms);
    } else {
        DrvSt3215_NoteIoFail(&s_st_ctx, err_bits);
    }
    return DrvSt3215_ClassifyIoResult(&s_st_ctx, pr, false, tick_ms);
}

static St3215_IoResult_t do_read_feedback(uint32_t           tick_ms,
                                          St3215_Feedback_t *out)
{
    uint8_t tx[ST3215_TX_BUF_SIZE];
    uint8_t rx[ST3215_RX_BUF_SIZE];

    uint8_t tx_len = DrvSt3215_EncodeReadFeedback(tx, s_st_ctx.servo_id);
    if (tx_len == 0U) {
        return ST3215_IO_BAD_FRAME;
    }

    AraStatus_t kr = BSP_UART_HalfDuplex_Transact(BSP_UART_ST3215, tx, tx_len,
                                                  rx, ST3215_FEEDBACK_FRAME_LEN);
    if (kr != ARA_OK) {
        DrvSt3215_NoteIoFail(&s_st_ctx, 0U);
        return DrvSt3215_ClassifyIoResult(&s_st_ctx, ST3215_PARSE_BAD_HEADER,
                                          false, tick_ms);
    }

    uint16_t rx_len = 0U;
    AraStatus_t rr  = BSP_UART_HD_WaitRx(BSP_UART_ST3215,
                                         ST3215_IO_TIMEOUT_MS_1M, &rx_len);
    if (rr == ARA_TIMEOUT) {
        BSP_UART_HD_AbortRx(BSP_UART_ST3215);
        DrvSt3215_NoteIoFail(&s_st_ctx, 0U);
        return DrvSt3215_ClassifyIoResult(&s_st_ctx, ST3215_PARSE_OK,
                                          true, tick_ms);
    }
    if (rr != ARA_OK) {
        DrvSt3215_NoteIoFail(&s_st_ctx, 0U);
        return DrvSt3215_ClassifyIoResult(&s_st_ctx, ST3215_PARSE_BAD_HEADER,
                                          false, tick_ms);
    }

    uint8_t err_bits = 0U;
    St3215_ParseResult_t pr = DrvSt3215_ParseFeedback(rx, (uint8_t)rx_len,
                                                      s_st_ctx.servo_id,
                                                      tick_ms, out, &err_bits);
    if (pr == ST3215_PARSE_OK) {
        DrvSt3215_NoteIoOk(&s_st_ctx, tick_ms);
        s_st_ctx.last_fb       = *out;
        s_st_ctx.last_fb_valid = true;
    } else {
        DrvSt3215_NoteIoFail(&s_st_ctx, err_bits);
    }
    return DrvSt3215_ClassifyIoResult(&s_st_ctx, pr, false, tick_ms);
}

void TaskMotion_Update(const MotionCmd_t *cmd,
                       uint32_t           tick_ms,
                       MotionState_t     *state)
{
    if ((cmd == NULL) || (state == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));

    int16_t target_steps = (int16_t)ST3215_DEG_TO_STEPS(
        clamp_angle_deg(cmd->target_angle_deg));

    /* Compute the deadband delta in int32 to avoid the int16 narrowing
     * trap: when s_last_written_steps == INT16_MIN (the sentinel before
     * the first successful write), target_steps - INT16_MIN = 32768,
     * which is unrepresentable as int16. Truncating that back to int16
     * silently flips it to INT16_MIN and breaks the very first write
     * detection. Doing the math in int32 keeps the diff exact, and
     * abs() on an int32 value is well-defined for everything except
     * INT32_MIN (which we never reach with int16 inputs). */
    int32_t target_i32  = (int32_t)target_steps;
    int32_t last_i32    = (int32_t)s_last_written_steps;
    int32_t diff_steps  = target_i32 - last_i32;
    if (diff_steps < 0) diff_steps = -diff_steps;

    bool changed   = (diff_steps > TASK_MOTION_DEADBAND_STEPS);
    bool keepalive = ((tick_ms - s_last_written_ms) > TASK_MOTION_KEEPALIVE_MS);
    bool need_write = cmd->torque_on && (changed || keepalive || cmd->force_keepalive);
    bool write_result_set = false;

    if (cmd->torque_on != s_torque_enabled) {
        state->last_write_result = do_set_torque(cmd->torque_on, tick_ms);
        write_result_set = true;
        if (state->last_write_result == ST3215_IO_OK) {
            s_torque_enabled = cmd->torque_on;
            if (!cmd->torque_on) {
                s_last_written_steps = INT16_MIN;
                s_last_written_ms    = 0U;
            }
        } else {
            need_write = false;
        }
    }

    if (need_write) {
        uint16_t speed = (cmd->target_speed != 0U) ? cmd->target_speed
                                                   : TASK_MOTION_DEFAULT_SPEED;
        uint8_t  acc   = (cmd->target_acc   != 0U) ? cmd->target_acc
                                                   : TASK_MOTION_DEFAULT_ACC;
        state->last_write_result = do_write_pos(target_steps, speed, acc, tick_ms);
        write_result_set = true;
        if (state->last_write_result == ST3215_IO_OK) {
            s_last_written_steps = target_steps;
            s_last_written_ms    = tick_ms;
        }
    } else if (!write_result_set) {
        state->last_write_result = ST3215_IO_OK;
    }

    /* Always read feedback. */
    state->last_read_result = do_read_feedback(tick_ms, &state->feedback);
    if (state->last_read_result == ST3215_IO_OK) {
        state->feedback_valid = true;
    } else if (s_st_ctx.last_fb_valid) {
        state->feedback       = s_st_ctx.last_fb;
        state->feedback_valid = false;
    }

    state->servo_online  = DrvSt3215_IsOnline(&s_st_ctx, tick_ms);
    state->motion_status = state->feedback_valid
        ? DrvSt3215_ClassifyMotion(&state->feedback, target_steps, NULL)
        : ST3215_MOTION_UNKNOWN;
}

#endif /* TASK_MOTION_USE_MOCK */
