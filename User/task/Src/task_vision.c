/**
 * @file task_vision.c
 * @brief Vision runnable for demo_v7.
 */

#include "task_vision.h"

#if TASK_VISION_USE_HC13
#include "bsp_uart.h"
#include "drv_hc13.h"
#include <stdio.h>
#endif

#include "FreeRTOS.h"
#include "task.h"

static struct {
    bool     mock_active;
    int16_t  mock_angle_deg;
    uint32_t mock_expires_ms;
    uint32_t mock_injected_ms;
} s_vision;

#if TASK_VISION_USE_HC13
/*
 * Return a low-rate application acknowledgement to the PC.  This confirms
 * that a complete V command passed the MCU parser; it does not claim that the
 * servo has already reached the requested angle.
 */
#define TASK_VISION_ACK_PERIOD_MS (200U)
static uint32_t s_last_ack_ms;
static uint32_t s_last_ack_frame;
#endif

void TaskVision_Init(void)
{
    taskENTER_CRITICAL();
    s_vision.mock_active      = false;
    s_vision.mock_angle_deg   = 0;
    s_vision.mock_expires_ms  = 0U;
    s_vision.mock_injected_ms = 0U;
    taskEXIT_CRITICAL();

#if TASK_VISION_USE_HC13
    DrvHC13_Init();
    s_last_ack_ms    = 0U;
    s_last_ack_frame = 0U;
#endif
}

void TaskVision_MockInjectTarget(int16_t  angle_deg,
                                 uint32_t duration_ms,
                                 uint32_t tick_ms)
{
    if (duration_ms == 0U) {
        duration_ms = 1000U; /* sensible default */
    }
    taskENTER_CRITICAL();
    s_vision.mock_active      = true;
    s_vision.mock_angle_deg   = angle_deg;
    s_vision.mock_injected_ms = tick_ms;
    s_vision.mock_expires_ms  = tick_ms + duration_ms;
    taskEXIT_CRITICAL();
}

void TaskVision_MockClear(void)
{
    taskENTER_CRITICAL();
    s_vision.mock_active = false;
    taskEXIT_CRITICAL();
}

void TaskVision_Update(uint32_t tick_ms, VisionIntent_t *out)
{
    if (out == NULL) {
        return;
    }

#if TASK_VISION_USE_HC13
    HC13VisionSample_t hc13;
    if (DrvHC13_PollVision(tick_ms, &hc13)) {
        const uint32_t frame_count = DrvHC13_GetVisionFrames();
        if ((frame_count != s_last_ack_frame) &&
            ((s_last_ack_frame == 0U) ||
             ((uint32_t)(tick_ms - s_last_ack_ms) >= TASK_VISION_ACK_PERIOD_MS))) {
            char ack[48];
            const int ack_len = snprintf(ack,
                                         sizeof(ack),
                                         "ACK,%lu,%d,%u\r\n",
                                         (unsigned long)frame_count,
                                         (int)hc13.target_angle_deg,
                                         (unsigned int)hc13.confidence);
            if ((ack_len > 0) && ((size_t)ack_len < sizeof(ack))) {
                BSP_UART_HC13_Send((const uint8_t *)ack, (uint16_t)ack_len);
                s_last_ack_ms    = tick_ms;
                s_last_ack_frame = frame_count;
            }
        }

        out->target_present       = hc13.target_present;
        out->target_angle_deg     = hc13.target_angle_deg;
        out->target_speed         = hc13.target_speed;
        out->confidence           = hc13.confidence;
        out->last_update_tick_ms  = hc13.timestamp_ms;
        return;
    }
#endif

    bool     active    = false;
    int16_t  angle     = 0;
    uint32_t injected  = 0U;
    uint32_t expires   = 0U;

    taskENTER_CRITICAL();
    active   = s_vision.mock_active;
    angle    = s_vision.mock_angle_deg;
    injected = s_vision.mock_injected_ms;
    expires  = s_vision.mock_expires_ms;
    taskEXIT_CRITICAL();

    /* Auto-expire. */
    if (active && ((int32_t)(tick_ms - expires) >= 0)) {
        TaskVision_MockClear();
        active = false;
    }

    if (active) {
        out->target_present       = true;
        out->target_angle_deg     = angle;
        out->target_speed         = 1000U;  /* default 1000 step/s */
        out->confidence           = 90U;
        out->last_update_tick_ms  = tick_ms;
    } else {
        out->target_present       = false;
        out->target_angle_deg     = 0;
        out->target_speed         = 0U;
        out->confidence           = 0U;
        out->last_update_tick_ms  = injected; /* preserve last */
    }
}
