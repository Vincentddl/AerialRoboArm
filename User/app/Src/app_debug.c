/**
 * @file app_debug.c
 * @brief L5 DebugTask: Console input, DataHub status dump, LED render.
 *
 * Minimal skeleton for demo_v7. Existing richer menu logic in the legacy
 * app_testbench.c remains in the tree (compiled out via CMake EXCLUDE in a
 * later phase) to serve as reference while this file grows.
 */

#include "app_debug.h"
#include "datahub.h"
#include "debug_request.h"
#include "bsp_uart.h"
#include "dev_status.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include <string.h>

static osThreadId_t s_debug_handle = NULL;

/* =============================================================================
 * Console input
 * ============================================================================= */

static void console_poll(uint32_t tick_ms)
{
    uint8_t buf[16];
    uint16_t n = BSP_UART_Read(BSP_UART_DEBUG, buf, sizeof(buf));
    for (uint16_t i = 0; i < n; i++) {
        switch (buf[i]) {
        case 'v': /* mock vision: 120 deg for 3 seconds */
            DebugRequest_Post(DBG_REQ_MOCK_VISION_TARGET, 120, 3000);
            BSP_UART_Printf("[DBG] mock vision target 120 deg / 3s\r\n");
            break;
        case 'c': /* clear mock vision */
            DebugRequest_Post(DBG_REQ_MOCK_VISION_CLEAR, 0, 0);
            BSP_UART_Printf("[DBG] mock vision cleared\r\n");
            break;
        case 'f': /* clear fault */
            DebugRequest_Post(DBG_REQ_CLEAR_FAULT, 0, 0);
            BSP_UART_Printf("[DBG] clear fault requested\r\n");
            break;
        case '?':
            BSP_UART_Printf("[DBG] keys: v=mock-vision, c=clear-vision, f=clear-fault\r\n");
            break;
        default:
            break;
        }
    }
    (void)tick_ms;
}

/* =============================================================================
 * LED render from DataHub pattern
 * ============================================================================= */

static void led_render(uint32_t tick_ms)
{
    AraLedPattern_t p = DataHub_GetLedPattern();
    bool on = false;
    switch (p) {
    case LED_PATTERN_OFF:               on = false; break;
    case LED_PATTERN_BOOT_FAST_BLINK:   on = ((tick_ms / 100U) & 1U); break;
    case LED_PATTERN_IDLE_SLOW_BLINK:   on = ((tick_ms / 500U) & 1U); break;
    case LED_PATTERN_MANUAL_HEARTBEAT:  on = ((tick_ms % 1000U) < 80U); break;
    case LED_PATTERN_AUTO_SOLID:        on = true; break;
    case LED_PATTERN_ERROR_SOS:         on = ((tick_ms / 150U) & 1U); break;
    default:                             on = false; break;
    }
    DevStatus_LedSet(on);
}

/* =============================================================================
 * Periodic snapshot print
 * ============================================================================= */

static uint32_t s_snapshot_print_counter = 0U;

static void periodic_snapshot(void)
{
    s_snapshot_print_counter++;
    if ((s_snapshot_print_counter % 10U) != 0U) {
        return; /* print once per second */
    }
    DataHub_t s;
    DataHub_Read(&s);
    BSP_UART_Printf("[DBG] mode=%d arb=%d rc=%d vis=%d pos=%d tgt=%d load=%d\r\n",
                    (int)s.current_mode,
                    (int)s.arbiter_reason_code,
                    (int)s.rc_link_up,
                    (int)s.vision_link_up,
                    (int)s.servo_position_angle_deg,
                    (int)s.servo_target_angle_deg,
                    (int)s.servo_load);
}

/* =============================================================================
 * Task body
 * ============================================================================= */

static void DebugTaskEntry(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    BSP_UART_Printf("\r\n[DBG] ARA demo_v7 up. press '?' for help\r\n");

    for (;;) {
        const uint32_t now_ms = osKernelGetTickCount();
        console_poll(now_ms);
        periodic_snapshot();
        led_render(now_ms);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ARA_PERIOD_DEBUG_MS));
    }
}

/* =============================================================================
 * Lifecycle
 * ============================================================================= */

void App_Debug_InitDeps(void)
{
    /* Nothing pre-scheduler for now. */
}

void App_Debug_Init(void)
{
    const osThreadAttr_t attr = {
        .name       = "DebugTask",
        .stack_size = ARA_STACK_DEBUG_BYTES,
        .priority   = ARA_PRIO_DEBUG,
    };
    s_debug_handle = osThreadNew(DebugTaskEntry, NULL, &attr);
}
