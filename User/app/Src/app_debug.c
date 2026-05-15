/**
 * @file app_debug.c
 * @brief L5 DebugTask: Console input, DataHub status dump, LED render.
 *
 * Minimal skeleton for demo_v7. Existing richer menu logic in the legacy
 * app_testbench.c remains in the tree (compiled out via CMake EXCLUDE in a
 * later phase) to serve as reference while this file grows.
 */

#include "app_debug.h"
#include "app_control.h"
#include "datahub.h"
#include "debug_request.h"
#include "bsp_uart.h"
#include "dev_status.h"
#include "task_motion.h"   /* TASK_MOTION_ANGLE_MIN_DEG / MAX_DEG */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include <string.h>

static osThreadId_t s_debug_handle = NULL;

/* =============================================================================
 * Console input
 * ============================================================================= */

/* Force-mode bring-up state mirrored locally so we can apply ±/preset keys
 * without round-tripping through DataHub. Authoritative copy lives in
 * app_control via DBG_REQ_FORCE_GOTO_ANGLE; this is just an echo for the
 * input UI. */
static int16_t s_console_force_angle = 0;

/* Multi-char line buffer used by the 'g <angle>' command. The buffer
 * is only read/written from DebugTask, so no locking is needed. */
#define CONSOLE_LINE_MAX  16
static char     s_line_buf[CONSOLE_LINE_MAX];
static uint8_t  s_line_len = 0;
static bool     s_line_active = false;   /* true after the user presses 'g' */

static void console_print_help(void)
{
    BSP_UART_Printf(
        "[DBG] keys:\r\n"
        "  v=mock-vision 120deg/3s   c=clear-vision    f=clear-fault\r\n"
        "  e=force ON (lock 0deg)    k=force OFF\r\n"
        "  1=goto 0  2=goto 90  3=goto 180\r\n"
        "  +=+10deg  -=-10deg\r\n"
        "  g <deg>=force to arbitrary angle, eg 'g 47<enter>'\r\n"
        "  ?=this help\r\n");
}

static void console_send_force(int16_t angle_deg, bool enable)
{
    s_console_force_angle = angle_deg;
    DebugRequest_Post(DBG_REQ_FORCE_GOTO_ANGLE,
                      (int32_t)angle_deg,
                      enable ? 0 : -1);
    if (enable) {
        BSP_UART_Printf("[DBG] force ON, tgt=%d deg\r\n", (int)angle_deg);
    } else {
        BSP_UART_Printf("[DBG] force OFF\r\n");
    }
}

static int parse_signed_int(const char *s, uint8_t len, int32_t *out)
{
    if ((s == NULL) || (len == 0U)) return -1;
    uint8_t  i   = 0U;
    int32_t  acc = 0;
    int32_t  sign = 1;
    if (s[0] == '-') { sign = -1; i = 1U; }
    else if (s[0] == '+') { i = 1U; }
    if (i >= len) return -1;
    for (; i < len; i++) {
        if (s[i] < '0' || s[i] > '9') return -1;
        acc = acc * 10 + (s[i] - '0');
        if (acc > 100000) return -1;   /* overflow guard */
    }
    *out = sign * acc;
    return 0;
}

static void console_finish_line(void)
{
    /* The line is expected to start with 'g' followed by whitespace and
     * a signed decimal angle. Any deviation just resets the buffer with
     * a brief error. */
    if ((s_line_len < 2U) || (s_line_buf[0] != 'g')) {
        BSP_UART_Printf("[DBG] bad cmd, try 'g <angle>'\r\n");
    } else {
        uint8_t i = 1U;
        while ((i < s_line_len) && (s_line_buf[i] == ' ')) i++;
        int32_t deg;
        if ((i >= s_line_len) ||
            (parse_signed_int(&s_line_buf[i], (uint8_t)(s_line_len - i), &deg) != 0)) {
            BSP_UART_Printf("[DBG] bad angle, try 'g 47'\r\n");
        } else {
            if (deg < 0)   deg = 0;
            if (deg > 359) deg = 359;
            console_send_force((int16_t)deg, true);
        }
    }
    s_line_len    = 0U;
    s_line_active = false;
}

static void console_handle_line_char(char c)
{
    if ((c == '\r') || (c == '\n')) {
        console_finish_line();
        return;
    }
    if ((c == 0x08) || (c == 0x7F)) {  /* backspace / DEL */
        if (s_line_len > 0U) {
            s_line_len--;
            BSP_UART_Printf("\b \b");
        }
        return;
    }
    if (s_line_len < (CONSOLE_LINE_MAX - 1U)) {
        s_line_buf[s_line_len++] = c;
        /* Echo so the operator can see what they typed. */
        char echo[2] = { c, 0 };
        BSP_UART_Printf("%s", echo);
    }
}

static void console_poll(uint32_t tick_ms)
{
    uint8_t buf[16];
    uint16_t n = BSP_UART_Read(BSP_UART_DEBUG, buf, sizeof(buf));
    for (uint16_t i = 0; i < n; i++) {
        char c = (char)buf[i];

        if (s_line_active) {
            console_handle_line_char(c);
            continue;
        }

        switch (c) {
        case 'v':
            DebugRequest_Post(DBG_REQ_MOCK_VISION_TARGET, 120, 3000);
            BSP_UART_Printf("[DBG] mock vision target 120 deg / 3s\r\n");
            break;
        case 'c':
            DebugRequest_Post(DBG_REQ_MOCK_VISION_CLEAR, 0, 0);
            BSP_UART_Printf("[DBG] mock vision cleared\r\n");
            break;
        case 'f':
            DebugRequest_Post(DBG_REQ_CLEAR_FAULT, 0, 0);
            BSP_UART_Printf("[DBG] clear fault requested\r\n");
            break;
        case 'e': {
            /* Lock force mode at the servo's current physical position so
             * enabling force does NOT command an unwanted slew back to 0
             * deg. After 'e' the servo holds where it already is, then the
             * operator can step from there with +/-/1/2/3/g. */
            DataHub_t hub;
            DataHub_Read(&hub);
            int32_t cur = hub.servo_position_angle_deg;
            if (cur < TASK_MOTION_ANGLE_MIN_DEG) cur = TASK_MOTION_ANGLE_MIN_DEG;
            if (cur > TASK_MOTION_ANGLE_MAX_DEG) cur = TASK_MOTION_ANGLE_MAX_DEG;
            console_send_force((int16_t)cur, true);
            break;
        }
        case 'k':
            console_send_force(s_console_force_angle, false);
            break;
        case '1':
            console_send_force(0, true);
            break;
        case '2':
            console_send_force(90, true);
            break;
        case '3':
            console_send_force(180, true);
            break;
        case '+': {
            int32_t a = (int32_t)s_console_force_angle + 10;
            if (a > 359) a = 359;
            console_send_force((int16_t)a, true);
            break;
        }
        case '-': {
            int32_t a = (int32_t)s_console_force_angle - 10;
            if (a < 0) a = 0;
            console_send_force((int16_t)a, true);
            break;
        }
        case 'g':
            s_line_active = true;
            s_line_len    = 0U;
            s_line_buf[s_line_len++] = 'g';
            BSP_UART_Printf("g");
            break;
        case '?':
            console_print_help();
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
    int16_t force_angle = 0;
    bool    force_on    = App_Control_GetForceState(&force_angle);
    BSP_UART_Printf("[DBG] mode=%d arb=%d rc=%d vis=%d pos=%d tgt=%d load=%d%s\r\n",
                    (int)s.current_mode,
                    (int)s.arbiter_reason_code,
                    (int)s.rc_link_up,
                    (int)s.vision_link_up,
                    (int)s.servo_position_angle_deg,
                    (int)(force_on ? force_angle : s.servo_target_angle_deg),
                    (int)s.servo_load,
                    force_on ? " [FORCE]" : "");
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
