/**
 * @file test_rc_console.c
 * @brief RC Controller Modular & Integration Test Implementation
 */

#include "test_rc_console.h"
#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "task.h"

/* --- Includes for Modules to Test --- */
#include "drv_elrs.h"           // L2
#include "mod_rc_semantic.h"    // L3
#include "datahub.h"            // L4 Blackboard
#include "drv_bldc_power.h"     // L2 Actuator (For Integration Test)

/* --- Configuration --- */
#define CONSOLE_BUF_SIZE 64
#define TEST_MOTOR_DUTY  300    // Safe open-loop testing duty cycle
#define CONSOLE_INPUT_GUARD_MS 500U
#define CONSOLE_CMD_BUF_SIZE 8U

/* --- Global Instances --- */
static DrvElrs_Context_t       elrs_ctx;
static ModRcSemantic_Context_t semantic_ctx;
static DrvBldc_Context_t       bldc_ctx;

static uint8_t rx_buffer[CONSOLE_BUF_SIZE];
static bool    is_init = false;
static char    cmd_buffer[CONSOLE_CMD_BUF_SIZE];
static uint8_t cmd_len = 0;

/* --- Control Mode --- */
typedef enum {
    MODE_IDLE = 0,
    MODE_RAW_CHANNELS,   // 1. L2: Test CRSF Parsing
    MODE_SEMANTIC,       // 2. L3: Test Logic Translation
    MODE_DATAHUB,        // 3. L4: Test Blackboard & Failsafe
    MODE_MOTOR_LINK      // 4. SYS: RC -> Motor Control
} TestRcMode_e;

static TestRcMode_e current_mode = MODE_IDLE;
static RcControlData_t current_intent;
static uint32_t console_input_enable_tick = 0;

/* --- Prototypes --- */
static void PrintMenu(void);
static void ExecuteCommand(char cmd);
static void GenerateDefaultChannels(uint16_t *chs);
static bool IsValidCommand(char cmd);
static void PrintRxDebug(const uint8_t *buf, uint16_t len);
static void ProcessConsoleByte(char ch);

/* ==========================================
 * Init
 * ========================================== */
void TestRcConsole_Init(void) {
    uint8_t flush_buf[CONSOLE_BUF_SIZE];

    BSP_UART_Printf("\r\n=== ARA PLATFORM: RC INTEGRATION TEST ===\r\n");

    /* 1. Init DataHub */
    DataHub_Init();

    /* 2. Init L2: ELRS Driver */
    DrvElrs_Init(&elrs_ctx, BSP_UART_ELRS);

    /* 3. Init L3: Semantic Logic */
    uint16_t default_chs[DRV_ELRS_MAX_CHANNELS];
    GenerateDefaultChannels(default_chs);
    ModRcSemantic_Init(&semantic_ctx, default_chs);

    /* 4. Init Motor Power Stage (For Linkage Test) */
    DrvBldc_Init(&bldc_ctx, BSP_GPIO_MOTOR_EN);
    DrvBldc_Enable(&bldc_ctx, false);

    /* 5. Flush any startup garbage lingering in the DEBUG UART DMA buffer */
    while (BSP_UART_Read(BSP_UART_DEBUG, flush_buf, CONSOLE_BUF_SIZE) > 0) {
    }

    /* Ignore any serial noise during the first 500ms after boot */
    console_input_enable_tick = (uint32_t)xTaskGetTickCount() + CONSOLE_INPUT_GUARD_MS;

    is_init = true;
    PrintMenu();
}

/* ==========================================
 * Main Loop
 * ========================================== */
void TestRcConsole_TaskLoop(void) {
    if (!is_init) return;

    /* 1. Handle UART Commands */
    uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    uint16_t len = BSP_UART_Read(BSP_UART_DEBUG, rx_buffer, CONSOLE_BUF_SIZE);
    if (len > 0) {
        PrintRxDebug(rx_buffer, len);

        if (now_ms >= console_input_enable_tick) {
            for (uint16_t i = 0; i < len; i++) {
                char cmd = (char)rx_buffer[i];
                ProcessConsoleByte(cmd);
            }
        }
    }

    /* 2. Update L2 Hardware Driver (Process UART RingBuffer) */
    DrvElrs_Update(&elrs_ctx, now_ms);
    bool is_link_up = DrvElrs_IsLinkUp(&elrs_ctx);

    /* 3. Update L3 Semantic Logic */
    if (is_link_up) {
        ModRcSemantic_Process(&semantic_ctx, elrs_ctx.channels, now_ms, &current_intent);
        current_intent.is_link_up = true;
    } else {
        /* Manual Failsafe Generation for Test Console */
        current_intent.is_link_up = false;
        current_intent.req_mode = ARA_MODE_IDLE;
        current_intent.estop_state = ESTOP_ACTIVE;
        current_intent.arm_cmd = ARM_CMD_HOLD;
    }

    /* Write to DataHub for MODE_DATAHUB test */
    DataHub_WriteRcData(&current_intent);

    /* 4. Execute Mode Logic & Print Telemetry (Decimated) */
    static uint32_t print_ts = 0;
    bool should_print = (now_ms - print_ts > 200); // 5Hz Print Rate

    switch (current_mode) {
        case MODE_IDLE:
            DrvBldc_Enable(&bldc_ctx, false);
            break;

        case MODE_RAW_CHANNELS:
            if (should_print) {
                BSP_UART_Printf("[L2] Link:%d | CH5:%04d | CH6:%04d | CH7:%04d | CH10:%04d\r\n",
                                is_link_up,
                                elrs_ctx.channels[MOD_RC_IDX_SA],
                                elrs_ctx.channels[MOD_RC_IDX_SE],
                                elrs_ctx.channels[MOD_RC_IDX_SC],
                                elrs_ctx.channels[MOD_RC_IDX_SD]);
            }
            break;

        case MODE_SEMANTIC:
            if (should_print) {
                BSP_UART_Printf("[L3] ModeReq:%d | ArmCmd:%d | Grip:%d | EStop:%d | Pulse:%d | Aux:%d\r\n",
                                current_intent.req_mode,
                                current_intent.arm_cmd,
                                current_intent.gripper_cmd,
                                current_intent.estop_state,
                                current_intent.sys_reset_pulse,
                                current_intent.aux_knob_val);
            }
            break;

        case MODE_DATAHUB:
            if (should_print) {
                RcControlData_t dh_data;
                AraStatus_t s = DataHub_ReadRcData(&dh_data);
                BSP_UART_Printf("[L4] HubRead:%d | LinkUp:%d | EStop:%d\r\n",
                                s, dh_data.is_link_up, dh_data.estop_state);
            }
            break;

        case MODE_MOTOR_LINK:
            /* Integration Test: RC -> Motor */
            if (current_intent.estop_state == ESTOP_ACTIVE) {
                /* STRICT ESTOP: Kill power */
                DrvBldc_Enable(&bldc_ctx, false);
                BSP_PWM_StopAll();
                if (should_print) BSP_UART_Printf("[SYS] EMERGENY STOP ACTIVE!\r\n");
            }
            else if (current_intent.req_mode == ARA_MODE_MANUAL) {
                /* Manual Mode: Obey Arm Command */
                DrvBldc_Enable(&bldc_ctx, true);

                if (current_intent.arm_cmd == ARM_CMD_EXTEND) {
                    /* Spin Forward (Open-loop test) */
                    DrvBldc_SetDuties(&bldc_ctx, TEST_MOTOR_DUTY, 0, 0);
                    if (should_print) BSP_UART_Printf("[SYS] Motor: EXTENDING >>>\r\n");
                }
                else if (current_intent.arm_cmd == ARM_CMD_RETRACT) {
                    /* Spin Backward (Open-loop test) */
                    DrvBldc_SetDuties(&bldc_ctx, 0, TEST_MOTOR_DUTY, 0);
                    if (should_print) BSP_UART_Printf("[SYS] Motor: <<< RETRACTING (Pulse)\r\n");
                }
                else {
                    /* Hold */
                    DrvBldc_SetDuties(&bldc_ctx, 0, 0, 0);
                    if (should_print) BSP_UART_Printf("[SYS] Motor: HOLD\r\n");
                }
            }
            else {
                /* Auto Mode: Ignore Manual Arm Command */
                DrvBldc_SetDuties(&bldc_ctx, 0, 0, 0);
                if (should_print) BSP_UART_Printf("[SYS] Auto Mode: RC Ignored.\r\n");
            }
            break;
    }

    if (should_print) print_ts = now_ms;
}

/* ==========================================
 * Helper Functions
 * ========================================== */
static void PrintMenu(void) {
    BSP_UART_Printf("\r\n--- RC Integration Test Menu ---\r\n");
    BSP_UART_Printf("[r] L2: Raw CRSF Channels (Link Test)\r\n");
    BSP_UART_Printf("[s] L3: Semantic Intent (Logic Test)\r\n");
    BSP_UART_Printf("[d] L4: DataHub Failsafe Read (Hub Test)\r\n");
    BSP_UART_Printf("[m] SYS: Manual Motor Linkage (E-Stop + Arm Test)\r\n");
    BSP_UART_Printf("[i] Stop / Idle\r\n");
    BSP_UART_Printf("--------------------------------\r\n> ");
}

static void ExecuteCommand(char cmd) {
    switch (cmd) {
        case 'r':
            current_mode = MODE_RAW_CHANNELS;
            BSP_UART_Printf("MODE: RAW CHANNELS. Please move RC sticks.\r\n");
            break;
        case 's':
            current_mode = MODE_SEMANTIC;
            BSP_UART_Printf("MODE: SEMANTIC LOGIC. Test buttons and switches.\r\n");
            break;
        case 'd':
            current_mode = MODE_DATAHUB;
            BSP_UART_Printf("MODE: DATAHUB. Turn off RC transmitter to test Failsafe.\r\n");
            break;
        case 'm':
            current_mode = MODE_MOTOR_LINK;
            BSP_UART_Printf("MODE: SYSTEM LINKAGE. CAUTION: Motor will move!\r\n");
            break;
        case 'i':
            current_mode = MODE_IDLE;
            DrvBldc_Enable(&bldc_ctx, false);
            BSP_PWM_StopAll();
            BSP_UART_Printf("STOPPED.\r\n");
            break;
        case 'h': PrintMenu(); break;
        default: break;
    }
}

static bool IsValidCommand(char cmd) {
    switch (cmd) {
        case 'r':
        case 's':
        case 'd':
        case 'm':
        case 'i':
        case 'h':
            return true;
        default:
            return false;
    }
}

static void ProcessConsoleByte(char ch) {
    if (ch == '\r' || ch == '\n') {
        if (cmd_len == 1U && IsValidCommand(cmd_buffer[0])) {
            ExecuteCommand(cmd_buffer[0]);
        }
        cmd_len = 0;
        return;
    }

    if (ch == ' ' || ch == '\t') {
        return;
    }

    /* Discard non-printable / non-ASCII bytes (e.g. UART line noise) */
    if ((uint8_t)ch < 0x20U || (uint8_t)ch > 0x7EU) {
        return;
    }

    /* Direct single-key execution (for terminals without line terminator) */
    if (cmd_len == 0 && IsValidCommand(ch)) {
        ExecuteCommand(ch);
        return;
    }

    if (cmd_len < (CONSOLE_CMD_BUF_SIZE - 1U)) {
        cmd_buffer[cmd_len++] = ch;
        cmd_buffer[cmd_len] = '\0';
    } else {
        cmd_len = 0;
    }
}

static void PrintRxDebug(const uint8_t *buf, uint16_t len) {
    static uint32_t rx_packet_count = 0;
    uint16_t print_len = (len > 16U) ? 16U : len;

    BSP_UART_Printf("[RXDBG] pkt=%lu len=%u :", (unsigned long)(++rx_packet_count), len);
    for (uint16_t i = 0; i < print_len; i++) {
        BSP_UART_Printf(" %02X", buf[i]);
    }
    if (len > print_len) {
        BSP_UART_Printf(" ...");
    }
    BSP_UART_Printf(" |");
    for (uint16_t i = 0; i < print_len; i++) {
        char ch = (char)buf[i];
        if (ch >= 32 && ch <= 126) {
            BSP_UART_Printf("%c", ch);
        } else {
            BSP_UART_Printf(".");
        }
    }
    if (len > print_len) {
        BSP_UART_Printf("...");
    }
    BSP_UART_Printf("|\r\n");
}

static void GenerateDefaultChannels(uint16_t *chs) {
    for (int i = 0; i < DRV_ELRS_MAX_CHANNELS; i++) {
        chs[i] = MOD_RC_VAL_MID;
    }
    chs[MOD_RC_IDX_SD] = MOD_RC_SW_LOW - 100; // Default to E-STOP ACTIVE
}
