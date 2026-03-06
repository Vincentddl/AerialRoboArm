/**
 * @file test_console.c
 * @brief L4 closed-loop control test console.
 */

#include "test_console.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* --- Includes --- */
#include "drv_as5600.h"
#include "drv_bldc_power.h"
#include "alg_pid.h"
#include "alg_voltage_foc.h"

/* --- Configuration --- */
#define CONSOLE_BUF_SIZE 64
#define POLE_PAIRS       7
#define PWM_PERIOD       1440

/* Conservative starting values for the 2808 motor. */
/* Q8.8 format: 1.0 = 256. */
#define PID_KP            192
#define PID_KI            8
#define PID_KD            0
#define VOLT_LIMIT        7000
#define ALIGN_VOLTAGE_Q15 4500
#define TORQUE_TEST_UQ    4500
#define UQ_RAMP_STEP      15

/* --- Global Instances --- */
static DrvAS5600_Context_t as5600_ctx;
static DrvBldc_Context_t   bldc_ctx;
static AlgPid_Context_t    pid_vel_ctx;
static AlgFoc_Context_t    foc_ctx;

static SemaphoreHandle_t sem_as5600_done = NULL;
static uint8_t rx_buffer[CONSOLE_BUF_SIZE];
static bool    is_init = false;

/* --- Runtime Variables --- */
static float    velocity_filtered = 0.0f; /* rad/s */
static uint16_t prev_angle_raw = 0;
static uint32_t prev_velocity_ts = 0;

/* --- Control Mode --- */
typedef enum {
    MODE_IDLE = 0,
    MODE_VELOCITY_CHECK,
    MODE_VOLTAGE_LOOP,
    MODE_SPEED_LOOP
} ControlMode_e;

static ControlMode_e current_mode = MODE_IDLE;
static float         target_val = 0.0f; /* voltage command or speed target */
static int16_t       last_uq_cmd = 0;

/* --- Prototypes --- */
static void AS5600_Callback_ISR(void);
static void Auto_Align_Zero(void);
static void Update_Velocity(uint16_t current_raw);
static void Loop_Control_Task(void);
static void PrintMenu(void);
static void ExecuteCommand(char cmd);
static void ResetSensorRecoveryState(void);
static int16_t RampTowards(int16_t current, int16_t target, int16_t step);

/* ==========================================
 * Init
 * ========================================== */
void TestConsole_Init(void) {
    BSP_UART_Printf("\r\n=== ARA PLATFORM: L4 CLOSED LOOP ===\r\n");

    sem_as5600_done = xSemaphoreCreateBinary();

    /* 1. Driver init */
    DrvAS5600_Init(&as5600_ctx, BSP_I2C_MOTION, AS5600_Callback_ISR);
    DrvBldc_Init(&bldc_ctx, BSP_GPIO_MOTOR_EN);

    /* 2. Algorithm init */
    AlgPid_Init(&pid_vel_ctx);
    AlgPid_SetGains(&pid_vel_ctx, PID_KP, PID_KI, PID_KD, VOLT_LIMIT, 2000000);
    AlgFoc_Init(&foc_ctx, POLE_PAIRS, PWM_PERIOD);

    /* 3. Automatic zero alignment */
    Auto_Align_Zero();

    is_init = true;
    PrintMenu();
}

void TestConsole_TaskLoop(void) {
    uint16_t len = BSP_UART_Read(BSP_UART_DEBUG, rx_buffer, CONSOLE_BUF_SIZE);
    if (len > 0) {
        for (uint16_t i = 0; i < len; i++) {
            ExecuteCommand((char)rx_buffer[i]);
        }
    }

    if (is_init) {
        Loop_Control_Task();
    }
}

/* ==========================================
 * Control Logic
 * ========================================== */

/* Force the rotor to a known electrical angle and record sensor zero. */
static void Auto_Align_Zero(void) {
    BSP_UART_Printf("[Align] Aligning Sensor Zero...\r\n");

    DrvBldc_Enable(&bldc_ctx, true);
    AlgFoc_Run(&foc_ctx, 0, 0, ALIGN_VOLTAGE_Q15);
    DrvBldc_SetDuties(&bldc_ctx, foc_ctx.duty_a, foc_ctx.duty_b, foc_ctx.duty_c);

    HAL_Delay(700);

    DrvAS5600_TriggerUpdate(&as5600_ctx);
    HAL_Delay(2);
    uint16_t zero_pos = DrvAS5600_GetRawAngle(&as5600_ctx);

    AlgFoc_SetZeroOffset(&foc_ctx, zero_pos);

    DrvBldc_SetDuties(&bldc_ctx, 0, 0, 0);
    DrvBldc_Enable(&bldc_ctx, false);

    BSP_UART_Printf("[Align] Done. Zero Offset = %d\r\n", zero_pos);

    prev_angle_raw = zero_pos;
    prev_velocity_ts = HAL_GetTick();
}

/* Estimate shaft speed with simple first-order low-pass filtering. */
static void Update_Velocity(uint16_t current_raw) {
    uint32_t now = HAL_GetTick();
    float dt = (now - prev_velocity_ts) * 0.001f;
    if (dt <= 0.0f) {
        return;
    }

    int32_t delta = (int32_t)current_raw - (int32_t)prev_angle_raw;
    if (delta > 2048) {
        delta -= 4096;
    }
    if (delta < -2048) {
        delta += 4096;
    }

    float delta_rad = delta * 0.001534f;
    float raw_vel = delta_rad / dt;

    velocity_filtered = 0.9f * velocity_filtered + 0.1f * raw_vel;

    prev_angle_raw = current_raw;
    prev_velocity_ts = now;
}

static uint8_t sensor_error_count = 0;
static uint8_t sensor_busy_count = 0;
static uint8_t sensor_dma_error_count = 0;

static void Loop_Control_Task(void) {
    if (current_mode == MODE_IDLE) {
        return;
    }

    AraStatus_t trig_status = DrvAS5600_TriggerUpdate(&as5600_ctx);

    if (trig_status != ARA_OK) {
        if (trig_status == ARA_BUSY) {
            sensor_busy_count++;
            if (sensor_busy_count >= 50) {
                BSP_UART_Printf("[Warn] Sensor busy for %d cycles\r\n", sensor_busy_count);
                sensor_busy_count = 0;
            }
            return;
        }

        if (trig_status == ARA_ERR_DMA) {
            sensor_dma_error_count++;
            if (sensor_dma_error_count >= 5) {
                BSP_UART_Printf("[Warn] Sensor DMA start failed, recovering I2C (HAL=0x%08lX)\r\n", BSP_I2C_GetLastError());
                BSP_I2C_Init();
                BSP_I2C_ClearLastError();
                sensor_dma_error_count = 0;
            }
            return;
        }

        sensor_busy_count = 0;
        sensor_dma_error_count = 0;
        BSP_UART_Printf("[Err] Sensor Trigger Failed! Code: %d\r\n", trig_status);
        vTaskDelay(pdMS_TO_TICKS(200));
        return;
    }
    sensor_busy_count = 0;
    sensor_dma_error_count = 0;

    if (xSemaphoreTake(sem_as5600_done, pdMS_TO_TICKS(10)) != pdTRUE) {
        sensor_error_count++;
        if (sensor_error_count > 5) {
            BSP_UART_Printf("[Err] Sensor DMA Timeout! Interrupt missing?\r\n");
            sensor_error_count = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        return;
    }
    sensor_error_count = 0;

    uint16_t raw_angle = DrvAS5600_GetRawAngle(&as5600_ctx);
    Update_Velocity(raw_angle);

    switch (current_mode) {
        case MODE_IDLE:
            if (bldc_ctx.is_enabled) {
                DrvBldc_Enable(&bldc_ctx, false);
            }
            last_uq_cmd = 0;
            break;

        case MODE_VELOCITY_CHECK:
            if (bldc_ctx.is_enabled) {
                DrvBldc_Enable(&bldc_ctx, false);
            }
            last_uq_cmd = 0;
            break;

        case MODE_VOLTAGE_LOOP:
            if (!bldc_ctx.is_enabled) {
                DrvBldc_Enable(&bldc_ctx, true);
            }
            /* High-torque test mode with a slow Uq ramp. */
            last_uq_cmd = RampTowards(last_uq_cmd, (int16_t)target_val, UQ_RAMP_STEP);
            AlgFoc_Run(&foc_ctx, raw_angle, last_uq_cmd, 0);
            DrvBldc_SetDuties(&bldc_ctx, foc_ctx.duty_a, foc_ctx.duty_b, foc_ctx.duty_c);
            break;

        case MODE_SPEED_LOOP: {
            if (!bldc_ctx.is_enabled) {
                DrvBldc_Enable(&bldc_ctx, true);
            }

            int16_t t_vel_int = (int16_t)(target_val * 10.0f);
            int16_t m_vel_int = (int16_t)(velocity_filtered * 10.0f);
            int16_t u_q_cmd = AlgPid_Compute(&pid_vel_ctx, t_vel_int, m_vel_int);

            last_uq_cmd = RampTowards(last_uq_cmd, u_q_cmd, UQ_RAMP_STEP);
            AlgFoc_Run(&foc_ctx, raw_angle, last_uq_cmd, 0);
            DrvBldc_SetDuties(&bldc_ctx, foc_ctx.duty_a, foc_ctx.duty_b, foc_ctx.duty_c);
            break;
        }
    }

    static uint32_t print_ts = 0;
    if (HAL_GetTick() - print_ts > 200) {
        print_ts = HAL_GetTick();
        if (current_mode != MODE_IDLE) {
            BSP_UART_Printf("M:%d | Tgt:%.1f | Vel:%.2f | Ang:%d | Uq:%d\r\n",
                            current_mode, target_val, velocity_filtered, foc_ctx.electric_angle,
                            last_uq_cmd);
        }
    }
}

/* ==========================================
 * Helper
 * ========================================== */
static void AS5600_Callback_ISR(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (sem_as5600_done != NULL) {
        xSemaphoreGiveFromISR(sem_as5600_done, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void ResetSensorRecoveryState(void) {
    sensor_error_count = 0;
    sensor_busy_count = 0;
    sensor_dma_error_count = 0;
    BSP_I2C_Init();
}

static int16_t RampTowards(int16_t current, int16_t target, int16_t step) {
    if (current < target) {
        current += step;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= step;
        if (current < target) {
            current = target;
        }
    }
    return current;
}

static void PrintMenu(void) {
    BSP_UART_Printf("\r\n--- L4 Control Menu ---\r\n");
    BSP_UART_Printf("[v] Check Velocity Signal (Hand Spin)\r\n");
    BSP_UART_Printf("[t] High Torque Test (Ramp to Uq=4500)\r\n");
    BSP_UART_Printf("[c] Closed Loop Speed (Target=5.0)\r\n");
    BSP_UART_Printf("[s] Stop / Idle\r\n");
    BSP_UART_Printf("[z] Re-Align Zero\r\n");
    BSP_UART_Printf("-----------------------\r\n> ");
}

static void ExecuteCommand(char cmd) {
    switch (cmd) {
        case 'v':
            ResetSensorRecoveryState();
            current_mode = MODE_VELOCITY_CHECK;
            target_val = 0;
            last_uq_cmd = 0;
            BSP_UART_Printf("MODE: Velocity Check. Spin motor by hand.\r\n");
            break;

        case 't':
            ResetSensorRecoveryState();
            current_mode = MODE_VOLTAGE_LOOP;
            target_val = TORQUE_TEST_UQ;
            last_uq_cmd = 0;
            AlgPid_Reset(&pid_vel_ctx);
            BSP_UART_Printf("MODE: High Torque Test. Uq ramps to %d.\r\n", TORQUE_TEST_UQ);
            break;

        case 'c':
            ResetSensorRecoveryState();
            current_mode = MODE_SPEED_LOOP;
            target_val = 5.0f;
            last_uq_cmd = 0;
            AlgPid_Reset(&pid_vel_ctx);
            BSP_UART_Printf("MODE: Speed Loop. Target=5.0 rad/s\r\n");
            break;

        case 's':
            current_mode = MODE_IDLE;
            target_val = 0;
            last_uq_cmd = 0;
            DrvBldc_Enable(&bldc_ctx, false);
            BSP_PWM_StopAll();
            ResetSensorRecoveryState();
            BSP_UART_Printf("STOPPED.\r\n");
            break;

        case 'z':
            current_mode = MODE_IDLE;
            target_val = 0;
            last_uq_cmd = 0;
            AlgPid_Reset(&pid_vel_ctx);
            ResetSensorRecoveryState();
            Auto_Align_Zero();
            BSP_UART_Printf("MODE: Idle after re-align.\r\n");
            break;

        case 'h':
            PrintMenu();
            break;

        default:
            break;
    }
}
