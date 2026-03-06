/**
 * @file test_console.c
 * @brief L4 Closed-Loop Control System (Final Integration)
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

// [PID 调参] 保守起步参数
// Q8.8 Format: 1.0 = 256.
// 建议初值: Kp=0.5 (128), Ki=0.01 (2), Kd=0
#define PID_KP           128
#define PID_KI           10
#define PID_KD           0
#define VOLT_LIMIT       6000  // Q15 输出限幅

/* --- Global Instances --- */
static DrvAS5600_Context_t as5600_ctx;
static DrvBldc_Context_t   bldc_ctx;
static AlgPid_Context_t    pid_vel_ctx;
static AlgFoc_Context_t    foc_ctx;

static SemaphoreHandle_t   sem_as5600_done = NULL;
static uint8_t rx_buffer[CONSOLE_BUF_SIZE];
static bool    is_init = false;

/* --- Runtime Variables --- */
static float    velocity_filtered = 0.0f; // rad/s
static uint16_t prev_angle_raw = 0;
static uint32_t prev_velocity_ts = 0;

/* --- Control Mode --- */
typedef enum {
    MODE_IDLE = 0,
    MODE_VELOCITY_CHECK, // 1. 手动拨动测速
    MODE_VOLTAGE_LOOP,   // 2. 电压/力矩模式
    MODE_SPEED_LOOP      // 3. 速度闭环模式
} ControlMode_e;

static ControlMode_e current_mode = MODE_IDLE;
static float         target_val = 0.0f; // 目标电压或目标速度
static int16_t       last_uq_cmd = 0;

/* --- Prototypes --- */
static void AS5600_Callback_ISR(void);
static void Auto_Align_Zero(void);
static void Update_Velocity(uint16_t current_raw);
static void Loop_Control_Task(void);
static void PrintMenu(void);
static void ExecuteCommand(char cmd);

/* ==========================================
 * Init
 * ========================================== */
void TestConsole_Init(void) {
    BSP_UART_Printf("\r\n=== ARA PLATFORM: L4 CLOSED LOOP ===\r\n");

    sem_as5600_done = xSemaphoreCreateBinary();

    // 1. 初始化驱动
    DrvAS5600_Init(&as5600_ctx, BSP_I2C_MOTION, AS5600_Callback_ISR);
    DrvBldc_Init(&bldc_ctx, BSP_GPIO_MOTOR_EN);

    // 2. 初始化算法
    AlgPid_Init(&pid_vel_ctx);
    AlgPid_SetGains(&pid_vel_ctx, PID_KP, PID_KI, PID_KD, VOLT_LIMIT, 2000000); // MaxOut, MaxInt

    AlgFoc_Init(&foc_ctx, POLE_PAIRS, PWM_PERIOD);

    // 3. 自动对齐并记录零点偏移
    Auto_Align_Zero();

    is_init = true;
    PrintMenu();
}

void TestConsole_TaskLoop(void) {
    // 1. 处理串口命令
    uint16_t len = BSP_UART_Read(BSP_UART_DEBUG, rx_buffer, CONSOLE_BUF_SIZE);
    if (len > 0) {
        for (uint16_t i = 0; i < len; i++) ExecuteCommand((char)rx_buffer[i]);
    }

    // 2. 主控制循环
    if (is_init) {
        Loop_Control_Task();
    }
}

/* ==========================================
 * Control Logic
 * ========================================== */

// 自动进行传感器零点对齐
static void Auto_Align_Zero(void) {
    BSP_UART_Printf("[Align] Aligning Sensor Zero...\r\n");

    // 将电压矢量强制指向电角度 0。
    // 对 FOC 而言，就是 Angle=0, Uq=0, Ud=对齐电压。
    DrvBldc_Enable(&bldc_ctx, true);
    AlgFoc_Run(&foc_ctx, 0, 0, 4000);
    DrvBldc_SetDuties(&bldc_ctx, foc_ctx.duty_a, foc_ctx.duty_b, foc_ctx.duty_c);

    // 等待转子稳定。
    HAL_Delay(700);

    // 读取当前角度，作为零点偏移。
    DrvAS5600_TriggerUpdate(&as5600_ctx);
    HAL_Delay(2); // Wait for DMA
    uint16_t zero_pos = DrvAS5600_GetRawAngle(&as5600_ctx);

    // 保存零点偏移。
    AlgFoc_SetZeroOffset(&foc_ctx, zero_pos);

    // 释放电机。
    DrvBldc_SetDuties(&bldc_ctx, 0, 0, 0);
    DrvBldc_Enable(&bldc_ctx, false);

    BSP_UART_Printf("[Align] Done. Zero Offset = %d\r\n", zero_pos);

    // 重置速度估计状态。
    prev_angle_raw = zero_pos;
    prev_velocity_ts = HAL_GetTick();
}

// 速度估计，带一个简单的一阶低通滤波
static void Update_Velocity(uint16_t current_raw) {
    uint32_t now = HAL_GetTick();
    float dt = (now - prev_velocity_ts) * 0.001f;
    if (dt <= 0.0f) return;

    // 计算 0-4095 编码器范围内的回绕角度差。
    int32_t delta = (int32_t)current_raw - (int32_t)prev_angle_raw;
    if (delta > 2048)  delta -= 4096;
    if (delta < -2048) delta += 4096;

    // 将编码器增量换算为弧度。
    float delta_rad = delta * 0.001534f;

    // 原始角速度。
    float raw_vel = delta_rad / dt;

    // 一阶低通滤波。
    // 系数越小，滤波越强，但延迟也越大。
    velocity_filtered = 0.9f * velocity_filtered + 0.1f * raw_vel;

    prev_angle_raw = current_raw;
    prev_velocity_ts = now;
}

static uint8_t sensor_error_count = 0;
static uint8_t sensor_busy_count = 0;
static uint8_t sensor_dma_error_count = 0;

static void Loop_Control_Task(void) {
    // 1. 启动传感器 DMA 读取，并获取底层状态。
    AraStatus_t trig_status = DrvAS5600_TriggerUpdate(&as5600_ctx);

    if (trig_status != ARA_OK) {
        // ARA_BUSY 表示上一次 DMA 传输还没结束，本周期直接跳过。
        if (trig_status == ARA_BUSY) {
            sensor_busy_count++;
            if (sensor_busy_count >= 50) {
                BSP_UART_Printf("[Warn] Sensor busy for %d cycles\r\n", sensor_busy_count);
                sensor_busy_count = 0;
            }
            return;
        }

        // ARA_ERR_DMA 表示 DMA 启动失败，尝试做一次 I2C 软恢复。
        if (trig_status == ARA_ERR_DMA) {
            sensor_dma_error_count++;
            if (sensor_dma_error_count >= 5) {
                BSP_UART_Printf("[Warn] Sensor DMA start failed, recovering I2C\r\n");
                BSP_I2C_Init();
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

    // 2. 等待 DMA 完成信号量。
    if (xSemaphoreTake(sem_as5600_done, pdMS_TO_TICKS(10)) != pdTRUE) {
        // DMA 已启动，但完成中断没有回来。
        sensor_error_count++;
        if (sensor_error_count > 5) {
            BSP_UART_Printf("[Err] Sensor DMA Timeout! Interrupt missing?\r\n");
            sensor_error_count = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        return;
    }
    sensor_error_count = 0;

    // 3. 读取最新传感器数据。
    uint16_t raw_angle = DrvAS5600_GetRawAngle(&as5600_ctx);

    // 4. 更新速度估计。
    Update_Velocity(raw_angle);

    // 5. 执行控制模式状态机。
    switch (current_mode) {
        case MODE_IDLE:
            if (bldc_ctx.is_enabled) DrvBldc_Enable(&bldc_ctx, false);
            last_uq_cmd = 0;
            break;

        case MODE_VELOCITY_CHECK:
            // 只读取不驱动，用于手动拨动测速。
            if (bldc_ctx.is_enabled) DrvBldc_Enable(&bldc_ctx, false);
            last_uq_cmd = 0;
            break;

        case MODE_VOLTAGE_LOOP: // 电压/力矩模式
            if (!bldc_ctx.is_enabled) DrvBldc_Enable(&bldc_ctx, true);
            // 直接给定 Uq，并保持 Ud 为 0。
            // 这里使用传感器实测角度，不是开环累加角度。
            last_uq_cmd = (int16_t)target_val;
            AlgFoc_Run(&foc_ctx, raw_angle, last_uq_cmd, 0);
            DrvBldc_SetDuties(&bldc_ctx, foc_ctx.duty_a, foc_ctx.duty_b, foc_ctx.duty_c);
            break;

        case MODE_SPEED_LOOP: // 速度闭环模式
            if (!bldc_ctx.is_enabled) DrvBldc_Enable(&bldc_ctx, true);

            // PID: 目标速度与实际速度比较，输出 Uq 指令。
            // 当前先按 1 rad/s -> 10 内部单位做缩放。
            int16_t t_vel_int = (int16_t)(target_val * 10.0f);
            int16_t m_vel_int = (int16_t)(velocity_filtered * 10.0f);

            int16_t u_q_cmd = AlgPid_Compute(&pid_vel_ctx, t_vel_int, m_vel_int);
            last_uq_cmd = u_q_cmd;

            // 用最新 Uq 指令执行 FOC。
            AlgFoc_Run(&foc_ctx, raw_angle, u_q_cmd, 0);
            DrvBldc_SetDuties(&bldc_ctx, foc_ctx.duty_a, foc_ctx.duty_b, foc_ctx.duty_c);
            break;
    }

    // 6. 降频输出遥测信息。
    static uint32_t print_ts = 0;
    if (HAL_GetTick() - print_ts > 200) {
        print_ts = HAL_GetTick();
        if (current_mode != MODE_IDLE) {
            // 打印模式、目标值、实际速度、电角度和 Uq。
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
    if (sem_as5600_done != NULL) xSemaphoreGiveFromISR(sem_as5600_done, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void PrintMenu(void) {
    BSP_UART_Printf("\r\n--- L4 Control Menu ---\r\n");
    BSP_UART_Printf("[v] Check Velocity Signal (Hand Spin)\r\n");
    BSP_UART_Printf("[t] Torque/Voltage Mode (Uq=400)\r\n");
    BSP_UART_Printf("[c] Closed Loop Speed (Target=10.0)\r\n");
    BSP_UART_Printf("[s] Stop / Idle\r\n");
    BSP_UART_Printf("[z] Re-Align Zero\r\n");
    BSP_UART_Printf("-----------------------\r\n> ");
}

static void ExecuteCommand(char cmd) {
    switch (cmd) {
        case 'v':
            current_mode = MODE_VELOCITY_CHECK;
            target_val = 0;
            BSP_UART_Printf("MODE: Velocity Check. Spin motor by hand.\r\n");
            break;

        case 't':
            current_mode = MODE_VOLTAGE_LOOP;
            target_val = 3000.0f; // 约等效 3.3V
            // 重置 PID 状态，避免历史积分残留。
            AlgPid_Reset(&pid_vel_ctx);
            BSP_UART_Printf("MODE: Voltage FOC. Uq=3.1V. Motor should accelerate.\r\n");
            break;

        case 'c':
            current_mode = MODE_SPEED_LOOP;
            target_val = 10.0f; // 默认目标: 10 rad/s
            AlgPid_Reset(&pid_vel_ctx);
            BSP_UART_Printf("MODE: Speed Loop. Target=10.0 rad/s\r\n");
            break;

        case 's':
            current_mode = MODE_IDLE;
            target_val = 0;
            last_uq_cmd = 0;
            DrvBldc_Enable(&bldc_ctx, false);
            BSP_PWM_StopAll();
            BSP_UART_Printf("STOPPED.\r\n");
            break;

        case 'z':
            Auto_Align_Zero();
            break;

        case 'h':
            PrintMenu();
            break;

        default:
            break;
    }
}