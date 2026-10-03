#include "app.h"

#include "app_config.h"
#include "i2c.h"
#include "main.h"
#include "tofsense_f2p_i2c.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>

static TOFSense_F2P_I2C s_sensor;
static uint32_t s_successful_reads;
static uint32_t s_i2c_errors;
static HAL_StatusTypeDef s_last_i2c_status = HAL_OK;
static uint32_t s_last_read_tick;
static uint32_t s_last_poll_tick;
static uint32_t s_last_stats_tick;

static void Debug_Printf(const char *format, ...);
static void TOFSense_HandleResult(const TOFSense_F2P_I2C_Result *result);

void App_Init(void)
{
    if (!TOFSense_F2P_I2C_Init(&s_sensor, &hi2c1, TOFSENSE_I2C_ID)) {
        Error_Handler();
    }

    Debug_Printf("\r\nTOFSense-F2 P STM32F103 I2C monitor\r\n");
    Debug_Printf("sensor: I2C1 PB6/PB7 @ %lu Hz, id=%u, address=0x%02X\r\n",
                 (unsigned long)TOFSENSE_I2C_CLOCK_HZ,
                 TOFSENSE_I2C_ID,
                 (unsigned int)(s_sensor.hal_address >> 1U));
    Debug_Printf("debug : USART2 PA2/PA3 @ %lu\r\n",
                 (unsigned long)DEBUG_UART_BAUDRATE);

    s_last_i2c_status = TOFSense_F2P_I2C_IsReady(
        &s_sensor, 3U, TOFSENSE_I2C_TIMEOUT_MS);
    Debug_Printf("probe : %s\r\n",
                 s_last_i2c_status == HAL_OK ? "device ready" : "no ACK");
}

void App_Loop(void)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - s_last_poll_tick) >= TOFSENSE_POLL_PERIOD_MS) {
        TOFSense_F2P_I2C_Result result;
        s_last_poll_tick = now;
        s_last_i2c_status = TOFSense_F2P_I2C_Read(
            &s_sensor, &result, TOFSENSE_I2C_TIMEOUT_MS);

        if (s_last_i2c_status == HAL_OK) {
            s_successful_reads++;
            s_last_read_tick = HAL_GetTick();
            TOFSense_HandleResult(&result);
        } else {
            s_i2c_errors++;
        }
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_stats_tick) >= 1000U) {
        s_last_stats_tick = now;
        Debug_Printf("STAT reads=%lu i2c_err=%lu hal=%u last_age=%lu ms\r\n",
                     (unsigned long)s_successful_reads,
                     (unsigned long)s_i2c_errors,
                     (unsigned int)s_last_i2c_status,
                     (unsigned long)(now - s_last_read_tick));
    }
}

static void Debug_Printf(const char *format, ...)
{
    char buffer[192];
    va_list args;

    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length <= 0) {
        return;
    }
    if ((size_t)length >= sizeof(buffer)) {
        length = (int)sizeof(buffer) - 1;
    }
    (void)HAL_UART_Transmit(&huart2,
                            (uint8_t *)buffer,
                            (uint16_t)length,
                            DEBUG_UART_TIMEOUT_MS);
}

static void TOFSense_HandleResult(const TOFSense_F2P_I2C_Result *result)
{
    if (TOFSense_F2P_I2C_IsDistanceValid(result)) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    if ((TOFSENSE_DEBUG_DECIMATION == 0U)
        || ((s_successful_reads % TOFSENSE_DEBUG_DECIMATION) == 0U)) {
        Debug_Printf("F2P time=%lu ms distance=%ld mm status=%u(%s) signal=%u precision=%u cm rate=%u Hz filter=%u\r\n",
                     (unsigned long)result->system_time_ms,
                     (long)result->distance_mm,
                     result->distance_status,
                     TOFSense_F2P_I2C_IsDistanceValid(result) ? "VALID" : "INVALID",
                     result->signal_strength,
                     result->range_precision_cm,
                     result->refresh_rate_hz,
                     result->filter_factor);
    }
}
