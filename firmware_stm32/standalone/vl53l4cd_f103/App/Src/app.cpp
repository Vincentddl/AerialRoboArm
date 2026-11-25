#include "app.h"

#include "Arduino.h"
#include "Wire.h"
#include "app_config.h"
#include "i2c.h"
#include "main.h"
#include "usart.h"
#include "vl53l4cd_class.h"

#include <cstdarg>
#include <cstdio>

static TwoWire s_wire(&hi2c1);
static VL53L4CD s_sensor(&s_wire, PIN_VL53_XSHUT);
static bool s_running;
static uint32_t s_measurements;
static uint32_t s_valid_results;
static uint32_t s_warning_results;
static uint32_t s_driver_errors;
static uint32_t s_range_errors;
static uint32_t s_last_measurement_tick;
static uint32_t s_last_poll_tick;
static uint32_t s_last_retry_tick;
static uint32_t s_last_stats_tick;
static VL53L4CD_ERROR s_last_status = VL53L4CD_ERROR_NONE;
static uint16_t s_sensor_id;

enum class RangeQuality : uint8_t {
    Valid,
    Warning,
    Error
};

static void Debug_Printf(const char *format, ...);
static bool Sensor_Start(void);
static RangeQuality Range_GetQuality(uint8_t status);
static const char *Range_GetQualityText(RangeQuality quality);
static const char *Range_GetStatusText(uint8_t status);

extern "C" void App_Init(void)
{
    Debug_Printf("\r\nVL53L4CD STM32F103 proximity monitor\r\n");
    Debug_Printf("I2C1 PB6/PB7 @ 400 kHz, address=0x%02X (7-bit 0x%02X)\r\n",
                 VL53L4CD_I2C_ADDRESS_8BIT,
                 VL53L4CD_I2C_ADDRESS_8BIT >> 1U);
    Debug_Printf("XSHUT=PB0, GPIO1=PB1, USART2=115200\r\n");

    s_wire.begin();
    s_sensor.begin();
    s_running = Sensor_Start();
}

extern "C" void App_Loop(void)
{
    uint32_t now = HAL_GetTick();

    if (!s_running) {
        if ((uint32_t)(now - s_last_retry_tick) >= VL53L4CD_RETRY_PERIOD_MS) {
            s_last_retry_tick = now;
            s_running = Sensor_Start();
        }
    } else if ((uint32_t)(now - s_last_poll_tick) >= VL53L4CD_POLL_PERIOD_MS) {
        s_last_poll_tick = now;
        uint8_t ready = 0U;
        s_last_status = s_sensor.VL53L4CD_CheckForDataReady(&ready);
        if (s_last_status != VL53L4CD_ERROR_NONE) {
            s_driver_errors++;
            s_running = false;
            Debug_Printf("sensor read-ready error=%u\r\n", s_last_status);
        } else if (ready != 0U) {
            VL53L4CD_Result_t result = {};
            s_last_status = s_sensor.VL53L4CD_GetResult(&result);
            if (s_last_status == VL53L4CD_ERROR_NONE) {
                s_last_status = s_sensor.VL53L4CD_ClearInterrupt();
            }

            if (s_last_status == VL53L4CD_ERROR_NONE) {
                s_measurements++;
                s_last_measurement_tick = HAL_GetTick();
                RangeQuality quality = Range_GetQuality(result.range_status);
                if (quality == RangeQuality::Valid) {
                    s_valid_results++;
                    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                } else if (quality == RangeQuality::Warning) {
                    s_warning_results++;
                } else {
                    s_range_errors++;
                }

                if ((VL53L4CD_PRINT_DECIMATION == 0U)
                    || ((s_measurements % VL53L4CD_PRINT_DECIMATION) == 0U)) {
                    Debug_Printf("RANGE quality=%s status=%u(%s) distance=%u mm signal=%u ambient=%u sigma=%u mm spads=%u\r\n",
                                 Range_GetQualityText(quality),
                                 result.range_status,
                                 Range_GetStatusText(result.range_status),
                                 result.distance_mm,
                                 result.signal_per_spad_kcps,
                                 result.ambient_per_spad_kcps,
                                 result.sigma_mm,
                                 result.number_of_spad);
                }
            } else {
                s_driver_errors++;
                s_running = false;
            }
        }
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_stats_tick) >= 1000U) {
        s_last_stats_tick = now;
        Debug_Printf("STAT reads=%lu valid=%lu warn=%lu range_err=%lu i2c_driver_err=%lu driver=%u last_age=%lu ms running=%u\r\n",
                     (unsigned long)s_measurements,
                     (unsigned long)s_valid_results,
                     (unsigned long)s_warning_results,
                     (unsigned long)s_range_errors,
                     (unsigned long)s_driver_errors,
                     s_last_status,
                     (unsigned long)(now - s_last_measurement_tick),
                     s_running ? 1U : 0U);
    }
}

static bool Sensor_Start(void)
{
    s_last_status = s_sensor.InitSensor(VL53L4CD_I2C_ADDRESS_8BIT);
    if (s_last_status == VL53L4CD_ERROR_NONE) {
        s_last_status = s_sensor.VL53L4CD_GetSensorId(&s_sensor_id);
    }
    if (s_last_status == VL53L4CD_ERROR_NONE) {
        s_last_status = s_sensor.VL53L4CD_SetRangeTiming(
            VL53L4CD_TIMING_BUDGET_MS,
            VL53L4CD_APP_INTERMEASUREMENT_MS);
    }
    if (s_last_status == VL53L4CD_ERROR_NONE) {
        s_last_status = s_sensor.VL53L4CD_StartRanging();
    }

    if (s_last_status == VL53L4CD_ERROR_NONE) {
        Debug_Printf("probe: VL53L4CD ready, id=0x%04X, ranging started (%u ms timing budget)\r\n",
                     s_sensor_id,
                     VL53L4CD_TIMING_BUDGET_MS);
        return true;
    }

    s_driver_errors++;
    Debug_Printf("probe: VL53L4CD init failed, driver=%u; retrying\r\n",
                 s_last_status);
    return false;
}

static RangeQuality Range_GetQuality(uint8_t status)
{
    if (status == 0U) {
        return RangeQuality::Valid;
    }
    if ((status == 1U) || (status == 2U) || (status == 6U)) {
        return RangeQuality::Warning;
    }
    return RangeQuality::Error;
}

static const char *Range_GetQualityText(RangeQuality quality)
{
    switch (quality) {
        case RangeQuality::Valid:
            return "VALID";
        case RangeQuality::Warning:
            return "WARNING";
        default:
            return "ERROR";
    }
}

static const char *Range_GetStatusText(uint8_t status)
{
    switch (status) {
        case 0U: return "valid";
        case 1U: return "sigma-high";
        case 2U: return "signal-low";
        case 3U: return "below-threshold";
        case 4U: return "phase-limit";
        case 5U: return "hardware-fail";
        case 6U: return "phase-valid-no-wrap-check";
        case 7U: return "wrapped-target";
        case 8U: return "processing-fail";
        case 9U: return "xtalk-fail";
        case 10U: return "interrupt-error";
        case 11U: return "merged-target";
        case 12U: return "signal-too-low";
        case 255U: return "boot-or-other-error";
        default: return "unknown";
    }
}

static void Debug_Printf(const char *format, ...)
{
    char buffer[224];
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
                            reinterpret_cast<uint8_t *>(buffer),
                            (uint16_t)length,
                            DEBUG_UART_TIMEOUT_MS);
}
