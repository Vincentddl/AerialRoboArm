#include "app.h"

#include "Arduino.h"
#include "Wire.h"
#include "app_config.h"
#include "i2c.h"
#include "main.h"
#include "usart.h"
#include "vl53lx_class.h"

#include <cstdarg>
#include <cstdio>

static TwoWire s_wire(&hi2c1);
static VL53LX s_sensor(&s_wire, PIN_VL53_XSHUT);
static bool s_running;
static uint32_t s_frames;
static uint32_t s_driver_errors;
static uint32_t s_last_frame_tick;
static uint32_t s_last_poll_tick;
static uint32_t s_last_retry_tick;
static uint32_t s_last_stats_tick;
static VL53LX_Error s_last_status = VL53LX_ERROR_NONE;
static VL53LX_DeviceInfo_t s_device_info = {};

static void Debug_Printf(const char *format, ...);
static bool Sensor_Start(void);
static bool Range_IsValid(uint8_t status);
static void Print_Fixed1616(FixPoint1616_t value, char *buffer, size_t size);

extern "C" void App_Init(void)
{
    Debug_Printf("\r\nVL53L3CX STM32F103 multi-target monitor\r\n");
    Debug_Printf("I2C1 PB6/PB7 @ 400 kHz, address=0x%02X (7-bit 0x%02X)\r\n",
                 VL53L3CX_I2C_ADDRESS_8BIT,
                 VL53L3CX_I2C_ADDRESS_8BIT >> 1U);
    Debug_Printf("XSHUT=PB0, GPIO1=PB1, USART2=115200\r\n");

    s_wire.begin();
    s_sensor.begin();
    s_running = Sensor_Start();
}

extern "C" void App_Loop(void)
{
    uint32_t now = HAL_GetTick();

    if (!s_running) {
        if ((uint32_t)(now - s_last_retry_tick) >= VL53L3CX_RETRY_PERIOD_MS) {
            s_last_retry_tick = now;
            s_running = Sensor_Start();
        }
    } else if ((uint32_t)(now - s_last_poll_tick) >= VL53L3CX_POLL_PERIOD_MS) {
        s_last_poll_tick = now;
        uint8_t ready = 0U;
        s_last_status = s_sensor.VL53LX_GetMeasurementDataReady(&ready);
        if (s_last_status != VL53LX_ERROR_NONE) {
            s_driver_errors++;
            s_running = false;
            Debug_Printf("sensor read-ready error=%d\r\n", (int)s_last_status);
        } else if (ready != 0U) {
            VL53LX_MultiRangingData_t data = {};
            s_last_status = s_sensor.VL53LX_GetMultiRangingData(&data);
            if (s_last_status == VL53LX_ERROR_NONE) {
                s_frames++;
                s_last_frame_tick = HAL_GetTick();

                int16_t nearest = 32767;
                for (uint8_t index = 0U; index < data.NumberOfObjectsFound; ++index) {
                    const VL53LX_TargetRangeData_t &target = data.RangeData[index];
                    if (Range_IsValid(target.RangeStatus)
                        && (target.RangeMilliMeter >= 0)
                        && (target.RangeMilliMeter < nearest)) {
                        nearest = target.RangeMilliMeter;
                    }
                }

                if (nearest != 32767) {
                    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                }

                if ((VL53L3CX_PRINT_DECIMATION == 0U)
                    || ((s_frames % VL53L3CX_PRINT_DECIMATION) == 0U)) {
                    Debug_Printf("FRAME count=%u objects=%u nearest=%d mm\r\n",
                                 data.StreamCount,
                                 data.NumberOfObjectsFound,
                                 nearest == 32767 ? -1 : nearest);
                    for (uint8_t index = 0U; index < data.NumberOfObjectsFound; ++index) {
                        const VL53LX_TargetRangeData_t &target = data.RangeData[index];
                        char signal[20];
                        char ambient[20];
                        Print_Fixed1616(target.SignalRateRtnMegaCps, signal, sizeof(signal));
                        Print_Fixed1616(target.AmbientRateRtnMegaCps, ambient, sizeof(ambient));
                        Debug_Printf("  target[%u] status=%u(%s) distance=%d mm min=%d max=%d signal=%s Mcps ambient=%s Mcps\r\n",
                                     index,
                                     target.RangeStatus,
                                     Range_IsValid(target.RangeStatus) ? "VALID" : "INVALID",
                                     target.RangeMilliMeter,
                                     target.RangeMinMilliMeter,
                                     target.RangeMaxMilliMeter,
                                     signal,
                                     ambient);
                    }
                }
            } else {
                s_driver_errors++;
            }

            VL53LX_Error clear_status = s_sensor.VL53LX_ClearInterruptAndStartMeasurement();
            if (clear_status != VL53LX_ERROR_NONE) {
                s_last_status = clear_status;
                s_driver_errors++;
                s_running = false;
            }
        }
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_stats_tick) >= 1000U) {
        s_last_stats_tick = now;
        Debug_Printf("STAT frames=%lu errors=%lu driver=%d last_age=%lu ms running=%u\r\n",
                     (unsigned long)s_frames,
                     (unsigned long)s_driver_errors,
                     (int)s_last_status,
                     (unsigned long)(now - s_last_frame_tick),
                     s_running ? 1U : 0U);
    }
}

static bool Sensor_Start(void)
{
    s_last_status = s_sensor.InitSensor(VL53L3CX_I2C_ADDRESS_8BIT);
    if (s_last_status == VL53LX_ERROR_NONE) {
        s_last_status = s_sensor.VL53LX_GetDeviceInfo(&s_device_info);
    }
    if (s_last_status == VL53LX_ERROR_NONE) {
        s_last_status = s_sensor.VL53LX_StartMeasurement();
    }

    if (s_last_status == VL53LX_ERROR_NONE) {
        Debug_Printf("probe: VL53L3CX ready, type=0x%02X revision=%u.%u, ranging started\r\n",
                     s_device_info.ProductType,
                     s_device_info.ProductRevisionMajor,
                     s_device_info.ProductRevisionMinor);
        return true;
    }

    s_driver_errors++;
    Debug_Printf("probe: VL53L3CX init failed, driver=%d; retrying\r\n",
                 (int)s_last_status);
    return false;
}

static bool Range_IsValid(uint8_t status)
{
    return status == VL53LX_RANGESTATUS_RANGE_VALID
        || status == VL53LX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
}

static void Print_Fixed1616(FixPoint1616_t value, char *buffer, size_t size)
{
    uint32_t whole = value >> 16U;
    uint32_t fraction = ((value & 0xFFFFU) * 1000U) >> 16U;
    (void)snprintf(buffer, size, "%lu.%03lu",
                   (unsigned long)whole,
                   (unsigned long)fraction);
}

static void Debug_Printf(const char *format, ...)
{
    char buffer[256];
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
