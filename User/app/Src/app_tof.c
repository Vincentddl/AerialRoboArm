/**
 * @file app_tof.c
 * @brief Background gripper ranging task for VL53L1X.
 */

#include "app_tof.h"

#include "task_tof_gate.h"
#include "ara_prio.h"
#include "drv_vl53l1x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include <string.h>

#define TOF_RETRY_DELAY_MS       (500U)
#define TOF_RESTART_ERROR_LIMIT  (5U)

static osThreadId_t s_tof_handle = NULL;
static AppTofSnapshot_t s_snapshot;

static void snapshot_set_offline(AraStatus_t status)
{
    taskENTER_CRITICAL();
    s_snapshot.last_status = status;
    s_snapshot.online = false;
    s_snapshot.measurement_valid = false;
    s_snapshot.in_window = false;
    s_snapshot.grasp_confirmed = false;
    s_snapshot.range_status = 0xFFU;
    s_snapshot.consecutive_hits = 0U;
    taskEXIT_CRITICAL();
}

static void snapshot_publish(const DrvVL53L1X_Context_t *sensor,
                             const DrvVL53L1X_Result_t *sample,
                             const TaskTofGate_t *gate,
                             bool measurement_valid,
                             uint32_t previous_timestamp_ms)
{
    taskENTER_CRITICAL();
    s_snapshot.last_status = ARA_OK;
    s_snapshot.sensor_kind = APP_RANGE_SENSOR_VL53L1X;
    s_snapshot.sensor_id = sensor->sensor_id;
    s_snapshot.distance_mm = sample->distance_mm;
    s_snapshot.signal_rate_kcps = sample->signal_rate_kcps;
    s_snapshot.ambient_rate_kcps = sample->ambient_rate_kcps;
    s_snapshot.range_status = sample->range_status;
    s_snapshot.consecutive_hits = gate->consecutive_valid_samples;
    s_snapshot.online = true;
    s_snapshot.measurement_valid = measurement_valid;
    s_snapshot.in_window = TaskTofGate_IsDistanceInWindow(sample->distance_mm);
    s_snapshot.grasp_confirmed =
        TaskTofGate_IsConfirmed(gate, sample->timestamp_ms);
    s_snapshot.sample_timestamp_ms = sample->timestamp_ms;
    s_snapshot.measured_period_ms = (previous_timestamp_ms == 0U) ? 0U :
        (uint32_t)(sample->timestamp_ms - previous_timestamp_ms);
    s_snapshot.sample_count++;
    taskEXIT_CRITICAL();
}

static void TofTaskEntry(void *argument)
{
    (void)argument;
    DrvVL53L1X_Context_t sensor;
    TaskTofGate_t gate;
    uint32_t previous_timestamp_ms = 0U;
    uint8_t consecutive_errors = 0U;

    memset(&sensor, 0, sizeof(sensor));
    TaskTofGate_Init(&gate);

    for (;;) {
        if (!sensor.initialized) {
            AraStatus_t status = DrvVL53L1X_Init(
                &sensor,
                DRV_VL53L1X_DEFAULT_ADDRESS_7BIT,
                DRV_VL53L1X_DEFAULT_TIMEOUT_MS);
            if (status == ARA_OK) {
                status = DrvVL53L1X_StartContinuous(&sensor);
            }
            if (status != ARA_OK) {
                snapshot_set_offline(status);
                taskENTER_CRITICAL();
                s_snapshot.error_count++;
                taskEXIT_CRITICAL();
                memset(&sensor, 0, sizeof(sensor));
                vTaskDelay(pdMS_TO_TICKS(TOF_RETRY_DELAY_MS));
                continue;
            }

            TaskTofGate_Init(&gate);
            previous_timestamp_ms = 0U;
            consecutive_errors = 0U;
            taskENTER_CRITICAL();
            s_snapshot.sensor_id = sensor.sensor_id;
            s_snapshot.online = true;
            s_snapshot.last_status = ARA_BUSY;
            taskEXIT_CRITICAL();
        }

        DrvVL53L1X_Result_t sample;
        AraStatus_t status = DrvVL53L1X_ReadIfReady(&sensor, &sample);
        if (status == ARA_BUSY) {
            vTaskDelay(pdMS_TO_TICKS(ARA_PERIOD_TOF_POLL_MS));
            continue;
        }
        if (status != ARA_OK) {
            consecutive_errors++;
            taskENTER_CRITICAL();
            s_snapshot.last_status = status;
            s_snapshot.error_count++;
            taskEXIT_CRITICAL();
            if (consecutive_errors >= TOF_RESTART_ERROR_LIMIT) {
                (void)DrvVL53L1X_Stop(&sensor);
                memset(&sensor, 0, sizeof(sensor));
                snapshot_set_offline(status);
            }
            vTaskDelay(pdMS_TO_TICKS(TASK_TOF_GATE_SAMPLE_PERIOD_MS));
            continue;
        }

        consecutive_errors = 0U;
        const bool valid = DrvVL53L1X_IsRangeValid(&sample);
        TaskTofGate_Update(&gate, valid, sample.distance_mm, sample.timestamp_ms);
        snapshot_publish(&sensor,
                         &sample,
                         &gate,
                         valid,
                         previous_timestamp_ms);
        previous_timestamp_ms = sample.timestamp_ms;
        vTaskDelay(pdMS_TO_TICKS(ARA_PERIOD_TOF_POLL_MS));
    }
}

void App_Tof_Init(void)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.last_status = ARA_ERR_DISCONNECTED;
    s_snapshot.sensor_kind = APP_RANGE_SENSOR_VL53L1X;

    const osThreadAttr_t attributes = {
        .name = "TofTask",
        .stack_size = ARA_STACK_TOF_BYTES,
        .priority = ARA_PRIO_TOF,
    };
    s_tof_handle = osThreadNew(TofTaskEntry, NULL, &attributes);
}

bool App_Tof_GetSnapshot(AppTofSnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    taskENTER_CRITICAL();
    *snapshot = s_snapshot;
    taskEXIT_CRITICAL();
    return true;
}
