/**
 * @file task_tof_gate.c
 * @brief ToF distance-window debounce and freshness logic.
 */

#include "task_tof_gate.h"

void TaskTofGate_Init(TaskTofGate_t *gate)
{
    if (gate == NULL) {
        return;
    }

    gate->last_distance_mm = 0U;
    gate->last_sample_ms = 0U;
    gate->consecutive_valid_samples = 0U;
    gate->sample_available = false;
    gate->confirmed = false;
}

bool TaskTofGate_IsDistanceInWindow(uint16_t distance_mm)
{
    return (distance_mm >= TASK_TOF_GATE_MIN_DISTANCE_MM) &&
           (distance_mm <= TASK_TOF_GATE_MAX_DISTANCE_MM);
}

void TaskTofGate_Update(TaskTofGate_t *gate,
                        bool measurement_valid,
                        uint16_t distance_mm,
                        uint32_t timestamp_ms)
{
    if (gate == NULL) {
        return;
    }

    const bool previous_is_stale = gate->sample_available &&
        ((uint32_t)(timestamp_ms - gate->last_sample_ms) >
         TASK_TOF_GATE_STALE_TIMEOUT_MS);
    if (previous_is_stale) {
        gate->consecutive_valid_samples = 0U;
        gate->confirmed = false;
    }

    gate->last_distance_mm = distance_mm;
    gate->last_sample_ms = timestamp_ms;
    gate->sample_available = true;

    if (measurement_valid && TaskTofGate_IsDistanceInWindow(distance_mm)) {
        if (gate->consecutive_valid_samples < TASK_TOF_GATE_REQUIRED_SAMPLES) {
            gate->consecutive_valid_samples++;
        }
        gate->confirmed =
            gate->consecutive_valid_samples >= TASK_TOF_GATE_REQUIRED_SAMPLES;
    } else {
        gate->consecutive_valid_samples = 0U;
        gate->confirmed = false;
    }
}

bool TaskTofGate_IsConfirmed(const TaskTofGate_t *gate, uint32_t now_ms)
{
    if ((gate == NULL) || !gate->sample_available || !gate->confirmed) {
        return false;
    }
    return (uint32_t)(now_ms - gate->last_sample_ms) <=
           TASK_TOF_GATE_STALE_TIMEOUT_MS;
}
