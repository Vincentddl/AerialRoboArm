/**
 * @file task_tof_gate.h
 * @brief Sensor-independent ToF confirmation gate for the gripper.
 */

#ifndef TASK_TOF_GATE_H
#define TASK_TOF_GATE_H

#include "ara_def.h"

#define TASK_TOF_GATE_MIN_DISTANCE_MM       (50U)
#define TASK_TOF_GATE_MAX_DISTANCE_MM       (120U)
#define TASK_TOF_GATE_SAMPLE_PERIOD_MS      (20U)
#define TASK_TOF_GATE_REQUIRED_SAMPLES      (2U)
#define TASK_TOF_GATE_STALE_TIMEOUT_MS      (60U)

typedef struct {
    uint16_t last_distance_mm;
    uint32_t last_sample_ms;
    uint8_t consecutive_valid_samples;
    bool sample_available;
    bool confirmed;
} TaskTofGate_t;

void TaskTofGate_Init(TaskTofGate_t *gate);

bool TaskTofGate_IsDistanceInWindow(uint16_t distance_mm);

void TaskTofGate_Update(TaskTofGate_t *gate,
                        bool measurement_valid,
                        uint16_t distance_mm,
                        uint32_t timestamp_ms);

bool TaskTofGate_IsConfirmed(const TaskTofGate_t *gate, uint32_t now_ms);

#endif /* TASK_TOF_GATE_H */
