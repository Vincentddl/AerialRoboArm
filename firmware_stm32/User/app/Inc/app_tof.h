/**
 * @file app_tof.h
 * @brief Continuous VL53L1X acquisition and gripper-distance confirmation.
 */

#ifndef APP_TOF_H
#define APP_TOF_H

#include "ara_def.h"

typedef enum {
    APP_RANGE_SENSOR_UNKNOWN = 0,
    APP_RANGE_SENSOR_VL53L1X,
} AppRangeSensorKind_t;

typedef struct {
    AraStatus_t last_status;
    AppRangeSensorKind_t sensor_kind;
    uint16_t sensor_id;
    uint16_t distance_mm;
    uint16_t signal_rate_kcps;
    uint16_t ambient_rate_kcps;
    uint8_t range_status;
    uint8_t consecutive_hits;
    bool online;
    bool measurement_valid;
    bool in_window;
    bool grasp_confirmed;
    uint32_t sample_timestamp_ms;
    uint32_t measured_period_ms;
    uint32_t sample_count;
    uint32_t error_count;
} AppTofSnapshot_t;

void App_Tof_Init(void);
bool App_Tof_GetSnapshot(AppTofSnapshot_t *snapshot);

#endif /* APP_TOF_H */
